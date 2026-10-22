"""Batch marking of captured poses, with saved drafts and image-only suggestions."""
import json
import numpy as np
from PySide6.QtWidgets import QCheckBox, QPushButton, QMessageBox
from PySide6.QtCore import QTimer
from point_inspector import PointInspector
from campaign_storage import atomic, encoded


class CampaignInspector(PointInspector):
    def __init__(self,store,index,item,parent,on_saved):
        self.store=store;self.pose_index=index;self.on_saved=on_saved;self.draft_enabled=False
        super().__init__(lambda:item,parent,manual_mode=True,manual_callback=self.save_points)
        self.setWindowTitle(f'Campaign — pose {index+1}/{len(store.doc["poses"])} — place physical joints')
        self.capture_button.hide();self.save_button.hide()
        self.profile_tabs.setTabText(0,'Batch review')
        self.sweep_profile_button.setText('Save & next pose')
        self.auto_next=QCheckBox('After a click, select the next joint');self.auto_next.setChecked(True)
        self.profile_tabs.widget(0).layout().insertWidget(0,self.auto_next)
        self.suggest_button=QPushButton('Suggest from previous annotated image')
        self.profile_tabs.widget(0).layout().insertWidget(1,self.suggest_button);self.suggest_button.clicked.connect(self.suggest)
        self.profile_tabs.setFixedHeight(205)
        path=self.store.path(self.store.doc['poses'][index]['path'])/'draft.json'
        data=json.loads(path.read_text()) if path.exists() else None
        annotations=self.store.doc['poses'][index].get('annotations')
        if data:
            self.mode.setCurrentIndex(data.get('mode',0));self.manual=data['manual'];self.excluded=set(data['excluded'])
        elif annotations:
            self.mode.setCurrentIndex(int(annotations['mode']=='centres'))
            self.manual={r['id']:r['physical_px'] for r in annotations['rows'] if r['physical_px'] is not None}
            self.excluded={r['id'] for r in annotations['rows'] if not r['included']}
        elif not self.manual:self.auto_place_dots(silent=True)
        if (data or annotations) and self.mode.currentIndex()==0:
            self.edit_hint.setText('Showing your saved points as JOINTS. If you placed them on the black element dots, press Auto-place marker dots to re-mark them as element centres.')
        self.rebuild();self.table.selectRow(0);self.draft_enabled=True
        self.stationary.toggled.connect(self.persist_draft)
        self.caption.setText(f'Pose {index+1}: {store.doc["poses"][index]["command"]["label"]}. Place every included joint, or explicitly uncheck obscured points. Drafts save after each edit; confirm the image before Save & next.'+(' Model prediction unavailable at current k₂; annotation remains available.' if item['result'].get('model_source')=='equilibrium unavailable' else ''))

    def manual_readiness(self,*args):
        super().manual_readiness(*args)
        if not hasattr(self,'rows'):return
        missing=[r for r in self.rows if r['included'] and r['physical_px'] is None]
        if missing:
            self.sweep_profile_button.setEnabled(False)
            self.manual_status.setText(f'{len(missing)} included points still need placement. Uncheck only genuinely obscured points; at least four are required.')

    def rebuild(self,*args):
        super().rebuild(*args)
        if self.draft_enabled:self.persist_draft()

    def persist_draft(self,*_):
        if not self.draft_enabled:return
        path=self.store.path(self.store.doc['poses'][self.pose_index]['path'])/'draft.json'
        try:atomic(path,encoded(dict(manual=self.manual,excluded=sorted(self.excluded),mode=self.mode.currentIndex())))
        except OSError as exc:
            self.caption.setText('Draft could not be saved: '+str(exc))

    def clicked_image(self,event):
        row=self.table.currentRow();before=dict(self.manual)
        super().clicked_image(event)
        if hasattr(self,'auto_next') and self.auto_next.isChecked() and self.manual!=before:
            self.table.selectRow(min(row+1,len(self.rows)-1))

    def apply_profile(self,save=False,sweep=False):
        try:self.save_points(self.source_item,self.rows,'centres' if self.mode.currentIndex() else 'joints',self.stationary.isChecked())
        except (ValueError,OSError) as exc:QMessageBox.warning(self,'Points not saved',str(exc))

    def save_points(self,item,rows,mode,stationary):
        from manual_analysis import complete_capture
        if any(r['included'] and r['physical_px'] is None for r in rows):raise ValueError('Place or explicitly exclude every point first.')
        frozen=complete_capture(item,rows,mode,stationary,allow_missing_model=True)
        self.store.set_observation(self.pose_index,frozen['observed'],'Manual — reviewed',dict(rows=rows,mode=mode,stationary=stationary))
        self.persist_draft()
        self.on_saved(self.pose_index)
        QTimer.singleShot(0,self.close)

    def suggest(self):
        import cv2
        earlier=next((p for p in reversed(self.store.doc['poses'][:self.pose_index]) if p.get('annotations') and
                      p['annotations']['mode']==('centres' if self.mode.currentIndex() else 'joints')),None)
        if not earlier:
            self.caption.setText('Annotate an earlier pose first. Suggestions follow image features only; every suggested point needs your review.');return
        previous=self.store.load_item(earlier['number']-1)['rgb'];current=self.source_item['rgb']
        rows=[r for r in earlier['annotations']['rows'] if r['included'] and r['physical_px'] is not None]
        pts=np.array([r['physical_px'] for r in rows],np.float32).reshape(-1,1,2)
        a=cv2.cvtColor(previous,cv2.COLOR_RGB2GRAY);b=cv2.cvtColor(current,cv2.COLOR_RGB2GRAY)
        moved,ok,err=cv2.calcOpticalFlowPyrLK(a,b,pts,None,winSize=(31,31),maxLevel=4)
        if moved is None:self.caption.setText('No reliable image matches. Place points manually.');return
        back,rev,_=cv2.calcOpticalFlowPyrLK(b,a,moved,None,winSize=(31,31),maxLevel=4)
        if back is None:self.caption.setText('Image matches could not be verified. Place points manually.');return
        good=(ok.ravel()>0)&(rev.ravel()>0)&(err.ravel()<20)&(np.linalg.norm(back-pts,axis=2).ravel()<1.5)
        h,w=b.shape;count=0
        for row,point,valid in zip(rows,moved.reshape(-1,2),good):
            if valid and np.isfinite(point).all() and 16<=point[0]<w-16 and 16<=point[1]<h-16:
                before=cv2.getRectSubPix(a,(31,31),tuple(map(float,row['physical_px'])))
                after=cv2.getRectSubPix(b,(31,31),tuple(map(float,point)))
                # Flow's internal error can look small in a repetitive/blurred
                # patch. Require direct local texture agreement as well.
                if min(float(before.std()),float(after.std()))<6:continue
                if float(cv2.matchTemplate(after,before,cv2.TM_CCOEFF_NORMED)[0,0])<.85:continue
                if row['id'] not in self.manual:self.manual[row['id']]=point.tolist();count+=1
        self.stationary.setChecked(False);self.rebuild()
        self.caption.setText(f'{count} image-based draft suggestions added. Inspect and correct every point before confirming. MuJoCo points were not used as physical measurements.')

    def closeEvent(self,event):
        self.persist_draft();event.accept()
