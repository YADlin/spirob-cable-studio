"""Atomic, bounded campaign records inside the owning session's data folder."""
import base64
import copy
import hashlib
import io
import json
import os
import shutil
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path
import numpy as np
from PIL import Image


def digest(data):return hashlib.sha256(data).hexdigest()


def encoded(value):
    def default(v):
        if isinstance(v,np.ndarray):return v.tolist()
        if isinstance(v,np.generic):return v.item()
        raise TypeError(type(v).__name__)
    return (json.dumps(value,indent=2,allow_nan=False,default=default)+'\n').encode()


def atomic(path,data):
    path=Path(path);tmp=path.with_name(path.name+'.tmp')
    with tmp.open('wb') as stream:
        stream.write(data);stream.flush();os.fsync(stream.fileno())
    tmp.replace(path)


class CampaignStore:
    def __init__(self,root,recover=True):
        self.root=Path(root).resolve();self.lock=threading.RLock()
        self.doc=json.loads((self.root/'campaign.json').read_text())
        if self.doc.get('kind')!='spirob_campaign' or self.doc.get('schema')!=1:raise ValueError('Not a supported SpiRob campaign.')
        if recover:self.recover_committed_frames()

    @classmethod
    def create(cls,parent,plan,rows,item,config,fit,mode='automatic',budget_mib=256):
        root=Path(parent)/('campaign_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:6])
        root.mkdir(parents=True,exist_ok=False)
        model=Path(config['model']).resolve()
        files=[p for p in model.parent.rglob('*') if p.is_file()]
        if sum(p.stat().st_size for p in files)>32*1048576:raise ValueError('Model snapshot exceeds 32 MiB.')
        if any(p.is_symlink() for p in model.parent.rglob('*')):raise ValueError('Model snapshots must contain real files, not symlinks.')
        shutil.copytree(model.parent,root/'model')
        cfg=copy.deepcopy(config);cfg['model']='model/'+model.name
        cfg['comparison_setup']=copy.deepcopy(item['setup'])
        fixed=copy.deepcopy(fit);fixed['candidate']['parameters']=copy.deepcopy(item['parameters'])
        metadata=dict(kind='spirob_campaign',schema=1,version='0.9.1',created_utc=datetime.now(timezone.utc).isoformat(),
            session=item['packet']['session'],state='collecting',reason='',mode=mode,plan=plan,rows=rows,
            config=cfg,fit=fixed,d2=item['d2'],poses=[],image_bytes=0,budget_bytes=int(budget_mib*1048576),
            model_hashes={p.relative_to(root).as_posix():digest(p.read_bytes()) for p in (root/'model').rglob('*') if p.is_file()},
            retention='Owned by the session. Save / Discard is decided after disconnect; interrupted runs remain pending.',
            settling='Target arrival within 0.5 mm, encoder range at most 0.15 mm over 2 seconds, plus the chosen dwell. '
                     'Encoder hold alone does not prove absence of physical vibration. Review the retained photograph.')
        atomic(root/'campaign.json',encoded(metadata))
        store=cls(root,recover=False)
        # Fail before any campaign motion if the copied model cannot stand alone.
        from spirob_analysis.model import SpiRobModel
        SpiRobModel(store.config()['model'])
        return store

    def recover_committed_frames(self):
        """Recover the bounded write-before-index crash window; never move hardware."""
        from campaign import held_packet
        changed=False
        for index in range(len(self.doc['poses']),len(self.doc['plan']['poses'])):
            relative='poses/'+f'{index+1:04d}';folder=self.path(relative)
            if not (folder/'frame.jpg').is_file() or not (folder/'snapshot.json').is_file():break
            jpeg=(folder/'frame.jpg').read_bytes();body=(folder/'snapshot.json').read_bytes()
            try:
                snap=json.loads(body);pose=self.doc['plan']['poses'][index];packet=snap['packet']
                with Image.open(io.BytesIO(jpeg)) as im:im.verify()
                if packet['session']!=self.doc['session'] or not held_packet(packet,pose['lengths_mm'],pose['settle_s']+pose['dwell_s'],self.doc['config']['live']['max_input_gap_s']):break
            except (ValueError,KeyError,OSError):break
            self.doc['poses'].append(dict(number=index+1,path=relative,frame_sha256=digest(jpeg),snapshot_sha256=digest(body),
                frame_index=packet['frame_index'],t_s=packet['t_s'],command=copy.deepcopy(pose),status='Needs points',
                reason='Recovered committed frame after interrupted index write; review physical joints',
                observation=None,annotation_revision=0,excluded=pose.get('role')=='return',
                exclusion_reason='Return image retained for comparison; include it to fit this pose.' if pose.get('role')=='return' else ''))
            self.doc['image_bytes']+=len(jpeg);changed=True
        if self.doc['state'] in ('collecting','next','ack','moving','saving'):
            self.doc.update(state='interrupted',reason='Acquisition interrupted. Review retained poses; motion will not resume.');changed=True
        if changed:
            self.invalidate_analysis();self.save()

    def save(self):
        with self.lock:atomic(self.root/'campaign.json',encoded(self.doc))

    def state(self,state,reason,events=None):
        with self.lock:
            self.doc.update(state=state,reason=reason)
            if events is not None:self.doc['events']=list(events)
            self.save()

    def config(self):
        cfg=copy.deepcopy(self.doc['config']);cfg['model']=str(self.path(cfg['model']));return cfg

    def path(self,relative):
        p=(self.root/relative).resolve()
        if not p.is_relative_to(self.root):raise ValueError('Campaign path leaves its data folder.')
        return p

    def verify_model(self):
        for name,sha in self.doc['model_hashes'].items():
            if digest(self.path(name).read_bytes())!=sha:raise ValueError('Frozen campaign model has changed: '+name)

    def capture(self,index,item,pose):
        with self.lock:
            if index!=len(self.doc['poses']):raise ValueError('Capture order mismatch.')
            packet=copy.deepcopy(item['packet']);jpeg=base64.b64decode(packet.pop('image_b64'),validate=True)
            if len(jpeg)>8*1048576:raise ValueError('Image exceeds 8 MiB.')
            # Decode once as a format/integrity check, never recompress or resize.
            with Image.open(io.BytesIO(jpeg)) as im:im.verify()
            if self.doc['image_bytes']+len(jpeg)>self.doc['budget_bytes']:raise ValueError('Campaign image budget reached. Prior poses are retained.')
            if shutil.disk_usage(self.root).free<max(32*1048576,4*len(jpeg)):raise OSError('Disk space reserve reached.')
            window=pose['settle_s']+pose['dwell_s']+1
            packet['motor']=[r for r in packet['motor'] if packet['t_s']-window<=r['t_s']<=packet['t_s']+1]
            snap={k:copy.deepcopy(item[k]) for k in ('parameters','d2','setup','displayed_age_s','recorded_demo') if k in item}
            snap.update(packet=packet,message='Campaign held pose',observed=None,setup_preview=False)
            number=f'{index+1:04d}';folder=self.root/'poses'/number;folder.mkdir(parents=True,exist_ok=True)
            body=encoded(snap)
            if len(body)>2*1048576:raise ValueError('Pose metadata exceeds 2 MiB.')
            atomic(folder/'frame.jpg',jpeg);atomic(folder/'snapshot.json',body)
            info=dict(number=index+1,path='poses/'+number,frame_sha256=digest(jpeg),snapshot_sha256=digest(body),
                frame_index=packet['frame_index'],t_s=packet['t_s'],command=copy.deepcopy(pose),
                status='Captured',reason='Waiting for automatic measurement' if self.doc['mode']=='automatic' else 'Waiting for manual joints',
                observation=None,annotation_revision=0,excluded=pose.get('role')=='return',
                exclusion_reason='Return image retained for comparison; include it to fit this pose.' if pose.get('role')=='return' else '')
            self.doc['poses'].append(info);self.doc['image_bytes']+=len(jpeg);self.save()
            return info

    def load_item(self,index):
        pose=self.doc['poses'][index];folder=self.path(pose['path'])
        jpeg=(folder/'frame.jpg').read_bytes();body=(folder/'snapshot.json').read_bytes()
        if digest(jpeg)!=pose['frame_sha256'] or digest(body)!=pose['snapshot_sha256']:raise ValueError('Pose image or encoder snapshot changed.')
        item=json.loads(body);item['packet']['image_b64']=base64.b64encode(jpeg).decode()
        item['rgb']=np.array(Image.open(io.BytesIO(jpeg)).convert('RGB'));return item

    def prepared(self,index):
        from manual_analysis import prepare_capture
        self.verify_model()
        return prepare_capture(self.load_item(index),self.config(),self.doc['fit'],allow_missing_model=True)

    def annotation(self,index,rows,mode,stationary):
        from manual_analysis import complete_capture
        with self.lock:
            item=self.prepared(index)
            frozen=complete_capture(item,rows,mode,stationary,allow_missing_model=True)
            self.set_observation(index,frozen['observed'],'Manual',dict(rows=rows,mode=mode,stationary=stationary))
            return frozen

    def invalidate_analysis(self):
        self.doc.pop('analysis',None)
        previous=self.root/'fit_results.json'
        if previous.exists():previous.replace(self.root/'fit_results_previous.json')
        for name in ('fit_results.csv','fit_results.png'):
            (self.root/name).unlink(missing_ok=True)

    def set_observation(self,index,observation,source,annotations=None):
        with self.lock:
            pose=self.doc['poses'][index]
            pose.update(observation=copy.deepcopy(observation),status='Ready',reason=source,
                annotation_revision=pose['annotation_revision']+1)
            if annotations is not None:pose['annotations']=copy.deepcopy(annotations)
            self.invalidate_analysis();self.save()

    def exclude(self,index,value,reason='User excluded this pose'):
        with self.lock:
            pose=self.doc['poses'][index];pose['excluded']=bool(value);pose['exclusion_reason']=reason
            self.invalidate_analysis();self.save()
