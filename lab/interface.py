"""Window and background preview loop. Read the three experiment modules first.

Qt draws controls; Preview computes comparisons; WorkstationSession transports
commands. Only actuation.py on the laptop can give commands to the motor worker.
"""
import base64
import copy
import hashlib
import io
import json
import queue
import threading
import time
import csv
from contextlib import nullcontext
from datetime import datetime, timezone
from pathlib import Path
import numpy as np
from PIL import Image
from PySide6.QtCore import QThread, Signal, QTimer, Qt, QPointF, QUrl
from PySide6.QtGui import QImage, QPainter, QPen, QColor, QPolygonF, QDesktopServices
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QDoubleSpinBox, QCheckBox, QComboBox,
    QSpinBox, QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
    QFileDialog, QMessageBox, QScrollArea, QStackedWidget, QSplitter, QTabWidget, QSizePolicy)
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from appearance import apply_theme, theme_figure
from analysis import Comparator
from spirob_analysis.model import SpiRobModel
from connection import WorkstationSession, CameraSetupSession, camera_get
from run_storage import review_entries, finish_review
from spirob_analysis.measurement import measure


def recorded_packet(config, step):
    from spirob_analysis.session import Session
    session = Session(config['session'])
    jpeg, provenance = session.held_frame(step)
    t = provenance['t_s']
    motor, rest = [], session.trial['rest_lengths_mm']
    for axis, (times, lengths) in enumerate(session.motor_series()):
        for ti, length in zip(times, lengths):
            if t-4 <= ti <= t+1:
                motor.append(dict(axis=axis, t_s=float(ti), length_mm=float(length+rest[axis]),
                    target_mm=float(session.displacement_mm(step)[axis]+rest[axis]), state='HOLDING'))
    return dict(state='RECORDING', session=session.run['session_id'], t_s=t, frame_index=provenance['frame_index'],
                motor=motor, motor_active=True, rest_lengths_mm=rest, frame_age_s=0., motor_age_s=[0., 0.],
                image_b64=base64.b64encode(jpeg).decode(), phase=f'RECORDED HOLD {step}', clock_uncertainty_s=None)


class Preview(QThread):
    """Network reader keeps one latest frame; analysis never blocks that reader."""
    updated = Signal(object)
    video = Signal(object)
    applied = Signal(object)

    def __init__(self, config, fit, session=None, demo=False, setup_only=False):
        super().__init__()
        self.config, self.fit, self.session, self.demo = config, fit, session, demo
        self.setup_only = setup_only
        self.stop = threading.Event()
        self.requests = queue.Queue()
        self.incoming = queue.Queue(maxsize=1)
        self.engine = None
        self.step = 2
        self.last_key = None
        self.last_result = None
        self.latest_scan = None
        self.reader_error = None
        self.tracker = None
        self.analysis_mode = config.get('analysis_mode','vision')
        self.vision_method = config.get('vision_method','silhouette')
        self.vision_epoch = 0
        self.vision = None
        self.last_vision_message = 'Image processing started; camera and MuJoCo continue independently.'
        self.reference_revision = None
        self.reference = None

    def read_video(self):
        while not self.stop.is_set():
            began = time.monotonic()
            try:
                packet = camera_get(self.session.url, self.session.token, '/packet', timeout=1)
                if 'image_b64' in packet:
                    transport = time.monotonic()-began
                    packet['frame_age_s'] += transport
                    packet['motor_age_s'] = [None if x is None else x+transport for x in packet['motor_age_s']]
                    rgb = np.array(Image.open(io.BytesIO(base64.b64decode(packet['image_b64']))).convert('RGB'))
                    item = dict(rgb=rgb, packet=packet, result=None, observed=None,
                        message='Live video; waiting for a current comparison', raw=True,
                        displayed_age_s=packet['frame_age_s'] + time.monotonic()-began-transport,
                        received=time.monotonic(), processing_s=time.monotonic()-began)
                    self.video.emit(item)
                    try: self.incoming.get_nowait()
                    except queue.Empty: pass
                    self.incoming.put_nowait(item)
                    self.reader_error = None
                else:
                    self.reader_error = packet.get('message', 'Waiting for camera')
            except Exception as exc:
                self.reader_error = 'Video transport: '+str(exc)
            self.stop.wait(max(.01, 1/self.config['live']['poll_hz']-(time.monotonic()-began)))

    def publish_item(self, item, log):
        result=item.get('result')
        compact=None if result is None else {k:v for k,v in result.items()
            if k not in ('observation','predicted_curve_px','predicted_points_px','landmarks','landmark_rows','setup')}
        log.write(json.dumps(dict(type='vision_comparison' if item.get('vision_result') else 'model_comparison',
            session=item['packet']['session'],frame_index=item['packet']['frame_index'],
            frame_age_s=item['displayed_age_s'],processing_s=item['processing_s'],
            analysis_mode=self.analysis_mode,result=compact,message=item['message']),allow_nan=False)+'\n')
        log.flush();self.updated.emit(item)

    def drain_vision(self, log):
        try:item,observed,error,elapsed,epoch=self.vision.finished.get_nowait()
        except queue.Empty:return
        if epoch!=self.vision_epoch or self.analysis_mode!='vision':return
        result=item.get('result')
        if not error and observed is not None and result:
            try:result=self.engine.attach_measurement(result,observed,item['packet'],self.demo)
            except (ValueError,KeyError) as exc:error=str(exc)
        if error or observed is None:
            self.engine.history.clear()
            message='Image measurement unavailable: '+str(error or 'no extracted points')+'. Manual points mode remains available.'
        else:message=result['message'] if result else 'Image measured; waiting for referenced encoder inputs.'
        age=item['displayed_age_s']+time.monotonic()-item['received']
        completed=dict(item,result=result,observed=observed,message=message,vision_result=True,
            vision_error=error,displayed_age_s=0. if self.demo else age,received=time.monotonic(),
            processing_s=item['processing_s']+elapsed,vision_processing_s=elapsed)
        self.last_vision_message=message
        self.publish_item(completed,log)

    def reference_result(self, packet, error):
        # Keep an honestly labelled reference visible while feedback is absent.
        # This separate model never resets the dynamic model's state.
        if self.reference_revision!=self.engine.revision:
            cfg=dict(self.config,comparison_setup=self.engine.setup())
            fit=copy.deepcopy(self.fit);fit['candidate']['parameters']=dict(self.engine.p)
            self.reference=Comparator(cfg,fit).compare_setup(packet,None)
            self.reference_revision=self.engine.revision
        result=copy.deepcopy(self.reference)
        result.update(t_s=packet['t_s'],revision=self.engine.revision,
            message='Reference backbone only — '+str(error),model_ready=False)
        return result

    def run(self):
        from live_vision import VisionWorker
        reader=None
        self.engine=Comparator(self.config,self.fit)
        self.configure_tracker()
        self.vision=VisionWorker(measure,self.config);self.vision.start()
        if not self.demo:
            reader=threading.Thread(target=self.read_video,daemon=True);reader.start()
        log_dir=getattr(self.session,'log_dir',None)
        if log_dir:Path(log_dir).mkdir(parents=True,exist_ok=True)
        name=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_analysis.jsonl'
        last_error=0.
        try:
            with ((Path(log_dir)/name).open('x') if log_dir else nullcontext(NullLog())) as log:
                log.write(json.dumps(dict(type='metadata',version='0.9.1',config=self.config,
                    parameters=self.engine.p,d2=self.engine.d2,recorded_demo=self.demo,
                    source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in [*Path('.').glob('*.py'),*Path('spirob_live').glob('*.py')]}))+'\n')
                log.flush();logged_revision=None
                while not self.stop.is_set():
                    began=time.monotonic()
                    try:
                        refresh=self.handle_requests()
                        self.drain_vision(log)
                        if self.demo:
                            if self.last_key is not None and not refresh:
                                self.stop.wait(.025);continue
                            packet=recorded_packet(self.config,self.step)
                            rgb=np.array(Image.open(io.BytesIO(base64.b64decode(packet['image_b64']))).convert('RGB'))
                            item=dict(packet=packet,rgb=rgb,received=time.monotonic(),displayed_age_s=0.)
                        else:
                            try:item=self.incoming.get(timeout=.03)
                            except queue.Empty:
                                if self.reader_error:raise RuntimeError(self.reader_error)
                                continue
                            packet=dict(item['packet']);rgb=item['rgb']
                            delay=time.monotonic()-item['received']
                            packet['frame_age_s']+=delay
                            packet['motor_age_s']=[None if v is None else v+delay for v in packet['motor_age_s']]
                        key=(packet['session'],packet['frame_index'])
                        if key==self.last_key and not refresh:continue
                        self.last_key=key
                        try:
                            if self.setup_only:result=self.engine.compare_setup(packet,None)
                            elif self.demo:result=self.engine.compare_still(packet,None)
                            else:result=self.engine.compare(packet,None)
                            result['model_ready']=not result.get('setup_preview',False)
                        except Exception as exc:result=self.reference_result(packet,exc)
                        if self.analysis_mode=='manual' and result.get('model_ready'):
                            message='Manual points mode — camera + MuJoCo running; capture a held pose to place points and sweep. Image processing is off.'
                        elif self.analysis_mode=='vision' and result.get('model_ready'):
                            message=self.last_vision_message
                        else:message=result['message']
                        self.last_result=result
                        elapsed=time.monotonic()-began
                        age=0. if self.demo else item['displayed_age_s']+time.monotonic()-item['received']
                        if logged_revision!=self.engine.revision:
                            log.write(json.dumps(dict(type='comparison_setup',revision=self.engine.revision,
                                setup=self.engine.setup(),parameters=self.engine.p),allow_nan=False)+'\n')
                            logged_revision=self.engine.revision
                        frame=dict(rgb=rgb,packet=packet,result=result,observed=None,message=message,
                            parameters=dict(self.engine.p),d2=self.engine.d2,
                            landmarks=result['landmarks'],landmark_rows=[],setup=self.engine.setup(),
                            recorded_demo=self.demo,setup_preview=self.setup_only,analysis_mode=self.analysis_mode,
                            displayed_age_s=age,received=time.monotonic(),processing_s=elapsed)
                        if not self.demo or self.analysis_mode=='manual':self.publish_item(frame,log)
                        if self.analysis_mode=='vision':self.vision.submit(frame,self.tracker,self.vision_epoch)
                    except Exception as exc:
                        self.last_result=None
                        if time.monotonic()-last_error>2:
                            log.write(json.dumps(dict(type='error',error=type(exc).__name__+': '+str(exc)))+'\n');log.flush()
                            last_error=time.monotonic()
                        self.updated.emit(dict(message=str(exc),result=None))
                    self.stop.wait(.005)
        finally:
            self.stop.set();self.vision.stop.set();self.vision.clear_pending();self.vision.join(1.)
            if reader:reader.join(1.5)

    def configure_tracker(self, reference=None):
        from joint_tracking import ManualJointTracker
        profile = self.engine.profile
        if self.analysis_mode == 'vision' and self.vision_method == 'dots':
            from dot_tracking import DotTracker, expected_spacing_px
            self.tracker = DotTracker(expected_spacing_px(self.engine.model, self.engine.p['pixels_per_m']),
                                      self.config.get('marker_base_px'))
            return
        if self.analysis_mode != 'vision' or not profile or profile.get('measurement_mode') != 'manual_joints':
            self.tracker = None
        elif reference is not None or self.tracker is None or self.tracker.profile['calibration_id'] != profile['calibration_id']:
            self.tracker = ManualJointTracker(profile, self.config, reference)

    def handle_requests(self):
        refresh = False
        while True:
            try: action, values = self.requests.get_nowait()
            except queue.Empty: return refresh
            if action == 'parameters':
                self.vision_epoch+=1
                self.engine.set_parameters(*values); refresh = True
                self.applied.emit(dict(kind='parameters', k2=self.engine.p['k2'], d2=self.engine.d2,
                    stiffness_ratio=self.engine.p['stiffness_ratio'], stiffness_exponent=self.engine.p['stiffness_exponent'], revision=self.engine.revision))
            elif action == 'joint_values':
                self.vision_epoch+=1
                self.engine.set_joint_values(*values); refresh = True
                self.applied.emit(dict(kind='parameters', k2=self.engine.p['k2'], d2=self.engine.d2,
                    stiffness_ratio=self.engine.p['stiffness_ratio'], stiffness_exponent=self.engine.p['stiffness_exponent'],
                    joint_values=True, revision=self.engine.revision))
            elif action == 'setup':
                self.vision_epoch+=1
                setup, reference = values if isinstance(values, tuple) else (values, None)
                self.engine.set_setup(setup)
                self.configure_tracker(reference); refresh = True
                self.applied.emit(dict(kind='setup', setup=self.engine.setup(), revision=self.engine.revision))
            elif action == 'step':
                self.vision_epoch+=1
                self.step = values; refresh = True
            elif action == 'analysis_mode':
                self.analysis_mode=values
                self.vision_epoch+=1;self.engine.history.clear();self.configure_tracker()
                if self.vision:self.vision.clear_pending()
                refresh=True
                self.applied.emit(dict(kind='analysis_mode',mode=values,revision=self.engine.revision))
            elif action == 'vision_method':
                self.vision_method=values;self.tracker=None
                self.vision_epoch+=1;self.engine.history.clear();self.configure_tracker()
                if self.vision:self.vision.clear_pending()
                refresh=True
                self.applied.emit(dict(kind='vision_method',method=values,revision=self.engine.revision))


class NullLog:
    def write(self, _): pass
    def flush(self): pass


class CameraView(QWidget):
    """Draw one image and its time-matched model; never stretch either separately."""
    base_picked = Signal(float, float)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(640, 360)
        self.frame = None
        self.predicted = self.observed = None
        self.stale = False
        self.landmarks = []
        self.base_px = None
        self.pick_base = False
        self.show_camera = self.show_backbone = True

    def set_visibility(self, camera, backbone):
        self.show_camera, self.show_backbone = bool(camera), bool(backbone)
        if not self.show_camera:
            self.pick_base=False;self.unsetCursor()
        self.update()

    def set_reference(self, result):
        """Display a local reference without inventing a camera/motor packet."""
        self.frame=None;self.stale=False;self.observed=None;self.landmarks=[]
        self.predicted=[result['base_px']]+result['predicted_curve_px']
        self.base_px=result['base_px'];self.update()

    def set_frame(self, rgb, predicted, observed, landmarks=(), base_px=None):
        height, width = rgb.shape[:2]
        self.frame = QImage(rgb.data, width, height, width*3, QImage.Format_RGB888).copy()
        self.predicted, self.observed, self.stale = predicted, observed, False
        self.landmarks, self.base_px = landmarks, base_px
        self.update()

    def mousePressEvent(self, event):
        if not self.show_camera or not self.pick_base or self.frame is None or self.stale or event.button()!=Qt.LeftButton:return
        scale=min(self.width()/self.frame.width(),self.height()/self.frame.height())
        x=(event.position().x()-(self.width()-self.frame.width()*scale)/2)/scale
        y=(event.position().y()-(self.height()-self.frame.height()*scale)/2)/scale
        if 0<=x<self.frame.width() and 0<=y<self.frame.height():
            self.pick_base=False;self.unsetCursor();self.base_picked.emit(x,y)

    def paintEvent(self, _):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#17212b'))
        if not self.show_camera and not self.show_backbone:
            painter.setPen(QColor('white'))
            painter.drawText(self.rect(), Qt.AlignCenter, 'Camera and MuJoCo backbone hidden')
            return
        if self.frame is None:
            if not self.show_backbone or not self.predicted:
                painter.setPen(QColor('white'))
                painter.drawText(self.rect(), Qt.AlignCenter, 'Waiting for camera preview')
                return
            # Standalone reference uses a fitted viewport, not camera pixels.
            # Camera alignment is unchanged and resumes with the next real image.
            painter.setPen(QColor('white'))
            painter.drawText(18,26,'MuJoCo reference — no measured motor inputs')
            points=np.asarray(self.predicted);lo=points.min(0);hi=points.max(0)
            extent=np.maximum(hi-lo,1.)
            scale=min((self.width()-80)/extent[0],(self.height()-100)/extent[1])
            centre=(lo+hi)/2
            painter.translate(self.width()/2,self.height()/2+12)
            painter.scale(scale,scale);painter.translate(-centre[0],-centre[1])
        else:
            scale = min(self.width()/self.frame.width(), self.height()/self.frame.height())
            painter.translate((self.width()-self.frame.width()*scale)/2, (self.height()-self.frame.height()*scale)/2)
            painter.scale(scale, scale)
            if self.show_camera:painter.drawImage(0, 0, self.frame)
        if not self.stale:
            painter.setRenderHint(QPainter.Antialiasing)
            for points, colour in ((self.predicted if self.show_backbone else None, '#ff493b'),
                                   (self.observed if self.show_camera else None, '#00ad8e')):
                if points:
                    pen=QPen(QColor(colour),2);pen.setCosmetic(True);painter.setPen(pen)
                    painter.drawPolyline(QPolygonF([QPointF(*p) for p in points]))
            for row in self.landmarks:
                if not row['included']: continue
                p=row['physical_px'] if self.show_camera else None
                q=row['model_px'] if self.show_backbone else None
                if p is not None and q is not None:
                    painter.setPen(QPen(QColor('#ffe27c'),1));painter.drawLine(QPointF(*p),QPointF(*q))
                for point,colour in ((p,'#00efb6'),(q,'#ff493b')):
                    if point is not None:
                        painter.setPen(QPen(QColor(colour),2));painter.drawEllipse(QPointF(*point),5,5)
            if self.show_backbone and self.base_px is not None:
                point=painter.transform().map(QPointF(*self.base_px))
                painter.save();painter.resetTransform();x,y=point.x(),point.y()
                painter.setPen(QPen(QColor('#ff493b'),2))
                painter.drawLine(QPointF(x-6,y),QPointF(x+6,y));painter.drawLine(QPointF(x,y-6),QPointF(x,y+6))
                painter.drawText(QPointF(x+10,y+4),'j_001 base');painter.restore()
        else:
            painter.resetTransform();painter.fillRect(0,0,self.width(),34,QColor('#7d2330'))
            painter.setPen(QColor('white'))
            font = painter.font(); font.setPixelSize(14); painter.setFont(font)
            painter.drawText(12,22,'STALE IMAGE — no current comparison' if self.show_camera else 'MODEL PAUSED — stale camera feedback')


class StatusLabel(QLabel):
    """Telemetry may change every frame; it must not move nearby controls."""
    def __init__(self, text='', lines=2):
        super().__init__(text)
        self.setWordWrap(True);self.setMinimumWidth(0)
        self.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Fixed)
        self.setFixedHeight(lines*self.fontMetrics().lineSpacing()+8)
        self.setToolTip(text)

    def setText(self, text):
        super().setText(text);self.setToolTip(text)


class Window(QMainWindow):
    def __init__(self, config, fit, rig, demo=False):
        super().__init__()
        self.config, self.fit, self.rig, self.demo = copy.deepcopy(config), copy.deepcopy(fit), rig, demo
        config, fit = self.config, self.fit
        from motion_settings import model_profile
        from motion_profile import validate_profile
        self.motion_profile=validate_profile(config['motion_profile']) if config.get('motion_profile') else model_profile(SpiRobModel(config['model']))
        self.theme = config.get('theme', 'light')
        apply_theme(QApplication.instance(), self.theme)
        self.startup_setup_warning = None
        try:
            engine = Comparator(config, fit)
        except (ValueError, KeyError, TypeError) as exc:
            if not config.get('comparison_setup'): raise
            config.pop('comparison_setup')
            engine = Comparator(config, fit)
            self.startup_setup_warning = 'Saved alignment could not be loaded; supplied calibration is in memory: '+str(exc)
        self.comparison_setup = engine.setup()
        self.current_d2 = engine.d2
        self.fit['candidate']['parameters'].update(stiffness_ratio=engine.p['stiffness_ratio'],
                                                  stiffness_exponent=engine.p['stiffness_exponent'])
        self.supplied_setup = Comparator({k:v for k,v in config.items() if k!='comparison_setup'},fit).setup()
        self.base_m = engine.model.base_point().copy()
        self.reference_layout = engine.joint_preview()
        self.standalone_reference = engine.compare_setup({'t_s':0.},None)
        self.setup_dialog = None
        self.setup_revision = 0
        self.sweep_result = None
        self.sweep_snapshot = None
        self.surface_dialog = None
        self.campaign_window = None
        self.remote_received = 0.
        self.selected_k = None
        self.scan_setup_revision = None
        self.review_busy = False
        self.review_queue = []
        self.last_summary = 0.
        self.analysis_message = 'Waiting for analysis'
        self.analysis_error = None
        self.parameter_revision = -1
        self.pending_comparison_change = False
        self.session = self.preview = self.connect_thread = None
        self.setup_session = None
        self.setup_connecting = self.setup_stopping = self.connect_after_setup = False
        self.notifications = queue.Queue()
        self.connecting = self.closing = self.close_when_done = False
        self.remote = {}
        self.latest = None
        self.last_analysis = None
        self.last_model = self.last_video = self.last_vision = None
        self.last_vision_display = 0.
        self.manual_capture_busy = False
        self.analysis_mode = config.get('analysis_mode','vision')
        if self.analysis_mode not in ('vision','manual'):self.analysis_mode='vision'
        self.config['analysis_mode']=self.analysis_mode
        self.inspector = None
        self.last_pair_received = 0.
        self.last_display = self.last_plot = 0.
        self.history = []
        self.scan_busy = False
        self.setWindowTitle('SpiRob Lab 0.9.1 — connect · move · compare')
        self.setMinimumSize(1040,650)
        available=QApplication.primaryScreen().availableGeometry()
        self.resize(min(1440,max(1040,available.width()-48)),min(900,max(650,available.height()-64)))
        root=QWidget();self.setCentralWidget(root);layout=QVBoxLayout(root)
        layout.setContentsMargins(10,8,10,8);layout.setSpacing(6)
        self.banner=StatusLabel('RECORDED EXAMPLE — motors unavailable' if demo else 'Disconnected',1)
        self.banner.setStyleSheet('font-size:15px;font-weight:600')
        header=QHBoxLayout();layout.addLayout(header);header.addWidget(self.banner,1)
        self.theme_box=QComboBox();self.theme_box.addItems(['Light','Dark']);header.addWidget(self.theme_box)
        self.theme_box.setCurrentText(self.theme.title());self.theme_box.currentTextChanged.connect(self.change_theme)
        self.alignment_button=QPushButton('Alignment / cable mapping…');header.addWidget(self.alignment_button)
        self.alignment_button.clicked.connect(self.open_setup)
        manual_button=QPushButton('Operating manual');header.addWidget(manual_button)
        manual_button.clicked.connect(lambda:QDesktopServices.openUrl(QUrl.fromLocalFile(str(Path(__file__).resolve().parent/'docs/OPERATING_MANUAL.html'))))

        # Session controls stay outside every scrolling or tabbed region.
        actions=QHBoxLayout();layout.addLayout(actions)
        self.connect_button=QPushButton('Connect');self.arm_button=QPushButton('Enable at rest')
        self.hold_button=QPushButton('STOP — hold both');self.disconnect_button=QPushButton('Disconnect')
        self.hold_button.setStyleSheet('font-weight:800;background:#a32121;color:white;border:2px solid #ffb5b5;padding:7px 16px')
        for button in (self.connect_button,self.arm_button,self.hold_button,self.disconnect_button):actions.addWidget(button)
        self.at_rest=QCheckBox('Robot is at its declared physical rest');actions.addWidget(self.at_rest,1)
        self.at_rest.setToolTip('Confirm both actual physical cable lengths match their configured rest before enabling or declaring rest.')

        self.workspace=QSplitter(Qt.Horizontal);self.workspace.setChildrenCollapsible(False);layout.addWidget(self.workspace,1)
        viewer=QWidget();left=QVBoxLayout(viewer);left.setContentsMargins(0,0,6,0);left.setSpacing(5)
        viewer.setMinimumWidth(520);self.workspace.addWidget(viewer)
        views=QHBoxLayout();left.addLayout(views)
        self.camera_toggle=QPushButton('Camera: OFF');self.camera_toggle.setCheckable(True);self.camera_toggle.setChecked(demo)
        self.backbone_toggle=QPushButton('MuJoCo backbone: ON');self.backbone_toggle.setCheckable(True);self.backbone_toggle.setChecked(True)
        views.addWidget(self.camera_toggle);views.addWidget(self.backbone_toggle)
        self.camera_toggle.toggled.connect(self.toggle_camera);self.backbone_toggle.toggled.connect(self.update_view_visibility)
        self.preview_button=self.camera_toggle
        self.backbone_toggle.setToolTip('Show or hide the red backbone independently. Before Connect this is the local reference; during a run the model continues for analysis.')
        self.inspect_button=QPushButton('Inspect / place points…');views.addWidget(self.inspect_button)
        self.inspect_button.clicked.connect(self.inspect_points)
        self.display_status=StatusLabel();left.addWidget(self.display_status)
        self.camera=CameraView();self.camera.setMinimumSize(360,170);left.addWidget(self.camera,1)
        self.camera.set_reference(self.standalone_reference);self.camera.base_picked.connect(self.base_picked)
        self.view_legend=StatusLabel(lines=1);left.addWidget(self.view_legend)
        self.status=StatusLabel('Camera opens setup preview. Connect starts the motor session.');left.addWidget(self.status)

        # Compact task pages replace the old whole-window vertical scroll.
        self.motor_tabs=QTabWidget();self.motor_tabs.setFixedHeight(258);left.addWidget(self.motor_tabs)
        motion_page=QWidget();motion=QVBoxLayout(motion_page);motion.setSpacing(4)
        self.motor_tabs.addTab(motion_page,'Motion')
        motion_row=QHBoxLayout();motion.addLayout(motion_row)
        self.motor_mode=QComboBox();self.motor_mode.addItems(['Mean + full difference','Independent cable lengths'])
        motion_row.addWidget(QLabel('Analysis motion'));motion_row.addWidget(self.motor_mode,1)
        self.motion_stack=QStackedWidget();motion.addWidget(self.motion_stack,1)
        self.motor_mode.currentIndexChanged.connect(self.motion_stack.setCurrentIndex)
        self.mean=self.number(sum(self.motion_profile['rest_lengths_mm'])/2,.001,10000.,3);self.difference=self.number(0.,-150.,150.,3);self.speed=self.number(1.,.05,20.,2)
        mean_panel=QWidget();mean_grid=QGridLayout(mean_panel);mean_grid.setContentsMargins(0,0,0,0);self.motion_stack.addWidget(mean_panel)
        for i,(label,box) in enumerate((('Mean length [mm]',self.mean),('Full difference [mm]',self.difference),('Speed [mm/s]',self.speed))):
            mean_grid.addWidget(QLabel(label),0,i);mean_grid.addWidget(box,1,i);box.valueChanged.connect(self.show_targets)
        self.move_button=QPushButton('Move mean + difference');mean_grid.addWidget(self.move_button,1,3)
        self.target_label=StatusLabel();mean_grid.addWidget(self.target_label,2,0,1,3);self.show_targets()
        self.mean_button=QPushButton('Move mean only');mean_grid.addWidget(self.mean_button,2,3)
        self.mean_button.setToolTip('Moves both cables by the same amount, preserving the measured full difference')
        self.mean_button.clicked.connect(lambda:self.command('mean',mean_mm=self.mean.value(),speed_mm_s=self.speed.value()))
        mean_grid.setRowStretch(3,1)
        independent_panel=QWidget();independent=QGridLayout(independent_panel);independent.setContentsMargins(0,0,0,0);independent.setSpacing(4)
        self.motion_stack.addWidget(independent_panel)
        self.length_targets=[];self.length_speeds=[];self.length_buttons=[]
        for col,label in enumerate(['Cable','Target [mm]','Speed [mm/s]','']):independent.addWidget(QLabel(label),0,col)
        for axis in (0,1):
            independent.addWidget(QLabel(f'Cable {axis+1}'),1+axis,0)
            target=self.number(self.motion_profile['rest_lengths_mm'][axis],.001,10000.,6);speed=self.number(1.,.05,20.,2)
            self.length_targets.append(target);self.length_speeds.append(speed)
            independent.addWidget(target,1+axis,1);independent.addWidget(speed,1+axis,2)
            button=QPushButton(f'Move cable {axis+1} only');self.length_buttons.append(button);independent.addWidget(button,1+axis,3)
            button.clicked.connect(lambda checked=False,a=axis:self.command('axis',axis=a,length_mm=self.length_targets[a].value(),speed_mm_s=self.length_speeds[a].value()))
        self.same_speed=QCheckBox('Same speed for both motors');self.same_speed.setChecked(True);independent.addWidget(self.same_speed,3,0,1,2)
        self.same_speed.toggled.connect(self.sync_length_speeds);self.length_speeds[0].valueChanged.connect(self.sync_length_speeds);self.sync_length_speeds()
        self.length_move=QPushButton('Move both to selected lengths');independent.addWidget(self.length_move,3,2,1,2)
        self.length_move.clicked.connect(lambda:self.command('independent',lengths_mm=[b.value() for b in self.length_targets],speeds_mm_s=[b.value() for b in self.length_speeds]))
        self.length_hint=StatusLabel();independent.addWidget(self.length_hint,4,0,1,4)

        self.setup_panel=QWidget();cable=QGridLayout(self.setup_panel);cable.setSpacing(4)
        self.motor_tabs.addTab(self.setup_panel,'Jog / rest')
        self.setup_enable=QPushButton('Enable for setup jogs');self.reference_button=QPushButton('Declare rest here')
        cable.addWidget(self.setup_enable,0,0,1,2);cable.addWidget(self.reference_button,0,2,1,2)
        self.setup_enable.clicked.connect(lambda:self.command('setup_enable'))
        self.reference_button.clicked.connect(lambda:self.command('reference',at_rest=self.at_rest.isChecked()))
        self.jog_step=self.number(.5,.05,1000.,2);self.jog_speed=self.number(.5,.05,20.,2)
        cable.addWidget(QLabel('Jog step [mm]'),1,0);cable.addWidget(self.jog_step,1,1)
        cable.addWidget(QLabel('Jog speed [mm/s]'),1,2);cable.addWidget(self.jog_speed,1,3)
        self.jog_buttons=[];self.axis_buttons=self.length_buttons;self.axis_targets=self.length_targets
        for axis in (0,1):
            cable.addWidget(QLabel(f'Cable {axis+1}'),2+axis,0)
            row=QHBoxLayout();cable.addLayout(row,2+axis,1,1,3)
            for direction,text in ((-1,'− Step'),(1,'+ Step')):
                button=QPushButton(text);row.addWidget(button);self.jog_buttons.append(button)
                button.clicked.connect(lambda checked=False,a=axis,d=direction:self.command('jog',axis=a,delta_mm=d*self.jog_step.value(),speed_mm_s=self.jog_speed.value()))
        both_row=QHBoxLayout();cable.addLayout(both_row,4,0,1,4)
        for title,direction in [('Both −',-1),('Both +',1)]:
            button=QPushButton(title);both_row.addWidget(button);self.jog_buttons.append(button)
            button.clicked.connect(lambda checked=False,d=direction:self.command('jog_pair',delta_mm=d*self.jog_step.value(),speed_mm_s=self.jog_speed.value()))
        hint=StatusLabel('− reduces / + increases cable length. Wait for HOLDING before the next step. Declare rest only at the known physical rest; Stop / Hold retains torque.',3)
        cable.addWidget(hint,5,0,1,4)

        telemetry_page=QWidget();telemetry_layout=QVBoxLayout(telemetry_page);self.motor_tabs.addTab(telemetry_page,'Lengths / timing')
        self.telemetry=StatusLabel('Lengths [mm] — MuJoCo follows encoder changes from physical rest.');telemetry_layout.addWidget(self.telemetry)
        self.length_table=QTableWidget(2,5);telemetry_layout.addWidget(self.length_table)
        self.length_table.setHorizontalHeaderLabels(['Physical → model','Demand [mm]','Encoder [mm]','Model demand','Model actual'])
        self.length_table.verticalHeader().hide();self.length_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.length_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch);self.length_table.setFixedHeight(92)
        for row in range(2):
            for col in range(5):self.length_table.setItem(row,col,QTableWidgetItem('—'))
        self.details=QCheckBox('Show timing details (updates twice per second)');telemetry_layout.addWidget(self.details)
        self.video_info=StatusLabel(lines=1);telemetry_layout.addWidget(self.video_info);telemetry_layout.addStretch()

        session_page=QWidget();session_controls=QGridLayout(session_page);self.motor_tabs.addTab(session_page,'Session / data')
        self.duration=QComboBox()
        for minutes in (10,20,30,45,60):self.duration.addItem(f'{minutes} min',minutes*60)
        session_controls.addWidget(QLabel('Session duration'),0,0);session_controls.addWidget(self.duration,0,1)
        self.difference_limit=self.number(self.motion_profile['difference_mm'],1.,10000.,1)
        session_controls.addWidget(QLabel('Max difference [mm]'),0,2);session_controls.addWidget(self.difference_limit,0,3)
        self.difference_limit.valueChanged.connect(self.change_motion_limit)
        self.motion_hint=StatusLabel();session_controls.addWidget(self.motion_hint,1,0,1,4)
        self.record_button=QPushButton('Start recording');self.record_button.clicked.connect(self.toggle_recording);session_controls.addWidget(self.record_button,2,0)
        self.motion_settings_button=QPushButton('Cable references / limits…')
        session_controls.addWidget(self.motion_settings_button,4,0,1,4)
        self.motion_settings_button.clicked.connect(self.edit_motion_settings)
        self.motion_summary=StatusLabel('',3);session_controls.addWidget(self.motion_summary,5,0,1,4)
        self.record_status=StatusLabel('Preview only: no images saved. Record limit: 60 s / 250 MiB per session.');session_controls.addWidget(self.record_status,2,1,1,3)
        self.review_button=QPushButton('Review pending run data…');session_controls.addWidget(self.review_button,3,0,1,4);self.review_button.clicked.connect(self.recover_runs)
        campaign_page=QWidget();campaign_layout=QVBoxLayout(campaign_page);self.motor_tabs.addTab(campaign_page,'Campaign')
        campaign_button=QPushButton('Plan / run campaign · review captured poses…');campaign_layout.addWidget(campaign_button)
        campaign_button.clicked.connect(self.open_campaign)
        self.campaign_status=StatusLabel('Acquire a complete journey, then fit shared k₂ and review individual poses.',3);campaign_layout.addWidget(self.campaign_status)
        campaign_note=QLabel('Draft 73 poses or generate a random journey. Simulate and approve before Start.\nChoose session duration before Connect. One JPEG per held pose; STOP aborts the sequence.')
        campaign_note.setWordWrap(True);campaign_layout.addWidget(campaign_note);campaign_layout.addStretch()
        if demo:
            self.steps=QComboBox();self.steps.addItems(['Hold 1','Hold 2','Hold 3','Hold 4']);self.steps.setCurrentIndex(1)
            session_controls.addWidget(QLabel('Recorded pose'),6,0);session_controls.addWidget(self.steps,6,1,1,3)
            self.steps.currentIndexChanged.connect(lambda i:self.preview.requests.put(('step',i+1)))
        session_controls.setRowStretch(7,1)

        analysis_panel=QWidget();right=QVBoxLayout(analysis_panel);right.setContentsMargins(6,0,0,0);right.setSpacing(5)
        analysis_panel.setMinimumWidth(380);self.workspace.addWidget(analysis_panel);self.workspace.setSizes([820,500])
        self.analysis_tabs=QTabWidget();right.addWidget(self.analysis_tabs,1)
        self.analysis_scroll=QScrollArea();self.analysis_scroll.setWidgetResizable(True);self.analysis_scroll.setFrameShape(QScrollArea.NoFrame)
        analysis_form=QWidget();form=QVBoxLayout(analysis_form);form.setSpacing(5);self.analysis_scroll.setWidget(analysis_form)
        self.analysis_tabs.addTab(self.analysis_scroll,'Fit / capture')
        form.addWidget(QLabel('Measurement mode'))
        self.analysis_mode_box=QComboBox();self.analysis_mode_box.addItem('Image estimation — live green backbone','vision');self.analysis_mode_box.addItem('Manual points — no image processing','manual')
        self.analysis_mode_box.setCurrentIndex(int(self.analysis_mode=='manual'));form.addWidget(self.analysis_mode_box)
        self.analysis_mode_box.currentIndexChanged.connect(self.change_analysis_mode)
        self.dots_box=QCheckBox('Marker dots — one black dot per element (E1)')
        self.dots_box.setToolTip('Image estimation reads the black dot at each element centre (link_002…link_021) instead of the silhouette. Base element: two dots.')
        self.dots_box.setChecked(self.config.get('vision_method')=='dots');form.addWidget(self.dots_box)
        self.dots_box.toggled.connect(self.change_vision_method)
        parameters=QGridLayout();form.addLayout(parameters)
        self.kbox=self.number(engine.p['k2'],.001,10.,7)
        from spirob_analysis.model import SpiRobModel
        self.dbox=self.number(engine.d2,0.,2.,7)
        parameters.addWidget(QLabel('k₂ [N m/rad]'),0,0);parameters.addWidget(self.kbox,0,1)
        parameters.addWidget(QLabel('d₂ [N m s/rad]'),1,0);parameters.addWidget(self.dbox,1,1)
        self.apply_button=QPushButton('Apply to simulation');parameters.addWidget(self.apply_button,2,0,1,2)
        self.joint_editor_button=QPushButton('Edit each joint k and d…');form.addWidget(self.joint_editor_button)
        self.joint_editor_button.setToolTip('Per-joint stiffness and damping; Apply shows the MuJoCo backbone over the camera.')
        self.joint_editor_button.clicked.connect(self.open_joint_editor);self.joint_editor=None
        self.profile_edit=QCheckBox('Use exponential stiffness profile (replaces per-joint values)');form.addWidget(self.profile_edit)
        self.profile_panel=QWidget();profile_controls=QGridLayout(self.profile_panel)
        self.rbox=self.number(engine.p['stiffness_ratio'],.5,1.5,15)
        self.bbox=self.number(engine.p['stiffness_exponent'],0.,3.,10)
        profile_controls.addWidget(QLabel('Ratio r'),0,0);profile_controls.addWidget(self.rbox,0,1)
        profile_controls.addWidget(QLabel('Exponent multiplier b'),1,0);profile_controls.addWidget(self.bbox,1,1)
        profile_note=QLabel('kᵢ = k₂ · r^[b(i−2)], i ≥ 2. Default b = 1.\nEdits take effect with Apply to simulation.');profile_note.setWordWrap(True)
        profile_controls.addWidget(profile_note,2,0,1,2);form.addWidget(self.profile_panel)
        self.profile_panel.setVisible(False);self.profile_edit.toggled.connect(self.profile_panel.setVisible)
        self.applied_label=StatusLabel('Simulation parameters: waiting for comparison',3);form.addWidget(self.applied_label)
        sweep_controls=QGridLayout();form.addLayout(sweep_controls)
        self.scan_min=self.number(.02,.001,10.,6);self.scan_max=self.number(3.,.001,10.,6);self.scan_count=QSpinBox();self.scan_count.setRange(3,201);self.scan_count.setValue(31)
        for col,(label,box) in enumerate([('Minimum k₂',self.scan_min),('Maximum k₂',self.scan_max),('Grid samples',self.scan_count)]):
            sweep_controls.addWidget(QLabel(label),0,col);sweep_controls.addWidget(box,1,col)
        self.scan_button=QPushButton('Freeze measured pose + sweep k₂');form.addWidget(self.scan_button)
        self.sweep_ready=StatusLabel(lines=4);form.addWidget(self.sweep_ready)
        note=StatusLabel('Joint 1 fixed. Joint 2 sets the exponential k and d profiles. Alignment and cable offsets stay fixed. d₂ is not yet fitted.',3);form.addWidget(note);form.addStretch()
        plot_page=QWidget();plot_layout=QVBoxLayout(plot_page);plot_layout.setContentsMargins(2,2,2,2);self.analysis_tabs.addTab(plot_page,'J / k₂ plots')
        self.surface_check=QCheckBox('3D profile review (after a completed sweep)');plot_layout.addWidget(self.surface_check)
        self.surface_check.setToolTip('Review J versus k₂ and one constant using exactly the completed sweep’s frozen pose. The default plot remains J versus k₂.')
        self.surface_check.toggled.connect(self.toggle_surface)
        self.figure=Figure(figsize=(5,6),layout='constrained');self.canvas=FigureCanvasQTAgg(self.figure)
        self.canvas.setMinimumSize(300,180);self.canvas.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Expanding);plot_layout.addWidget(self.canvas)
        self.cost_ax,self.scan_ax=self.figure.subplots(2,1)
        self.cost_ax.set(title='Image error through time',xlabel='Session time [s]',ylabel='J [px² / point]')
        self.error_line,=self.cost_ax.plot([],[],color='#9aabb6',label='Instantaneous (diagnostic)');self.cost_line,=self.cost_ax.plot([],[],'o',ms=3,color='#1b69af',label='Both settled');self.cost_ax.legend(fontsize=8)
        self.scan_ax.set(title='Frozen-pose equilibrium sweep',xlabel='k₂ [N m/rad]',ylabel='J [px² / point]',xscale='log')
        self.scan_line,=self.scan_ax.plot([],[],'o-',ms=3,color='#9970cf');self.minimum_line,=self.scan_ax.plot([],[],'*',ms=12,color='#df6a00',label='Lowest sampled J')
        self.selection_line,=self.scan_ax.plot([],[],'o',ms=10,mfc='none',mec='#13a8ad',label='Selected');self.scan_ax.legend(fontsize=8)
        self.canvas.mpl_connect('button_press_event',self.select_scan_point)
        for ax in (self.cost_ax,self.scan_ax):ax.grid(alpha=.2)
        self.apply_plot_mode()
        self.sweep_label=StatusLabel('Choose the k₂ interval, capture a settled pose, then sweep. Results appear in J / k₂ plots.',3);right.addWidget(self.sweep_label)
        sweep_actions=QHBoxLayout();right.addLayout(sweep_actions)
        self.best_button=QPushButton('Select lowest J');self.best_button.clicked.connect(self.select_best);sweep_actions.addWidget(self.best_button)
        self.apply_scan_button=QPushButton('Apply selection');self.apply_scan_button.clicked.connect(self.apply_sweep);sweep_actions.addWidget(self.apply_scan_button)
        self.save_scan_button=QPushButton('Save sweep…');self.save_scan_button.clicked.connect(self.save_sweep);sweep_actions.addWidget(self.save_scan_button)
        self.capture_pose_button=QPushButton('Save pose screenshot + J + k₂ sweep…');right.addWidget(self.capture_pose_button);self.capture_pose_button.clicked.connect(self.save_pose)
        self.capture_pose_button.setToolTip('Exports the exact frozen image used for the completed sweep, with overlays and companion data.')
        self.command_status=StatusLabel();layout.addWidget(self.command_status)
        if demo:self.start_preview()
        self.connect_button.clicked.connect(self.connect)
        self.arm_button.clicked.connect(lambda: self.command('arm', at_rest=self.at_rest.isChecked()))
        self.move_button.clicked.connect(lambda: self.command('move', mean_mm=self.mean.value(), difference_mm=self.difference.value(), speed_mm_s=self.speed.value()))
        self.hold_button.clicked.connect(lambda: self.command('hold'))
        from PySide6.QtGui import QShortcut,QKeySequence
        self.stop_shortcut=QShortcut(QKeySequence('Escape'),self)
        self.stop_shortcut.setContext(Qt.ApplicationShortcut)
        self.stop_shortcut.activated.connect(lambda:self.command('hold'))
        self.disconnect_button.clicked.connect(self.disconnect)
        self.apply_button.clicked.connect(self.apply_parameters)
        self.scan_button.clicked.connect(self.scan)
        self.change_motion_limit(self.difference_limit.value())
        self.update_motion_widgets()
        self.update_view_visibility()
        self.timer = QTimer(self); self.timer.timeout.connect(self.tick); self.timer.start(100)
        theme_figure(self.figure,self.theme)
        self.tick()
        if self.startup_setup_warning:self.command_status.setText(self.startup_setup_warning)
        if not demo and not self.startup_setup_warning:
            self.command_status.setText('Ready. Save / Discard is requested after this run ends. Earlier pending runs: Session / data → Review pending run data.')

    def save_preferences(self, include_setup=False):
        path=Path('lab.local.json')
        preferences=json.loads(path.read_text()) if path.exists() else {}
        preferences['motion_profile']=self.motion_profile
        preferences['theme']=self.theme
        preferences['analysis_mode']=self.analysis_mode
        preferences['vision_method']=self.config.get('vision_method','silhouette')
        if include_setup:preferences['comparison_setup']=copy.deepcopy(self.comparison_setup)
        temporary=path.with_suffix('.tmp')
        temporary.write_text(json.dumps(preferences,indent=2,allow_nan=False)+'\n');temporary.replace(path)

    def change_theme(self, name):
        self.theme=name.lower();apply_theme(QApplication.instance(),self.theme)
        theme_figure(self.figure,self.theme);self.canvas.draw_idle()
        if self.inspector:self.inspector.redraw()
        if self.surface_dialog and self.surface_dialog.result:
            from surface_review import draw_surface
            draw_surface(self.surface_dialog.figure,self.surface_dialog.result,self.theme)
            self.surface_dialog.canvas.draw_idle()
        try:self.save_preferences()
        except (OSError,ValueError) as exc:self.command_status.setText('Theme applied but not saved: '+str(exc))

    def open_campaign(self):
        if self.campaign_window is None:
            from campaign_window import CampaignWindow
            self.campaign_window=CampaignWindow(self)
        self.campaign_window.show();self.campaign_window.raise_();self.campaign_window.activateWindow()

    def campaign_active(self):
        return bool(self.campaign_window and self.campaign_window.active)

    def open_setup(self):
        from display_setup import DisplaySetup
        if self.setup_dialog is None:
            self.setup_dialog=DisplaySetup(self.comparison_setup,self.base_m,self.apply_setup,
                                          self.pick_base,self.supplied_setup,self, scale_callback=self.matched_scale)
        else:self.setup_dialog.load(self.comparison_setup)
        self.setup_dialog.show();self.setup_dialog.raise_()

    def pick_base(self, dialog):
        if not self.camera.show_camera or self.camera.frame is None or self.camera.stale:
            dialog.show();dialog.notice.setText('Turn Camera ON and wait for fresh video, then pick the fixed joint-1 base.');return
        self.camera.pick_base=True;self.camera.setCursor(Qt.CrossCursor)
        self.command_status.setText('Click the physical fixed joint-1 base in the video. The dialog will reopen for Apply.')

    def base_picked(self,x,y):
        if self.setup_dialog:self.setup_dialog.picked(x,y)

    def apply_setup(self, setup, save=False, tracking_reference=None):
        if self.campaign_active():raise ValueError("Stop the campaign before changing alignment or joint calibration.")
        if self.session and self.remote.get('phase') not in ('HOLDING','READY_DISABLED','SETUP_HOLD'):
            raise ValueError('Wait until the robot is holding or disabled before changing comparison setup.')
        candidate=copy.deepcopy(self.config);candidate['comparison_setup']=copy.deepcopy(setup)
        checked = Comparator(candidate,self.fit)  # Validate before changing any shared state.
        if self.camera.frame is not None:
            points = checked.transform(np.vstack([checked.model.base_point(), checked.model.curve()]))
            visible = ((points[:,0]>=0)&(points[:,0]<self.camera.frame.width())&
                       (points[:,1]>=0)&(points[:,1]<self.camera.frame.height()))
            if not visible.any():
                raise ValueError('This setup places the whole reference backbone outside the image. Pick the base again or Load supplied image alignment.')
        self.comparison_setup=copy.deepcopy(setup);self.config['comparison_setup']=copy.deepcopy(setup)
        self.setup_revision+=1;self.latest=None;self.last_analysis=None;self.history.clear()
        self.last_model=self.last_vision=None
        self.standalone_reference=checked.compare_setup({'t_s':0.},None)
        self.reference_layout=self.standalone_reference['landmarks']
        if not self.preview and not self.session and not self.setup_session:
            self.camera.set_reference(self.standalone_reference)
        self.last_plot=0.
        self.cost_line.set_data([],[]);self.error_line.set_data([],[]);self.canvas.draw_idle()
        self.analysis_message='Updating projection; error score waits for a fresh settled comparison'
        if self.preview:
            self.pending_comparison_change=True
            self.preview.requests.put(('setup',(copy.deepcopy(setup),tracking_reference)))
        if self.sweep_result:self.select_best()
        if save:self.save_preferences(True)
        self.command_status.setText('Comparison setup saved.' if save else 'Comparison setup applied in memory.')

    def matched_scale(self, setup):
        from landmarks import scale_from_profile
        return scale_from_profile(setup.get('profile') or {}, self.reference_layout)

    def use_profile(self, profile, save=False, frozen=None, reference=None):
        if frozen and self.scan_busy:raise ValueError('Wait for the current sweep to finish before applying another frozen pose.')
        if frozen:
            old=frozen['result']['setup']
            if any(old[k]!=self.comparison_setup[k] for k in ('mapping','signs','translation_x_px','translation_y_px','rotation_rad','pixels_per_m')):
                raise ValueError('Alignment or mapping changed after capture. Capture the frame again before sweeping.')
        setup=copy.deepcopy(self.comparison_setup);setup['profile']=profile
        self.apply_setup(setup,save,tracking_reference=reference)
        self.command_status.setText('Exact joint calibration applied to live image tracking'+(' and J / sweeps.' if profile['use_for_score'] else '; J samples the corrected curve.'))
        if frozen:
            from landmarks import correspondences
            from analysis import point_cost
            item=copy.deepcopy(frozen);result=item['result']
            if profile.get('measurement_mode') == 'manual_joints':
                from joint_tracking import manual_observation
                points=[profile['calibration_points_px'][n] for n in profile['chain_ids']]
                item['observed']=manual_observation(profile,points,self.config)
            rows=correspondences(item['observed'],result['landmarks'],profile['mode'],profile=profile)
            selected=[r for r in rows if r['included']]
            if any(r['physical_px'] is None for r in selected): raise ValueError('Calibrated joints do not fit this visible curve.')
            result['landmark_rows']=rows;result['setup']=copy.deepcopy(setup)
            if profile['use_for_score']:
                displacement=result['observation']['displacement_mm']
                obs=result['observation']=dict(copy.deepcopy(item['observed']),displacement_mm=displacement)
                obs.update(points_px=[r['physical_px'] for r in selected],landmark_ids=[r['id'] for r in selected],
                    landmark_mode=profile['mode'],tail_distance_px=[profile['tail_px'][r['id']] for r in selected])
                result['predicted_points_px']=[r['model_px'] for r in selected]
                result['cost_px2']=point_cost(obs['points_px'],result['predicted_points_px'])
                result['rmse_px']=float(np.sqrt(result['cost_px2']))
                result['motion_error_px2']=result['cost_px2']
            else:
                checked=Comparator(dict(self.config,comparison_setup=setup),self.fit)
                obs=checked.observation(item['observed'],np.array(result['lengths_mm']),item['packet'])
                # Interpolate on the captured model curve, preserving that pose.
                curve=np.array(result['predicted_curve_px'])
                arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(curve,axis=0),axis=1))]
                at=arc[-1]-np.array(obs['tail_distance_px'])
                if np.any(at<0):raise ValueError('Corrected curve samples exceed the model length.')
                predicted=np.c_[np.interp(at,arc,curve[:,0]),np.interp(at,arc,curve[:,1])]
                result.update(observation=obs,predicted_points_px=predicted.tolist(),cost_px2=point_cost(obs['points_px'],predicted))
                result['rmse_px']=float(np.sqrt(result['cost_px2']));result['motion_error_px2']=result['cost_px2']
            self.scan(item)

    def parameters_applied(self, message):
        self.parameter_revision=message['revision']
        self.pending_comparison_change=False
        if message['kind']=='parameters':
            self.current_d2=message['d2']
            self.fit['candidate']['parameters'].update({k:message[k] for k in ('k2','stiffness_ratio','stiffness_exponent') if k in message})
            self.applied_label.setText(self.parameter_text(message,'Simulation acknowledged'))
        elif message['kind']=='vision_method':
            self.command_status.setText('Image estimation now reads the black marker dots.' if message['method']=='dots' else 'Image estimation now uses the silhouette centreline.')
        elif message['kind']=='analysis_mode':
            self.command_status.setText('Manual points mode: image processing is off.' if message['mode']=='manual' else 'Live image estimation enabled.')
        else:
            self.command_status.setText('Alignment, mapping and landmark setup acknowledged by simulation.')

    def toggle_recording(self):
        enabled=not self.remote.get('recording',{}).get('active',False)
        self.command('record',enabled=enabled)

    def refresh_summary(self, item):
        result=item.get('result') if item else None
        latest=self.latest
        if self.preview:
            if self.analysis_error:self.status.setText(self.analysis_error)
            elif result:
                cost=result['cost_px2']
                prefix=f"J = {cost:.2f} px²/point | RMSE = {result['rmse_px']:.2f} px | " if cost is not None else ''
                self.status.setText(prefix+self.analysis_message)
            elif latest and latest.get('rgb') is not None:
                self.status.setText(self.analysis_message if item else 'Live video · waiting for a fresh comparison')
        if latest:
            if self.details.isChecked():
                packet=latest['packet']
                self.video_info.setText(f"Frame {packet['frame_index']} · age ≤ {latest['displayed_age_s']:.2f} s · processing {latest.get('processing_s',0)*1000:.0f} ms")
            else:self.video_info.setText('Recorded example' if self.demo else 'Live video · '+('comparison current' if result else 'analysis pending'))
        mapping=self.comparison_setup['mapping'];ids=self.remote.get('motor_ids',[])
        axes=self.remote.get('axes',[])
        for physical in range(2):
            tendon=mapping.index(physical)
            motor_id=f" (ID {ids[physical]})" if len(ids)==2 else ''
            values=[f"Cable {physical+1}{motor_id} → cable_{tendon}",None,None,None,None]
            if result:
                values[1:3]=[result['targets_mm'][physical],result['lengths_mm'][physical]]
                telemetry=result.get('tendons',{})
                if telemetry:
                    values[3:]=[telemetry['demanded_mm'][tendon],telemetry['actual_mm'][tendon]]
            elif len(axes)==2:
                a=axes[physical];values[1:3]=[a.get('target_absolute_mm'),a.get('absolute_mm')]
            for col,value in enumerate(values):
                self.length_table.item(physical,col).setText(value if isinstance(value,str) else '—' if value is None else f'{value:.3f}')
        self.telemetry.setText((f"Lengths [mm] at image t={result['t_s']:.3f} s. " if result else 'Physical lengths [mm]; comparison unavailable. ')+
            'MuJoCo demand follows encoder feedback; its geometric length can differ.')

    @staticmethod
    def number(value, low, high, decimals):
        box = QDoubleSpinBox(); box.setRange(low, high); box.setDecimals(decimals); box.setValue(value)
        box.setKeyboardTracking(False)
        return box

    def show_targets(self, *_):
        a, d = self.mean.value(), self.difference.value()
        self.target_label.setText(f'Requested targets: cable 1 = {a+d/2:.3f} mm; cable 2 = {a-d/2:.3f} mm')

    def change_motion_limit(self, value):
        self.difference.setRange(-value,value)
        self.motion_profile['difference_mm']=float(value)
        self.motion_hint.setText(f'Mean/difference: |L₁−L₂| ≤ {value:g} mm, within the cable bounds. Limits lock at Connect.')
        if hasattr(self,'length_targets'):self.update_length_bounds()

    def update_length_bounds(self):
        policy=self.remote.get('motion_limits') or self.motion_profile
        ranges=[]
        for axis,(box,(lo,hi)) in enumerate(zip(self.length_targets,policy['absolute_bounds_mm'])):
            box.setRange(lo,hi);ranges.append(f'Cable {axis+1}: {lo:g}–{hi:g} mm')
        self.length_hint.setText('; '.join(ranges)+f". Speed ceiling {policy['max_speed_mm_s']:g} mm/s.")

    def update_motion_widgets(self):
        p=self.motion_profile
        self.mean.setValue(sum(p['rest_lengths_mm'])/2)
        self.difference.setValue(p['rest_lengths_mm'][0]-p['rest_lengths_mm'][1])
        self.update_length_bounds()
        for box,value in zip(self.length_targets,p['rest_lengths_mm']):box.setValue(value)
        for box in [self.speed,self.jog_speed,*self.length_speeds]:box.setMaximum(p['max_speed_mm_s'])
        self.motion_summary.setText(f"GUI references: {p['rest_lengths_mm'][0]:.3f} / {p['rest_lengths_mm'][1]:.3f} mm. "
            f"Speed ≤ {p['max_speed_mm_s']:g} mm/s; setup jog travel ±{p['setup_travel_mm']:g} mm. Sent to WSL at Connect.")
        self.show_targets()

    def edit_motion_settings(self):
        if self.session or self.connecting or self.closing:return
        from motion_settings import MotionSettings
        dialog=MotionSettings(self)
        if dialog.exec():
            self.motion_profile=dialog.profile;self.config['motion_profile']=copy.deepcopy(dialog.profile)
            self.update_motion_widgets();self.save_preferences()
            self.command_status.setText('Saved next-session references and bounds. Mean and cable targets updated. Re-preview campaigns after changes.')

    def sync_length_speeds(self, *_):
        linked=self.same_speed.isChecked()
        if linked:self.length_speeds[1].setValue(self.length_speeds[0].value())
        self.length_speeds[1].setEnabled(not linked)

    def change_vision_method(self, checked):
        method='dots' if checked else 'silhouette'
        self.config['vision_method']=method
        self.last_vision=None;self.history.clear()
        if self.preview:self.preview.requests.put(('vision_method',method))
        try:self.save_preferences()
        except (OSError,ValueError) as exc:self.command_status.setText('Method changed; preferences were not saved: '+str(exc))

    def change_analysis_mode(self, *_):
        self.analysis_mode=self.analysis_mode_box.currentData()
        self.config['analysis_mode']=self.analysis_mode
        self.last_vision=None;self.last_analysis=None;self.history.clear()
        self.cost_line.set_data([],[]);self.error_line.set_data([],[]);self.canvas.draw_idle()
        self.apply_plot_mode()
        if self.preview:self.preview.requests.put(('analysis_mode',self.analysis_mode))
        if self.inspector:self.inspector.close();self.inspector=None
        try:self.save_preferences()
        except (OSError,ValueError) as exc:self.command_status.setText('Mode changed; preferences were not saved: '+str(exc))
        self.tick()

    def apply_plot_mode(self):
        manual=self.analysis_mode=='manual'
        self.cost_ax.set_visible(not manual);self.cost_ax.set_in_layout(not manual)
        grid=self.figure.add_gridspec(1 if manual else 2,1)
        self.scan_ax.set_subplotspec(grid[0 if manual else 1])
        if not manual:self.cost_ax.set_subplotspec(grid[0])
        self.canvas.draw_idle()

    def update_view_visibility(self, *_):
        camera,backbone=self.camera_toggle.isChecked(),self.backbone_toggle.isChecked()
        self.camera.set_visibility(camera,backbone)
        self.camera_toggle.setText('Camera: '+('ON' if camera else 'OFF'))
        self.backbone_toggle.setText('MuJoCo backbone: '+('ON' if backbone else 'OFF'))
        if self.session or self.connecting or self.closing:
            text='View switches only: camera acquisition and MuJoCo analysis continue during the motor run.'
            tooltip='Show or hide the camera image. Acquisition continues for feedback freshness and analysis until Disconnect.'
        elif self.demo:
            text='Recorded example: camera image and MuJoCo backbone can be shown independently.'
            tooltip='Show or hide the recorded image.'
        else:
            text=('Camera setup preview; motors disconnected and session timer not started.' if camera else 'Camera closed; motors disconnected and session timer not started.')
            if self.setup_connecting:text='Opening camera; motors disconnected and session timer not started.'
            elif self.setup_stopping:text='Closing setup camera; motors disconnected.'
            if backbone:text+=' Backbone shows the MuJoCo reference pose.'
            tooltip='Open or close camera-only setup preview before Connect. The backbone switch is independent.'
        self.display_status.setText(text);self.camera_toggle.setToolTip(tooltip)
        if self.camera.frame is None:
            legend='Red: local MuJoCo reference; no measured motor inputs' if backbone else 'Turn a view ON to display it.'
        elif camera and backbone:legend='Red: MuJoCo backbone    Green: camera measurement    Same image time'
        elif camera:legend='Camera only; green physical measurements appear when available.'
        elif backbone:legend='MuJoCo backbone only; the camera image is hidden.'
        else:legend='Both views hidden.'
        self.view_legend.setText(legend)

    def set_camera_checked(self, checked):
        self.camera_toggle.blockSignals(True)
        self.camera_toggle.setChecked(checked)
        self.camera_toggle.blockSignals(False)
        self.update_view_visibility()

    def toggle_camera(self, checked):
        self.update_view_visibility()
        if self.demo or self.session or self.connecting or self.closing:return
        if not checked:
            if self.setup_session or self.setup_connecting:self.stop_setup_preview()
            return
        if self.setup_session or self.setup_connecting or self.setup_stopping or self.review_busy:return
        self.start_setup_camera()

    def toggle_setup_preview(self, checked=None):
        self.camera_toggle.setChecked(not self.camera_toggle.isChecked() if checked is None else checked)

    def start_setup_camera(self):
        if self.session or self.connecting or self.closing or self.review_busy: return
        self.setup_connecting=True
        self.banner.setText('Opening setup camera — motors disconnected; session timer has not started')
        def work():
            try:
                self.setup_session=CameraSetupSession(self.rig)
                self.setup_session.connect()
                self.notifications.put(('setup_connected',None))
            except Exception as exc:self.notifications.put(('setup_failed',str(exc)))
        self.connect_thread=threading.Thread(target=work,daemon=True);self.connect_thread.start()

    def stop_setup_preview(self, connect_after=False):
        self.connect_after_setup=connect_after
        if self.setup_stopping:return
        self.setup_stopping=True
        if self.setup_session:self.setup_session.cancel.set()
        def work():
            error=None
            if self.connect_thread and self.connect_thread.is_alive():self.connect_thread.join()
            if self.preview:self.preview.stop.set();self.preview.wait()
            try:
                if self.setup_session:self.setup_session.disconnect()
            except Exception as exc:error=str(exc)
            self.notifications.put(('setup_stopped',error))
        threading.Thread(target=work,daemon=True).start()

    def connect(self):
        if self.review_busy or self.session or self.connecting or self.closing:return
        if self.setup_session or self.setup_connecting:
            self.stop_setup_preview(connect_after=True);return
        self.connecting = True
        seconds=self.duration.currentData();limit=self.difference_limit.value()
        profile=copy.deepcopy(self.motion_profile)
        self.banner.setText('Connecting — SSH may ask for your key passphrase in the terminal')
        self.remote = {}; self.latest = None; self.last_analysis = None; self.last_model=self.last_video=self.last_vision=None
        self.at_rest.setChecked(False); self.history.clear()
        def work():
            try:
                self.session = WorkstationSession(self.rig, 'logs')
                self.session.max_seconds=seconds; self.session.difference_limit_mm=limit;self.session.motion_profile=profile
                if self.closing:self.session.cancel.set()
                self.session.connect()
                self.notifications.put(('connected', None))
            except Exception as exc:
                self.notifications.put(('connect_failed', str(exc)))
        self.connect_thread = threading.Thread(target=work, daemon=True); self.connect_thread.start()

    def start_preview(self):
        self.parameter_revision=-1;self.pending_comparison_change=False
        self.preview = Preview(self.config, self.fit, self.session or self.setup_session, self.demo, bool(self.setup_session))
        self.preview.updated.connect(self.show_comparison)
        self.preview.video.connect(self.show_video)
        self.preview.applied.connect(self.parameters_applied)
        self.preview.start()

    def command(self, action, **values):
        if action=='hold' and self.campaign_window and self.campaign_window.simulation_dialog:
            self.campaign_window.simulation_dialog.cancel.set()
        if self.campaign_active():
            if action=="hold":self.campaign_window.abort("User pressed Stop / Hold");return
            self.command_status.setText("Campaign owns motion. Stop the campaign before using manual motion.");return
        try:
            self.session.send(action, **values)
            self.command_status.setText('Sent: '+action+'; waiting for laptop acknowledgement')
        except Exception as exc:
            self.command_status.setText(str(exc))

    def parameter_text(self, values, prefix):
        fallback=self.fit['candidate']['parameters']
        ratio=values.get('stiffness_ratio',fallback['stiffness_ratio'])
        exponent=values.get('stiffness_exponent',fallback['stiffness_exponent'])
        if values.get('joint_values') or ratio is None:
            return f"{prefix}: per-joint values; k₂ = {values['k2']:.7g}; d₂ = {values['d2']:.7g}"
        return (f"{prefix}: k₂ = {values['k2']:.7g}; d₂ = {values['d2']:.7g}; "
                f"r = {ratio:.7g}; b = {exponent:.7g}")

    def open_joint_editor(self):
        from joint_editor import JointEditor
        if self.joint_editor is None:
            self.joint_editor=JointEditor(self,self.config['model'],self.apply_joint_values)
        self.joint_editor.show();self.joint_editor.raise_()

    def apply_joint_values(self, stiffness, damping):
        if self.campaign_active():
            raise ValueError('Campaign parameters are frozen. Stop the campaign before changing them.')
        if not self.preview:
            raise ValueError('Simulation is not running yet.')
        self.pending_comparison_change=True
        self.preview.requests.put(('joint_values',([float(v) for v in stiffness],[float(v) for v in damping])))
        self.kbox.blockSignals(True);self.kbox.setValue(float(stiffness[0]));self.kbox.blockSignals(False)
        self.dbox.blockSignals(True);self.dbox.setValue(float(damping[0]));self.dbox.blockSignals(False)
        self.profile_edit.setChecked(False)
        self.latest=None;self.last_analysis=None;self.last_model=self.last_vision=None;self.last_plot=0.
        self.applied_label.setText('Applying per-joint values… waiting for simulation acknowledgement')

    def apply_parameters(self):
        if self.campaign_active():
            self.command_status.setText("Campaign parameters are frozen. Stop the campaign before changing them.");return
        if self.preview:
            self.pending_comparison_change=True
            exponential=self.profile_edit.isChecked()
            self.preview.requests.put(('parameters', (self.kbox.value(), self.dbox.value(),
                self.rbox.value() if exponential else None,self.bbox.value() if exponential else None)))
            self.latest = None; self.last_analysis = None
            self.last_model=self.last_vision=None
            self.last_plot=0.
            self.analysis_message='Updating simulation parameters; waiting for a fresh comparison'
            self.applied_label.setText('Applying… waiting for simulation acknowledgement')
            self.history.append((self.history[-1][0] if self.history else 0., np.nan, np.nan))

    def show_video(self, item):
        self.last_video=item
        if self.hold_measured_view():return
        # Keep a recently completed synchronized pair briefly. Otherwise show raw
        # live video, with no old model painted on a different camera frame.
        if self.latest and self.latest.get('result') and time.monotonic()-self.last_pair_received < .25:
            return
        self.show_comparison(item)

    def hold_measured_view(self):
        """Keep a valid image/curve pair through the next vision calculation."""
        item=self.last_vision
        if self.analysis_mode!='vision' or not item or not item.get('observed'):return False
        elapsed=time.monotonic()-self.last_vision_display
        budget=min(1.,max(.25,1.5*item.get('vision_processing_s',0.)+.1))
        age=item['displayed_age_s']+time.monotonic()-item['received']
        return elapsed<budget and (self.demo or age<self.config['live']['max_age_s'])

    def show_comparison(self, item):
        item = dict(item)
        if 'received' in item and not self.demo:
            item['displayed_age_s'] += time.monotonic()-item['received']
            item['received'] = time.monotonic()
        if 'rgb' not in item:
            self.analysis_error='Analysis: '+item['message']
            return
        if item.get('analysis_mode',self.analysis_mode)!=self.analysis_mode:return
        result=item.get('result')
        if result and (self.pending_comparison_change or result.get('revision',0)<self.parameter_revision):
            return
        age_limit=5. if item.get('vision_result') else self.config['live']['max_age_s']
        if not self.demo and not item.get('raw') and item['displayed_age_s'] > age_limit:
            self.analysis_error='Analysis behind live video; score paused'
            return
        if item.get('vision_result'):
            self.last_vision=item
        elif not item.get('raw'):
            self.last_model=item
            # Display a measured pair briefly, always on its own camera image.
            # Incoming model-only frames still remain available for capture.
            if self.hold_measured_view():return
        self.latest = item
        if not item.get('raw'):
            self.last_analysis = item
        result, packet = item.get('result'), item.get('packet')
        if not item.get('raw'):
            self.analysis_message=item['message'];self.analysis_error=None
        if 'rgb' not in item:
            self.camera.stale = True; self.camera.update()
            return
        self.last_display = time.monotonic()
        if result: self.last_pair_received = self.last_display
        if item.get('vision_result'):self.last_vision_display=self.last_display
        if self.session and not self.demo:
            self.session.preview_received(item['displayed_age_s'])
        stale = not self.demo and item['displayed_age_s'] > age_limit
        full_curve=([result['base_px']]+result['predicted_curve_px']) if result and result.get('base_px') else result['predicted_curve_px'] if result else None
        if full_curve:
            points=np.asarray(full_curve);h,w=item['rgb'].shape[:2]
            if not ((points[:,0]>=0)&(points[:,0]<w)&(points[:,1]>=0)&(points[:,1]<h)).any():
                self.analysis_message='MuJoCo is outside the image. Open Alignment / cable mapping → Load supplied image alignment, then pick the base again.'
        self.camera.set_frame(item['rgb'], full_curve,
                              item['observed']['curve_px'] if item.get('observed') else None,
                              (result.get('landmark_rows') or item.get('landmark_rows',[])) if result else item.get('landmark_rows',[]),result.get('base_px') if result else None)
        if stale:
            self.camera.stale = True
            self.latest['result'] = None
            self.status.setText('Preview is stale; no current stiffness score')
            self.camera.update()
            return
        if result:
            cost=result['cost_px2']
            self.applied_label.setText(self.parameter_text(result,'Applied in simulation'))
            if item.get('vision_result') or result.get('observation'):
                self.history.append((result['t_s'], result['motion_error_px2'] if result['motion_error_px2'] is not None else np.nan,
                                     cost if cost is not None else np.nan))
                self.history = self.history[-1200:]
        if time.monotonic()-self.last_plot > .5:
            if self.history:
                t, instant, held = np.array(self.history).T
                self.error_line.set_data(t, instant); self.cost_line.set_data(t, held)
                self.cost_ax.relim(); self.cost_ax.autoscale_view()
            self.canvas.draw_idle(); self.last_plot = time.monotonic()

    def inspector_snapshot(self):
        item=self.last_model if self.analysis_mode=='manual' else self.last_vision or self.last_analysis
        if item is None:return None
        age=item['displayed_age_s']+time.monotonic()-item['received']
        limit=5. if item.get('vision_result') else self.config['live']['max_age_s']
        if not self.demo and (not (self.session or self.setup_session) or age>limit):
            if item is self.last_vision:
                item=self.last_model
                if item is None:return None
                age=item['displayed_age_s']+time.monotonic()-item['received']
                if age>self.config['live']['max_age_s']:return None
            else:return None
        return dict(item, displayed_age_s=0. if self.demo else age)

    def inspect_points(self):
        if self.analysis_mode=='manual':return self.capture_manual()
        from point_inspector import PointInspector
        if self.inspector is None:
            self.inspector=PointInspector(self.inspector_snapshot,self,apply_callback=self.use_profile)
        else:self.inspector.capture()
        self.inspector.show();self.inspector.raise_();self.inspector.activateWindow()

    def manual_source(self):
        """Prefer the latest raw frame; manual equilibrium is computed separately."""
        item=self.last_model if self.demo else self.last_video or self.last_model
        if item is None:return None
        delay=0. if self.demo else time.monotonic()-item['received']
        age=item['displayed_age_s']+delay
        if not self.demo and (not (self.session or self.setup_session) or age>self.config['live']['max_age_s']):return None
        parameters=dict(self.fit['candidate']['parameters'])
        parameters.update({k:self.comparison_setup[k] for k in ('translation_x_px','translation_y_px','rotation_rad','pixels_per_m')})
        packet=dict(item['packet'])
        packet['frame_age_s']+=delay
        packet['motor_age_s']=[None if v is None else v+delay for v in packet['motor_age_s']]
        return dict(item,packet=packet,parameters=parameters,d2=self.current_d2,setup=copy.deepcopy(self.comparison_setup),
            landmarks=self.reference_layout,recorded_demo=self.demo,setup_preview=bool(self.setup_session),
            displayed_age_s=0. if self.demo else age)

    def capture_manual(self):
        if self.manual_capture_busy:return
        if self.pending_comparison_change:
            self.command_status.setText('Wait for simulation acknowledgement before capturing changed settings.');return
        item=self.manual_source()
        if item is None:
            self.command_status.setText('No fresh camera frame. Start preview or reconnect the camera.');return
        self.manual_capture_busy=True
        self.command_status.setText('Freezing this camera frame and its measured cable inputs…')
        config=copy.deepcopy(self.config);fit=copy.deepcopy(self.fit)
        def work():
            from manual_analysis import prepare_capture
            try:self.notifications.put(('manual_capture',prepare_capture(item,config,fit)))
            except Exception as exc:self.notifications.put(('manual_capture_failed',str(exc)))
        threading.Thread(target=work,daemon=True).start()

    def manual_sweep(self, item, rows, mode, stationary_confirmed):
        if self.scan_busy:raise ValueError('A sweep is already running; wait for it to finish.')
        original=item['result']['setup']
        if any(original[k]!=self.comparison_setup[k] for k in ('mapping','signs','translation_x_px','translation_y_px','rotation_rad','pixels_per_m')):
            raise ValueError('Alignment or cable mapping changed after capture. Capture again before sweeping.')
        from manual_analysis import complete_capture
        self.scan(complete_capture(item,rows,mode,stationary_confirmed))

    def scan(self, item=None):
        if not isinstance(item,dict) and self.analysis_mode=='manual':return self.capture_manual()
        if not isinstance(item,dict):item=self.inspector_snapshot()
        if self.scan_busy or not item or not item.get('result'):return
        result=copy.deepcopy(item['result'])
        if not result['settled']:
            self.command_status.setText('Wait until both shapes have settled');return
        lower,upper,count=self.scan_min.value(),self.scan_max.value(),self.scan_count.value()
        if lower>=upper:
            self.command_status.setText('Sweep minimum must be below maximum.');return
        fit=copy.deepcopy(self.fit);fit['candidate']['parameters']=dict(item['parameters'])
        config=copy.deepcopy(self.config);config['comparison_setup']=copy.deepcopy(self.comparison_setup)
        self.scan_busy=True;self.scan_setup_revision=self.setup_revision
        snapshot=copy.deepcopy(item); revision=self.setup_revision
        self.command_status.setText('Sweeping one frozen observation; video and motor controls remain active')
        def work():
            try:
                model=Comparator(config,fit);scan=model.scan(result,count,lower,upper)
                scan.update(session=item['packet']['session'],frame_index=item['packet']['frame_index'],
                            frozen_d2=item['d2'],
                            setup_revision=revision,recorded_demo=bool(item.get('recorded_demo')),
                            model_sha256=hashlib.sha256(Path(model.model_path).read_bytes()).hexdigest(),
                            frame_sha256=hashlib.sha256(base64.b64decode(item['packet']['image_b64'])).hexdigest())
                self.notifications.put(('scan',(scan,snapshot)))
            except Exception as exc:self.notifications.put(('scan_failed',str(exc)))
        threading.Thread(target=work,daemon=True).start()

    def toggle_surface(self, checked):
        if not checked:
            if self.surface_dialog:self.surface_dialog.close()
            return
        if self.scan_busy or not self.sweep_result:
            self.command_status.setText('Complete a normal frozen-pose k₂ sweep first, then check 3D profile review. It will reuse those exact physical points and cable inputs.')
            self.surface_check.blockSignals(True);self.surface_check.setChecked(False);self.surface_check.blockSignals(False)
            return
        from surface_review import SurfaceReview
        self.surface_dialog=SurfaceReview(self.config['model'],self.sweep_result,self.use_surface_minimum,self)
        self.surface_dialog.finished.connect(self.surface_closed)
        self.surface_dialog.show()

    def surface_closed(self, _):
        self.surface_check.blockSignals(True);self.surface_check.setChecked(False);self.surface_check.blockSignals(False)

    def use_surface_minimum(self, surface):
        if surface['source']['setup_revision']!=self.setup_revision:
            raise ValueError('Alignment or point setup changed. Capture and sweep again before using this minimum.')
        if surface['source']['model_sha256']!=hashlib.sha256(Path(self.config['model']).read_bytes()).hexdigest():
            raise ValueError('Model XML changed after capture. Capture and sweep again.')
        p=surface['minimum']['parameters']
        self.kbox.setValue(p['k2']);self.rbox.setValue(p['stiffness_ratio']);self.bbox.setValue(p['stiffness_exponent'])
        self.profile_edit.setChecked(True);self.analysis_tabs.setCurrentIndex(0)
        self.command_status.setText('Surface minimum is in the k₂, r and b fields. Click Apply to simulation to use it; the live model is unchanged until then.')

    def select_best(self):
        if self.sweep_result:self.select_sweep_index(self.sweep_result['minimum']['index'])

    def select_sweep_index(self,index):
        scan=self.sweep_result
        self.selected_k=scan['k2'][index];self.kbox.setValue(self.selected_k)
        self.selection_line.set_data([self.selected_k],[scan['cost_px2'][index]])
        best=scan['minimum']
        warning=' Minimum is at the interval edge; consider a wider interval.' if best['at_boundary'] else ''
        if scan['setup_revision']!=self.setup_revision:warning+=' Setup changed: rerun before applying.'
        self.sweep_label.setText(f"Lowest sampled J = {best['cost_px2']:.5g} px²/point at k₂ = {best['k2']:.7g}. "
            f"Selected k₂ = {self.selected_k:.7g}."+warning)
        self.canvas.draw_idle()

    def select_scan_point(self,event):
        if event.inaxes!=self.scan_ax or event.xdata is None or not self.sweep_result:return
        valid=[i for i,c in enumerate(self.sweep_result['cost_px2']) if c is not None]
        index=min(valid,key=lambda i:abs(np.log(self.sweep_result['k2'][i])-np.log(max(event.xdata,1e-12))))
        self.select_sweep_index(index)

    def apply_sweep(self):
        if self.selected_k is None or self.sweep_result['setup_revision']!=self.setup_revision:return
        self.kbox.setValue(self.selected_k)
        fixed=self.sweep_result['fixed_parameters']
        self.rbox.setValue(fixed['stiffness_ratio']);self.bbox.setValue(fixed['stiffness_exponent'])
        self.apply_parameters()

    def export_sweep(self,path):
        if not self.sweep_result:raise ValueError('Run a sweep first.')
        scan=copy.deepcopy(self.sweep_result);scan['selected_k2']=self.selected_k
        scan['version']='0.9.1';scan['applied_k2']=self.fit['candidate']['parameters']['k2']
        scan['applied_parameters']=copy.deepcopy(self.fit['candidate']['parameters'])
        path=Path(path).with_suffix('.png')
        figure=Figure(figsize=(8,5),layout='constrained');ax=figure.subplots()
        ax.semilogx(scan['k2'],[np.nan if c is None else c for c in scan['cost_px2']],'o-')
        best=scan['minimum'];ax.plot(best['k2'],best['cost_px2'],'*',ms=13,color='#c55b00')
        ax.set(title=f"Frozen frame {scan['frame_index']} — lowest sampled J = {best['cost_px2']:.6g} at k₂ = {best['k2']:.7g}",
               xlabel='k₂ [N m/rad]',ylabel='J [px² / point]')
        fixed=scan['fixed_parameters']
        ax.text(.02,.98,('Boundary minimum: extend interval' if best['at_boundary'] else 'Minimum over sampled values; alignment and offsets fixed')
                +f"\nFixed r = {fixed['stiffness_ratio']:.8g}, b = {fixed['stiffness_exponent']:.8g}",
                transform=ax.transAxes,va='top',fontsize=9)
        ax.grid(alpha=.25);figure.savefig(path,dpi=180)
        with path.with_suffix('.csv').open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(['k2_Nm_per_rad','J_px2_per_point','lowest_sample','error'])
            for i,(k,c,error) in enumerate(zip(scan['k2'],scan['cost_px2'],scan['errors'])):writer.writerow([k,c,i==best['index'],error or ''])
        path.with_suffix('.json').write_text(json.dumps(scan,indent=2,allow_nan=False)+'\n')
        return path

    def save_sweep(self):
        if not self.sweep_result:return
        name,_=QFileDialog.getSaveFileName(self,'Save frozen k₂ sweep','spirob_k2_sweep.png','PNG image (*.png)')
        if not name:return
        path=Path(name).with_suffix('.png')
        if any(path.with_suffix(s).exists() for s in ('.csv','.json')):
            if QMessageBox.question(self,'Replace companion files?','Replace existing CSV/JSON companions?')!=QMessageBox.Yes:return
        try:self.export_sweep(path);self.command_status.setText('Saved sweep PNG, CSV and JSON: '+str(path))
        except (OSError,ValueError) as exc:QMessageBox.warning(self,'Save failed',str(exc))

    def export_pose(self, path):
        from pose_capture import export_pose
        if not self.sweep_snapshot or not self.sweep_result:raise ValueError('Freeze a pose and complete a sweep first.')
        return export_pose(path,self.sweep_snapshot,self.sweep_result)

    def save_pose(self):
        if not self.sweep_snapshot:return
        name,_=QFileDialog.getSaveFileName(self,'Save frozen pose + sweep','spirob_pose.png','PNG image (*.png)')
        if not name:return
        path=Path(name).with_suffix('.png')
        companions=[path.with_suffix('.json'),path.with_name(path.stem+'_joints.csv'),path.with_name(path.stem+'_sweep.csv'),path.with_name(path.stem+'_frame.png')]
        if any(p.exists() for p in companions):
            if QMessageBox.question(self,'Replace companion files?','Replace existing pose data companions?')!=QMessageBox.Yes:return
        try:self.export_pose(path);self.command_status.setText('Saved frozen pose screenshot, source frame, J, joint table and k₂ sweep: '+str(path))
        except (OSError,ValueError) as exc:QMessageBox.warning(self,'Save failed',str(exc))

    def disconnect(self):
        if self.campaign_window:self.campaign_window.begin_shutdown()
        if self.closing:
            return
        self.closing = True
        self.banner.setText('Disconnecting — waiting for motor and camera shutdown acknowledgements')
        if self.session:
            self.session.cancel.set()
        def work():
            if self.connect_thread and self.connect_thread.is_alive():
                self.connect_thread.join(65)
            try:final = self.session.disconnect() if self.session else None
            except Exception as exc:
                final=None
                self.notifications.put(('storage_warning','Shutdown could not be confirmed: '+str(exc)))
            if self.preview:
                self.preview.stop.set()
                self.preview.wait()  # Background wait: never delete a log with a live writer.
            if self.session and hasattr(self.session,'finish_storage'):
                try:self.session.finish_storage()
                except OSError as exc:self.notifications.put(('storage_warning',str(exc)))
            if self.campaign_window:self.campaign_window.shutdown_wait()
            self.notifications.put(('disconnected', final))
        threading.Thread(target=work, daemon=True).start()

    def ask_run_decision(self, entry):
        box=QMessageBox(self)
        box.setWindowTitle('Keep this run’s data?')
        recording=entry.get('recording',{})
        images=(f"Recorded images: {recording.get('frames',0)}; {recording.get('bytes',0)/1048576:.1f} MiB."
                if recording.get('frames') else 'No continuous video was recorded. Any campaign images and results are included with this run’s logs.')
        confirmed=entry.get('shutdown_confirmed',False)
        box.setText('Keep the collected data for '+str(entry.get('session') or 'this connection attempt')+'?')
        box.setInformativeText(('Hardware shutdown confirmed.\n' if confirmed else
            'Hardware shutdown is not confirmed. Data will be retained; Discard is unavailable.\n')+
            images+'\n\nSave keeps motor/analysis logs, campaign poses/annotations/results and any recorded video. '
            'Discard removes only this run’s collected files. Separate image/graph exports remain. '
            'Decide later keeps the data pending. Review it explicitly through Session / data → Review pending run data.')
        save=box.addButton('Save',QMessageBox.AcceptRole)
        discard=box.addButton('Discard',QMessageBox.DestructiveRole);discard.setEnabled(confirmed)
        later=box.addButton('Decide later',QMessageBox.RejectRole)
        already_discarding=entry.get('decision')=='discard_requested' or entry.get('requested_decision')=='discard'
        if already_discarding:
            save.setEnabled(False)
            box.setInformativeText(box.informativeText()+'\n\nDiscard was already requested. Some files may already be removed; choose Discard to finish cleanup or Decide later to postpone.')
        box.setDefaultButton(discard if already_discarding and confirmed else later if already_discarding else save)
        box.setEscapeButton(later)
        box.exec()
        return 'keep' if box.clickedButton()==save else 'discard' if box.clickedButton()==discard else 'later'

    def recover_runs(self):
        if self.demo or self.session or self.setup_session or self.setup_connecting or self.connecting or self.closing or self.review_busy:return
        self.review_busy=True
        self.command_status.setText('Checking for run data awaiting your Save / Discard decision…')
        def work():
            try:self.notifications.put(('review_list',review_entries('logs',self.rig)))
            except Exception as exc:self.notifications.put(('review_failed',str(exc)))
        threading.Thread(target=work,daemon=True).start()

    def begin_review(self, entries):
        self.review_busy=True
        self.review_queue=list(entries)
        self.next_review()

    def next_review(self):
        if not self.review_queue:
            if self.campaign_window:self.campaign_window.end_shutdown()
            self.review_busy=False
            if self.close_when_done:self.close()
            return
        entry=self.review_queue.pop(0)
        choice=self.ask_run_decision(entry)
        if choice=='later':
            self.command_status.setText('Run retained pending your decision. Use Session / data → Review pending run data when ready.')
            QTimer.singleShot(0,self.next_review)
            return
        self.command_status.setText('Saving run data…' if choice=='keep' else 'Discarding this run after confirmed shutdown…')
        def work():
            try:self.notifications.put(('review_done',finish_review('logs',self.rig,entry,choice)))
            except Exception as exc:self.notifications.put(('review_failed',str(exc)))
        threading.Thread(target=work,daemon=True).start()

    def motion_enabled(self, button, enabled):
        # Disabling a focused jog used to advance Qt focus into a scroll area.
        # Keep keyboard focus on the fixed Stop control while motion is active.
        if not enabled and button.hasFocus() and self.hold_button.isEnabled():
            self.hold_button.setFocus(Qt.OtherFocusReason)
        button.setEnabled(enabled)

    def tick(self):
        if self.setup_session:
            self.setup_session.touch()
            while True:
                try:event=self.setup_session.events.get_nowait()
                except queue.Empty:break
                if event['type']=='preview_failed' and not self.setup_stopping:
                    self.command_status.setText(event['message']);self.stop_setup_preview()
        if self.session:
            self.session.touch()
            while True:
                try:
                    event = self.session.events.get_nowait()
                except queue.Empty:
                    break
                if self.campaign_window:self.campaign_window.motor_event(event)
                if event['type'] == 'status':
                    self.remote_received=time.monotonic()
                    self.remote = event
                    rec = event.get('recording', {})
                    self.record_button.setText('Stop recording' if rec.get('active') else 'Start recording')
                    self.record_status.setText(('RECORDING' if rec.get('active') else 'Preview only')+
                        f" | {rec.get('seconds',0):.1f}/60 s | {rec.get('bytes',0)/1048576:.1f}/250 MiB | "+rec.get('reason',''))
                    self.banner.setText(f'{event["session"]} | {event["phase"]} | {event["elapsed_s"]:.0f}/{event.get("session_limit_s",600)} s')
                    if event.get('rest_lengths_mm'):
                        self.update_length_bounds()
                elif event['type'] in ('accepted', 'rejected', 'starting', 'closing', 'startup_failed'):
                    self.command_status.setText(event.get('message', event['type']))
                    if event['type'] == 'startup_failed': self.disconnect()
                elif event['type'] == 'transport_closed' and not self.closing:
                    self.disconnect()
        while True:
            try:
                kind, value = self.notifications.get_nowait()
            except queue.Empty:
                break
            if kind == 'manual_capture':
                self.manual_capture_busy=False
                if self.analysis_mode!='manual' or self.closing:continue
                from point_inspector import PointInspector
                if self.inspector:self.inspector.close()
                self.inspector=PointInspector(lambda item=value:item,self,manual_mode=True,
                    manual_callback=self.manual_sweep,recapture_callback=self.capture_manual)
                self.inspector.show();self.inspector.raise_();self.inspector.activateWindow()
                self.command_status.setText(value['message'])
            elif kind == 'manual_capture_failed':
                self.manual_capture_busy=False;self.command_status.setText('Capture could not be prepared: '+value)
            elif kind == 'setup_connected':
                self.setup_connecting=False
                if not self.setup_stopping:
                    self.banner.setText('SETUP PREVIEW — motors disconnected; session timer has not started')
                    self.start_preview()
            elif kind == 'setup_failed':
                self.command_status.setText('Setup camera failed: '+value)
                self.setup_connecting=False
                if not self.setup_stopping:self.stop_setup_preview()
            elif kind == 'setup_stopped':
                self.setup_session=None;self.preview=None;self.setup_connecting=self.setup_stopping=False
                if not self.connect_after_setup or value:self.set_camera_checked(False)
                self.latest=self.last_analysis=self.last_model=self.last_video=self.last_vision=None
                self.camera.set_reference(self.standalone_reference)
                self.banner.setText('Setup preview stopped'+(': '+value if value else ''))
                start=self.connect_after_setup;self.connect_after_setup=False
                if self.close_when_done:self.close()
                elif start and not value:self.connect()
            elif kind == 'connected':
                self.connecting = False
                if not self.closing: self.start_preview()
            elif kind == 'connect_failed':
                self.connecting = False
                self.banner.setText('Connection failed: '+value)
                self.command_status.setText(value)
                self.disconnect()
            elif kind == 'disconnected':
                self.preview = None
                self.closing = self.connecting = False
                confirmed = value and (value.get('motor_shutdown') or {}).get('worker_exited') and (value.get('motor_shutdown') or {}).get('disable_confirmed') == [True, True]
                self.banner.setText('Disconnected — motor torque release confirmed' if confirmed else 'Disconnected — motor shutdown was NOT confirmed; inspect the laptop/run.json')
                entry=self.session.review_entry() if self.session and hasattr(self.session,'review_entry') else None
                self.command_status.setText(((value.get('storage_message','Saved')+': '+value['path'])+
                    (' | '+value['error'] if value.get('error') else '')) if value else 'No shutdown report received')
                self.session = None; self.remote = {}; self.latest = None; self.last_analysis = None
                self.last_model=self.last_video=self.last_vision=None
                self.set_camera_checked(False);self.camera.set_reference(self.standalone_reference)
                self.record_button.setText('Start recording')
                self.record_status.setText('Disconnected. Collected data is pending your Save / Discard decision.')
                self.begin_review([entry] if entry else [])
            elif kind == 'storage_warning':
                self.command_status.setText('Recovery journal warning; files retained: '+value)
            elif kind == 'review_list':
                if not value:self.command_status.setText('No pending run data to review.')
                self.begin_review(value)
            elif kind == 'review_done':
                self.command_status.setText('Run data saved.' if value['decision']=='keep' else 'This run’s collected data was discarded.')
                self.next_review()
            elif kind == 'review_failed':
                self.command_status.setText('Review incomplete; remaining files retained: '+value+' Use Review pending run data to retry.')
                if self.close_when_done:
                    QMessageBox.warning(self,'Run review incomplete',value+'\nRemaining files are retained. Reopen the app and use Review pending run data to retry.')
                self.next_review()
            elif kind == 'scan':
                self.scan_busy = False
                value,self.sweep_snapshot=value
                self.sweep_result=value
                self.scan_line.set_data(value['k2'],[np.nan if x is None else x for x in value['cost_px2']])
                best=value['minimum'];self.minimum_line.set_data([best['k2']],[best['cost_px2']])
                self.scan_ax.set_title(f"Frozen t={value['frame_t_s']:.2f} s — select, then apply")
                self.scan_ax.relim();self.scan_ax.autoscale_view();self.select_best()
                self.command_status.setText('Lowest sampled k₂ selected. Open J / k₂ plots to inspect; Apply selection updates the model. Save sweep exports the results.')
            elif kind == 'scan_failed':
                self.scan_busy = False; self.command_status.setText(value)
        if self.campaign_window:self.campaign_window.tick()
        camera_item=self.last_video or self.latest
        fresh = self.demo or bool(camera_item and 'rgb' in camera_item and
            camera_item['displayed_age_s'] + time.monotonic() - camera_item['received'] < self.config['live']['max_age_s'])
        if not fresh and self.camera.frame is not None:
            self.camera.stale = True; self.camera.update(); self.latest = None
            self.status.setText('No fresh comparison; displayed image is stale')
        ready = self.session is not None and not self.connecting and not self.closing
        self.camera_toggle.setEnabled(not self.connecting and not self.closing and not self.setup_stopping and not self.setup_connecting and not self.review_busy)
        self.update_view_visibility()
        self.motion_settings_button.setEnabled(not self.session and not self.connecting and not self.closing)
        self.duration.setEnabled(not self.session and not self.connecting and not self.closing)
        self.difference_limit.setEnabled(not self.session and not self.connecting and not self.closing)
        self.connect_button.setEnabled(not self.demo and self.session is None and not self.connecting and not self.closing and not self.setup_connecting and not self.setup_stopping and not self.review_busy)
        self.review_button.setEnabled(not self.demo and self.session is None and not self.setup_session and not self.setup_connecting and not self.connecting and not self.closing and not self.review_busy)
        self.disconnect_button.setEnabled(not self.demo and (self.session is not None or self.connecting) and not self.closing)
        self.arm_button.setEnabled(ready and self.remote.get('phase') == 'READY_DISABLED' and self.at_rest.isChecked() and bool(fresh))
        self.hold_button.setEnabled(self.session is not None and not self.closing)
        self.move_button.setEnabled(ready and self.remote.get('phase') == 'HOLDING' and bool(fresh))
        self.mean_button.setEnabled(ready and self.remote.get('phase')=='HOLDING' and bool(fresh))
        self.setup_enable.setEnabled(ready and self.remote.get('phase')=='READY_DISABLED' and bool(fresh))
        self.reference_button.setEnabled(ready and self.remote.get('phase')=='SETUP_HOLD' and self.at_rest.isChecked() and bool(fresh))
        for button in self.jog_buttons:
            self.motion_enabled(button,ready and self.remote.get('phase') in ('HOLDING','SETUP_HOLD') and bool(fresh))
        for button in self.axis_buttons:self.motion_enabled(button,ready and self.remote.get('phase')=='HOLDING' and bool(fresh))
        self.motion_enabled(self.length_move,ready and self.remote.get('phase')=='HOLDING' and bool(fresh))
        snapshot=self.inspector_snapshot()
        if self.analysis_mode=='manual':snapshot=self.manual_source()
        self.inspect_button.setEnabled(snapshot is not None and snapshot.get('rgb') is not None and not self.manual_capture_busy)
        self.record_button.setEnabled(ready and bool(fresh))
        self.apply_button.setEnabled(self.preview is not None)
        item=self.inspector_snapshot();result=item.get('result') if item else None
        if self.analysis_mode=='manual':
            self.scan_button.setText('Capture pose + place points for k₂')
            available=self.manual_source() is not None
            reason=('Capture available. Place at least four physical points, confirm the pose was stationary, then sweep. No image estimation or tracking.' if available else 'Start camera preview or Connect to capture an image. Manual mode does not need a green curve.')
            if self.manual_capture_busy:reason='Preparing the captured image and its encoder-based equilibrium model…'
            self.scan_button.setEnabled(available and not self.scan_busy and not self.manual_capture_busy and not self.pending_comparison_change)
        else:
            self.scan_button.setText('Freeze measured pose + sweep k₂')
            if not item:reason='Waiting for a camera/model frame. Start preview or Connect. Manual points mode is available without extraction.'
            elif not result or result.get('setup_preview'):reason=item.get('message','Connect, enable and declare rest to provide model inputs.')
            elif not item.get('observed'):reason=item.get('message','Waiting for image measurement. Try Manual points mode.')
            elif not result.get('settled'):reason=result.get('message','Waiting for a stable image measurement.')
            else:reason=f"Ready: measured frame {item['packet']['frame_index']} · age {item['displayed_age_s']:.2f} s. This exact image will be swept."
            self.scan_button.setEnabled(bool(result and result.get('settled')) and not self.scan_busy and not self.pending_comparison_change)
        if self.scan_busy:reason='Sweeping the frozen pose. Live camera and motor controls continue.'
        elif self.pending_comparison_change:reason='Waiting for the simulation to acknowledge changed parameters or alignment.'
        self.sweep_ready.setText(reason)
        self.scan_button.setToolTip(reason)
        self.best_button.setEnabled(self.sweep_result is not None)
        self.save_scan_button.setEnabled(self.sweep_result is not None)
        self.capture_pose_button.setEnabled(self.sweep_result is not None and self.sweep_snapshot is not None and not self.scan_busy)
        self.apply_scan_button.setEnabled(bool(self.sweep_result and self.preview and
            self.sweep_result['setup_revision']==self.setup_revision and self.selected_k is not None))
        active_campaign=self.campaign_active()
        self.alignment_button.setEnabled(not active_campaign)
        self.analysis_mode_box.setEnabled(not active_campaign)
        if active_campaign:
            for widget in (self.move_button,self.mean_button,self.setup_enable,self.reference_button,self.length_move,
                self.arm_button,self.apply_button,self.apply_scan_button,self.scan_button,self.inspect_button,*self.jog_buttons,*self.axis_buttons):
                self.motion_enabled(widget,False)
        if time.monotonic()-self.last_summary>=.5:
            self.refresh_summary(item);self.last_summary=time.monotonic()

    def closeEvent(self, event):
        if self.setup_session or self.setup_connecting or self.setup_stopping:
            self.close_when_done=True;self.stop_setup_preview();event.ignore()
        elif self.review_busy:
            self.close_when_done=True
            event.ignore()
        elif self.session or self.connecting or self.closing:
            self.close_when_done = True
            self.disconnect()
            event.ignore()
        else:
            if self.preview:
                self.preview.stop.set(); self.preview.wait(2500)
            if self.inspector:self.inspector.close()
            if self.surface_dialog:self.surface_dialog.close()
            if self.campaign_window:self.campaign_window.close()
            event.accept()
