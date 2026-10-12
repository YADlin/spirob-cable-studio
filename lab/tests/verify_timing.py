"""A refreshed host offset restores valid bracketing without extrapolation."""
from pathlib import Path
import sys
import tempfile
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from spirob_live.packets import make_packet
from analysis import inputs_at

with tempfile.TemporaryDirectory() as directory:
    session=Path(directory)/'SESSION';(session/'camera').mkdir(parents=True);(session/'motor').mkdir()
    (session/'motor/samples.csv').write_text('axis,state,absolute_mm,target_absolute_mm,read_start_s,read_end_s,velocity_mm_s\n'+''.join(
        f'{axis},HOLDING,220,220,{t},{t},0\n' for t in (11.7,11.8,11.9,12.) for axis in (0,1)))
    class Capture:
        path=session/'camera';session_id='SESSION'
        context=dict(coordinator_monotonic_s=100.,motor_origin_s=0.,motor_log='motor',
            clock_before={'best':{'offset_s':899.},'network_uncertainty_s':.003},trial={'rest_lengths_mm':[220,220]})
        def status(self,renew=False):return {'state':'STREAMING'}
        def snapshot(self):
            return [dict(frame_index=i,jpeg=b'jpeg',read_end_s=t) for i,t in enumerate([1011.8,1011.9,1012.])], self.context
    capture=Capture()
    packet=make_packet(capture,now=1012.1)
    try:inputs_at(packet,packet['t_s'],.65)
    except ValueError:pass
    else:raise AssertionError('Incorrect clock offset silently extrapolated encoders')
    capture.context['clock_before']['best']['offset_s']=900.
    packet=make_packet(capture,now=1012.1)
    lengths,_=inputs_at(packet,packet['t_s'],.65)
    assert packet['frame_index']==2 and all(x==220 for x in lengths)
    assert all(0<=age<.2 for age in packet['motor_age_s'])
print('PASS: refreshed clock offset restores image/encoder bracketing; stale mapping is not extrapolated')
