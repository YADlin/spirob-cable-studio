"""Serializable per-session limits; no hardware imports or register writes."""
import base64
import json
import math

MAX_SPEED = 20.0  # Matches the existing motor controller's application ceiling.


def validate_profile(profile, rests=None):
    p = json.loads(json.dumps(profile, allow_nan=False))
    def number(v):
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise ValueError('Motion settings must be finite numbers.')
        return float(v)
    rr = p['rest_lengths_mm']
    bb = p['absolute_bounds_mm']
    if len(rr) != 2 or len(bb) != 2 or any(len(b) != 2 for b in bb):
        raise ValueError('Provide two references and two min/max length pairs.')
    rr = [number(v) for v in rr]
    bb = [[number(v) for v in b] for b in bb]
    if any(not 70 < r <= 10000 or not 0 < lo < hi <= 10000 or not lo <= r <= hi or
           max(r-lo, hi-r) > 1000 for r, (lo, hi) in zip(rr, bb)):
        raise ValueError('References must exceed 70 mm, lie within positive limits, and travel must not exceed 1000 mm from reference.')
    if rests is not None and any(abs(a-b)>1e-6 for a,b in zip(rr,rests)):
        raise ValueError('Motion profile and session reference lengths disagree.')
    speed = number(p.get('max_speed_mm_s', MAX_SPEED))
    setup = number(p.get('setup_travel_mm', 150.))
    difference = number(p.get('difference_mm', max(bb[0][1]-bb[1][0],bb[1][1]-bb[0][0])))
    if not .05 <= speed <= MAX_SPEED or not .5 <= setup <= 1000 or not 1 <= difference <= 10000:
        raise ValueError('Speed ceiling: 0.05..20 mm/s; setup travel: 0.5..1000 mm; difference: 1..10000 mm.')
    return dict(rest_lengths_mm=rr, absolute_bounds_mm=bb, max_speed_mm_s=speed,
                setup_travel_mm=setup, difference_mm=difference,
                travel_mm=max(max(r-lo,hi-r) for r,(lo,hi) in zip(rr,bb)))


def encode_profile(profile):
    return base64.urlsafe_b64encode(json.dumps(validate_profile(profile),allow_nan=False).encode()).decode().rstrip('=')


def decode_profile(text):
    if len(text)>8192:raise ValueError('Motion profile too large')
    return validate_profile(json.loads(base64.urlsafe_b64decode(text+'='*(-len(text)%4))))
