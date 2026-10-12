"""Image-only landmark spacing; no Qt, motor commands or parameter fitting."""
import numpy as np


def rigid_points(path, profile):
    """Walk backwards from the visible tip, intersecting fixed-radius circles.

    Distances are projected joint-to-joint chords, not smoothed arc lengths.
    No model pose or stiffness is used. Missing coverage stays missing.
    """
    path = np.asarray(path, float)
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    at = arc[-1] - profile['anchor_tail_px']
    names = profile['chain_ids']
    points = {name: None for name in names}
    if not 0 <= at <= arc[-1]: return points
    point = np.array([np.interp(at, arc, path[:, j]) for j in (0, 1)])
    points[names[-1]] = point.tolist()
    for name, radius in zip(names[-2::-1], profile['link_lengths_px'][::-1]):
        # Truncate at the preceding intersection; search in distal-to-base order.
        i = int(np.searchsorted(arc, at, side='left')) - 1
        end = point.copy()
        found = False
        while i >= 0:
            start = path[i]; vector = start - end
            a = float(vector @ vector)
            b = float(2 * ((end-point) @ vector))
            c = float((end-point) @ (end-point) - radius**2)
            disc = b*b - 4*a*c
            if a > 1e-14 and disc >= -1e-8:
                roots = sorted(((-b-np.sqrt(max(0., disc)))/(2*a),
                                (-b+np.sqrt(max(0., disc)))/(2*a)))
                valid = [u for u in roots if 0 <= u <= 1+1e-9]
                if valid:
                    u = min(1., valid[0]); point = end + u*vector
                    at = arc[i] + np.linalg.norm(point-start)
                    points[name] = point.tolist(); found = True; break
            end = start; i -= 1
        if not found: break
    return points


def correspondences(observed, layout, mode='joints', tip_offset_px=0., arc_scale=1., manual=None, excluded=(), profile=None):
    """Do not clamp invisible landmarks onto the first/last visible pixel."""
    manual = manual or {}
    path = np.asarray((observed or {}).get('curve_px', []), dtype=float).reshape(-1, 2)
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    tails = np.asarray(layout['tail_m']) * layout['pixels_per_m']
    predicted = None if layout['model_px'] is None else np.asarray(layout['model_px'])
    ids = list(layout['ids'])
    if mode == 'centres':
        tails = (tails[:-1]+tails[1:])/2
        predicted = None if predicted is None else (predicted[:-1]+predicted[1:])/2
        ids = ['link_'+name.split('_')[-1] for name in ids[:-1]]
    rows = []
    authoritative = bool(profile and profile.get('measurement_mode') == 'manual_joints')
    tracked = (observed or {}).get('joint_points_px', {}) if authoritative and (observed or {}).get('calibration_id') == profile['calibration_id'] else {}
    rigid = rigid_points(path, profile) if len(path) > 1 and profile and profile.get('kind') == 'rigid_links' and not authoritative else None
    for i, (name, tail) in enumerate(zip(ids, tails)):
        saved = (profile or {}).get('tail_px', {}).get(name)
        at = arc[-1]-((saved if saved is not None else tail)*arc_scale+tip_offset_px)
        physical, source = None, 'outside visible arc'
        if name in manual:
            physical, source = list(manual[name]), 'manual'
        elif authoritative:
            physical = tracked.get(name)
            source = (observed or {}).get('measurement_source', 'image tracked') if physical is not None else 'awaiting image tracking'
        elif rigid is not None and name in rigid:
            physical = rigid[name]
            source = 'fixed-length estimate' if physical is not None else 'outside rigid-chain coverage'
        elif len(path) > 1 and 0 <= at <= arc[-1]:
            physical = [float(np.interp(at, arc, path[:, j])) for j in (0, 1)]
            source = 'profile estimate' if saved is not None else 'estimated'
        virtual = None if predicted is None else predicted[i].tolist()
        delta = None if physical is None or virtual is None else (np.array(virtual)-physical).tolist()
        rows.append(dict(id=name, physical_px=physical, model_px=virtual, source=source,
            included=name not in excluded and (not profile or name in profile.get('score_ids', profile['tail_px'])), dx_px=None if delta is None else delta[0],
            dy_px=None if delta is None else delta[1],
            distance_px=None if delta is None else float(np.linalg.norm(delta))))
    return rows


def placement_profile(observed, rows, mode, frame_index, image_shape, use_for_score=False, rigid=False):
    """Project verified clicks onto the image curve and store distance from tip.

    Frozen x,y pixels are never replayed on moving video. Large cross-track
    edits are rejected because this feature cannot repair a wrong centreline.
    """
    path = np.asarray(observed['curve_px'], float)
    segment = np.diff(path, axis=0)
    length = np.linalg.norm(segment, axis=1)
    arc = np.r_[0., np.cumsum(length)]
    tails, deviations, projected = {}, {}, {}
    for row in rows:
        if (not rigid and not row['included']) or row['physical_px'] is None: continue
        point = np.asarray(row['physical_px'], float)
        u = np.clip(np.sum((point-path[:-1])*segment, axis=1)/np.maximum(length**2, 1e-12), 0., 1.)
        nearest = path[:-1]+u[:, None]*segment
        distance = np.linalg.norm(nearest-point, axis=1)
        i = int(np.argmin(distance))
        if distance[i] > 15:
            raise ValueError(f"{row['id']} is {distance[i]:.1f} px from the measured centreline. Fix the centreline/ROI first.")
        tails[row['id']] = float(arc[-1]-arc[i]-u[i]*length[i])
        deviations[row['id']] = float(distance[i])
        projected[row['id']] = nearest[i].tolist()
    if len(tails) < 4:
        raise ValueError('Include at least four visible, correctly ordered points.')
    if any(b >= a for a, b in zip(tails.values(), list(tails.values())[1:])):
        raise ValueError('Points must remain in base-to-tip order, with distinct positions.')
    result = dict(mode=mode, tail_px=tails, use_for_score=bool(use_for_score),
                source_frame=frame_index, image_shape=list(image_shape[:2]),
                projection_error_px=deviations,
                method='Fixed tip-relative arc distances along each new image centreline; estimated, not detected joints.')
    if rigid:
        if mode != 'joints': raise ValueError('Rigid lengths require joint anchors, not element centres.')
        indices = [i for i, r in enumerate(rows) if r['id'] in projected]
        if indices != list(range(indices[0], len(rows))) or rows[-1]['id'] != 'tip':
            raise ValueError('Place every joint from the first visible joint through the tip. Hidden proximal joints may be omitted.')
        names = list(projected)
        lengths = np.linalg.norm(np.diff(list(projected.values()), axis=0), axis=1)
        if np.any(lengths < 1.): raise ValueError('Adjacent joints must be at least 1 pixel apart.')
        score_ids = [r['id'] for r in rows if r['included'] and r['id'] in projected]
        if len(score_ids) < 4: raise ValueError('Include at least four visible points in J.')
        result.update(kind='rigid_links', chain_ids=names, link_lengths_px=lengths.tolist(),
                      anchor_tail_px=tails[names[-1]], calibration_points_px=projected,
                      score_ids=score_ids,
                      method='Fixed projected chord lengths, anchored at tip and followed along the measured curve. Planar motion and fixed camera required.')
    return result


def scale_from_profile(profile, layout):
    """Compare the same adjacent joint intervals, never base-to-tip chord length."""
    if profile.get('kind') != 'rigid_links': raise ValueError('Apply a rigid joint calibration first.')
    ids = layout['ids']; tails = dict(zip(ids, layout['tail_m']))
    names = profile['chain_ids']
    model_length = sum(tails[a]-tails[b] for a,b in zip(names,names[1:]))
    if model_length <= 0: raise ValueError('No matching model length.')
    return float(sum(profile['link_lengths_px']) / model_length)
