"""Read-only pelvis tracking diagnostics in the policy's reference world frame."""
from __future__ import annotations

import numpy as np


def pelvis_tracking(clip, frame, position):
    """Compare against the current frame before core.step advances it.

    This is a pelvis-position diagnostic, not the network's future torso-anchor
    feature. Do not alter the measured pose, reference, or control decisions.
    """
    missing = np.full(3, np.nan)
    if position is None or clip.root_pos is None:
        return missing.copy(), missing
    current = np.asarray(position, dtype=float)
    reference = np.asarray(clip.root_pos[min(max(int(frame), 0), clip.num_frames - 1)], dtype=float)
    if current.shape != (3,) or not np.isfinite(current).all() or not np.isfinite(reference).all():
        return missing.copy(), missing
    return reference.copy(), current - reference


def pelvis_error_text(error):
    e = np.asarray(error, dtype=float)
    if e.shape != (3,) or not np.isfinite(e).all():
        return 'pelvis err unavailable'
    return (f'pelvis err XYZ=[{e[0]:+.3f}, {e[1]:+.3f}, {e[2]:+.3f}] m '
            f'(actual-ref), XY={np.linalg.norm(e[:2]):.3f} m')


def summarize_pelvis(z, meta):
    """Report each recorded control phase/clip, without reopening mutable clips."""
    if 'root_pos_error' not in z:
        return  # Old logs did not store their reference poses.
    error = z['root_pos_error']
    state = z['root_tracking_state']
    clip = z['clip']
    if not len(error):
        return
    cut = np.flatnonzero((state[1:] != state[:-1]) | (clip[1:] != clip[:-1])) + 1
    for a, b in zip(np.r_[0, cut], np.r_[cut, len(error)]):
        if state[a] < 0:
            continue
        valid = np.flatnonzero(np.isfinite(error[a:b]).all(axis=1)) + a
        if not len(valid):
            continue
        e = error[valid]
        xy = np.linalg.norm(e[:, :2], axis=1)
        peak = valid[int(xy.argmax())]
        phase = meta['states'][int(state[a])]
        name = meta['clips'][int(clip[a])]
        mean = ', '.join(f'{v:+.3f}' for v in e.mean(axis=0))
        print(f'[summary] pelvis {name} {phase}: XYZ mean=[{mean}] m (actual-ref); '
              f'XY mean/max {xy.mean():.3f}/{xy.max():.3f} m; '
              f'Z min/max {e[:, 2].min():+.3f}/{e[:, 2].max():+.3f} m; '
              f'XY peak ref frame {int(z["root_ref_frame"][peak])} '
              f'({z["root_ref_time_s"][peak]:.2f} s), run t={z["t"][peak]:.3f} s')
