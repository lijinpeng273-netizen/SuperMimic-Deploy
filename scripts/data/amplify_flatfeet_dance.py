"""Increase pitch and late waist motion of an already repaired 30 FPS H1 CSV.

Preserves frame timing and all eight arm trajectories. The first 0.6 s keeps
the original pitch, with a smooth gain ramp through 2 s. Waist amplification
ramps between 40% and 55% of the motion. Planted-foot IK is rebuilt after the
upper-body changes. The source and existing outputs are never overwritten.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from repair_dance_flatfeet import rebuild_planted_feet
from h1_deploy.data.motion_convert import load_csv, convert_csv_to_npz
from h1_deploy.deploy.export import export_clip
from h1_deploy.robots.h1 import load_h1_model


def smoothstep(x):
    x = np.clip(x, 0., 1.)
    return x*x*x*(10.+x*(-15.+6.*x))


def amplify(source, *, forward_gain, backward_gain, late_waist_gain):
    gains = np.array([forward_gain, backward_gain, late_waist_gain])
    if not np.isfinite(gains).all() or (gains < 1).any():
        raise ValueError('amplitude gains must be finite and >= 1')
    raw = load_csv(source)
    if len(raw) < 61:
        raise ValueError('expected a repaired 30 FPS dance longer than 2 seconds')
    euler = Rotation.from_quat(raw[:, 3:7]).as_euler('xyz')
    if np.max(np.abs(euler[:, [0, 2]])) > 1e-5 or np.max(np.abs(raw[:, 1])) > 1e-5:
        raise ValueError('source must have fixed stance, zero pelvis yaw/roll and y')
    t = np.arange(len(raw))/30.
    pitch = euler[:, 1]
    # Smoothly choose stronger backward amplification without a derivative kink
    # at upright. Do not perturb the initial posture or the input time axis.
    backward_weight = .5*(1.-np.tanh(pitch/np.deg2rad(3.)))
    gain = forward_gain + (backward_gain-forward_gain)*backward_weight
    pitch = pitch*(1.+smoothstep((t-.6)/1.4)*(gain-1.))
    q = raw[:, 7:].copy()
    waist_envelope = smoothstep((t/t[-1]-.40)/.15)
    q[:, 10] *= 1.+(late_waist_gain-1.)*waist_envelope
    out, info = rebuild_planted_feet(pitch, raw[:, 2], q)
    return out, dict(source=str(source), forward_gain=forward_gain,
                     backward_gain=backward_gain, late_waist_gain=late_waist_gain,
                     input_fps=30, output_fps=30, frames=len(out), duration_s=t[-1],
                     waist_ramp_s=[float(.4*t[-1]), float(.55*t[-1])],
                     pitch_range_deg=np.rad2deg([pitch.min(), pitch.max()]).tolist(),
                     waist_range_deg=np.rad2deg([q[:, 10].min(), q[:, 10].max()]).tolist(),
                     **info)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source', type=Path, default=Path('data/use/dance_smooth_flatfeet_bolder.csv'))
    ap.add_argument('--name', required=True)
    ap.add_argument('--forward-gain', type=float, required=True, help='pitch amplitude multiplier when leaning forward')
    ap.add_argument('--backward-gain', type=float, required=True, help='pitch amplitude multiplier when leaning backward')
    ap.add_argument('--late-waist-gain', type=float, required=True, help='waist multiplier after the smooth 40%-55% ramp')
    ap.add_argument('--data-dir', type=Path, default=Path('data/use'))
    ap.add_argument('--deploy-dir', type=Path, default=Path('deploy_artifacts/use'))
    a = ap.parse_args()
    if Path(a.name).name != a.name or a.name in ('', '.', '..'):
        ap.error('--name must be a filename stem without directories')
    csv = a.data_dir/(a.name+'.csv')
    motion = a.data_dir/(a.name+'.npz')
    deploy = a.deploy_dir/(a.name+'.npz')
    paths = [csv, motion, deploy]
    if len({p.resolve() for p in paths}) != 3 or a.source.resolve() in {p.resolve() for p in paths}:
        ap.error('source, CSV, motion NPZ and deploy NPZ must be different paths')
    if any(p.exists() for p in paths):
        ap.error('output already exists; choose a new --name')
    out, info = amplify(a.source, forward_gain=a.forward_gain,
                        backward_gain=a.backward_gain, late_waist_gain=a.late_waist_gain)
    model = load_h1_model()
    # Check the reference, independently of the controller's runtime protections.
    from h1_deploy.deploy.models import load_scene
    scene = load_scene(torque=True)
    limits = scene.jnt_range[1:]
    if not np.isfinite(out).all() or (out[:, 7:] < limits[:, 0]).any() or (out[:, 7:] > limits[:, 1]).any():
        raise ValueError('amplified reference violates a hard joint limit or has nonfinite values')
    a.data_dir.mkdir(parents=True, exist_ok=True)
    a.deploy_dir.mkdir(parents=True, exist_ok=True)
    np.savetxt(csv, out, delimiter=',', fmt='%.10f')
    info['conversion'] = convert_csv_to_npz(csv, motion, input_fps=30, output_fps=50, model=model)
    export_clip(motion, deploy)
    info['deployment'] = str(deploy)
    print(json.dumps(info, indent=2))


if __name__ == '__main__':
    main()
