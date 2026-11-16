"""Create a conservative H1 dance from a 30 FPS headerless 26-column CSV.

The new reference keeps both feet flat and planted, reduces pelvis pitch,
transfers part of pelvis yaw to the waist, and preserves the order of arm motion.
Planar leg IK and a whole-body COM constraint replace the original lower-body
trajectory. This intentionally removes tiptoe, stepping, and lateral travel.

Example (from repository root, in the supermimic environment):
  python -B scripts/data/repair_dance_flatfeet.py --source data/use/dance_smooth.csv

For a moderately larger bow, arm motion and waist rotation, with the same timing:
  python -B scripts/data/repair_dance_flatfeet.py --name dance_smooth_flatfeet_bolder \
    --pitch-gain .75 --pitch-limit-deg 24 --arm-gain 1 --waist-gain 1.15

Output: data/use/dance_smooth_flatfeet.csv (30 FPS), matching motion NPZ (50 FPS),
and deploy_artifacts/use/dance_smooth_flatfeet.npz. Existing files are protected.
No deployment code, controller settings, or source data are changed.
"""
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco
from scipy.ndimage import gaussian_filter1d
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation
from h1_deploy.data.motion_convert import load_csv, convert_csv_to_npz
from h1_deploy.deploy.export import export_clip
from h1_deploy.deploy.models import load_scene
from h1_deploy.robots.h1 import load_h1_model


def repair(source, *, slowdown=1.25, pitch_gain=.55, arm_gain=.90,
           height=.99, com_x=.035, pitch_limit_deg=20., waist_gain=1.):
    if not np.isfinite([slowdown, pitch_gain, arm_gain, height, com_x,
                        pitch_limit_deg, waist_gain]).all():
        raise ValueError('repair parameters must be finite')
    if slowdown < 1 or not 0 < pitch_gain <= 1 or not 0 < arm_gain <= 1:
        raise ValueError('slowdown >= 1 and gains in (0, 1] required')
    if pitch_limit_deg <= 0 or waist_gain <= 0:
        raise ValueError('pitch_limit_deg and waist_gain must be positive')
    raw = load_csv(source)
    fps = 30.
    duration = (len(raw)-1) / fps
    sm = raw.copy()
    euler = np.unwrap(Rotation.from_quat(raw[:, 3:7]).as_euler('xyz'), axis=0)
    euler = gaussian_filter1d(euler, 3., axis=0, mode='nearest')
    sm[:, 3:7] = Rotation.from_euler('xyz', euler).as_quat()
    sm[:, :3] = gaussian_filter1d(raw[:, :3], 3., axis=0, mode='nearest')
    sm[:, 7:] = gaussian_filter1d(raw[:, 7:], 3., axis=0, mode='nearest')
    # Smooth clock at the beginning/end; output remains a headerless 30 FPS CSV.
    ramp = .6
    total = duration * slowdown + ramp
    t = np.arange(int(np.floor(total * fps)) + 1) / fps
    integral = t-ramp/2
    left = t < ramp
    integral[left] = .5*(t[left] - ramp/np.pi*np.sin(np.pi*t[left]/ramp))
    right = t > total-ramp
    remain = total-t[right]
    integral[right] = total-ramp-.5*(remain-ramp/np.pi*np.sin(np.pi*remain/ramp))
    source_t = np.clip(integral / slowdown, 0, duration)
    orig_t = np.arange(len(raw))/fps
    q = np.column_stack([np.interp(source_t, orig_t, sm[:, j]) for j in range(7, 26)])
    angles = np.column_stack([np.interp(source_t, orig_t, euler[:, j]) for j in range(3)])
    zs = np.interp(source_t, orig_t, sm[:, 2])
    pitch_limit = np.deg2rad(pitch_limit_deg)
    pitch = pitch_limit * np.tanh(pitch_gain * angles[:, 1] / pitch_limit)
    q[:, 11:] *= arm_gain
    q[:, 10] += .7*(angles[:, 2]-angles[0, 2])
    q[:, 10] *= waist_gain
    q[:, :10] = 0.
    heights = height + .01*np.tanh((zs-np.median(zs))/.03)
    out, ik_info = rebuild_planted_feet(pitch, heights, q, com_x=com_x)
    return out, dict(input_frames=len(raw), output_frames=len(out), input_fps=30, output_fps=30,
                     slowdown=slowdown, pitch_gain=pitch_gain, arm_gain=arm_gain,
                     pitch_limit_deg=pitch_limit_deg, waist_gain=waist_gain,
                     pelvis_height_center=height, com_target_x=com_x, **ik_info,
                     duration_s=(len(out)-1)/fps, fixed_stance=True)


def rebuild_planted_feet(pitch, heights, joint_pos, *, com_x=.035):
    """Recompute root x and leg IK for flat planted feet and a static COM x target.

    Inputs are a pitch/height trajectory and the 19 joint angles. Arms and waist
    are retained, while all ten leg joints are reconstructed. This geometry
    constraint is not a dynamic balance guarantee; validate with closed-loop sim.
    """
    pitch = np.asarray(pitch, dtype=float)
    heights = np.asarray(heights, dtype=float)
    q = np.array(joint_pos, dtype=float, copy=True)
    if pitch.ndim != 1 or heights.shape != pitch.shape or q.shape != (len(pitch), 19):
        raise ValueError('expected pitch/height [T] and joints [T, 19]')
    if not (np.isfinite(pitch).all() and np.isfinite(heights).all()
            and np.isfinite(q).all() and np.isfinite(com_x)):
        raise ValueError('planted-foot inputs must be finite')
    q[:, :10] = 0.
    xyz = np.zeros((len(pitch), 3))
    xyz[:, 2] = heights
    quat = Rotation.from_euler('y', pitch[:, None]).as_quat()
    m = load_scene(torque=True)
    d = mujoco.MjData(m)
    # Height of the flat sole below the ankle, from the actual collision mesh.
    d.qpos[:] = 0.; d.qpos[2] = 1.1; d.qpos[3] = 1.
    mujoco.mj_kinematics(m, d)
    foot = m.body('left_ankle_link').id
    geom = m.geom('left_ankle_link_collision').id
    mesh = m.geom_dataid[geom]
    a, n = m.mesh_vertadr[mesh], m.mesh_vertnum[mesh]
    verts = m.mesh_vert[a:a+n] @ d.geom_xmat[geom].reshape(3,3).T + d.geom_xpos[geom]
    ankle_z = d.xpos[foot, 2] - verts[:, 2].min()
    hx = float(m.body('left_hip_roll_link').pos[0])
    hz = float(m.body('left_hip_yaw_link').pos[2])
    l1 = abs(float(m.body('left_knee_link').pos[2]))
    l2 = abs(float(m.body('left_ankle_link').pos[2]))
    mass = m.body_mass
    max_com_error = 0.
    for i in range(len(pitch)):
        theta, z = pitch[i], xyz[i, 2]
        hip_dx = hx*np.cos(theta) + hz*np.sin(theta)
        hip_dz = -hx*np.sin(theta) + hz*np.cos(theta)

        def solve(x):
            dx = -(x+hip_dx)
            down = z+hip_dz-ankle_z
            ck = (dx*dx+down*down-l1*l1-l2*l2)/(2*l1*l2)
            if not -1 < ck < 1:
                raise ValueError('Requested pelvis/feet geometry is unreachable')
            knee = np.arccos(ck)
            thigh_world = np.arctan2(-dx, down)-np.arctan2(l2*np.sin(knee), l1+l2*np.cos(knee))
            hip = thigh_world-theta
            ankle = -thigh_world-knee
            q[i, [2,3,4,7,8,9]] = [hip,knee,ankle,hip,knee,ankle]
            d.qpos[:3] = [x,0,z];d.qpos[3:7] = quat[i,[3,0,1,2]];d.qpos[7:] = q[i]
            mujoco.mj_kinematics(m,d)
            return np.dot(mass,d.xipos[:,0])/mass.sum() - com_x

        # At larger backward pitches, a fixed search endpoint can lie beyond
        # leg reach even when the balanced solution is reachable. Restrict the
        # numerical search interval to feasible hip-to-ankle distances.
        down = z+hip_dz-ankle_z
        reach_sq = (l1+l2)**2-down**2
        if reach_sq <= 0:
            raise ValueError('Requested pelvis height exceeds planted leg reach')
        reach = np.sqrt(reach_sq)*(1.-1e-9)
        lo, hi = max(-.18, -reach-hip_dx), min(.18, reach-hip_dx)
        if lo >= hi:
            raise ValueError('No reachable planted-foot root position')
        x = brentq(solve,lo,hi,xtol=1e-10)
        max_com_error = max(max_com_error,abs(solve(x)))
        xyz[i,0] = x
    out = np.c_[xyz,quat,q]
    return out, dict(max_reference_com_x_error=max_com_error, ankle_height=ankle_z)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source', type=Path, default=Path('data/use/dance_smooth.csv'))
    ap.add_argument('--data-dir', type=Path, default=Path('data/use'))
    ap.add_argument('--deploy-dir', type=Path, default=Path('deploy_artifacts/use'))
    ap.add_argument('--name', default='dance_smooth_flatfeet')
    ap.add_argument('--slowdown', type=float, default=1.25, help='motion duration multiplier, plus 0.6 s smooth clock ramps')
    ap.add_argument('--pitch-gain', type=float, default=.55, help='pelvis pitch gain before the smooth bound')
    ap.add_argument('--pitch-limit-deg', type=float, default=20., help='smooth pelvis pitch bound in degrees')
    ap.add_argument('--arm-gain', type=float, default=.9, help='scale eight arm joint angles about zero')
    ap.add_argument('--waist-gain', type=float, default=1., help='scale waist motion after transferring pelvis yaw')
    ap.add_argument('--height', type=float, default=.99, help='central pelvis height in metres; small vertical motion is retained')
    ap.add_argument('--com-x', type=float, default=.035, help='whole-body COM x relative to planted ankle origins, metres')
    a = ap.parse_args()
    if Path(a.name).name != a.name or a.name in ('', '.', '..'):
        ap.error('--name must be a filename stem without directories')
    csv = a.data_dir / (a.name + '.csv')
    motion = a.data_dir / (a.name + '.npz')
    deploy = a.deploy_dir / (a.name + '.npz')
    paths = [csv, motion, deploy]
    if len({p.resolve() for p in paths}) != 3 or a.source.resolve() in {p.resolve() for p in paths}:
        ap.error('source, CSV, motion NPZ and deployment NPZ must be different paths')
    if any(p.exists() for p in paths):
        ap.error('output already exists; use a new --name or output directories')
    raw, info = repair(a.source, slowdown=a.slowdown, pitch_gain=a.pitch_gain,
                       arm_gain=a.arm_gain, height=a.height, com_x=a.com_x,
                       pitch_limit_deg=a.pitch_limit_deg, waist_gain=a.waist_gain)
    if not np.isfinite(raw).all():
        raise ValueError('repair produced nonfinite motion')
    a.data_dir.mkdir(parents=True, exist_ok=True)
    a.deploy_dir.mkdir(parents=True, exist_ok=True)
    np.savetxt(csv, raw, delimiter=',', fmt='%.10f')
    info['conversion'] = convert_csv_to_npz(csv, motion, input_fps=30, output_fps=50, model=load_h1_model())
    export_clip(motion, deploy)
    info['deployment'] = str(deploy)
    print(json.dumps(info, indent=2))


if __name__ == '__main__':
    main()
