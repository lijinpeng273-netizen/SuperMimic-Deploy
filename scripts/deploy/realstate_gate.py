"""Restart H1 simulation from recorded policy states and evaluate perturbations.

Logs use policy joint order and pelvis gyro. Anchor-enabled replay requires logged
root_pos and anchor_quat; missing global state is not replaced by reference feedback.
"""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))
import argparse
import itertools
import json
import numpy as np
import sim2sim as S
from h1_deploy.deploy.core import DeployCore


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--logs', nargs='+', required=True, type=Path)
    ap.add_argument('--artifacts', type=Path, default=S.ART)
    ap.add_argument('--ckpt', default='ckpt_latest')
    ap.add_argument('--clip', required=True)
    ap.add_argument('--frames', type=int, nargs='+', required=True)
    ap.add_argument('--lat', type=int, nargs='+', default=[0, 10, 20])
    ap.add_argument('--kps', type=float, nargs='+', default=[0.85, 1.0, 1.15])
    ap.add_argument('--mass', type=float, nargs='+', default=[0., 1.8])
    ap.add_argument('--fric', type=float, nargs='+', default=[0.6, 1.0])
    ap.add_argument('--steps', type=int, default=250)
    ap.add_argument('--out', type=Path, default=Path('outputs/sim2sim/realstate.json'))
    a = ap.parse_args()
    if a.steps < 1:
        ap.error('--steps must be positive')
    S.ART = a.artifacts.resolve()
    S.UNITREE_XML = S.ART / 'models/h1_torque_scene.xml'
    cfg, clip, raw = S.load_cfg(a.ckpt), S.load_clip(a.clip), S.load_clip_raw(a.clip)
    policy = S.load_policy(a.ckpt)
    results = []
    for path in a.logs:
        with np.load(path, allow_pickle=False) as z:
            log = {k:z[k] for k in z.files}
        for frame in a.frames:
            ids = np.where((log['frame'] == frame) & (log['state'] == 3))[0]
            if not len(ids):
                raise ValueError(f'{path}: no POLICY row at frame {frame}')
            row = int(ids[-1])
            ref_frame = max(0, frame - 1)
            if cfg.needs_root_pose:
                position, quat = log['root_pos'][row], log['anchor_quat'][row]
                if not np.isfinite(position).all() or not np.isfinite(quat).all():
                    raise ValueError(f'{path}: missing measured global pose for anchor replay')
            else:
                position, quat = raw['root_pos'][ref_frame], log['quat'][row]
            for latency, kp_scale, mass, friction in itertools.product(a.lat, a.kps, a.mass, a.fric):
                model = S.unitree_model(mass, friction)
                robot = S.Robot(model, cfg.joint_names)
                robot.set_state(position, quat, log['q'][row], raw['root_lin_vel_w'][ref_frame],
                                S.quat_to_mat(quat) @ log['gyro'][row], log['dq'][row])
                if not cfg.needs_root_pose:
                    robot.place_on_ground()
                sim = S.PDSim(robot, cfg, cfg.effort_limit)
                core = DeployCore(cfg, policy, clip)
                core.reset(ref_frame)
                if row > 0 and 'action' in log:
                    core.last_action = log['action'][row-1].copy()
                if row > 0 and 'obs_hist' in log and log['obs_hist'].shape[0] == len(log['frame']):
                    core.hist = log['obs_hist'][row-1].reshape(10, cfg.obs_frame_dim).copy()
                abort, fell = None, False
                for k in range(a.steps):
                    result = core.step(*robot.state(), root_pos=robot.root()[0])
                    if result.abort:
                        abort = result.abort
                        break
                    sim.command(result.q_target, latency, cfg.kp * kp_scale, cfg.kd)
                    for _ in range(20): sim.step_ms(record=False)
                    if S.fell(robot):
                        fell = True
                        break
                    if result.done: break
                results.append(dict(log=str(path), frame=frame, latency_ms=latency, kp_scale=kp_scale,
                                    mass_add=mass, friction=friction, steps=k+1, abort=abort, fell=fell,
                                    root_velocity_source='reference; not measured'))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(results, indent=2))
    print(a.out)


if __name__ == '__main__':
    main()
