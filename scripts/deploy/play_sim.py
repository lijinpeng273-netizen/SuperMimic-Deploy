"""Interactive or headless H1 policy playback with the shared deployment core."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))

import argparse
import time
import numpy as np
import mujoco
from h1_deploy.deploy.core import Clip, DeployConfig, DeployCore
from h1_deploy.deploy.policy import load_policy
from h1_deploy.deploy.models import load_scene
from sim2sim import Robot, PDSim


def main():
    root = Path(__file__).resolve().parents[2]
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--artifacts', type=Path, default=root / 'deploy_artifacts/ckpt_latest')
    ap.add_argument('--clip', required=True, type=Path)
    ap.add_argument('--scene', type=Path)
    ap.add_argument('--headless', action='store_true')
    ap.add_argument('--seconds', type=float, default=0, help='0: play one clip')
    ap.add_argument('--latency-ms', type=int, default=0)
    ap.add_argument('--log', type=Path, default=root / 'outputs/sim2sim/playback.npz')
    a = ap.parse_args()
    cfg = DeployConfig.load(a.artifacts / 'deploy_config.json')
    clip = Clip.load(a.clip)
    if clip.root_pos is None:
        raise ValueError('playback needs a complete H1 clip with root poses')
    model = load_scene(a.scene, torque=True)
    robot = Robot(model, cfg.joint_names)
    robot.set_state(clip.root_pos[0], clip.root_quat_wxyz[0], clip.joint_pos[0])
    robot.place_on_ground()
    sim = PDSim(robot, cfg, cfg.effort_limit)
    core = DeployCore(cfg, load_policy(a.artifacts), clip)
    viewer = None
    if not a.headless:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, robot.d)
    rows = []
    try:
        count = min(clip.num_frames - 1, int(a.seconds / cfg.control_dt)) if a.seconds > 0 else clip.num_frames - 1
        for _ in range(count):
            start = time.monotonic()
            result = core.step(*robot.state(), root_pos=robot.root()[0])
            if result.abort:
                print(f'abort at frame {result.frame}: {result.abort}')
                break
            rows.append((robot.d.qpos.copy(), result.action.copy(), result.q_target.copy()))
            sim.command(result.q_target, a.latency_ms)
            for _ in range(round(cfg.control_dt / model.opt.timestep)):
                sim.step_ms(record=False)
            if viewer:
                if not viewer.is_running():
                    break
                viewer.sync()
                time.sleep(max(0, cfg.control_dt - (time.monotonic() - start)))
    finally:
        if viewer:
            viewer.close()
        a.log.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(a.log, qpos=np.asarray([x[0] for x in rows]), action=np.asarray([x[1] for x in rows]),
                            q_target=np.asarray([x[2] for x in rows]))


if __name__ == '__main__':
    main()
