"""Prepare H1 deploy artifacts without importing the training environment."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from h1_deploy.robots.h1 import (H1_JOINT_NAMES, H1_ACTION_SCALE, default_pos, stiffness,
                               damping, effort_limit, velocity_limit, load_h1_model)
from h1_deploy.data.motion_format import validate_motion, checkpoint_identity

REPO = Path(__file__).resolve().parents[3]


def make_config(anchor_obs=False):
    model = load_h1_model()
    return dict(robot='h1', joint_names=list(H1_JOINT_NAMES), default_joint_pos=default_pos,
                action_scale=[H1_ACTION_SCALE[n] for n in H1_JOINT_NAMES], kp=stiffness, kd=damping,
                effort_limit=effort_limit, velocity_limit=velocity_limit,
                joint_lower=model.jnt_range[1:, 0].tolist(), joint_upper=model.jnt_range[1:, 1].tolist(),
                soft_limit_factor=0.9, control_dt=0.02, sim_dt=0.005, decimation=4,
                anchor_obs=bool(anchor_obs), robot_identity=checkpoint_identity())


def export_clip(source, destination):
    with np.load(source, allow_pickle=False) as z:
        names = validate_motion(z, source)
        root = names.index('pelvis')
        quat = z['body_quat_w'][:, root].astype(float)
        rotation = Rotation.from_quat(quat[:, [1, 2, 3, 0]])
        grav = rotation.inv().apply(np.tile([0., 0., -1.], (len(quat), 1)))
        omega = rotation.inv().apply(z['body_ang_vel_w'][:, root])
        w, x, y, zz = quat.T
        yaw = np.arctan2(2 * (w * zz + x * y), 1 - 2 * (y*y + zz*zz))
        heading = Rotation.from_euler('z', yaw[:, None])
        linear = heading.inv().apply(z['body_lin_vel_w'][:, root])
        goal = np.concatenate([z['joint_pos'] - np.array(default_pos), z['joint_vel'], grav, omega, linear,
                               z['body_pos_w'][:, root, 2:3]], axis=1).astype(np.float32)
        extra = {k: z[k] for k in z.files if k.startswith('segment_')}
        out = dict(name=np.array(Path(source).stem), fps=np.float32(50), joint_names=np.array(H1_JOINT_NAMES),
                   default_joint_pos=np.array(default_pos), goal=goal, joint_pos=z['joint_pos'],
                   joint_vel=z['joint_vel'], proj_grav=grav, root_pos=z['body_pos_w'][:, root],
                   root_quat_wxyz=quat, root_lin_vel_w=z['body_lin_vel_w'][:, root],
                   root_ang_vel_w=z['body_ang_vel_w'][:, root], **extra)
    Path(destination).parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(destination, **out)


def main():
    import os
    import shutil
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--cmd', choices=('config', 'clips', 'onnx', 'models', 'all'), default='all')
    ap.add_argument('--checkpoint', type=Path, default=REPO / 'checkpoints/ckpt_latest.pt')
    ap.add_argument('--npz-dir', type=Path, default=REPO / 'data/h1')
    ap.add_argument('--clips', nargs='+')
    ap.add_argument('--output', type=Path, default=REPO / 'deploy_artifacts')
    a = ap.parse_args()
    run = lambda key: a.cmd in (key, 'all')
    bundle = a.output / a.checkpoint.stem
    if run('config') or run('onnx'):
        from h1_deploy.deploy.policy import TorchPolicy
        policy = TorchPolicy(a.checkpoint)
        bundle.mkdir(parents=True, exist_ok=True)
    if run('config'):
        config = make_config(policy.deploy_metadata['anchor_obs'])
        config.update(policy.deploy_metadata)
        # A self-contained package: copy a checkpoint only when outside this repository.
        target = REPO / 'checkpoints' / a.checkpoint.name
        target.parent.mkdir(exist_ok=True)
        if a.checkpoint.resolve() != target.resolve():
            shutil.copy2(a.checkpoint, target)
        config['policy_checkpoint'] = os.path.relpath(target, bundle)
        (bundle / 'deploy_config.json').write_text(json.dumps(config, indent=2))
    if run('clips'):
        files = [a.npz_dir / f'{name}.npz' for name in a.clips] if a.clips else sorted(a.npz_dir.glob('*.npz'))
        if not files:
            raise FileNotFoundError(f'No H1 motion NPZ found in {a.npz_dir}; the checkpoint contains no reference trajectories')
        for source in files:
            export_clip(source, a.output / 'clips' / source.name)
    if run('models'):
        from h1_deploy.deploy.models import scene_xml
        import xml.etree.ElementTree as ET
        out = a.output / 'models'
        out.mkdir(parents=True, exist_ok=True)
        shutil.copytree(REPO / 'h1_data/meshes', out / 'meshes', dirs_exist_ok=True)
        for torque, name in ((False, 'h1_train_scene.xml'), (True, 'h1_torque_scene.xml')):
            root = ET.fromstring(scene_xml(torque))
            root.find('compiler').set('meshdir', 'meshes/h1')
            ET.ElementTree(root).write(out / name, encoding='unicode')
    if run('onnx'):
        import torch
        import onnx
        obs, goal = policy.input_shapes
        program = torch.onnx.export(policy.model, (torch.zeros(1, obs), torch.zeros(1, goal)),
                                    input_names=['obs', 'goal'], output_names=['action'],
                                    opset_version=18, dynamo=True, external_data=False)
        program.save(str(bundle / 'policy.onnx'), external_data=False)
        onnx.checker.check_model(onnx.load(bundle / 'policy.onnx'))
    print(f'Artifacts: {a.output}')


if __name__ == '__main__':
    main()
