"""Inference-only PyTorch checkpoint and ONNX backends; no trainer/mjlab dependency.
PT策略加载 """
from __future__ import annotations

import json
from pathlib import Path

import numpy as np


class TorchPolicy:
    def __init__(self, path, threads=1):
        import torch
        from h1_deploy.data.motion_format import validate_checkpoint
        from h1_deploy.policy.nets import Policy, PolicyCfg
        torch.set_num_threads(threads)
        checkpoint = torch.load(path, map_location='cpu', weights_only=False)
        if 'robot_identity' not in checkpoint:
            raise ValueError('checkpoint must include H1 robot_identity')
        self.path = Path(path)
        if checkpoint.get('checkpoint_format') == 'blinkmimic':
            from h1_deploy.policy.blinkmimic import BlinkPolicy, validate_blink_checkpoint
            validate_blink_checkpoint(checkpoint)
            self.model = BlinkPolicy(checkpoint).eval()
            self.config = checkpoint['args']
            self.input_shapes = (720, 530)
            self.deploy_metadata = dict(
                policy_format='blinkmimic', anchor_obs=False, anchor_body='pelvis', reference_position=True,
                history=10, future_frames=10, obs_frame_dim=72, obs_dim=720, goal_dim=530, action_dim=19,
                history_layout='frame_major', normalization_eps=0.01,
                pi_ref_skip=self.model.ref_skip, goal_q_offset=15, policy_state_key='pi',
                robot_identity=checkpoint['robot_identity'])
            return
        validate_checkpoint(checkpoint)
        config = checkpoint['config']
        if config['env']['future_frames'] != 10 or config['policy'].get('history', 10) != 10:
            raise ValueError('runtime requires history=10 and future_frames=10')
        self.config = config
        self.model = Policy(PolicyCfg(**config['policy'])).eval()
        self.model.load_state_dict(checkpoint['policy'], strict=True)
        expected_goal = 489 if config['env'].get('anchor_obs', False) else 480
        if config['policy']['goal_dim'] != expected_goal:
            raise ValueError('checkpoint goal_dim disagrees with anchor_obs')
        self.input_shapes = (630, expected_goal)
        self.deploy_metadata = dict(policy_format='supermimic', anchor_obs=config['env'].get('anchor_obs', False),
                                    history=10, future_frames=10, obs_frame_dim=63, obs_dim=630,
                                    goal_dim=expected_goal, action_dim=19, history_layout='term_major')

    def __call__(self, obs, goal):
        import torch
        with torch.inference_mode():
            out = self.model(torch.as_tensor(np.asarray(obs, np.float32))[None],
                             torch.as_tensor(np.asarray(goal, np.float32))[None])
        return out[0].numpy().astype(float)

# 检查点加载函数，支持 ONNX 和 PyTorch 格式
def load_policy(path, threads=1):
    from h1_deploy.deploy.core import OnnxPolicy
    path = Path(path)
    if path.is_dir():
        path = path / 'policy.onnx'
    if path.exists():
        if path.suffix == '.onnx':
            return OnnxPolicy(path, threads)
        if path.suffix == '.pt':
            return TorchPolicy(path, threads)
        raise ValueError(f'unsupported policy format: {path}')
    if path.name == 'policy.onnx':
        config_path = path.parent / 'deploy_config.json'
        if config_path.exists():
            config = json.loads(config_path.read_text())
            checkpoint = config.get('policy_checkpoint')
            if checkpoint:
                return TorchPolicy((config_path.parent / checkpoint).resolve(), threads)
    raise FileNotFoundError(f'policy missing: {path}; export ONNX or set policy_checkpoint in deploy_config.json')
