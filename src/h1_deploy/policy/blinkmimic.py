"""Inference-only BlinkMimic MLP with the checkpoint's frozen running statistics.

The input contract follows blinkmimic.networks.Policy; no training package is needed.
"""
from __future__ import annotations

import numpy as np
import torch
from torch import nn


class FrozenNorm(nn.Module):
    def __init__(self, dim):
        super().__init__()
        self.register_buffer("mean", torch.zeros(dim))
        self.register_buffer("var", torch.ones(dim))
        self.register_buffer("count", torch.zeros(()))

    def forward(self, x):
        return (x - self.mean) / torch.sqrt(self.var + 1e-4)


def _mlp(widths):
    layers = []
    for i, (n_in, n_out) in enumerate(zip(widths[:-1], widths[1:])):
        layers.append(nn.Linear(n_in, n_out))
        if i < len(widths) - 2:
            layers.append(nn.ELU())
    return nn.Sequential(*layers)


class BlinkPolicy(nn.Module):
    """Flat frame-major obs[720], goal[530] -> action[19], including reference skip."""
    def __init__(self, checkpoint):
        super().__init__()
        args, identity, weights = checkpoint['args'], checkpoint['robot_identity'], checkpoint['pi']
        self.ref_skip = bool(args.get('pi_ref_skip', False))
        widths = lambda key: [int(x) for x in args[key].split(',')]
        self.enc = _mlp([530, *widths('enc_hidden'), 32])
        self.dec = _mlp([752, *widths('dec_hidden'), 19])
        self.obs_norm = FrozenNorm(72)
        self.goal_norm = FrozenNorm(530)
        if self.ref_skip:
            self.register_buffer('ref_q0', torch.tensor(identity['default_q'], dtype=torch.float32))
            self.register_buffer('ref_alpha', torch.tensor(identity['action_scale'], dtype=torch.float32))
            for key, expected in (('ref_q0', identity['default_q']), ('ref_alpha', identity['action_scale'])):
                if not np.allclose(weights[key].numpy(), expected, rtol=0, atol=1e-6):
                    raise ValueError(f'BlinkMimic {key} disagrees with robot_identity')
        self.load_state_dict(weights, strict=True)
        for key, value in weights.items():
            if not torch.isfinite(value).all():
                raise ValueError(f'non-finite BlinkMimic parameter: {key}')
        if torch.any(self.obs_norm.var < 0) or torch.any(self.goal_norm.var < 0):
            raise ValueError('BlinkMimic normalization variance must be nonnegative')

    def forward(self, obs, goal):
        o = self.obs_norm(obs.reshape(-1, 10, 72)).flatten(-2)
        z = torch.tanh(self.enc(self.goal_norm(goal)))
        a = self.dec(torch.cat((z, o), dim=-1))
        if self.ref_skip:
            a = a + (goal[..., 15:34] - self.ref_q0) / self.ref_alpha
        return a


def validate_blink_checkpoint(checkpoint):
    """Accept the H1/pelvis input contract supported here, not arbitrary equal-size policies."""
    from h1_deploy.data.motion_format import asset_sha256
    from h1_deploy.robots.h1 import H1_JOINT_NAMES, H1_ACTION_SCALE, default_pos, stiffness, damping, effort_limit

    identity, args = checkpoint['robot_identity'], checkpoint['args']
    expected = dict(schema=1, robot='h1', asset_sha256=asset_sha256(), joint_names=list(H1_JOINT_NAMES),
                    root_body='pelvis', anchor_body='pelvis', reference_position=True,
                    control_dt=0.02, physics_dt=0.005)
    if checkpoint.get('checkpoint_version') != 1 or checkpoint.get('robot') != 'h1':
        raise ValueError('unsupported BlinkMimic checkpoint version or robot')
    for key, value in expected.items():
        if identity.get(key) != value:
            raise ValueError(f'unsupported BlinkMimic robot_identity.{key}: expected {value!r}')
    for key, value in (('default_q', default_pos), ('action_scale', [H1_ACTION_SCALE[n] for n in H1_JOINT_NAMES])):
        if np.shape(identity.get(key)) != (19,) or not np.allclose(identity[key], value, rtol=0, atol=1e-7):
            raise ValueError(f'BlinkMimic {key} does not match this H1 control configuration')
    controls = identity['controls']
    if len(controls) != 19:
        raise ValueError('BlinkMimic must define 19 motor controls')
    for i, control in enumerate(controls):
        required = dict(stiffness=stiffness[i], damping=damping[i], effort_limit=effort_limit[i],
                        armature=0.1, viscous_damping=1.0, delay_min_lag=0, delay_max_lag=0)
        if any(control.get(k) != v for k, v in required.items()):
            raise ValueError(f'BlinkMimic controls mismatch for {H1_JOINT_NAMES[i]}')
    if args.get('hist') != 10 or args.get('goal_frames') != 10:
        raise ValueError('BlinkMimic runtime requires hist=10 and goal_frames=10')
    for key, off in dict(pi_arch='mlp', pi_fourier=None, pi_goal_feet=False).items():
        if args.get(key, off) != off:
            raise ValueError(f'unsupported BlinkMimic network option {key}')
    for key in ('imu', 'joint_vel_filter', 'delay_hold_prob'):
        if identity['sensing'].get(key) is not None:
            raise ValueError(f'BlinkMimic sensing.{key} needs a separate deployment adapter')
