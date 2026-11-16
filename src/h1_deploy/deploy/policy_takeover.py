"""Blend a fixed HOLD target into the live policy without changing policy history."""
from __future__ import annotations

import numpy as np


class PolicyTakeover:
    def __init__(self, hold_target, duration_s):
        self.hold_target = np.asarray(hold_target, dtype=float).copy()
        self.duration_s = float(duration_s)
        if not np.isfinite(self.duration_s) or self.duration_s < 0:
            raise ValueError('policy blend duration must be finite and nonnegative')
        if not np.isfinite(self.hold_target).all():
            raise ValueError('policy blend requires a finite HOLD target')
        self.alpha = 0.0
        self.completed_at = None

    def target(self, policy_target, elapsed_s, q, dq, cfg, safety):
        """Blend only targets; reapply the original torque/position bounds afterwards.

        The network's last_action and observation history keep their existing
        meaning. In particular, the blended target is not fed back as an action.
        """
        self.alpha = 1.0 if self.duration_s == 0 else float(np.clip(elapsed_s / self.duration_s, 0, 1))
        if self.alpha >= 1:
            return np.asarray(policy_target).copy(), 0
        blended = (1 - self.alpha) * self.hold_target + self.alpha * policy_target
        target = blended.copy()
        # A convex blend of targets is not necessarily torque-safe at the new
        # measured q/dq: the old HOLD target may be outside today's bounds.
        if safety.clamp_torque:
            tau = safety.torque_clamp_factor * cfg.effort_limit
            target = np.clip(target, q + (-tau + cfg.kd * dq) / cfg.kp,
                             q + (tau + cfg.kd * dq) / cfg.kp)
        if safety.clamp_soft_limits:
            target = np.clip(target, cfg.soft_lower, cfg.soft_upper)
        return target, int(np.count_nonzero(np.abs(target - blended) > 1e-9))

    def submitted(self, now):
        """Return True once, after a full-policy target was successfully submitted."""
        if self.alpha >= 1 and self.completed_at is None:
            self.completed_at = float(now)
            return True
        return False

    def standby_elapsed(self, now):
        return None if self.completed_at is None else max(0.0, float(now) - self.completed_at)
