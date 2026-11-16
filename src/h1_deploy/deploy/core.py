"""Hardware-agnostic H1 policy runtime utilities.

Shared by the standalone H1 simulator and hardware runners. Anchor feedback
requires reference root poses and measured pelvis position; no zero padding is used.

Uses numpy, optional ONNX/PyTorch inference, and CPU MuJoCo FK for anchor feedback.
Everything that must match training lives here:

BlinkMimic uses obs[720] (10 frame-major 72D frames) and goal[530]; its
reference features live in deploy.blinkmimic and normalization in the policy.
The following legacy layout remains unchanged for SuperMimic:

* 50 Hz; joint order = MJCF joint order = 19-DoF Unitree H1 tree order (H1_JOINT_NAMES).
* obs frame (63) = [q - q0 (19), dq (19), pelvis gyro (3), pelvis projected gravity (3), last raw action (19)];
  history 10, flattened term-major, oldest -> newest; the first frame after start back-fills all 10 slots.
* goal (480, or 489 with anchor_obs) = goal table rows k+1 .. k+10 (clamped at the clip's last frame) when the robot is at frame k.
* q_target = a * scale + q0, dq_des = 0, tau_ff = 0, gains = training gains.
配置与动作读取、观测历史、目标构造、动作缩放和保护判定
"""
# 仿真和实机通用的H1策略运行时工具。
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from h1_deploy.deploy.blinkmimic import BlinkReference

HISTORY = 10
FUTURE = 10
NUM_JOINTS = 19
# (name, dim) of the policy observation terms, in training order.
OBS_TERMS = (
    ("joint_pos", NUM_JOINTS),
    ("joint_vel", NUM_JOINTS),
    ("base_ang_vel", 3),
    ("projected_gravity", 3),
    ("actions", NUM_JOINTS),
)


def quat_rotate_inverse(q_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    """R(q)^T v for a unit quaternion (w, x, y, z)."""
    w, xyz = q_wxyz[0], q_wxyz[1:]
    t = 2.0 * np.cross(xyz, v)
    return v - w * t + np.cross(xyz, t)


def projected_gravity(q_wxyz: np.ndarray) -> np.ndarray:
    return quat_rotate_inverse(q_wxyz, np.array([0.0, 0.0, -1.0]))


@dataclass
class DeployConfig:
    """Robot constants dumped from the training model by h1_deploy.deploy.export (deploy_config.json)."""

    joint_names: list[str]
    default_joint_pos: np.ndarray
    action_scale: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    effort_limit: np.ndarray
    joint_lower: np.ndarray  # hard MJCF range
    joint_upper: np.ndarray
    velocity_limit: np.ndarray
    soft_limit_factor: float = 0.9
    control_dt: float = 0.02
    anchor_obs: bool = False
    policy_format: str = 'supermimic'
    anchor_body: str = 'torso_link'
    reference_position: bool = False

    def __post_init__(self):
        if self.policy_format not in ('supermimic', 'blinkmimic'):
            raise ValueError(f'unsupported policy_format: {self.policy_format}')
        if self.policy_format == 'blinkmimic' and (
                self.anchor_obs or self.anchor_body != 'pelvis' or not self.reference_position):
            raise ValueError('BlinkMimic requires pelvis reference_position; legacy anchor_obs must be false')

    @property
    def obs_frame_dim(self):
        return 72 if self.policy_format == 'blinkmimic' else 63

    @property
    def input_shapes(self):
        return (720, 530) if self.policy_format == 'blinkmimic' else (630, 489 if self.anchor_obs else 480)

    @property
    def needs_root_pose(self):
        return self.anchor_obs or self.policy_format == 'blinkmimic'

    @classmethod
    def load(cls, path: str | Path) -> "DeployConfig":
        d = json.loads(Path(path).read_text())
        arr = lambda k: np.asarray(d[k], dtype=np.float64)  # noqa: E731
        from h1_deploy.robots.h1 import H1_JOINT_NAMES
        if d["joint_names"] != list(H1_JOINT_NAMES) or float(d.get("control_dt", 0.02)) != 0.02:
            raise ValueError("expected H1 model joint order and 50 Hz control")
        for key in ("default_joint_pos", "action_scale", "kp", "kd", "effort_limit", "joint_lower", "joint_upper", "velocity_limit"):
            if arr(key).shape != (19,) or not np.isfinite(arr(key)).all():
                raise ValueError(f"invalid H1 configuration field {key}")
        config = cls(
            joint_names=list(d["joint_names"]),
            default_joint_pos=arr("default_joint_pos"),
            action_scale=arr("action_scale"),
            kp=arr("kp"),
            kd=arr("kd"),
            effort_limit=arr("effort_limit"),
            joint_lower=arr("joint_lower"),
            joint_upper=arr("joint_upper"),
            velocity_limit=arr("velocity_limit"),
            soft_limit_factor=float(d.get("soft_limit_factor", 0.9)),
            control_dt=float(d.get("control_dt", 0.02)),
            anchor_obs=bool(d.get("anchor_obs", False)),
            policy_format=d.get('policy_format', 'supermimic'),
            anchor_body=d.get('anchor_body', 'torso_link'),
            reference_position=bool(d.get('reference_position', False)),
        )
        expected = dict(history=HISTORY, future_frames=FUTURE, obs_frame_dim=config.obs_frame_dim,
                        obs_dim=config.input_shapes[0], goal_dim=config.input_shapes[1], action_dim=NUM_JOINTS,
                        history_layout='frame_major' if config.policy_format == 'blinkmimic' else 'term_major')
        if config.policy_format == 'blinkmimic':
            expected.update(normalization_eps=0.01, goal_q_offset=15, policy_state_key='pi')
        for key, value in expected.items():
            if key in d and d[key] != value:
                raise ValueError(f'deploy_config.{key} must be {value!r} for this policy layout')
        return config

    @property
    def soft_lower(self) -> np.ndarray:
        mid, half = 0.5 * (self.joint_lower + self.joint_upper), 0.5 * (self.joint_upper - self.joint_lower)
        return mid - self.soft_limit_factor * half

    @property
    def soft_upper(self) -> np.ndarray:
        mid, half = 0.5 * (self.joint_lower + self.joint_upper), 0.5 * (self.joint_upper - self.joint_lower)
        return mid + self.soft_limit_factor * half


@dataclass
class Clip:
    """Precomputed per-clip reference (h1_deploy.deploy.export goal tables)."""

    name: str
    goal: np.ndarray  # (T, 48) goal features, fp32
    joint_pos: np.ndarray  # (T, 19) reference joint positions (start / end poses)
    proj_grav: np.ndarray  # (T, 3) reference pelvis projected gravity (abort check)

    root_pos: np.ndarray | None = None
    root_quat_wxyz: np.ndarray | None = None

    @classmethod
    def load(cls, path: str | Path) -> "Clip":
        from h1_deploy.robots.h1 import H1_JOINT_NAMES
        with np.load(path, allow_pickle=False) as z:
            if float(np.asarray(z["fps"]).reshape(-1)[0]) != 50:
                raise ValueError(f"{path}: expected 50 Hz")
            if list(z["joint_names"].astype(str)) != list(H1_JOINT_NAMES):
                raise ValueError(f"{path}: joint order mismatch")
            clip = cls(
                name=str(z["name"]), goal=z["goal"].astype(np.float32),
                joint_pos=z["joint_pos"].astype(float), proj_grav=z["proj_grav"].astype(float),
                root_pos=z["root_pos"].astype(float) if "root_pos" in z else None,
                root_quat_wxyz=z["root_quat_wxyz"].astype(float) if "root_quat_wxyz" in z else None,
            )
        n = clip.num_frames
        for name, width in (("goal", 48), ("joint_pos", 19), ("proj_grav", 3)):
            value = getattr(clip, name)
            if n < 2 or value.shape != (n, width) or not np.isfinite(value).all():
                raise ValueError(f"{path}: invalid {name}")
        for name, width in (("root_pos", 3), ("root_quat_wxyz", 4)):
            value = getattr(clip, name)
            if value is not None and (value.shape != (n, width) or not np.isfinite(value).all()):
                raise ValueError(f"{path}: invalid {name}")
        if clip.root_quat_wxyz is not None and not np.allclose(np.linalg.norm(clip.root_quat_wxyz, axis=1), 1, atol=2e-3):
            raise ValueError(f"{path}: root quaternion must be unit wxyz")
        return clip

    @property
    def num_frames(self) -> int:
        return self.goal.shape[0]


@dataclass
class SafetyCfg:
    # Training does not clamp joint targets to soft limits. Runtime clamping is optional.
    clamp_soft_limits: bool = False
    clamp_torque: bool = True
    # MuJoCo saturates force at every 5 ms substep, while this clamps the target once per 20 ms step from the
    # sampled state (stricter as the joint moves), so allow headroom; lower it (e.g. 0.5) for first hardware runs.
    torque_clamp_factor: float = 1.5
    max_tilt_err: float = 0.5  # rad between robot and reference pelvis gravity directions
    max_vel_frac: float = 0.8  # of the joint velocity limit
    max_abs_action: float = 50.0  # raw network output; |a| this large means something is badly wrong


@dataclass
class StepResult:
    q_target: np.ndarray  # (19,) clamped joint targets to send
    action: np.ndarray  # (19,) raw network output (fed back as last action)
    frame: int  # reference frame the robot should be at after this step
    done: bool  # clip finished (reached its last frame)
    abort: str | None  # reason to drop to damping, or None
    n_clamped: int  # how many joints were clamped this step


class OnnxPolicy:
    def __init__(self, path: str | Path, threads: int = 1):
        import onnxruntime as ort

        so = ort.SessionOptions()
        so.intra_op_num_threads = threads
        so.inter_op_num_threads = 1
        # Spinning worker threads steal CPU from the 500 Hz command thread; block instead.
        so.add_session_config_entry("session.intra_op.allow_spinning", "0")
        self.sess = ort.InferenceSession(str(path), sess_options=so, providers=["CPUExecutionProvider"])
        inputs = self.sess.get_inputs()
        outputs = self.sess.get_outputs()
        self.input_shapes = tuple(x.shape[1] for x in inputs)
        if ([x.name for x in inputs] != ["obs", "goal"] or self.input_shapes not in ((630, 480), (630, 489), (720, 530))
                or any(len(x.shape) != 2 or x.shape[0] != 1 for x in inputs)
                or len(outputs) != 1 or outputs[0].name != "action" or outputs[0].shape != [1, 19]):
            raise ValueError("ONNX requires (obs, goal) dimensions (630,480|489) or (720,530), batch=1, action[1,19]")

    def __call__(self, obs: np.ndarray, goal: np.ndarray) -> np.ndarray:
        out = self.sess.run(["action"], {"obs": obs[None].astype(np.float32), "goal": goal[None].astype(np.float32)})
        return out[0][0].astype(np.float64)


@dataclass
class DeployCore:
    cfg: DeployConfig
    policy: object  # callable (flat obs, goal) -> action[19]; dimensions determined by cfg
    clip: Clip
    safety: SafetyCfg = field(default_factory=SafetyCfg)

    def __post_init__(self):
        if hasattr(self.policy, 'input_shapes') and tuple(self.policy.input_shapes) != self.cfg.input_shapes:
            raise ValueError('policy input dimensions disagree with deploy_config')
        self._blink_reference = None
        self.reset()

    def blink_reference(self):
        if self._blink_reference is None or self._blink_reference.clip is not self.clip:
            self._blink_reference = BlinkReference(self.clip, self.cfg.control_dt)
        return self._blink_reference

    @property
    def num_joints(self) -> int:
        return len(self.cfg.joint_names)

    @property
    def obs_terms(self) -> tuple[tuple[str, int], ...]:
        nj = self.num_joints
        return (
            ("joint_pos", nj),
            ("joint_vel", nj),
            ("base_ang_vel", 3),
            ("projected_gravity", 3),
            ("actions", nj),
        )

    # ------------------------------------------------------------------ state

    def reset(self, start_frame: int = 0) -> None:
        """Call right before the first policy step; the robot is assumed to be at `start_frame`."""
        self.frame = int(start_frame)
        self.last_action = np.zeros(self.num_joints)
        self.hist: np.ndarray | None = None  # (HISTORY, cfg.obs_frame_dim), oldest first

    def obs_frame(self, q: np.ndarray, dq: np.ndarray, gyro: np.ndarray, quat_wxyz: np.ndarray) -> np.ndarray:
        return np.concatenate([q - self.cfg.default_joint_pos, dq, gyro, projected_gravity(quat_wxyz), self.last_action])

    def observation(self, q, dq, gyro, quat_wxyz, *, root_pos=None) -> np.ndarray:
        """Push one frame; SuperMimic flattens by term, BlinkMimic by frame."""
        f = self.obs_frame(np.asarray(q, float), np.asarray(dq, float), np.asarray(gyro, float), np.asarray(quat_wxyz, float))
        if self.cfg.policy_format == 'blinkmimic':
            relative = self.blink_reference().relative_pose(self.frame, root_pos, quat_wxyz)
            f = np.concatenate((f, relative))
        if self.hist is None:
            self.hist = np.repeat(f[None], HISTORY, axis=0)
        else:
            self.hist = np.concatenate([self.hist[1:], f[None]], axis=0)
        if self.cfg.policy_format == 'blinkmimic':
            return self.hist.reshape(-1)
        parts, o = [], 0
        for _, d in self.obs_terms:
            parts.append(self.hist[:, o : o + d].reshape(-1))  # term-major: all 10 frames of one term
            o += d
        return np.concatenate(parts)
    # 未来帧选择，目标拼接、anchor 拼接
    def goal(self, anchor: np.ndarray | None = None) -> np.ndarray:
        if self.cfg.policy_format == 'blinkmimic':
            return self.blink_reference().goal(self.frame)
        last = self.clip.num_frames - 1
        idx = np.minimum(self.frame + 1 + np.arange(FUTURE), last)
        g = self.clip.goal[idx].reshape(-1)
        if self.cfg.anchor_obs:
            if anchor is None or np.asarray(anchor).shape != (9,) or not np.isfinite(anchor).all():
                raise ValueError("anchor_obs requires a finite 9D relative torso pose")
            g = np.concatenate([g, np.asarray(anchor, dtype=g.dtype)])
        return g

    # ------------------------------------------------------------------ step

    def switch_clip(self, clip: "Clip", frame: int = 0) -> None:
        """Continue with another reference without resetting the observation history or the last action
        (standby -> motion -> standby hand-offs stay seamless for the policy)."""
        self.clip = clip
        self.frame = int(np.clip(frame, 0, clip.num_frames - 1))

    def step(self, q, dq, gyro, quat_wxyz, advance: bool = True, *, root_pos=None) -> StepResult:
        """advance=False keeps the reference frame fixed (standby on a static reference); done stays False."""
        # 一个策略周期：检查状态 -> 更新观测历史 -> 构造目标并推理 -> 校验动作 -> 裁剪目标角 -> 推进参考帧。
        # 本函数只返回结果；runner 根据 abort 决定进入阻尼，或将 q_target 配合 PD 增益发送给电机。
        # q/dq 按策略关节顺序排列，gyro 为骨盆局部角速度，quat_wxyz 为骨盆世界姿态。
        q, dq, gyro = np.asarray(q, float), np.asarray(dq, float), np.asarray(gyro, float)
        quat_wxyz = np.asarray(quat_wxyz, float)
        # 检查状态有限性、四元数范数、相对参考的倾角和关节速度，返回首个失败原因或 None。
        # abort 非空也不会在此提前返回，后续仍尝试计算；调用方必须检查 abort 后再决定是否发送目标。
        abort = self._check(q, dq, gyro, quat_wxyz)
        n = np.linalg.norm(quat_wxyz)
        # 为观测归一化四元数；范数不可用时以单位四元数兜底，但保留上面记录的 abort。
        # observation() 追加本周期观测并整理历史输入，其中 action 项仍是上一周期的 last_action。
        obs = self.observation(q, dq, gyro, quat_wxyz / n if np.isfinite(n) and n > 0.5 else np.array([1.0, 0.0, 0.0, 0.0]), root_pos=root_pos)
        anchor = None
        if self.cfg.anchor_obs:
            # 传统 anchor_obs：计算下一参考帧相对当前躯干的 9 维反馈，随后由 goal() 拼到目标输入。
            # BlinkMimic 的骨盆相对位姿在 observation() 内计算，不走这个躯干锚点分支。
            from h1_deploy.deploy.anchor import features
            anchor = features(self.clip, self.frame, q, root_pos, quat_wxyz)
        # 用当前参考帧构造未来目标，与历史观测一起送入策略；a 尚未乘 action_scale，也不是电机力矩。
        a = np.asarray(self.policy(obs, self.goal(anchor)), float)
        # 输出必须为每关节一个动作的一维向量；接口维度错误直接抛异常交给 runner 处理。
        if a.shape != (self.num_joints,):
            raise ValueError(f"policy output must have {self.num_joints} actions, got {a.shape}")
        if not np.all(np.isfinite(a)):
            # abort or ... 保留更早的失败原因；将整组非有限动作替换为零，避免其进入后续目标换算。
            # 零动作对应默认姿态偏移为零，并不代表零力矩；本周期仍需按 abort 执行阻尼。
            abort = abort or "non-finite action"
            a = np.zeros(self.num_joints)
        elif np.max(np.abs(a)) > self.safety.max_abs_action:
            # 任一动作绝对值超限就记录中止原因，不把动作截到阈值后继续控制。
            abort = abort or f"action magnitude {np.max(np.abs(a)):.1f}"
        # 原始目标角 q_raw = default_joint_pos + action_scale * a，再按启用的力矩/软限位约束裁剪。
        # n_clamped 统计最终目标相对 q_raw 发生变化的关节数，用于日志分析。
        q_t, n_clamped = self.targets(a, q, dq)
        # 下一周期观测反馈策略动作，而不是裁剪后的目标角；非有限输出时反馈上面替换的零动作。
        self.last_action = a
        # advance=False 只冻结参考帧：观测历史、推理和 last_action 仍持续更新，以支持策略待机。
        if advance:
            self.frame += 1
        # 播放模式下推进到最后一帧即标记完成；待机时 done 始终为 False。
        done = advance and self.frame >= self.clip.num_frames - 1
        # 返回的 frame 是推进后的帧号；本次观测、目标构造和推理使用的是推进前的帧号。
        return StepResult(q_target=q_t, action=a, frame=self.frame, done=done, abort=abort, n_clamped=n_clamped)

    # 网络输出转换为目标角度，并按启用的力矩/软限位约束裁剪；返回裁剪后的目标角和裁剪关节数。
    def targets(self, a: np.ndarray, q: np.ndarray, dq: np.ndarray) -> tuple[np.ndarray, int]:
        c = self.cfg
        raw = a * c.action_scale + c.default_joint_pos  # 关节计算目标角 = 默认关节角 + 动作缩放 * 策略输出
        q_t = raw.copy()
        if self.safety.clamp_torque:
            # PD with dq_des = 0: tau = kp (q_t - q) - kd dq, kept within +-effort (what the sim actuator clamps).
            tau = self.safety.torque_clamp_factor * c.effort_limit  # 目标角裁剪的力矩阈值 = 配置力矩上限 * 裁剪系数
            lo = q + (-tau + c.kd * dq) / c.kp  # 目标角裁剪下界 = 当前关节角 + (-力矩阈值 + 阻尼 * 当前关节速度) / 比例增益
            hi = q + (tau + c.kd * dq) / c.kp   # 目标角裁剪上界 = 当前关节角 + (力矩阈值 + 阻尼 * 当前关节速度) / 比例增益
            q_t = np.clip(q_t, lo, hi)
        if self.safety.clamp_soft_limits:
            q_t = np.clip(q_t, c.soft_lower, c.soft_upper)
        return q_t, int(np.count_nonzero(np.abs(q_t - raw) > 1e-9))

    def _check(self, q: np.ndarray, dq: np.ndarray, gyro: np.ndarray, quat_wxyz: np.ndarray) -> str | None:
        s = self.safety
        # An uninitialised IMU (all-zero quaternion) would otherwise read as perfectly upright.
        if not (np.all(np.isfinite(quat_wxyz)) and np.all(np.isfinite(gyro))):
            return "non-finite IMU"
        if abs(np.linalg.norm(quat_wxyz) - 1.0) > 0.05:
            return f"bad IMU quaternion norm {np.linalg.norm(quat_wxyz):.3f}"
        g_robot = projected_gravity(quat_wxyz / np.linalg.norm(quat_wxyz))
        g_ref = self.clip.proj_grav[min(self.frame, self.clip.num_frames - 1)]
        cosang = float(np.clip(np.dot(g_robot, g_ref) / (np.linalg.norm(g_robot) * np.linalg.norm(g_ref) + 1e-9), -1.0, 1.0))
        tilt = float(np.arccos(cosang))
        if tilt > s.max_tilt_err:
            return f"tilt error {np.degrees(tilt):.0f} deg"
        over = np.abs(dq) > s.max_vel_frac * self.cfg.velocity_limit
        if np.any(over):
            j = int(np.argmax(np.abs(dq) / self.cfg.velocity_limit))
            return f"joint velocity {self.cfg.joint_names[j]} {dq[j]:.1f} rad/s"
        if not (np.all(np.isfinite(q)) and np.all(np.isfinite(dq))):
            return "non-finite state"
        return None


# Goal feature layout per frame (48): q-q0 [0:19] | dq [19:38] | proj grav [38:41] | ang vel body [41:44] |
# lin vel heading [44:47] | height [47].
_GOAL_RATE_COLS = np.r_[NUM_JOINTS : 2 * NUM_JOINTS, 2 * NUM_JOINTS + 3 : 2 * NUM_JOINTS + 9]


def static_clip(clip: Clip, frame: int) -> Clip:
    """A standing-still reference at one frame of `clip`: same pose, gravity and height, all rates zero.
    Used for policy standby before a motion starts (frame 0) and after it ends (last frame)."""
    frame = int(np.clip(frame, 0, clip.num_frames - 1))
    g = clip.goal[frame].copy()
    nj = clip.joint_pos.shape[-1]
    rate_cols = np.r_[nj : 2 * nj, 2 * nj + 3 : 2 * nj + 9]
    g[rate_cols] = 0.0
    return Clip(
        name=f"{clip.name}@{frame}_static",
        goal=np.repeat(g[None], 2, 0),
        joint_pos=np.repeat(clip.joint_pos[frame][None], 2, 0),
        proj_grav=np.repeat(clip.proj_grav[frame][None], 2, 0),
        root_pos=None if clip.root_pos is None else np.repeat(clip.root_pos[frame][None], 2, 0),
        root_quat_wxyz=None if clip.root_quat_wxyz is None else np.repeat(clip.root_quat_wxyz[frame][None], 2, 0),
    )


def interpolate_pose(q_from: np.ndarray, q_to: np.ndarray, t: float, duration: float) -> np.ndarray:
    """Smooth (cosine) interpolation used to bring the robot to a clip's first-frame pose."""
    s = float(np.clip(t / max(duration, 1e-6), 0.0, 1.0))
    s = 0.5 - 0.5 * np.cos(np.pi * s)
    return q_from + s * (q_to - q_from)
