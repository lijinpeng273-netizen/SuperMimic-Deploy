# H1 部署公共工具：统一控制脚本使用的电机映射、传感器状态和遥控器接口。
# 将 19 个策略关节与 GO 消息的 20 个电机槽位互相映射，并生成电机工作模式。
# 结合腰部角度与角速度，把躯干 IMU 姿态和角速度换算到骨盆，封装为 H1State。
# 解析手柄按键，提供组合键去抖、释放后重新触发以及互斥组合键检查。
# 提供 DDS 初始化、内置运动模式只读查询，以及基于 MuJoCo 模型的关节正方向提示。
"""H1 GO DDS helpers: policy/hardware mapping, torso IMU conversion and remote keys.
IMU转换，电机映射"""
from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# 根据文件位置定位仓库，允许脚本直接运行时导入 src 下的 h1_deploy 模块。
REPO = Path(__file__).resolve().parents[2]
if str(REPO / "src") not in sys.path:
    sys.path.insert(0, str(REPO / "src"))

NUM_JOINTS = 19
NUM_MOTORS_GO = 20
# 第 i 个元素是策略关节 i 对应的硬件电机槽位；索引 9 未参与这 19 个关节的控制。
# 接收时用 values[JOINT_TO_MOTOR] 重排，发送时把策略数组写回这些槽位。
JOINT_TO_MOTOR = np.array([7, 3, 4, 5, 10, 8, 0, 1, 2, 11, 6, 16, 17, 18, 19, 12, 13, 14, 15], dtype=int)
# 这里使用硬件电机编号；弱电机与其他受控电机使用不同的 GO mode 值。
WEAK_MOTORS = {10, 11, 12, 13, 14, 15, 16, 17, 18, 19}
DEFAULT_ARTIFACTS = REPO / "deploy_artifacts" / "ckpt_latest"
DEFAULT_MJCF = REPO / "h1_data" / "h1_training.xml"
# No hardware torso-lock mode is supported. These empty sets retain the runner log schema.
LOCKED_WAIST_IDX = ()
LOCK_WAIST_STANDBY_ALLOWED_CLIPS = ()


def motor_modes():
    # 生成 20 槽 mode：未使用槽为 0，弱电机为 1，其余受控电机为 0x0A。
    return bytes(0 if i not in JOINT_TO_MOTOR else (1 if i in WEAK_MOTORS else 0x0A) for i in range(NUM_MOTORS_GO))


def to_motor(values):
    # 将策略顺序的 19 维 q/kp/kd 等数组扩展为硬件顺序的 20 维数组，空槽保持零。
    result = np.zeros(NUM_MOTORS_GO, dtype=float)
    result[JOINT_TO_MOTOR] = values
    return result

# 把 torso IMU 转换为 pelvis IMU
def pelvis_imu(torso_quat, torso_gyro, waist_q, waist_dq):
    from scipy.spatial.transform import Rotation
    q = np.asarray(torso_quat, float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 0.05:
        raise ValueError("invalid torso IMU quaternion")
    # 腰部仅绕 z 轴转动：R_world_torso = R_world_pelvis @ Rz(waist_q)。
    # 因此骨盆姿态为 R_world_torso @ Rz.T，去掉腰部相对骨盆的转角。
    rz = Rotation.from_rotvec([0, 0, float(waist_q)]).as_matrix()
    # SDK 四元数顺序为 wxyz，SciPy 使用 xyzw；输入和输出均需重排。
    world_torso = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    pelvis_quat = Rotation.from_matrix(world_torso @ rz.T).as_quat()[[3, 0, 1, 2]]
    # 先把躯干局部角速度转到骨盆坐标系，再减去腰部自身绕 z 轴的相对角速度。
    pelvis_gyro = rz @ np.asarray(torso_gyro, float) - np.array([0, 0, waist_dq])
    return pelvis_quat, pelvis_gyro

# ---------------------------------------------------------------------------------------------- buttons
# Buttons are packed into wireless_remote bytes 2 and 3.
BUTTON_BITS = {
    "R1": 0, "L1": 1, "start": 2, "select": 3, "R2": 4, "L2": 5, "F1": 6, "F2": 7,
    "A": 8, "B": 9, "X": 10, "Y": 11, "up": 12, "right": 13, "down": 14, "left": 15,
}


def remote_keys(wireless_remote) -> int:
    # 第 2 字节为低 8 位，第 3 字节为高 8 位，合成 16 位按键状态。
    return int(wireless_remote[2]) | (int(wireless_remote[3]) << 8)


def pressed(keys: int) -> list[str]:
    # 按 BUTTON_BITS 中的位号，列出当前按下的所有按键名称。
    return [n for n, b in BUTTON_BITS.items() if keys >> b & 1]


def combo_mask(*names: str) -> int:
    # 对组合键各个位做按位或；(keys & mask) == mask 表示这些键全部按下。
    m = 0
    for n in names:
        m |= 1 << BUTTON_BITS[n]
    return m


def keys_from_names(names) -> tuple[int, int]:
    """(wireless_remote[2], wireless_remote[3]) for a set of button names (used by the sim to inject presses)."""
    # remote_keys 的反向操作，供仿真构造手柄按键数据。
    k = combo_mask(*names) if names else 0
    return k & 0xFF, (k >> 8) & 0xFF


class ComboEdge:
    """Edge-triggered combo detector: fires once when all buttons of the combo become pressed together.
    (Kept for reference; the runner uses ComboDebounce.)"""

    def __init__(self, combos: dict[str, int]):
        self.combos = combos
        self.prev = 0

    def update(self, keys: int) -> list[str]:
        # 只检测组合从“不完整”到“全部按下”的边沿；此旧实现不做连续采样去抖。
        fired = [name for name, m in self.combos.items() if (keys & m) == m and (self.prev & m) != m]
        self.prev = keys
        return fired


class ComboDebounce:
    """Debounced combo detector for the runner.

    * the key word must be identical for >= `stable_n` consecutive messages before it counts;
    * a combo fires once when it becomes (stably) held, and only if it was ARMED: armed means the combo was
      seen (stably) released for >= `release_s`. Nothing is armed at construction, so a combo that is held
      when the program starts never fires until it has been released and pressed again;
    * combos listed in `no_arm` (DAMP) fire whenever they are stably held (idempotent, safe direction),
      at most once per `release_s` while held.
    * `exclusive` combos require exactly their own buttons. If another button is held with any of
      their buttons, all their buttons must be released before rearming (including brief overlaps).
    """

    def __init__(self, combos: dict[str, int], stable_n: int = 3, release_s: float = 0.1, no_arm=("damp",), exclusive=()):
        # stable_n 是连续相同按键消息数；release_s 是重新触发前要求的稳定释放时间。
        # no_arm 默认包含阻尼键；exclusive 要求只能按下该组合自身的按键。
        self.combos, self.stable_n, self.release_s, self.no_arm = combos, stable_n, release_s, set(no_arm)
        self.exclusive = set(exclusive)
        self.blocked = {k: False for k in self.exclusive}
        self.cand, self.cand_n = None, 0
        # 普通组合初始不可触发：必须先观察到释放，防止启动时一直按住的键直接生效。
        self.armed = {k: False for k in combos}
        self.released_t = {k: None for k in combos}
        self.last_fire = {k: -1e9 for k in combos}

    def update(self, keys: int, t: float) -> list[str]:
        # 互斥组合只要与其他键重叠就锁住，连尚未通过去抖的瞬时重叠也要处理。
        # Disarm on raw samples too: releasing a modifier must not turn an
        # emergency/ambiguous chord into a fresh policy-takeover request.
        for name in self.exclusive:
            m = self.combos[name]
            if (keys & m) and (keys & ~m):
                self.blocked[name] = True
                self.armed[name] = False
                self.released_t[name] = None
        # 对整个按键状态字去抖，连续 stable_n 条消息相同后才进入触发判断。
        if keys == self.cand:
            self.cand_n += 1
        else:
            self.cand, self.cand_n = keys, 1
        if self.cand_n < self.stable_n:
            return []
        fired = []
        for name, m in self.combos.items():
            # 被重叠按键锁住的组合，必须所有组成键都释放满 release_s 才能重新启用。
            if self.blocked.get(name, False):
                if keys & m:
                    self.released_t[name] = None
                else:
                    if self.released_t[name] is None:
                        self.released_t[name] = t
                    if t - self.released_t[name] >= self.release_s:
                        self.blocked[name] = False
                        self.armed[name] = True
                continue
            if (keys & m) == m:
                self.released_t[name] = None
                if name in self.no_arm:
                    # 阻尼键无需事先释放；稳定按住时允许每隔 release_s 再次触发。
                    if t - self.last_fire[name] >= self.release_s:
                        fired.append(name)
                        self.last_fire[name] = t
                elif self.armed[name]:
                    # 普通组合触发一次后撤销资格，持续按住不会重复触发。
                    fired.append(name)
                    self.armed[name] = False
            else:
                # 普通组合只要不再全部按下，就开始释放计时；互斥组合被锁住时走上面的严格分支。
                if self.released_t[name] is None:
                    self.released_t[name] = t
                if t - self.released_t[name] >= self.release_s:
                    self.armed[name] = True
        return fired


# ---------------------------------------------------------------------------------------------- state
# runner 共用的状态结构：关节数组统一采用策略顺序，姿态和角速度统一采用骨盆约定。
@dataclass
class H1State:
    q: np.ndarray
    dq: np.ndarray
    tau: np.ndarray
    temp: np.ndarray  # (19, 2); GO single temperature duplicated for legacy log shape
    gyro: np.ndarray  # pelvis frame
    quat: np.ndarray  # pelvis orientation, wxyz
    acc: np.ndarray   # raw torso accelerometer; not a pelvis acceleration estimate
    rpy: np.ndarray   # raw torso rpy, diagnostic only
    tick: int
    reserved_mode: int
    reserved_control: int
    keys: int
    motorstate: np.ndarray  # GO lost counter, not an HG motor fault bitmask


def state_from_arrays(q, dq, tau, temp, gyro, quat, acc, rpy, tick, keys, lost):
    # 将 DDS 或桥接器提供的硬件顺序数组统一转换为 19 维策略关节顺序。
    ids = JOINT_TO_MOTOR
    q, dq = np.asarray(q, float)[ids], np.asarray(dq, float)[ids]
    # 重排后第 10 个索引对应腰部，用它补偿躯干 IMU 中的腰部相对运动。
    quat, gyro = pelvis_imu(quat, gyro, q[10], dq[10])
    temp = np.asarray(temp, float)[ids]
    if temp.ndim == 1:
        # GO 的单温度值复制成两列以兼容日志结构，不代表实际读取了两个温度传感器。
        temp = np.repeat(temp[:, None], 2, axis=1)
    # acc/rpy 保留原始躯干数据；两个 reserved 字段填零，lost 保留为逐电机丢包计数。
    return H1State(q, dq, np.asarray(tau, float)[ids], temp, gyro, quat,
                   np.asarray(acc, float), np.asarray(rpy, float), int(tick), 0, 0,
                   int(keys), np.asarray(lost, np.int64)[ids])


# 从 GO LowState 提取关节、IMU、tick 和遥控按键；接收时刻与 CRC 检查由 runner 负责。
def extract_state(msg) -> H1State:
    # 固定使用 20 槽 H1 GO 消息布局，避免把其他协议的状态误当作 H1 电机数据。
    if len(msg.motor_state) != NUM_MOTORS_GO:
        raise ValueError("expected the 20-slot H1 GO LowState layout")
    ms, imu = msg.motor_state, msg.imu_state
    return state_from_arrays(
        [m.q for m in ms], [m.dq for m in ms], [m.tau_est for m in ms],
        [m.temperature for m in ms], imu.gyroscope, imu.quaternion, imu.accelerometer, imu.rpy,
        msg.tick, remote_keys(msg.wireless_remote), [m.lost for m in ms],
    )


def check_motion_mode(timeout: float = 2.0):
    """Read-only MotionSwitcher query; never selects or releases a controller."""
    # 查询内置运动服务是否有活动模式，返回状态码、模式名和原始结果，由 runner 决定是否拒绝启动。
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

    c = MotionSwitcherClient()
    c.SetTimeout(timeout)
    c.Init()
    code, res = c.CheckMode()
    name = res.get("name") if isinstance(res, dict) else None
    return code, name, res


def init_dds(domain: int, iface: str | None):
    # 指定 DDS 域和可选网卡；未指定 iface 时使用 SDK 的默认接口选择。
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    ChannelFactoryInitialize(domain, iface) if iface else ChannelFactoryInitialize(domain)


# ---------------------------------------------------------------------------------------------- sign hints
def sign_hints(cfg, mjcf: Path = DEFAULT_MJCF) -> dict[str, str]:
    """Human-readable 'what does +q do' per joint, from the training MJCF at the q0 pose (pelvis frame:
    +x forward, +y robot-left, +z up). Uses mujoco if available; otherwise returns {}."""
    # 只在模型中计算“关节正转时哪里往哪边动”的提示；不驱动机器人，缺少 MuJoCo 时返回空结果。
    try:
        import mujoco
    except Exception:  # noqa: BLE001
        return {}
    try:
        m = mujoco.MjModel.from_xml_path(str(mjcf))
    except Exception as e:  # noqa: BLE001
        print(f"[sign] cannot load {mjcf}: {e}")
        return {}
    d = mujoco.MjData(m)
    # 将浮动基座置于原点且姿态为单位四元数，使模型世界轴与这里的骨盆坐标轴对齐。
    d.qpos[:] = 0.0
    d.qpos[3] = 1.0  # identity pelvis orientation
    names = {}
    for j in range(m.njnt):
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or ""
        names[nm.split("/")[-1]] = j
    # 设置配置中的默认关节姿态，再做前向计算，得到该姿态下的关节轴、转轴位置和几何位置。
    for i, jn in enumerate(cfg.joint_names):
        d.qpos[m.jnt_qposadr[names[jn]]] = cfg.default_joint_pos[i]
    mujoco.mj_forward(m, d)
    words = {0: ("forward", "backward"), 1: ("to robot-left", "to robot-right"), 2: ("up", "down")}
    out = {}
    for jn in cfg.joint_names:
        j = names[jn]
        axis, anchor = d.xaxis[j].copy(), d.xanchor[j].copy()
        b = m.jnt_bodyid[j]
        # 在该关节带动的子树中，选距转轴最远的 geom 中心作为观察点，便于描述运动方向。
        # farthest geom (of the moving subtree) from the joint axis = the point that visibly moves
        geoms = [g for g in range(m.ngeom) if _is_desc(m, m.geom_bodyid[g], b)]
        lever = lambda g: np.linalg.norm(np.cross(axis, d.geom_xpos[g] - anchor))  # noqa: E731
        tip = max(geoms, key=lever)
        tip_name = (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[tip]) or "").split("/")[-1]
        tip_name = f"farthest point of {tip_name}"
        # 正向单位角速度下，观察点线速度 v = axis × (point - anchor)，方向遵循右手定则。
        v = np.cross(axis, d.geom_xpos[tip] - anchor)
        # 取最显著的至多两个坐标分量，用前后、左右、上下描述正转时的瞬时运动。
        order = np.argsort(-np.abs(v))
        parts = [words[k][0] if v[k] > 0 else words[k][1] for k in order[:2] if abs(v[k]) > 0.25 * np.abs(v).max()]
        ax = " ".join(f"{a:+.2f}" for a in axis)
        sgn = lambda k: 1 if axis[k] > 0 else -1  # noqa: E731
        rot = ""
        # 转轴足够接近某个主轴时，再补充 yaw/pitch/roll 的直观转动方向。
        if abs(axis[2]) > 0.8:  # yaw
            rot = " (yaw: CCW seen from above, link turns to robot-left)" if sgn(2) > 0 else " (yaw: CW seen from above)"
        elif abs(axis[1]) > 0.8:  # pitch about the left axis: front goes down
            rot = " (pitch: link's front edge goes DOWN, back edge up)" if sgn(1) > 0 else " (pitch: front edge goes UP)"
        elif abs(axis[0]) > 0.8:  # roll about the forward axis: left side goes up
            rot = " (roll: link's LEFT side goes UP, right side down)" if sgn(0) > 0 else " (roll: left side goes DOWN)"
        out[jn] = f"axis(pelvis)=[{ax}]{rot}; +q moves {tip_name} {' & '.join(parts)}"
    return out


def _is_desc(m, k: int, b: int) -> bool:
    # 从 body k 沿父节点向上查找 b，用于筛选会随该关节一起运动的几何体。
    while k > 0:
        if k == b:
            return True
        k = m.body_parentid[k]
    return False


def now() -> float:
    # 单调时钟用于时间间隔和超时判断，不受系统日期或校时变化影响。
    return time.monotonic()
