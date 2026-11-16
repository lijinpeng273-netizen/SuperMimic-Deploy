# 接收参考坐标系下的骨盆位姿，并为 H1 的 anchor_obs 计算躯干锚点反馈。
# 用 MuJoCo 正向运动学，将骨盆位置、姿态和关节角转换为躯干的世界位姿。
# 生成 9 维反馈：机器人躯干坐标系中的 3 维参考位移，以及 6 维相对旋转表示。
# 通过非阻塞 UDP 读取 JSON 位姿包，检查坐标系标签、数据形状、四元数和时间戳。
# 在约 20 ms 读取预算内处理积压包并等待新包；发送端负责参考坐标系对齐与时钟同步。
"""Dynamic torso-anchor feedback in the reference world frame (training rot6d order)."""
from __future__ import annotations

import json
import select
import socket
import time
from functools import lru_cache

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from h1_deploy.robots.h1 import H1_JOINT_NAMES, load_h1_model
from h1_deploy.deploy.odometry import ReferenceAlignment


@lru_cache(maxsize=1)
def _model():
    # 首次计算运动学时才加载模型，随后复用缓存；只加载运动学所需内容以避免编译无用网格。
    # Still initialized on the first FK call, without compiling unused meshes.
    return load_h1_model(kinematics_only=True)


def torso_pose(root_pos, root_quat, joints):
    model = _model()
    # 模型可共享，但每次使用独立的 MjData，避免当前姿态与参考姿态的计算状态互相覆盖。
    data = mujoco.MjData(model)
    # 浮动基座 qpos 布局：前 3 项为世界位置，接着 4 项为 wxyz 四元数，其余为模型顺序的关节角。
    data.qpos[:3] = root_pos
    data.qpos[3:7] = root_quat
    data.qpos[7:] = joints
    # 只计算正向运动学，不推进仿真时间；xpos/xmat 给出躯干在世界坐标系中的位置与旋转。
    mujoco.mj_kinematics(model, data)
    body = model.body('torso_link').id
    # 返回副本，使结果独立于 MuJoCo 数据缓冲区；R 将躯干局部向量转换为世界向量。
    return data.xpos[body].copy(), data.xmat[body].reshape(3, 3).copy()


def features(clip, frame, joints, root_pos, root_quat):
    # 实测骨盆位姿与动作参考必须处于同一世界坐标系，才能计算有意义的位置和朝向偏差。
    # 此处生成传统 anchor_obs 的躯干特征；BlinkMimic 的骨盆反馈由 blinkmimic.py 另行计算。
    if root_pos is None:
        raise ValueError('anchor_obs requires measured pelvis position in the reference world frame')
    if clip.root_pos is None or clip.root_quat_wxyz is None:
        raise ValueError(f'{clip.name}: anchor_obs requires root_pos and root_quat_wxyz in the clip')
    p = np.asarray(root_pos, dtype=float)
    if p.shape != (3,) or not np.isfinite(p).all():
        raise ValueError('invalid pelvis position')
    # 与策略目标的未来帧约定一致，使用 frame+1；动作末尾固定在最后一帧。
    i = min(frame + 1, clip.num_frames - 1)
    # 分别从当前实测姿态和下一参考帧姿态，求出两者的躯干世界位姿。
    robot_p, robot_R = torso_pose(p, root_quat, joints)
    ref_p, ref_R = torso_pose(clip.root_pos[i], clip.root_quat_wxyz[i], clip.joint_pos[i])
    # 位置误差 dp = R_robot.T @ (p_ref - p_robot)，相对旋转 dR = R_robot.T @ R_ref。
    # 前 3 项是机器人躯干坐标系下的参考位移，后 6 项取 dR 的前两列并按行展开。
    # rot6d 顺序为 [dR00, dR01, dR10, dR11, dR20, dR21]，必须与训练时一致。
    return np.r_[robot_R.T @ (ref_p - robot_p), (robot_R.T @ ref_R)[:, :2].reshape(6)]


class PoseReceiver:
    """Local UDP pose input with bounded recovery from an unread receive queue.

    JSON: {time_unix_s, frame_id:'reference', root_pos:[x,y,z], root_quat_wxyz:[w,x,y,z]}.
    The producer must align its world origin/heading with the current reference clip.
    Clock synchronization is required if the producer runs on another computer.
    Reads stay at the caller's original control stage. If only old packets are
    queued, draining them lets the next live packet arrive within one 50 Hz tick.
    """
    # 单次 read() 的处理/等待预算，对应一个 50 Hz 控制周期；不同于位姿允许的最大年龄。
    READ_BUDGET_S = 0.02

    def __init__(self, address, max_age=0.1):
        # address 是本地监听的 HOST:PORT；发送端应向该地址发送 UDP JSON 数据包。
        host, port = address.rsplit(':', 1)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, int(port)))
        # 无数据时 recv 立即抛 BlockingIOError，等待新包的时间由 read() 中的 select 控制。
        self.sock.setblocking(False)
        self.max_age = float(max_age)
        if self.max_age <= 0:
            raise ValueError('pose maximum age must be positive')
        # 缓存格式为 (发送端 Unix 时间戳, 骨盆世界位置, 归一化 wxyz 四元数)。
        self.latest = None

    def read(self):
        # 单调时钟控制读取耗时；下方使用 Unix 墙钟与发送端时间戳比较数据新鲜程度。
        deadline = time.monotonic() + self.READ_BUDGET_S
        while True:
            # 先在时间预算内尽量清空接收队列，避免旧包堆积挡住较新的位姿。
            # A packet-count limit can leave newer poses behind old queued ones.
            # Drain until empty, bounded by wall time even for a continuous flood.
            while time.monotonic() < deadline:
                try:
                    raw = self.sock.recv(4096)
                except BlockingIOError:
                    break
                try:
                    packet = json.loads(raw)
                    pos = np.asarray(packet['root_pos'], float)
                    quat = np.asarray(packet['root_quat_wxyz'], float)
                    timestamp = float(packet['time_unix_s'])
                    # 标签为 reference 只是协议约定，本接收器不会自动校准原点或航向。
                    # 要求位置为 3 维、四元数为 4 维且数据有限，四元数范数误差小于 0.01。
                    valid = (packet['frame_id'] == 'reference' and pos.shape == (3,) and quat.shape == (4,)
                             and np.isfinite(pos).all() and np.isfinite(quat).all() and np.isfinite(timestamp)
                             and abs(np.linalg.norm(quat) - 1) < 0.01)
                    # 按发送时间选最新有效包，乱序旧包不覆盖缓存；合格四元数再归一化后保存。
                    if valid and (self.latest is None or timestamp > self.latest[0]):
                        self.latest = (timestamp, pos, quat / np.linalg.norm(quat))
                except (ValueError, KeyError, TypeError):
                    # 单个包的 JSON、字段或数值转换错误不终止读取，继续尝试后续数据包。
                    continue
            if self.latest is not None:
                # age = 本机 Unix 时间 - 发送时间；允许最多 20 ms 的时钟超前，最大陈旧时间为 max_age。
                # 缓存仍在有效期内时可以复用，返回 (root_pos, root_quat_wxyz)。
                age = time.time() - self.latest[0]
                if -0.02 <= age <= self.max_age:
                    return self.latest[1:]
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            # 队列已清空但没有可用的新鲜位姿时，只在本次剩余预算内等待，避免无限阻塞控制循环。
            # A full UDP queue drops incoming datagrams. Once drained, wait only
            # for the remaining budget for a new packet; never reuse a stale pose.
            if not select.select([self.sock], [], [], remaining)[0]:
                break
        # 区分从未收到有效包与缓存过期/时钟不匹配，抛给 runner 执行其位姿失效处理。
        if self.latest is None:
            raise ValueError('no valid anchor pose received')
        age = time.time() - self.latest[0]
        raise ValueError(f'anchor pose stale or clock mismatch: age={age:.3f}s')

    def close(self):
        # 释放本地 UDP 套接字和占用的监听端口。
        self.sock.close()


class SimPoseReceiver(PoseReceiver):
    """Align MuJoCo world poses once per preparation, like unitree-odom.

    The existing UDP validation/age checks and read timing remain unchanged.
    Height stays in the simulator's ground frame; Start does not re-align.
    """

    def __init__(self, address, max_age=0.1):
        super().__init__(address, max_age)
        self.alignment = ReferenceAlignment()
        self.prepared = False
        self.alignment_event = None

    def reset_reference(self, clip):
        self.alignment.reset(clip.root_pos[0], clip.root_quat_wxyz[0])
        self.prepared = True
        self.alignment_event = None

    def read(self):
        if not self.prepared:
            raise ValueError('simulation reference has not been prepared')
        position, quat = super().read()
        first = self.alignment.rotation is None
        result = self.alignment.apply(position, Rotation.from_quat(quat[[1, 2, 3, 0]]))
        if first:
            self.alignment_event = (
                f'fixed XY/yaw alignment: yaw={np.degrees(self.alignment.yaw):+.3f} deg, '
                f'xy shift={self.alignment.translation[:2].round(4).tolist()} m; '
                f'sim ground height unchanged, actual z={result[0][2]:.4f} m; '
                'kept through Start and STANDBY_POST')
        return result
