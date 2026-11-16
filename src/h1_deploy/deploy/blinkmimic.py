# 为 H1 BlinkMimic 策略构造骨盆参考特征，供 DeployCore 生成观测与目标输入。
# 从动作文件读取骨盆世界位姿和关节角，用差分预计算参考关节速度及骨盆线速度、角速度。
# relative_pose 返回当前参考骨盆相对实测骨盆的 9 维特征，顺序为旋转 6 维、位置 3 维。
# goal 将未来 10 帧参考信息拼成 530 维向量，位置和朝向以当前参考帧的水平航向为基准。
# 特征顺序、坐标系、rot6d 展平方式和速度缩放须与训练一致；本文件不执行策略推理或发送控制命令。
"""NumPy reference features for the H1 BlinkMimic pelvis policy.

Matches motion.MotionLib.goal and mdp.MDP.obs with reference_position=True.
The legacy NPZ goal[48] table is deliberately not used for this policy.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def quat_matrix(quat):
    # 将 wxyz 四元数转换为旋转矩阵，支持单个 (4,) 或批量 (..., 4) 输入。
    q = np.asarray(quat, dtype=np.float32)
    # 沿最后一维归一化，分母下限避免除零；这一步不替代调用方对无效姿态的检查。
    q = q / np.maximum(np.linalg.norm(q, axis=-1, keepdims=True), 1e-12)
    w, x, y, z = np.moveaxis(q, -1, 0)
    # 按矩阵行顺序组装 R；对于骨盆世界姿态，R 将骨盆局部向量转到世界系，R.T 做反向转换。
    return np.stack((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w),
                     2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w),
                     2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)), axis=-1).reshape(q.shape[:-1] + (3, 3))


def _gradient(x, dt):
    # 对等间隔时间序列求导，输入至少两帧；dt 为相邻参考帧的时间间隔，单位秒。
    g = np.empty_like(x)
    # 内部用中心差分 dx/dt ≈ (x[k+1] - x[k-1]) / (2*dt)，两端用单边差分。
    g[1:-1] = (x[2:] - x[:-2]) / (2 * dt)
    g[0], g[-1] = (x[1] - x[0]) / dt, (x[-1] - x[-2]) / dt
    return g


class BlinkReference:
    def __init__(self, clip, dt=0.02):
        # 骨盆世界位姿是必需数据；这里直接从轨迹构造特征，不读取旧版 clip.goal 的 48 维目标表。
        if clip.root_pos is None or clip.root_quat_wxyz is None:
            raise ValueError(f'{clip.name}: BlinkMimic needs root_pos and root_quat_wxyz')
        self.clip = clip
        self.p = np.asarray(clip.root_pos, np.float32)
        self.R = quat_matrix(clip.root_quat_wxyz)
        self.q = np.asarray(clip.joint_pos, np.float32)
        # q 为各帧 19 维关节角，qd 为差分得到的参考关节速度，单位 rad/s。
        self.qd = _gradient(self.q, dt)
        rt = self.R.transpose(0, 2, 1)
        w = np.empty_like(self.p)
        # 世界系角速度估计：w[k] ≈ rotvec(R[k+1] @ R[k-1].T) / (2*dt)。
        # as_rotvec() 给出轴角旋转向量，方向为旋转轴、模长为弧度；首末帧改用相邻一帧差分。
        # Central SO(3) differences inside a clip, one-sided differences at its ends.
        if len(self.p) > 2:
            w[1:-1] = Rotation.from_matrix(self.R[2:] @ rt[:-2]).as_rotvec() / (2 * dt)
        w[0] = Rotation.from_matrix(self.R[1] @ rt[0]).as_rotvec() / dt
        w[-1] = Rotation.from_matrix(self.R[-1] @ rt[-2]).as_rotvec() / dt
        # 分别用每帧的 R.T 将世界系线速度、角速度转到该参考帧自身的骨盆坐标系。
        self.v_local = (rt @ _gradient(self.p, dt)[..., None])[..., 0]
        self.w_local = (rt @ w[..., None])[..., 0]

    def relative_pose(self, frame, root_pos, quat):
        # 观测反馈使用当前参考帧 f；实测 root_pos/quat 必须已对齐动作参考的世界坐标系。
        p = np.asarray(root_pos, dtype=np.float32)
        if p.shape != (3,) or not np.isfinite(p).all():
            raise ValueError('BlinkMimic requires measured pelvis position in the reference world frame')
        f = min(frame, len(self.q) - 1)
        # R_robot.T 把世界向量转换到当前实测骨盆坐标系，用于表达参考姿态与位置偏差。
        rt = quat_matrix(quat).T
        # 相对旋转 dR = R_robot.T @ R_ref[f]；取前两列，按行展平为 [dR00,dR01,dR10,dR11,dR20,dR21]。
        # First two COLUMNS, flattened row-major; reference is frame f, not f+1.
        ori = (rt @ self.R[f])[:, :2].reshape(6)
        # 相对位置 dp = R_robot.T @ (p_ref[f] - p_robot)，单位米。
        pos = rt @ (self.p[f] - p)
        # 顺序为 [旋转6, 位置3]；DeployCore 将其附加到基础63维观测，得到每帧72维、10帧历史720维。
        return np.concatenate((ori, pos))

    def goal(self, frame):
        # 目标输入仅由参考轨迹生成：取 f+1 到 f+10，超出动作末尾的索引重复最后一帧。
        f = min(frame, len(self.q) - 1)
        future = np.minimum(f + np.arange(1, 11), len(self.q) - 1)
        # 当前参考骨盆的前向轴为 R[f] 第一列；投影到世界水平面后得到航向的 cos/sin。
        x, y = self.R[f, 0, 0], self.R[f, 1, 0]
        norm = np.sqrt(x*x + y*y + np.float32(1e-9))
        c, s = x / norm, y / norm
        # ht ≈ Rz(-heading_f)，只去除参考航向，保留未来姿态的俯仰和侧倾；不使用实测机器人朝向。
        ht = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]], dtype=np.float32)
        # 未来位置减去当前参考位置，再转到该航向坐标系；未来朝向也用同一个 ht 表达。
        dp = (ht @ (self.p[future] - self.p[f])[..., None])[..., 0]
        ori = (ht @ self.R[future])[..., :2].reshape(10, 6)
        # 每帧 53 维：[位置3, 旋转6, 局部线速度3, 局部角速度3, 关节角19, 关节速度19]。
        # v_local/w_local 仍属于各未来参考帧自身的骨盆系；关节角使用绝对值，关节速度乘训练约定的 0.1。
        # 按帧展平 10×53=530；第一未来帧的关节角位于 goal[15:34]，供策略参考跳连使用。
        return np.concatenate((dp, ori, self.v_local[future], self.w_local[future],
                               self.q[future], self.qd[future] * 0.1), axis=-1).reshape(530)
