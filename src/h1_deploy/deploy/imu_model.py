"""Optional generic saturation model for a body-frame IMU.

Clips gyro components and integrates an estimated quaternion. The default 500 deg/s
limit is a configurable simulation assumption, not a measured H1 sensor specification.
Hardware adapters must first transform the H1 torso IMU into the required pelvis frame."""

from __future__ import annotations

import os

import numpy as np

ENV_VAR = "SM_IMU_MODEL"  # "1" = on; unset / "" / "0" = off
GYRO_LIMIT = float(np.radians(500.0))  # Default simulation limit per axis; calibrate for the sensor.


def imu_model_from_env() -> bool:
    return os.environ.get(ENV_VAR, "").strip() not in ("", "0")


def quat_mul(a, b) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_exp(w, dt: float) -> np.ndarray:
    """Unit quaternion of the rotation w * dt (axis-angle vector)."""
    w = np.asarray(w, float)
    n = float(np.linalg.norm(w))
    a = n * dt
    if a < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return np.r_[np.cos(a / 2), np.sin(a / 2) * w / n]


def gravity_b(q_wxyz) -> np.ndarray:
    """World gravity direction (0, 0, -1) in the body frame (= h1_deploy.deploy.core.projected_gravity)."""
    w, x, y, z = q_wxyz
    return np.array([-2 * (x * z - w * y), -2 * (y * z + w * x), -(1 - 2 * (x * x + y * y))])


def angle_deg(a, b) -> float:
    a, b = np.asarray(a, float), np.asarray(b, float)
    return float(np.degrees(np.arctan2(np.linalg.norm(np.cross(a, b)), a @ b)))  # atan2: exact near 0 (arccos is not)


class ImuModel:
    """Saturating gyro + gyro-only attitude integrator, with per-run metrics.

    integrate(w_true, dt): every physics step (before mj_step, with the body-frame angular velocity of that step);
    observe(gyro_true, quat_true, frame): at every policy step -> (gyro, quat) the policy / DeployCore get.
    """

    def __init__(self, quat_wxyz, limit: float = GYRO_LIMIT):
        self.limit = float(limit)
        self.q = np.asarray(quat_wxyz, float).copy()
        self.q /= np.linalg.norm(self.q)
        self.lost_z = 0.0  # sum max(0, |wz| - limit) dt over the physics steps [rad]
        self.lost = 0.0  # same with |w - clip(w)| (all axes, vector norm) [rad]
        self.sat_frames: list[int] = []  # policy steps (reference frame) at which the reported gyro z is on the rail
        self.err_max, self.err_frame = 0.0, -1  # max angle between estimated and true gravity [deg] and its frame

    def clip(self, w) -> np.ndarray:
        return np.clip(np.asarray(w, float), -self.limit, self.limit)

    def integrate(self, w_true, dt: float) -> None:
        w = np.asarray(w_true, float)
        wc = self.clip(w)
        self.lost_z += max(0.0, abs(float(w[2])) - self.limit) * dt
        self.lost += float(np.linalg.norm(w - wc)) * dt
        self.q = quat_mul(self.q, quat_exp(wc, dt))
        self.q /= np.linalg.norm(self.q)

    def observe(self, gyro_true, quat_true, frame: int | None = None):
        g = np.asarray(gyro_true, float)
        if abs(float(g[2])) >= self.limit:
            self.sat_frames.append(-1 if frame is None else int(frame))
        e = angle_deg(gravity_b(self.q), gravity_b(quat_true))
        if e > self.err_max:
            self.err_max, self.err_frame = e, -1 if frame is None else int(frame)
        return self.clip(g), self.q.copy()

    def summary(self) -> dict:
        return {"imu_model": True, "imu_gyro_limit": self.limit, "imu_sat_z_frames": len(self.sat_frames),
                "imu_sat_z_frame_list": list(self.sat_frames), "imu_grav_err_max_deg": float(self.err_max),
                "imu_grav_err_max_frame": int(self.err_frame), "imu_rot_lost_z_rad": float(self.lost_z),
                "imu_rot_lost_rad": float(self.lost)}
