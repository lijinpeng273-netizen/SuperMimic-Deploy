"""Read-only pose diagnostics. Candidate offsets are never deployment calibration.

The world-z-offset hypothesis is fitted once, using the beginning of a stationary
recording. Foot geometry assumes a flat floor and explicitly specified contacts.
It cannot identify the physical origin of an undocumented odometry frame.
"""
from __future__ import annotations

from collections import Counter

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation


class SourceSequence:
    """Reject duplicate/regressed publications; do not compare robot and PC clocks."""

    def __init__(self):
        self.publisher = self.stamp = self.tick = None
        self.counts = Counter()
        self.problems: set[str] = set()

    def accept(self, stamp, publisher, tick=None):
        if stamp <= 0 or publisher <= 0:
            self.counts['invalid_metadata'] += 1
            self.problems.add('invalid DDS source timestamp or publisher handle')
            return False
        if self.publisher is not None and publisher != self.publisher:
            self.counts['publisher_changed'] += 1
            self.problems.add('DDS publisher changed')
            return False
        if self.stamp is not None and stamp <= self.stamp:
            reason = 'duplicate' if stamp == self.stamp else 'timestamp_regressed'
            self.counts[reason] += 1
            if stamp < self.stamp:
                self.problems.add('DDS source timestamp regressed')
            return False
        if tick is not None and self.tick is not None:
            delta = (tick - self.tick) % (2 ** 32)
            if delta == 0 or delta >= 2 ** 31:
                self.counts['tick_not_advancing'] += 1
                if delta >= 2 ** 31:
                    self.problems.add('lowstate tick regressed')
                return False
        self.publisher, self.stamp, self.tick = publisher, stamp, tick
        self.counts['accepted'] += 1
        return True


class FootGeometry:
    """Lowest collision-mesh vertices and ankle origins, relative to pelvis."""

    def __init__(self, model_path, joint_names):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        m = self.model
        if m.nq != 26 or m.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError('expected the free-base, 19-joint H1 model')
        if int(m.jnt_bodyid[0]) != m.body('pelvis').id:
            raise ValueError('the model free joint must belong to pelvis')
        self.qadr = np.array([m.joint(n).qposadr.item() for n in joint_names])
        self.ankles = [m.body(s + '_ankle_link').id for s in ('left', 'right')]
        self.meshes = []
        for side in ('left', 'right'):
            gid = m.geom(side + '_ankle_link_collision').id
            if m.geom_type[gid] != mujoco.mjtGeom.mjGEOM_MESH:
                raise ValueError('expected the H1 mesh-foot collision model')
            mesh = m.geom_dataid[gid]
            first, count = m.mesh_vertadr[mesh], m.mesh_vertnum[mesh]
            self.meshes.append((gid, m.mesh_vert[first:first + count].copy()))

    def heights(self, q, pelvis_quat):
        """Return required pelvis heights for each sole/ankle to have world z=0."""
        q, quat = np.asarray(q, float), np.asarray(pelvis_quat, float)
        if q.shape != (19,) or quat.shape != (4,) or not np.isfinite(q).all() or not np.isfinite(quat).all():
            raise ValueError('invalid H1 joints or pelvis quaternion')
        if abs(np.linalg.norm(quat) - 1) > 0.01:
            raise ValueError('pelvis quaternion is not normalized')
        d, m = self.data, self.model
        d.qpos[:3] = 0
        d.qpos[3:7] = quat / np.linalg.norm(quat)
        d.qpos[self.qadr] = q
        mujoco.mj_forward(m, d)
        sole = []
        for gid, vertices in self.meshes:
            z = vertices @ d.geom_xmat[gid].reshape(3, 3)[2] + d.geom_xpos[gid, 2]
            sole.append(-float(z.min()))
        return np.asarray(sole), -d.xpos[self.ankles, 2].copy()


def yaw_metrics(t, quats):
    quats = np.asarray(quats, float)
    if quats.shape != (len(t), 4) or not np.isfinite(quats).all():
        raise ValueError('invalid quaternion series')
    if np.max(np.abs(np.linalg.norm(quats, axis=1) - 1)) > 0.01:
        raise ValueError('quaternion series is not normalized')
    euler = Rotation.from_quat(quats[:, [1, 2, 3, 0]]).as_euler('xyz')
    yaw = np.unwrap(euler[:, 2])
    window = min(1.0, (t[-1] - t[0]) / 4)
    first, last = t <= t[0] + window, t >= t[-1] - window
    return dict(
        first_window_yaw_deg=float(np.degrees(np.mean(yaw[first]))),
        last_window_yaw_deg=float(np.degrees(np.mean(yaw[last]))),
        change_deg=float(np.degrees(np.mean(yaw[last]) - np.mean(yaw[first]))),
        slope_deg_s=float(np.degrees(np.polyfit(t - t[0], yaw, 1)[0])),
        roll_pitch_span_deg=np.degrees(np.ptp(euler[:, :2], axis=0)).tolist(),
    )


def analyze_pose(samples, geometry, *, measured_height=None, contact='unknown',
                 stationary=False, calibration_s=1.0, problems=(),
                 max_joint_span=0.02, max_contact_gap=0.02):
    """Analyze paired observations; no reference motion or policy is used.

    samples contains t [s], q [rad], pelvis_quat/odom_quat [wxyz], odom_pos [m].
    Calibration and hold-out windows are disjoint. Residuals use the same model,
    so they are consistency checks, not independent accuracy measurements.
    """
    a = {k: np.asarray(samples[k], float) for k in ('t', 'q', 'pelvis_quat', 'odom_quat', 'odom_pos')}
    t = a['t']
    n = len(t)
    if n < 10 or t.ndim != 1 or not np.isfinite(t).all() or np.any(np.diff(t) <= 0):
        raise ValueError('need >=10 paired samples with strictly increasing time')
    for key, width in (('q', 19), ('pelvis_quat', 4), ('odom_quat', 4), ('odom_pos', 3)):
        if a[key].shape != (n, width) or not np.isfinite(a[key]).all():
            raise ValueError(f'invalid paired array {key}')
    if t[-1] - t[0] < calibration_s + 1.0:
        raise ValueError('need at least one second after the calibration window')
    if measured_height is not None and (not np.isfinite(measured_height) or measured_height <= 0):
        raise ValueError('measured pelvis height must be positive and finite')
    if contact not in ('unknown', 'left', 'right', 'both'):
        raise ValueError('invalid contact selection')
    cal = t <= t[0] + calibration_s
    holdout = ~cal
    if np.count_nonzero(cal) < 5 or np.count_nonzero(holdout) < 5:
        raise ValueError('insufficient samples in calibration or hold-out window')
    heights = [geometry.heights(q, quat) for q, quat in zip(a['q'], a['pelvis_quat'])]
    soles = np.stack([v[0] for v in heights])
    ankles = np.stack([v[1] for v in heights])
    pos = a['odom_pos']
    ids = {'unknown': [], 'left': [0], 'right': [1], 'both': [0, 1]}[contact]
    fit_height = soles[:, ids].mean(1) if ids else None
    geometric_offset = float(np.median(fit_height[cal] - pos[cal, 2])) if ids else None
    manual_offset = float(measured_height - np.median(pos[cal, 2])) if measured_height is not None else None
    q_span = float(np.ptp(a['q'], axis=0).max())
    imu = yaw_metrics(t, a['pelvis_quat'])
    odom = yaw_metrics(t, a['odom_quat'])
    issues = list(problems)
    if q_span > max_joint_span:
        issues.append(f'joint angle span {q_span:.4f} rad exceeds {max_joint_span:g}; stationary fit withheld')
    if max(imu['roll_pitch_span_deg']) > 1.0:
        issues.append('pelvis roll/pitch varies by >1 degree; stationary fit withheld')
    if np.ptp(pos[:, 2]) > 0.01:
        issues.append('odometry height varies by >1 cm; stationary fit withheld')
    if contact == 'both' and np.max(np.abs(soles[:, 0] - soles[:, 1])) > max_contact_gap:
        issues.append('left/right contact heights disagree; check contact declaration, model and joint mapping')
    if manual_offset is not None and geometric_offset is not None and abs(manual_offset - geometric_offset) > max_contact_gap:
        issues.append('measured pelvis height disagrees with contact geometry; check the physical measurement point')
    if not stationary:
        issues.append('physical stationarity not confirmed with --stationary')
    if manual_offset is None and geometric_offset is None:
        issues.append('provide measured pelvis height or explicitly confirmed foot contact')
    candidate = manual_offset if manual_offset is not None else geometric_offset
    available = not issues and candidate is not None
    if not available:
        candidate = None
    first, last = t <= t[0] + min(1.0, (t[-1] - t[0]) / 4), t >= t[-1] - min(1.0, (t[-1] - t[0]) / 4)
    delta = pos[last].mean(0) - pos[first].mean(0)
    corrected = None
    if candidate is not None:
        z = pos[:, 2] + candidate  # One fixed offset, NEVER refit each frame.
        residual = z[:, None] - soles
        corrected = dict(
            mean_pelvis_height_m=float(z.mean()),
            height_span_m=float(np.ptp(z)),
            holdout_sole_z_mean_m=residual[holdout].mean(0).tolist(),
            holdout_sole_z_max_abs_m=np.abs(residual[holdout]).max(0).tolist(),
        )
    return dict(
        schema='h1_pose_diagnostics_v1', deployment_ready=False,
        hypothesis='fixed world-z datum offset; body-frame extrinsic is NOT identified',
        samples=n, span_s=float(t[-1] - t[0]), stationary_asserted=stationary,
        contact_asserted=contact, measured_pelvis_height_m=measured_height,
        calibration_window_s=calibration_s, holdout_samples=int(holdout.sum()),
        max_joint_span_rad=q_span, issues=sorted(set(issues)),
        geometry=dict(
            sole_required_height_mean_m=soles.mean(0).tolist(),
            ankle_required_height_mean_m=ankles.mean(0).tolist(),
            odom_minus_mean_ankle_height_mean_m=float((pos[:, 2] - ankles.mean(1)).mean()),
            odom_minus_mean_ankle_height_max_abs_m=float(np.abs(pos[:, 2] - ankles.mean(1)).max()),
            model_contact_height_difference_max_m=float(np.abs(soles[:, 0] - soles[:, 1]).max()),
        ),
        fit=dict(manual_world_z_offset_m=manual_offset, contact_world_z_offset_m=geometric_offset,
                 candidate_available=available, candidate_world_z_offset_m=candidate,
                 candidate_source='manual height' if manual_offset is not None else 'contact geometry',
                 corrected=corrected),
        odometry=dict(position_mean_m=pos.mean(0).tolist(), position_span_m=np.ptp(pos, axis=0).tolist(),
                      first_to_last_window_delta_m=delta.tolist(), xy_window_displacement_m=float(np.linalg.norm(delta[:2])),
                      position_slope_m_s=np.polyfit(t - t[0], pos, 1)[0].tolist(), yaw=odom),
        pelvis_imu=imu,
        limitations=[
            'A stationary recording does not identify robot_center or distinguish datum/body-frame offsets.',
            'Foot contact is a user assertion; zero GO foot-force fields do not establish contact.',
            'DDS publication progress does not prove internal estimator accuracy or measurement age.',
            'A constant height offset does not remove xy/yaw drift or align motion references.',
            'No controller consumes this report. It never certifies motion or hardware deployment.',
        ],
    )
