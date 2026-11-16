"""Same-host C++ odometry input and a fixed, per-preparation reference transform.

The z correction is a configured WORLD datum offset, not a body extrinsic.
No reference height, roll or pitch is fitted away. No DDS/control publishers here.
"""
from __future__ import annotations

import json
import select
import socket
import time

import numpy as np
from scipy.spatial.transform import Rotation


def _vector(value, size):
    out = np.asarray(value, dtype=float)
    if out.shape != (size,) or not np.isfinite(out).all():
        raise ValueError('invalid pose vector')
    return out


def _rotation(quat):
    q = _vector(quat, 4)
    if abs(np.linalg.norm(q) - 1) > 0.01:
        raise ValueError('invalid pose quaternion norm')
    return Rotation.from_quat(q[[1, 2, 3, 0]])


def _heading(rotation):
    axis = rotation.as_matrix()[:, 0]
    if np.linalg.norm(axis[:2]) < 0.1:
        raise ValueError('pelvis heading undefined near vertical forward axis')
    return np.arctan2(axis[1], axis[0])


class ReferenceAlignment:
    """Map actual odometry XY/yaw to clip frame 0 once; preserve measured z."""

    def reset(self, reference_pos, reference_quat):
        self.reference_pos = _vector(reference_pos, 3).copy()
        self.reference_heading = _heading(_rotation(reference_quat))
        self.rotation = None
        self.translation = None
        self.yaw = None

    def apply(self, position, rotation):
        position = _vector(position, 3)
        if self.rotation is None:
            self.yaw = float(self.reference_heading - _heading(rotation))
            self.yaw = float(np.arctan2(np.sin(self.yaw), np.cos(self.yaw)))
            self.rotation = Rotation.from_rotvec([0, 0, self.yaw])
            self.translation = self.reference_pos - self.rotation.apply(position)
            self.translation[2] = 0.0
        pos = self.rotation.apply(position) + self.translation
        quat = (self.rotation * rotation).as_quat()[[3, 0, 1, 2]]
        return pos, quat


class OdomPoseReceiver:
    """Consumes h1_odom_bridge packets on loopback; clocks must be on this PC.

    Source publisher/time checks and CRC/pairing run in C++. Session/sequence,
    pairing age and local monotonic age are checked again here. Restarts latch an
    error: an active reference transform must not silently adopt a new odom origin.
    """
    READ_BUDGET_S = 0.02

    def __init__(self, address, max_age, z_offset, orientation_body):
        host, port = address.rsplit(':', 1)
        if host != '127.0.0.1':
            raise ValueError('unitree-odom requires same-PC --pose-listen 127.0.0.1:PORT')
        if not np.isfinite(max_age) or max_age <= 0 or not np.isfinite(z_offset):
            raise ValueError('finite odom z offset and positive pose age required')
        if orientation_body not in ('torso', 'pelvis'):
            raise ValueError('explicit odom orientation body required: torso or pelvis')
        self.max_age, self.z_offset = float(max_age), float(z_offset)
        self.orientation_body = orientation_body
        self.alignment = ReferenceAlignment()
        self.prepared = False
        self.alignment_event = None
        self.latest = None
        self.session = None
        self.sequence = 0
        self.source_ns = 0
        self.fault = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, int(port)))
        self.sock.setblocking(False)

    def reset_reference(self, clip):
        # Called at the original HOLD -> STANDBY_PRE transition; no socket read.
        self.alignment.reset(clip.root_pos[0], clip.root_quat_wxyz[0])
        self.prepared = True
        self.alignment_event = None

    def _accept(self, packet, now):
        if packet['frame_id'] != 'unitree_odom_v1':
            return
        session = str(packet['session'])
        sequence, source = int(packet['sequence']), int(packet['source_ns'])
        stamp = float(packet['sample_monotonic_s'])
        pair_ms = float(packet['pair_ms'])
        pos = _vector(packet['position'], 3)
        rotation = _rotation(packet['quaternion_wxyz'])
        waist = float(packet['waist_q'])
        if (not session or sequence <= 0 or source <= 0 or not np.isfinite(waist)
                or not np.isfinite(stamp) or not np.isfinite(pair_ms) or not 0 <= pair_ms <= 5.0):
            return
        if self.session is not None and session != self.session:
            self.fault = 'odometry bridge restarted/changed; restart controller and prepare again'
            return
        if sequence < self.sequence or source < self.source_ns:
            self.fault = 'odometry sequence/source time regressed'
            return
        if sequence == self.sequence or source == self.source_ns:
            return  # duplicates never refresh age
        if not -0.02 <= now - stamp <= self.max_age:
            return
        if self.orientation_body == 'torso':
            # Odom-frame torso orientation -> odom-frame pelvis; waist motor 6.
            rotation = rotation * Rotation.from_rotvec([0, 0, -waist])
        pos = pos.copy()
        pos[2] += self.z_offset
        self.session, self.sequence, self.source_ns = session, sequence, source
        self.latest = (stamp, pos, rotation)

    def read(self):
        if not self.prepared:
            raise ValueError('odometry reference has not been prepared')
        deadline = time.monotonic() + self.READ_BUDGET_S
        while True:
            while time.monotonic() < deadline:
                try:
                    raw = self.sock.recv(4096)
                except BlockingIOError:
                    break
                try:
                    self._accept(json.loads(raw), time.monotonic())
                except (ValueError, KeyError, TypeError, OverflowError):
                    continue
            if self.fault:
                raise ValueError(self.fault)
            if self.latest is not None and -0.02 <= time.monotonic() - self.latest[0] <= self.max_age:
                first = self.alignment.rotation is None
                result = self.alignment.apply(self.latest[1], self.latest[2])
                if first:
                    self.alignment_event = (
                        f'fixed XY/yaw alignment: yaw={np.degrees(self.alignment.yaw):+.3f} deg, '
                        f'xy shift={self.alignment.translation[:2].round(4).tolist()} m; '
                        f'world z offset={self.z_offset:+.6f} m, actual z={result[0][2]:.4f} m; '
                        'kept through Start and STANDBY_POST')
                return result
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.sock], [], [], remaining)[0]:
                break
        raise ValueError('no fresh paired odometry; check h1_odom_bridge, source progress and same-PC UDP')

    def close(self):
        self.sock.close()
