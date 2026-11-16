"""GO DDS H1 simulator for the exact Python/C++ hardware runner interfaces.

Uses a nonzero DDS domain, H1 hardware motor slots and torso IMU outputs.
Physics waits for the first command. Optional files inject buttons, support and temperature.
"""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))

import argparse
import json
import signal
import socket
import threading
import time

import mujoco
import numpy as np

import h1_common as gc
from h1_deploy.deploy.core import Clip
from h1_deploy.deploy.models import load_scene
from h1_deploy.deploy.sim_support import BandSupportFile
from h1_deploy.robots.h1 import H1_JOINT_NAMES, default_pos, effort_limit
from sim2sim import Robot


# The model has no separate neck body. Attach near the neck base, above the
# shoulders (z ~= 0.43 m), in torso_link's local frame. Units are metres.
BAND_BODY = 'torso_link'
BAND_POINT_LOCAL = np.array([0., 0., 0.50])


class ElasticBand:
    """Original spring/damper, attached to the neck instead of the pelvis."""

    def __init__(self, model, data, length, anchor_offset_z=0.0):
        self.model = model
        self.body = model.body(BAND_BODY).id
        self.local_point = BAND_POINT_LOCAL.copy()
        self.length = length
        self.jac = np.zeros((3, model.nv))
        # Raise the fixed world anchor independently of the spring rest length.
        self.anchor = self.position(data) + np.array([0., 0., length + anchor_offset_z])

    def position(self, data):
        return data.xpos[self.body] + data.xmat[self.body].reshape(3, 3) @ self.local_point

    def reanchor_xy(self, data):
        # Move the gantry above the current neck once when support is restored.
        # Keep its world height/rest length; never follow the robot every tick.
        self.anchor[:2] = self.position(data)[:2]

    def apply(self, data, scale=1.0):
        point = self.position(data)
        mujoco.mj_jac(self.model, data, self.jac, None, point, self.body)
        velocity = self.jac @ data.qvel
        delta = self.anchor - point
        length = np.linalg.norm(delta)
        force = scale * (2000 * max(0, length - self.length) * delta / max(length, 1e-9) - 200 * velocity)
        # xfrc_applied acts at the body's COM in world coordinates. Include the
        # moment arm so the resulting wrench is a force applied at the neck.
        data.xfrc_applied[self.body, :3] += force
        data.xfrc_applied[self.body, 3:] += np.cross(point - data.xipos[self.body], force)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--domain', type=int, default=1)
    ap.add_argument('--iface', default='lo')
    ap.add_argument('--scene', type=Path)
    ap.add_argument('--clip', type=Path, help='initialize in this reference frame for anchor alignment')
    ap.add_argument('--dt', type=float, default=0.001)
    ap.add_argument('--duration', type=float, default=0, help='wall seconds, 0 = until interrupted')
    ap.add_argument('--init', choices=('zero', 'knees_bent'), default='knees_bent')
    ap.add_argument('--pelvis-z', type=float, default=1.05)
    ap.add_argument('--band-file', type=Path)
    ap.add_argument('--band-length', type=float, default=1.0)
    ap.add_argument('--band-anchor-offset-z', type=float, default=0.0,
                    help='fixed upper anchor vertical offset in metres; positive raises it without changing band length (default: 0)')
    ap.add_argument('--buttons-file', type=Path)
    ap.add_argument('--temp-file', type=Path, help='hardware motor index, base Celsius, rate Celsius/s')
    ap.add_argument('--motor-temp', type=int, default=30)
    ap.add_argument('--pose-udp', default=None, help='publish aligned pelvis pose to HOST:PORT')
    ap.add_argument('--log', type=Path)
    ap.add_argument('--log-tau', action='store_true')
    ap.add_argument('--status-every', type=float, default=1)
    ap.add_argument('--viewer', action='store_true')
    a = ap.parse_args()
    if a.domain == 0:
        ap.error('the simulator must use a nonzero DDS domain')
    if a.dt <= 0:
        ap.error('--dt must be positive')
    if not np.isfinite(a.band_anchor_offset_z):
        ap.error('--band-anchor-offset-z must be finite')
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
    from unitree_sdk2py.utils.crc import CRC
    model = load_scene(a.scene, torque=True)
    model.opt.timestep = a.dt
    robot = Robot(model, list(H1_JOINT_NAMES))
    if a.clip:
        clip = Clip.load(a.clip)
        if clip.root_pos is None:
            raise ValueError('initial clip requires root poses')
        robot.set_state(clip.root_pos[0], clip.root_quat_wxyz[0], clip.joint_pos[0])
    else:
        robot.set_state([0, 0, a.pelvis_z], [1, 0, 0, 0], default_pos if a.init == 'knees_bent' else np.zeros(19))
    data = robot.d
    lock = threading.Lock()
    shared = {'command': None, 'count': 0, 'time': 0.}
    crc = CRC()

    def command(message):
        if message.crc != crc.Crc(message):
            return
        motors = [message.motor_cmd[i] for i in gc.JOINT_TO_MOTOR]
        values = np.array([[m.q, m.dq, m.kp, m.kd, m.tau] for m in motors])
        if not np.isfinite(values).all():
            return
        enabled = np.array([m.mode != 0 for m in motors])
        values[~enabled, 2:] = 0
        with lock:
            shared.update(command=values, count=shared['count'] + 1, time=time.monotonic())

    gc.init_dds(a.domain, a.iface)
    pub = ChannelPublisher('rt/lowstate', LowState_)
    pub.Init()
    sub = ChannelSubscriber('rt/lowcmd', LowCmd_)
    sub.Init(command, 0)
    message = unitree_go_msg_dds__LowState_()
    message.head[0], message.head[1], message.level_flag = 0xFE, 0xEF, 0xFF
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    pose_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if a.pose_udp else None
    pose_address = (a.pose_udp.rsplit(':', 1)[0], int(a.pose_udp.rsplit(':', 1)[1])) if a.pose_udp else None
    viewer = None
    if a.viewer:
        import mujoco.viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)
    start = time.monotonic()
    deadline, next_status, next_pose, next_log = start, start, start, start
    next_view = start
    band = ElasticBand(model, data, a.band_length, a.band_anchor_offset_z)
    band_control = BandSupportFile(a.band_file)
    support_generation = 0
    temp_content, temp_start = '', start
    logs = []
    tick = 0
    try:
        while not stop.is_set() and (not a.duration or time.monotonic() - start < a.duration):
            now = time.monotonic()
            with lock:
                command_values, count, command_time = shared['command'], shared['count'], shared['time']
            band_scale = band_control.scale(now)
            band_on = band_scale > 0
            if band_on and band_control.support_generation != support_generation:
                band.reanchor_xy(data)
                support_generation = band_control.support_generation
                print(f'band ON: anchor XY moved above current neck: {band.anchor[:2].round(4).tolist()} m; '
                      f'anchor_z={band.anchor[2]:.3f}m unchanged', flush=True)
            data.xfrc_applied[:] = 0
            if band_on:
                band.apply(data, band_scale)
            pd = np.zeros(19)
            if command_values is not None:
                q, dq = robot.jq(), robot.jdq()
                if now - command_time > 0.1:
                    pd = -3 * dq
                else:
                    target, desired_vel, kp, kd, tau = command_values.T
                    pd = kp * (target - q) + kd * (desired_vel - dq) + tau
                robot.set_ctrl(np.clip(pd, -np.asarray(effort_limit), effort_limit))
                mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            q, dq = robot.jq(), robot.jdq()
            for i, motor_index in enumerate(gc.JOINT_TO_MOTOR):
                motor = message.motor_state[int(motor_index)]
                motor.q, motor.dq = float(q[i]), float(dq[i])
                motor.tau_est = float(data.actuator_force[robot.act[i]])
                motor.temperature = int(np.clip(a.motor_temp, 0, 255))
            text = a.temp_file.read_text() if a.temp_file and a.temp_file.exists() else ''
            if text != temp_content:
                temp_content, temp_start = text, now
            for line in text.splitlines():
                parts = line.split('#')[0].split()
                if len(parts) >= 3:
                    i, base, rate = int(parts[0]), float(parts[1]), float(parts[2])
                    if 0 <= i < 20:
                        message.motor_state[i].temperature = int(np.clip(round(base + rate * (now - temp_start)), 0, 255))
            imu = message.imu_state
            for sensor_name, field in (('imu_quat', 'quaternion'), ('imu_gyro', 'gyroscope'), ('imu_acc', 'accelerometer')):
                sensor = model.sensor(sensor_name)
                setattr(imu, field, data.sensordata[int(sensor.adr[0]):int(sensor.adr[0]) + int(sensor.dim[0])].tolist())
            tick += max(1, round(a.dt * 1000))
            message.tick = tick % (2**32)
            buttons = a.buttons_file.read_text().strip() if a.buttons_file and a.buttons_file.exists() else ''
            names = [x.strip() for x in buttons.replace(',', '+').split('+') if x.strip()]
            b2, b3 = gc.keys_from_names(names)
            message.wireless_remote[2], message.wireless_remote[3] = b2, b3
            message.crc = crc.Crc(message)
            pub.Write(message)
            if pose_socket and now >= next_pose:
                position, quaternion = robot.root()
                pose_socket.sendto(json.dumps(dict(time_unix_s=time.time(), frame_id='reference',
                    root_pos=position.tolist(), root_quat_wxyz=quaternion.tolist())).encode(), pose_address)
                next_pose = now + 0.01
            if a.log and (a.log_tau or now >= next_log):
                logs.append((data.time, data.qpos.copy(), pd.copy(), band_on, count, band_scale, band.anchor.copy()))
                next_log = now + 0.02
            if now >= next_status:
                print(f't={data.time:.2f}s z={robot.root()[0][2]:.3f} commands={count} band={band_on} support={100 * band_scale:.1f}% '
                      f'neck_z={band.position(data)[2]:.3f}m anchor_z={band.anchor[2]:.3f}m '
                      f'band_fz={data.xfrc_applied[band.body, 2]:.1f}N', flush=True)
                next_status = now + a.status_every
            if viewer and now >= next_view:
                viewer.sync()
                next_view = now + 1 / 60
                if not viewer.is_running(): break
            deadline += a.dt
            time.sleep(max(0, deadline - time.monotonic()))
            if deadline < time.monotonic() - 0.05:
                deadline = time.monotonic()
    finally:
        pub.Close(); sub.Close()
        if viewer: viewer.close()
        if pose_socket: pose_socket.close()
        if a.log:
            a.log.parent.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(a.log, t=[r[0] for r in logs], qpos=[r[1] for r in logs],
                                pd=[r[2] for r in logs], band=[r[3] for r in logs], n_cmd=[r[4] for r in logs],
                                band_scale=[r[5] for r in logs], band_anchor=[r[6] for r in logs])


if __name__ == '__main__':
    main()
