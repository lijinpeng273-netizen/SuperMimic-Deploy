"""H1 real/sim DDS runner: GO protocol, policy-order control and optional external anchor pose.

Remote: L2+up interpolate; R1+A standby; R1+X play; L2+B damping.
With --motion-select: L1+A/X/Y select; R1+B takes over from HOLD; Start plays when ready.
Preserves the original transition, watchdog, recording and multi-clip state machine.
See docs/SIM2REAL.md for interfaces and unverified hardware assumptions.
Python SDK 通信、状态机、控制循环、保护与日志
"""
from __future__ import annotations
import argparse
import collections
import gc as pygc
import json
import os
import queue
import select
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
import h1_common as gc
from h1_deploy.deploy.policy import load_policy
from h1_deploy.deploy.policy_takeover import PolicyTakeover
from h1_deploy.deploy.sim_support import write_band_command
from h1_deploy.deploy.tracking_report import pelvis_tracking, pelvis_error_text, summarize_pelvis
from h1_deploy.deploy.anchor import PoseReceiver, SimPoseReceiver
from h1_deploy.deploy.motion_selection import MotionSelection, MotionSelectionMixin, selection_combos, validate_selection_args
from h1_deploy.deploy.core import Clip, DeployConfig, DeployCore, OnnxPolicy, SafetyCfg, interpolate_pose, projected_gravity, static_clip
N = gc.NUM_JOINTS
STATES = ['PASSIVE', 'INTERP', 'HOLD', 'POLICY', 'END_HOLD', 'DAMP', 'STANDBY_PRE', 'STANDBY_POST']
SID = {s: i for i, s in enumerate(STATES)}
POLICY_STATES = ('STANDBY_PRE', 'POLICY', 'STANDBY_POST')
HEARTBEAT_STATES = ('INTERP', 'HOLD', 'END_HOLD') + POLICY_STATES
STANDBY_STATES = ('STANDBY_PRE', 'STANDBY_POST')
LEG_IDX = np.arange(10)
DAMP_KD = 3.0
LEG_KP = [150, 150, 150, 200, 40]
STAND_KP = np.array(LEG_KP * 2 + [300] * 1 + [100, 100, 100, 50] * 2, float)
COMBOS = {'interp': gc.combo_mask('L2', 'up'), 'policy': gc.combo_mask('R1', 'A'), 'start': gc.combo_mask('R1', 'X'), 'damp': gc.combo_mask('L2', 'B')}
COMBO_LABEL = {'interp': 'L2+up', 'policy': 'R1+A', 'start': 'R1+X'}
_CORE = SafetyCfg()
STAGES = {'air': dict(max_vel_frac=0.3, max_abs_action=10.0, torque_clamp_factor=0.5, max_tilt_hold=0.35), 'feet': dict(max_vel_frac=0.3, max_abs_action=10.0, torque_clamp_factor=0.5, max_tilt_hold=0.35), 'slack': dict(max_vel_frac=0.5, max_abs_action=15.0, torque_clamp_factor=1.0, max_tilt_hold=0.5), 'ground': dict(max_vel_frac=_CORE.max_vel_frac, max_abs_action=_CORE.max_abs_action, torque_clamp_factor=_CORE.torque_clamp_factor, max_tilt_hold=0.8)}
"""
max_vel_frac:实际关节速度阈值为配置速度上限的 80%，超过就触发保护
max_abs_action:策略输出各分量的绝对值上限为 10，超过就触发保护
torque_clamp_factor:用配置力矩上限的 100% 计算目标角度边界，超出后裁剪目标角度，继续控制
max_tilt_hold:固定 PD 阶段机最大倾斜保持角度为 0.45 弧度
"""

class QueueStream:
    """sys.stdout / sys.stderr replacement: never blocks, never raises; the printer thread does the I/O."""

    def __init__(self, q):
        self.q = q

    def write(self, s):
        if s:
            self.q.put(str(s))
        return len(s)

    def flush(self):
        pass

    def isatty(self):
        return False

class Printer:

    def __init__(self):
        self.q = queue.SimpleQueue()
        self.out = sys.__stdout__
        self.file = None
        self.n_fail = 0
        self.th = threading.Thread(target=self._loop, name='printer', daemon=True)
        self.th.start()

    def open_file(self, path: Path):
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            self.q.put(('__open__', path))
        except Exception:
            pass

    def _loop(self):
        while True:
            s = self.q.get()
            if s is None:
                return
            if isinstance(s, tuple):
                try:
                    self.file = open(s[1], 'a', buffering=1)
                except Exception:
                    self.file = None
                continue
            try:
                self.out.write(s)
                self.out.flush()
            except Exception:
                self.n_fail += 1
            if self.file is not None:
                try:
                    self.file.write(s)
                except Exception:
                    pass

    def close(self, timeout=2.0):
        self.q.put(None)
        self.th.join(timeout)
        try:
            if self.file is not None:
                self.file.close()
        except Exception:
            pass

class LowcmdWatcher:
    """Counts rt/lowcmd samples in a CHILD process (this file with --lowcmd-watcher-child).

    An in-process rt/lowcmd reader with a Python listener deadlocked the runner in the sim 'foreign' test (all
    threads in futex_wait once a second publisher was active): the DDS receive thread holds the reader while
    waiting for the GIL to run the listener, and the command thread holds the GIL inside Write() while the local
    delivery to that same reader waits for it (diagnosis inferred, gdb attach is not permitted here). In a child
    process nothing shares our GIL. The child prints its running sample count every 0.1 s; read non-blocking.
    """

    def __init__(self, domain: int, iface: str | None):
        self.p = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--lowcmd-watcher-child', str(domain), iface or ''], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, start_new_session=True)
        self.fd = self.p.stdout.fileno()
        os.set_blocking(self.fd, False)
        self.buf = b''
        self.count = 0
        self.t = 0.0
        self.ready = False
        self.dead = False

    def poll(self):
        if self.dead:
            return
        try:
            b = os.read(self.fd, 65536)
        except (BlockingIOError, InterruptedError):
            return
        except OSError:
            b = b''
        if b == b'':
            self.dead = True
            return
        self.buf += b
        *lines, self.buf = self.buf.split(b'\n')
        for ln in lines:
            if ln == b'ready':
                self.ready = True
            else:
                parts = ln.split()
                if len(parts) == 2 and parts[0].isdigit():
                    self.count, self.t = (int(parts[0]), float(parts[1]))

    def wait_ready(self, timeout: float) -> bool:
        t_end = time.monotonic() + timeout
        while not self.ready and (not self.dead) and (time.monotonic() < t_end):
            self.poll()
            time.sleep(0.02)
        return self.ready

    def count_for(self, secs: float) -> int:
        self.poll()
        time.sleep(0.15)
        self.poll()
        n0 = self.count
        time.sleep(secs + 0.15)
        self.poll()
        return self.count - n0

    def close(self):
        try:
            self.p.kill()
        except Exception:
            pass

def watcher_child(domain: int, iface: str):
    """--lowcmd-watcher-child: subscribe rt/lowcmd (count only), report on stdout, exit on stdin EOF."""
    gc.init_dds(domain, iface or None)
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
    cnt = [0]

    def h(_m):
        cnt[0] += 1
    sub = ChannelSubscriber('rt/lowcmd', LowCmd_)
    sub.Init(h, 0)
    out = sys.stdout
    try:
        out.write('ready\n')
        out.flush()
        while True:
            r, _, _ = select.select([0], [], [], 0.1)
            if r and os.read(0, 64) == b'':
                break
            out.write(f'{cnt[0]} {time.monotonic():.6f}\n')
            out.flush()
    except Exception:
        pass
    os._exit(0)

class Runner(MotionSelectionMixin):

    def __init__(self, args, printer: Printer):
        self.a = args
        self.printer = printer
        self.cfg = DeployConfig.load(args.artifacts / 'deploy_config.json')
        if self.cfg.needs_root_pose and (not args.pose_listen):
            raise ValueError('this checkpoint needs anchor feedback; pass --pose-listen for an aligned external pose stream')
        self.latest_anchor_pose = None
        self.pose_status_t = -np.inf
        receiver_type = SimPoseReceiver if args.sim else PoseReceiver
        self.pose_receiver = receiver_type(args.pose_listen, args.pose_max_age) if args.pose_listen else None
        assert len(self.cfg.joint_names) == N
        c = self.cfg
        self.lock_waist = bool(False)
        self.locked = np.zeros(N, bool)
        self.live = ~self.locked
        self.clip_notes = []
        self.clips = [Clip.load(p) for p in args.clip]
        self.motion_selection = MotionSelection.from_paths(args.clip) if args.motion_select else None
        self.sync_addr, self.sync_sock = (None, None)
        if args.sync_udp:
            host, port = args.sync_udp.rsplit(':', 1)
            self.sync_addr = (host, int(port))
            self.sync_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sync_sock.setblocking(False)
        self.f0s = [np.clip(cl.joint_pos[0], c.soft_lower, c.soft_upper) for cl in self.clips]
        self.fends = [np.clip(cl.joint_pos[-1], c.soft_lower, c.soft_upper) for cl in self.clips]
        for f in self.f0s + self.fends:
            f[self.locked] = 0.0
        self.static_pre = [static_clip(cl, 0) for cl in self.clips]
        self.static_post = [static_clip(cl, cl.num_frames - 1) for cl in self.clips]
        self.sb_win: collections.deque = collections.deque()
        self.standby_ok = [True for cl in self.clips]
        self.confirm: dict | None = None
        self.sbs: dict | None = None
        self.ref_speed = []
        for cl in self.clips:
            dq = np.abs(cl.goal[1:26, N:2 * N]).astype(float)
            dq[:, self.locked] = 0.0
            k, j = np.unravel_index(int(np.argmax(dq)), dq.shape) if dq.size else (0, 0)
            self.ref_speed.append((float(dq.max()) if dq.size else 0.0, c.joint_names[j], int(k) + 1, float(np.abs(cl.goal[0, N:2 * N]).max())))
        self.policy_fn = load_policy(args.artifacts / 'policy.onnx', threads=args.ort_threads)
        if tuple(self.policy_fn.input_shapes) != c.input_shapes:   # 维度检查
            raise ValueError('policy and deploy_config disagree on observation/anchor dimensions')
        self.n_policy_calls = 0

        def counted(obs, goal):
            self.n_policy_calls += 1
            a = self.policy_fn(obs, goal)
            if args.sim_inject_nan_step is not None and self.n_policy_calls >= args.sim_inject_nan_step:
                a = a * np.nan
            return a
        self.policy = counted
        prof = dict(STAGES[args.stage])
        for k in ('max_vel_frac', 'max_abs_action', 'torque_clamp_factor', 'max_tilt_hold'):
            v = getattr(args, k)
            if v is not None:
                prof[k] = v
        self.profile = prof
        self.max_tilt_hold = prof['max_tilt_hold']
        self.safety = SafetyCfg(max_vel_frac=prof['max_vel_frac'], max_abs_action=prof['max_abs_action'], torque_clamp_factor=prof['torque_clamp_factor'])
        self.stand_kp = STAND_KP.copy()
        # Fixed-PD damping follows this policy's deployment config unless overridden.
        self.stand_kd = c.kd.copy()
        if args.stand_waist_kd is not None:
            self.stand_kd[10:11] = args.stand_waist_kd
        if args.stand_arm_kd is not None:
            self.stand_kd[11:] = args.stand_arm_kd
        self.clip_idx = 0
        self.core: DeployCore | None = None
        self.policy_takeover: PolicyTakeover | None = None
        self.end: dict | None = None
        self.lock = threading.RLock()
        self.state = 'PASSIVE'
        self.state_t = 0.0
        self.damp_reason: str | None = None
        self.events: list[dict] = []
        self.pending: collections.deque = collections.deque()
        self.accept_buttons = False
        combos = selection_combos(gc.combo_mask) if self.motion_selection is not None else COMBOS
        self.combo = gc.ComboDebounce(combos, stable_n=args.button_stable_n, release_s=args.button_release_s,
                                      exclusive=('policy',) if self.motion_selection is not None else ())
        self.damp_req: str | None = None
        self.sig: int | None = None
        self.sig_r = None
        self.msg = None
        self.t_rx = 0.0
        self.n_rx = 0
        self.n_crc_bad = 0
        self.cb_errors = 0
        self.cb_last_err = ''
        self.last_tick = None
        self.t_tick_change = 0.0
        self.tick_moving = False
        self.mm_latched: int | None = None
        self.watcher: LowcmdWatcher | None = None
        self.self_delivery: float | None = None
        self.cmd = (None, np.zeros(N), np.full(N, DAMP_KD))
        self.cmd_lock = threading.Lock()
        self.t_main_cmd = time.monotonic()
        self.last_cmd_q: np.ndarray | None = None
        self.n_sent = 0
        self.n_write_fail = 0
        self.write_fail_consec = 0
        self.write_last_err = ''
        self.n_cmd_exc = 0
        self.stop = threading.Event()
        self.cmd_stop = threading.Event()
        self.cmd_started = False
        self.rt_info: dict = {}
        self.t0 = time.monotonic()
        self.wall0 = time.time()
        self._alloc_logs()

    def _alloc_logs(self):
        cap = int(self.a.cmd_log_max_s * self.a.cmd_hz * 1.15)
        self.cl_cap = cap
        self.cl_t = np.zeros(cap)
        self.cl_q = np.zeros((cap, N), np.float32)
        self.cl_kp = np.zeros((cap, N), np.float32)
        self.cl_kd = np.zeros((cap, N), np.float32)
        self.cl_s = np.zeros(cap, np.int8)
        self.cl_n = 0
        self.rec_cap = int(self.a.rec_max_s / self.cfg.control_dt)
        self.rec_warned = False
        self.rec: dict[str, list] = {k: [] for k in ('t', 'deadline', 'compute_ms', 'state', 'clip', 'frame', 'q', 'dq', 'gyro', 'quat', 'tick', 'state_age_ms', 'action', 'q_target', 'n_clamped', 'obs_hist', 'root_pos', 'anchor_quat', 'cmd_kp_mean', 'locked_temp', 'locked_mstat', 'tilt_err')}

    def event(self, kind: str, msg: str):
        """Thread-safe and non-blocking (only enqueues): callable from any thread."""
        t = time.monotonic() - self.t0
        self.events.append({'t': round(t, 4), 'kind': kind, 'msg': msg})
        line = f'[{t:8.3f}] {kind}: {msg}'
        if kind in ('ABORT', 'DAMP'):
            line = '\n' + '!' * 80 + '\n' + line + '\n' + '!' * 80
        self.printer.q.put(line + '\n')

    def set_state(self, new: str, reason: str=''):
        with self.lock:
            if self.state == 'DAMP':
                return False
            old = self.state
            self.state, self.state_t = (new, time.monotonic())
            if new in ('STANDBY_POST', 'END_HOLD'):
                self.sim_band_on(f'entered {new}: motion ended or fixed PD transition')
        self.event('STATE', f'{old} -> {new}' + (f' ({reason})' if reason else ''))
        return True

    def damp(self, reason: str):
        """Main or command thread only (never the DDS listener / a signal handler). Sends before logging."""
        with self.lock:
            if self.state == 'DAMP':
                return
            old = self.state
            self.state, self.state_t, self.damp_reason = ('DAMP', time.monotonic(), reason)
            with self.cmd_lock:
                self.cmd = (None, np.zeros(N), np.full(N, DAMP_KD))
        if self.cmd_started:
            try:
                self.send()
            except Exception:
                pass
        self.sim_band_on('DAMP: policy control stopped')
        self.event('DAMP', f'{old} -> DAMP: {reason}')

    def set_cmd(self, q, kp, kd, expect_state: str):
        """Install new targets unless the state machine moved (e.g. to DAMP) meanwhile. Also the main-loop heartbeat."""
        with self.lock:
            if self.state != expect_state:
                return False
            qq = None if q is None else np.asarray(q, float).copy()
            with self.cmd_lock:
                self.cmd = (qq, np.asarray(kp, float).copy(), np.asarray(kd, float).copy())
            self.t_main_cmd = time.monotonic()
            if qq is not None:
                self.last_cmd_q = qq
        return True

    def on_state(self, msg):
        try:
            t = time.monotonic()
            if self.crc_check and msg.crc != self.crc.Crc(msg):
                self.n_crc_bad += 1
                return
            tick = int(msg.tick)
            if self.last_tick is None or tick != self.last_tick:
                if self.last_tick is not None:
                    self.tick_moving = True
                self.last_tick, self.t_tick_change = (tick, t)
            self.msg, self.t_rx = (msg, t)
            self.n_rx += 1
            fired = self.combo.update(gc.remote_keys(msg.wireless_remote), t)
            if fired and self.accept_buttons:
                for name in fired:
                    if name == 'damp':
                        if self.damp_req is None:
                            self.damp_req = 'remote L2+B'
                    else:
                        self.pending.append((name, t))
        except Exception as e:
            self.cb_errors += 1
            self.cb_last_err = repr(e)

    def state_age(self, t: float | None=None) -> float:
        t = time.monotonic() if t is None else t
        age = t - self.t_rx
        if self.tick_moving:
            age = max(age, t - self.t_tick_change)
        return age

    def build_cmd(self):
        from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
        message = unitree_go_msg_dds__LowCmd_()
        message.head[0], message.head[1] = (254, 239)
        message.level_flag, message.gpio = (255, 0)
        for i, motor in enumerate(message.motor_cmd):
            motor.mode = gc.motor_modes()[i]
            motor.q, motor.dq, motor.tau, motor.kp, motor.kd = (0.0, 0.0, 0.0, 0.0, 0.0)
        return message

    def send(self) -> bool:
        """Publish one rt/lowcmd from the latest targets (500 Hz thread, right after policy steps, in damp())."""
        with self.cmd_lock:
            q, kp, kd = self.cmd
            msg = self.msg
            if q is None:
                q = np.zeros(N) if msg is None else np.array([msg.motor_state[i].q for i in gc.JOINT_TO_MOTOR], float)
            m = self.lowcmd
            mc = m.motor_cmd
            for i in range(N):
                c = mc[int(gc.JOINT_TO_MOTOR[i])]
                c.q, c.kp, c.kd = (float(q[i]), float(kp[i]), float(kd[i]))
            m.crc = self.crc.Crc(m)
            try:
                ok = bool(self.pub.Write(m))
            except Exception as e:
                ok = False
                self.write_last_err = repr(e)
            self.n_sent += 1
            if ok:
                self.write_fail_consec = 0
            else:
                self.n_write_fail += 1
                self.write_fail_consec += 1
            k = self.cl_n % self.cl_cap
            self.cl_t[k] = time.monotonic() - self.t0
            self.cl_q[k], self.cl_kp[k], self.cl_kd[k], self.cl_s[k] = (q, kp, kd, SID[self.state])
            self.cl_n += 1
            if self.cl_n == self.cl_cap:
                self.event('WARN', f'command log full ({self.a.cmd_log_max_s:.0f} s): ring buffer, keeping only the newest')
        return ok

    def _poll_signal(self) -> int | None:
        if self.sig_r is not None:
            try:
                b = os.read(self.sig_r, 64)
                if b:
                    return int(b[-1])
            except (BlockingIOError, InterruptedError):
                pass
            except OSError:
                pass
        return self.sig

    def _pin_cmd_thread(self):
        a, info = (self.a, self.rt_info)
        if a.cmd_cpu != 'none':
            try:
                cpus = sorted(os.sched_getaffinity(0))
                cpu = cpus[-1] if a.cmd_cpu == 'auto' else int(a.cmd_cpu)
                os.sched_setaffinity(threading.get_native_id(), {cpu})
                info['cmd_cpu'] = cpu
            except Exception as e:
                info['cmd_cpu_err'] = repr(e)
        if a.rt_prio > 0:
            try:
                os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(a.rt_prio))
                info['cmd_sched'] = f'SCHED_FIFO {a.rt_prio}'
            except Exception as e:
                info['cmd_sched_err'] = repr(e)
        self.event('RT', f'command thread: {info}')

    def cmd_loop(self):
        a = self.a
        self._pin_cmd_thread()
        period = 1.0 / a.cmd_hz
        hb_lim = (a.max_overrun + 0.5) * self.cfg.control_dt
        dl = time.monotonic()
        W = self.watcher
        win_t, win_rx, win_sent, calib, w_dead_noted = (dl, W.count if W else 0, self.n_sent, [], False)
        win_rx_t = W.t if W else 0.0
        while not self.cmd_stop.is_set():
            dl += period
            try:
                sig = self._poll_signal()
                if sig is not None and self.state != 'DAMP':
                    try:
                        name = signal.Signals(sig).name
                    except ValueError:
                        name = str(sig)
                    self.damp(f'signal {name}')
                if self.damp_req is not None and self.state != 'DAMP':
                    self.damp(self.damp_req)
                now = time.monotonic()
                st = self.state
                if st != 'DAMP':
                    age = self.state_age(now)
                    if age > a.stale_ms * 0.001:
                        self.damp(f'stale lowstate: {age * 1000.0:.1f} ms since last new state (limit {a.stale_ms} ms)')
                    elif st in HEARTBEAT_STATES and now - self.t_main_cmd > hb_lim:
                        self.damp(f'main loop stalled: no new command for {(now - self.t_main_cmd) * 1000.0:.1f} ms in {st} (limit {hb_lim * 1000.0:.0f} ms)')
                if W is not None:
                    W.poll()
                    if W.dead and (not w_dead_noted):
                        w_dead_noted = True
                        self.event('WARN', 'rt/lowcmd watcher process died: foreign-publisher watch OFF for the rest of the run')
                if W is not None and (not W.dead) and (now - win_t >= 1.0):
                    d_rx, d_sent = (W.count - win_rx, self.n_sent - win_sent)
                    rx_dt, tx_dt = (W.t - win_rx_t, now - win_t)
                    win_t, win_rx, win_sent, win_rx_t = (now, W.count, self.n_sent, W.t)
                    if self.self_delivery is None:
                        calib.append(d_rx / max(d_sent, 1))
                        if len(calib) >= 2:
                            self.self_delivery = 1.0 if max(calib) > 0.5 else 0.0
                            self.event('INFO', f'rt/lowcmd self-delivery ratio {max(calib):.2f} -> {self.self_delivery:.0f} (foreign-publisher watch armed)')
                    elif rx_dt > 0.5:
                        rx_rate, tx_rate = (d_rx / rx_dt, d_sent / max(tx_dt, 0.001))
                        thr = tx_rate * 1.3 + 10 if self.self_delivery > 0 else 10
                        if rx_rate > thr and self.state != 'DAMP':
                            self.damp(f'foreign rt/lowcmd publisher: {rx_rate:.0f} samples/s received vs {tx_rate:.0f}/s own sends')
                self.send()
                if self.write_fail_consec >= a.max_write_fail and self.state != 'DAMP':
                    self.damp(f'{self.write_fail_consec} consecutive rt/lowcmd write failures ({self.write_last_err})')
            except Exception as e:
                self.n_cmd_exc += 1
                self.event('ERROR', f'command thread exception {e!r}: {traceback.format_exc(limit=3)}')
                try:
                    self.damp(f'command thread exception {e!r}')
                except Exception:
                    pass
            rem = dl - time.monotonic()
            if rem > 0:
                time.sleep(rem)
            elif rem < -10 * period:
                dl = time.monotonic()

    def foreign_lowcmd(self, secs: float) -> int:
        n = self.watcher.count_for(secs)
        self.watcher.poll()
        if self.watcher.dead:
            raise SystemExit('rt/lowcmd watcher process died; cannot check for another rt/lowcmd publisher')
        return n

    def check_mode(self, when: str):
        a = self.a
        if not (a.domain == 0 or a.sim_try_check_mode):
            return
        try:
            code, name, raw = gc.check_motion_mode(a.check_mode_timeout)
        except Exception as e:
            code, name, raw = (-1, None, repr(e))
        print(f'[startup] MotionSwitcher CheckMode ({when}): code={code} result={raw}', flush=True)
        if name:
            raise SystemExit(f"Unitree motion mode '{name}' is ACTIVE (MotionSwitcher CheckMode). Enter debug mode / release the motion mode on the robot (this program never calls ReleaseMode) and retry.")
        if code != 0:
            msg = f'MotionSwitcher CheckMode did not answer (code {code}); cannot confirm the built-in controller is off (UNVERIFIED whether the service is visible from this PC). Relying on the rt/lowcmd check.'
            if a.require_check_mode:
                raise SystemExit(msg + ' --require-check-mode is set.')
            print(f'[startup] WARNING {msg}', flush=True)

    def startup(self):
        # 启动顺序：建立通信 -> 接收状态 -> 排查其他控制器 -> 确认并复查 -> 启动命令发送。
        # 前面的检查阶段只接收状态和查询模式，直到末尾启动命令线程才开始发送 lowcmd。
        a = self.a
        # 在指定 DDS 域和网卡上初始化通信，使用 H1 对应的 GO LowCmd/LowState 消息类型。
        gc.init_dds(a.domain, a.iface)
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC
        self.crc = CRC()
        # 真机默认校验收到的状态 CRC，校验失败由 on_state 丢弃；发送命令也使用 self.crc。
        self.crc_check = a.domain == 0 and (not a.no_state_crc)
        # 独立子进程监听 lowcmd，避免监视回调与本进程发送线程共享 GIL 引发死锁。
        self.watcher = LowcmdWatcher(a.domain, a.iface)
        # DDS 回调 on_state 更新最新状态 self.msg、接收计数和手柄事件。
        self.sub = ChannelSubscriber('rt/lowstate', LowState_)
        self.sub.Init(self.on_state, 0)
        # 等待第一条被回调接受的状态；超时抛出 SystemExit，由 main() 报告启动失败。
        t_end = time.monotonic() + a.wait_state_s
        while self.msg is None and time.monotonic() < t_end:
            time.sleep(0.01)
        if self.msg is None:
            raise SystemExit(f'no rt/lowstate within {a.wait_state_s} s on iface={a.iface} domain={a.domain} (CRC failures: {self.n_crc_bad})')
        # 约 1 秒内的接收增量用于估算状态频率；extract_state 按策略关节顺序提取状态。
        n0 = self.n_rx
        time.sleep(1.0)
        rate = self.n_rx - n0
        s = gc.extract_state(self.msg)
        print(f'[startup] lowstate rate {rate} Hz, reserved_mode={s.reserved_mode} reserved_control={s.reserved_control} tick={s.tick} CRC bad={self.n_crc_bad}', flush=True)
        self.mm_latched = 0

        # 此时自身尚未发布命令，监听到 lowcmd 就说明存在其他发布者，必须拒绝启动。
        if not self.watcher.wait_ready(10.0):
            raise SystemExit('rt/lowcmd watcher process did not start (cannot check for other controllers)')
        n = self.foreign_lowcmd(a.lowcmd_check_s)
        if n:
            raise SystemExit(f'{n} rt/lowcmd messages from ANOTHER publisher in {a.lowcmd_check_s} s (sport/locomotion service or another controller is active). Put the robot in debug mode (see docs/SIM2REAL.md#h1-model-ai-mode-and-debug-mode) and retry.')
        print(f'[startup] no other rt/lowcmd publisher seen in {a.lowcmd_check_s} s', flush=True)
        # 真机查询内置运动模式；仿真仅在显式要求时查询。这里只检查，不主动释放控制器。
        # 查到活动模式会拒绝启动；查询失败是否拒绝由 --require-check-mode 决定。
        self.check_mode('before confirmation')
        if self.crc_check and self.n_crc_bad:
            print(f'[startup] WARNING {self.n_crc_bad} lowstate CRC failures so far (those messages are dropped)')
        # 检查参与控制关节的最高温度；超过 max-temp 时停止启动流程。
        tmax = s.temp[self.live].max()
        if tmax > a.max_temp:
            raise SystemExit(f'motor temperature {tmax:.0f} C > --max-temp {a.max_temp}')
        # 用匹配模型输入维度的零数据预热 3 次，减少首次正式推理的初始化开销。
        # 这里直接调用 policy_fn，输出不用于控制，也不计入 counted 包装器的推理次数。
        in_shapes = self.policy_fn.input_shapes
        for _ in range(3):
            self.policy_fn(np.zeros(in_shapes[0]), np.zeros(in_shapes[1]))
        # 准备启动摘要：CPU 调频策略、机体坐标系重力方向，以及第一个动作的起始姿态。
        gov = _governor()
        g = projected_gravity(s.quat / max(np.linalg.norm(s.quat), 1e-09))
        f0 = self.f0s[0]
        # 统计各动作首帧有多少关节被软限位裁剪；实际裁剪已在构造 Runner 时完成。
        n_soft = [int(np.count_nonzero((np.abs(f - cl.joint_pos[0]) > 1e-09) & self.live)) for f, cl in zip(self.f0s, self.clips)]
        # 打印模型/动作信息、控制频率、安全阈值和 PD 增益，便于确认本次部署配置。
        print('=' * 80)
        print(f'artifacts : {a.artifacts}')
        print(f"clips     : {[c.name + f' ({c.num_frames} fr, {c.num_frames * self.cfg.control_dt:.1f} s)' for c in self.clips]}")
        print(f'start pose: clip frame 0 clipped to soft limits ({n_soft} joints clipped per clip)')
        for cl, (v, jn, fr, v0) in zip(self.clips, self.ref_speed):
            print(f'ref speed : {cl.name}: max reference joint speed over frames 1-25 {v:.2f} rad/s ({jn}, frame {fr}); frame 0 {v0:.2f} rad/s (info only)')
        print(f"iface/dom : {a.iface} / {a.domain}  {('SIM' if a.sim else 'REAL ROBOT')}  auto-sequence={a.auto_sequence}")
        print(f'control   : policy {1 / self.cfg.control_dt:.0f} Hz, lowcmd {a.cmd_hz:.0f} Hz, GO protocol, H1 motor mapping, ORT threads {a.ort_threads}')
        if isinstance(self.pose_receiver, SimPoseReceiver):
            print('pose      : MuJoCo pelvis, fixed XY/yaw alignment on each STANDBY_PRE first read; actual height preserved; kept through Start')
        print(f'STAGE     : {a.stage.upper()}  -> max_vel_frac {self.safety.max_vel_frac}, max_abs_action {self.safety.max_abs_action}, torque_clamp_factor {self.safety.torque_clamp_factor} x effort, clamp_soft_limits {self.safety.clamp_soft_limits}, core tilt err {np.degrees(self.safety.max_tilt_err):.0f} deg, hold tilt {np.degrees(self.max_tilt_hold):.0f} deg')
        if a.no_policy:
            print('NO-POLICY : --no-policy -> policy takeover is ALWAYS refused; this run only does PASSIVE / INTERP / HOLD / DAMP')
        print(f'stand kp  : {self.stand_kp.tolist()}')
        print(f'stand kd  : {self.stand_kd.tolist()}')
        print(f'takeover  : {a.policy_blend_s:g} s target blend in STANDBY_PRE; Start waits for completion and fresh stability/temperature samples')
        if a.sim and a.sim_band_file:
            print(f'sim band  : {a.sim_band_ramp_s:g} s smooth release after valid policy submission; readiness samples require full policy and zero support')
        # 说明动作结束后的行为：策略末帧待机，或旧模式下的固定 PD 保持。
        if self.motion_selection is not None:
            print('clip end  : STANDBY_POST; a queued selection starts the supported PD transition, otherwise wait for a motion key')
        elif a.end_hold_legacy:
            print(f"clip end  : {''}--end-hold-legacy -> END_HOLD, {a.end_blend_s} s blend to stand gains (CANNOT balance: hoist must carry)")
        else:
            print(f'clip end  : STANDBY_POST (policy keeps standing in the last-frame pose); L2+up TWICE within {a.confirm_s} s -> {a.end_blend_s} s blend to stand gains -> INTERP to the next clip (hoist must carry)')
        print(f'safety    : stale>{a.stale_ms} ms, overrun>{a.max_overrun} cycles, main-loop heartbeat>{(a.max_overrun + 0.5) * 20:.0f} ms, temp>{a.max_temp} C DAMP / >{a.warn_temp} C no start (UNVERIFIED thresholds), GO lost-count threshold 0x{a.max_motor_lost or 0:x}')
        # 分别展示策略接管和动作播放的门槛；当前仅打印，实际判断在运行状态机中执行。
        takeover = self.policy_button
        print(f'{takeover} req  : HOLD>={a.hold_min_s} s, max|q-f0|<{a.hold_tol} rad, max|dq|<{a.start_max_dq} rad/s, tilt vs clip<{a.start_max_tilt_deg} deg, temp<{a.warn_temp} C  -> STANDBY_PRE (policy stands still)')
        print(f'{self.play_button} req  : after blend, full-policy standby>={a.standby_min_s} s; over the last {a.standby_window_s} s tilt err<{a.standby_max_tilt_deg} deg, max|dq|<{a.standby_max_dq} rad/s; temp<{a.warn_temp} C  -> POLICY (motion)')
        print(f'standby   : status every {a.status_s} s; WARN |yaw change|>{a.warn_yaw_deg} deg, leg |dq|>{a.warn_leg_dq} rad/s for >{a.warn_leg_dq_s} s, temp>{a.warn_temp} C, standby>{a.standby_warn_s:.0f} s (documented cap)')
        print(f"cpu       : governor {gov}; command thread affinity '{a.cmd_cpu}', rt-prio {a.rt_prio}")
        print(f'robot now : proj_grav={np.round(g, 3)}  max|q - f0|={np.abs(s.q - f0)[self.live].max():.3f} rad  max temp={tmax:.0f} C' + '')
        # 根据是否启用 motion-select，展示对应的选动作、准备、播放和阻尼按键。
        if self.motion_selection is not None:
            print(f'buttons   : {self.motion_selection.button_summary(a.clip)} | R1+B = policy takeover from HOLD | Start = play when READY | L2+B = DAMP')
            print('            selecting starts supported PD preparation; during POLICY the selection waits for the current motion to finish')
            print('            HOLD stays in fixed PD until R1+B passes the takeover checks; early/refused requests are discarded')
            print('            release and press again to retry; R1+B with extra buttons is blocked until both are released; L2+B has priority')
            print('            standby checks then run automatically; Start plays when READY; the hoist must carry during fixed PD')
        else:
            print('buttons   : L2+up = interpolate to f0 (PASSIVE; END_HOLD -> next clip); in STANDBY_PRE (-> HOLD, same clip) and')
            print(f'            STANDBY_POST (-> next clip) it must be pressed TWICE within {a.confirm_s} s (stand gains: hoist must carry) |')
            print('            R1+A = policy STANDBY (from HOLD) | R1+X = start the MOTION (from STANDBY_PRE) | L2+B = DAMP (any state)')
        print('            (buttons pressed before the command thread starts are ignored; release and press again)')
        print('=' * 80, flush=True)
        # warn-temp 只在此提示后续策略接管会被拒绝；它与直接终止启动的 max-temp 不同。
        if tmax > a.warn_temp:
            print(f'[startup] WARNING max motor temperature {tmax:.0f} C > --warn-temp {a.warn_temp}: policy takeover will be refused')

        # 真机必须输入 yes 才允许发送命令；仿真默认跳过，sim-ask-yes 可测试同样的流程。
        if not a.sim or a.sim_ask_yes:
            ans = input("Robot on the hoist / area clear / L2+B tested this session? Type 'yes' to start sending rt/lowcmd (PASSIVE damping): ")
            if ans.strip() != 'yes':
                raise SystemExit('not confirmed')
            # 等待人工输入期间环境可能变化，因此再次检查其他发布者、运动模式和最新温度。
            n = self.foreign_lowcmd(a.lowcmd_recheck_s)
            if n:
                raise SystemExit(f"after 'yes': {n} rt/lowcmd messages from ANOTHER publisher in {a.lowcmd_recheck_s} s")
            self.check_mode('after confirmation')
            s2 = gc.extract_state(self.msg)
            if s2.temp[self.live].max() > a.max_temp:
                raise SystemExit(f"after 'yes': motor temperature {s2.temp[self.live].max():.0f} C > --max-temp {a.max_temp}")
            # 同时确认状态仍在更新；stale-ms 从毫秒换算成秒后与状态年龄比较。
            if self.state_age() > a.stale_ms * 0.001:
                raise SystemExit(f"after 'yes': lowstate stale ({self.state_age() * 1000.0:.0f} ms)")
            print(f"[startup] re-check after 'yes' OK (no foreign rt/lowcmd in {a.lowcmd_recheck_s} s)", flush=True)
        # --no-lowcmd-watch 只关闭运行期监视，上面的启动冲突检查仍然执行。
        if not a.lowcmd_watch:
            self.watcher.close()
            self.watcher = None

        # 检查通过后才创建命令发布器、初始化消息模板和发送线程；线程此时尚未 start。
        self.pub = ChannelPublisher('rt/lowcmd', LowCmd_)
        self.pub.Init()
        self.lowcmd = self.build_cmd()
        self.cmd_thread = threading.Thread(target=self.cmd_loop, name='lowcmd', daemon=True)
        # 清空旧按键事件并开始接收新事件，避免启动前的按键触发后续动作。
        self.pending.clear()
        self.accept_buttons = True
        if self.sig_r is not None:
            self._poll_signal()
        # 启动发送前回收垃圾，再冻结已有受 GC 跟踪对象，减少后续循环垃圾回收的扫描开销。
        pygc.collect()
        pygc.freeze()
        # 重置状态起始时间和主循环心跳，避免把启动检查的耗时算进运行期超时判断。
        self.state_t = time.monotonic()
        self.t_main_cmd = time.monotonic()
        # 标记已启动后开启发送线程；初始 PASSIVE 使用 kp=0、kd=DAMP_KD 的阻尼命令。
        # startup 返回后 main() 才调用 run()，后者负责状态切换及更新线程发送的控制目标。
        self.cmd_started = True
        self.cmd_thread.start()
        self.event('STATE', f'start in PASSIVE (stage {a.stage})')
        # 仿真器通过标志文件判断是否启用弹力带支撑；创建文件使启动阶段保持支撑。
        if a.sim and a.sim_band_file:
            self.sim_band_on(a.sim_band_file)
    # 控制逻辑
    def run(self):
        # 主控制循环：按 control_dt 读取状态、检查保护、处理事件、更新控制目标并记录数据。
        # 持续高频发送由 cmd_loop 线程负责；本循环在策略更新后也会立即发送一次命令。
        a = self.a
        c = self.cfg
        dt = c.control_dt
        dl = time.monotonic()
        self.band_off_t = None
        self.band_release_t = None
        interp_from = None
        self.end = None
        auto_damp_t = None
        stalled = False
        # END_HOLD 的比例力矩裁剪预算，最多取配置 effort_limit 的 1 倍。
        eff_lim = min(1.0, self.safety.torque_clamp_factor) * c.effort_limit
        while not self.stop.is_set():
            # 按累计截止时间调度，避免把每轮计算耗时累加到控制周期中。
            dl += dt
            rem = dl - time.monotonic()
            if rem > 0:
                time.sleep(rem)
            if self.stop.is_set():
                break
            t_start = time.monotonic()
            late = t_start - dl
            # 主循环落后超过允许周期数时切入阻尼。
            if late > a.max_overrun * dt and self.state not in ('DAMP',):
                self.damp(f'policy loop overrun: {late * 1000.0:.1f} ms late (> {a.max_overrun} cycles)')
            # 仿真故障注入：故意阻塞一次主循环，测试独立命令线程的心跳保护。
            if a.sim_stall_at_s is not None and (not stalled) and (self.state == 'POLICY') and (t_start - self.state_t >= a.sim_stall_at_s):
                stalled = True
                self.event('SIM', f'injecting a {a.sim_stall_s} s main-loop stall in POLICY')
                time.sleep(a.sim_stall_s)
                dl = time.monotonic()
                continue
            # 提取最新机器人状态、当前动作与首帧目标，并初始化本周期日志占位值。
            msg = self.msg
            st = gc.extract_state(msg)
            age = self.state_age(t_start)
            state = self.state
            t_in = t_start - self.state_t
            clip = self.clips[self.clip_idx]
            f0 = self.f0s[self.clip_idx]
            action = np.zeros(N)
            q_t = np.full(N, np.nan)
            n_cl = 0
            obs = None
            frame = -1
            tilt_err = np.nan
            root_ref_pos, root_pos_error = np.full(3, np.nan), np.full(3, np.nan)
            root_ref_frame, root_tracking_state = -1, -1
            root_ref_time_s = np.nan
            policy_q_target = np.full(N, np.nan)
            blend_alpha = np.nan
            blend_clamped = 0
            # 通用保护：过温、可选丢包计数、非有限状态和异常四元数；固定 PD 状态另查绝对倾角。
            if state != 'DAMP':
                merr = np.where(self.live, np.zeros(N, dtype=int) if a.max_motor_lost is None else np.maximum(st.motorstate - a.max_motor_lost, 0), 0)
                temp = np.where(self.live[:, None], st.temp, -np.inf)
                if temp.max() > a.max_temp:
                    j = int(np.unravel_index(np.argmax(temp), temp.shape)[0])
                    self.damp(f'motor temperature {temp.max():.0f} C on {c.joint_names[j]} > {a.max_temp} C')
                elif a.domain == 0 and np.any(merr != 0):
                    j = int(np.argmax(merr != 0))
                    self.damp(f'motor lost count {int(st.motorstate[j])} exceeds {a.max_motor_lost} on {c.joint_names[j]}')
                elif not np.all(np.isfinite(st.q)) or not np.all(np.isfinite(st.quat)) or (not np.all(np.isfinite(st.gyro))):
                    self.damp('non-finite lowstate')
                elif abs(float(np.linalg.norm(st.quat)) - 1.0) > 0.05:
                    self.damp(f'bad IMU quaternion (norm {float(np.linalg.norm(st.quat)):.3f})')
                elif state in ('INTERP', 'HOLD', 'END_HOLD'):
                    tilt = _tilt(st.quat)
                    if tilt > self.max_tilt_hold:
                        self.damp(f'tilt {np.degrees(tilt):.0f} deg in {state} > {np.degrees(self.max_tilt_hold):.0f} deg')
            # 保护检查可能已改变状态；取出按键事件时丢弃过期或不属于当前状态的接管请求。
            state = self.state
            evs = []
            while self.pending:
                name, t_ev = self.pending.popleft()
                if t_start - t_ev > a.button_max_age_s:
                    self.event('INFO', f'dropped stale button event {name} ({(t_start - t_ev) * 1000.0:.0f} ms old)')
                elif self.motion_selection is not None and name == 'policy' and t_ev < self.state_t:
                    self.event('INFO', 'R1+B from the previous state discarded; release and press again in HOLD')
                else:
                    evs.append(name)
            # 自动仿真产生等价操作事件，动作选择模式再处理选曲与准备流程。
            if a.auto_sequence and state != 'DAMP':
                evs += self.auto_events(state, t_in, st)
            evs = self.motion_events(state, t_in, st, evs)
            for ev in evs:
                if ev == 'select_motion':
                    # 应用已选动作，更新插值起点、当前动作及首帧目标。
                    origin = self.begin_selected_motion(st)
                    if origin is not None:
                        interp_from = origin
                    state, t_in = (self.state, 0.0)
                    clip, f0 = (self.clips[self.clip_idx], self.f0s[self.clip_idx])
                    continue
                elif ev == 'interp':
                    # 插值请求：从允许的状态准备首帧姿态；从策略待机退出需二次确认并先过渡到固定 PD。
                    if state not in ('PASSIVE', 'END_HOLD', 'STANDBY_PRE', 'STANDBY_POST'):
                        self.event('INFO', f'L2+up ignored in {state}')
                        continue
                    if state in ('END_HOLD', 'STANDBY_POST') and self.clip_idx + 1 >= len(self.clips) and (not (state == 'END_HOLD' and self.end is not None and self.end.get('then_hold'))):
                        self.event('INFO', f'L2+up in {state}: no next clip (all clips played)' + ('; the policy keeps standing: re-hoist (gantry over the robot, take up slack only), then L2+B' if state == 'STANDBY_POST' else ''))
                        continue
                    if state in STANDBY_STATES:
                        if not self.confirmed('interp', t_start, f"press L2+up again within {a.confirm_s:.0f} s: is the hoist carrying the robot? ({('back to HOLD of ' + clip.name if state == 'STANDBY_PRE' else 'next clip ' + self.clips[self.clip_idx + 1].name)}; stand gains from the 2nd press on: the robot CANNOT balance by itself)"):
                            continue
                        pre = state == 'STANDBY_PRE'
                        self.end = dict(q0=(self.last_cmd_q if self.last_cmd_q is not None else st.q).copy(), f_end=f0 if pre else self.fends[self.clip_idx], then_interp=not pre, then_hold=pre)
                        self.set_state('END_HOLD', f'L2+up x2 from {state}: {a.end_blend_s} s blend to stand gains, then ' + (f'HOLD of {clip.name} (same clip)' if pre else f'INTERP to f0 of {self.clips[self.clip_idx + 1].name}'))
                        self.event('WARN', 'stand gains from now on: the robot CANNOT balance by itself -> the HOIST MUST CARRY IT')
                        state, t_in = ('END_HOLD', 0.0)
                        continue
                    if state == 'END_HOLD':
                        if self.end is not None and (self.end.get('then_interp') or self.end.get('then_hold')):
                            self.event('INFO', 'L2+up in END_HOLD ignored: ' + ('HOLD of the same clip' if self.end.get('then_hold') else 'INTERP to the next clip') + ' follows the blend automatically')
                            continue
                        if t_in < a.end_blend_s:
                            self.event('INFO', f'L2+up in END_HOLD ignored: gain blend still running ({t_in:.2f} < {a.end_blend_s} s)')
                            continue
                        self.clip_idx += 1
                        clip, f0 = (self.clips[self.clip_idx], self.f0s[self.clip_idx])
                        interp_from = self.last_cmd_q.copy() if self.last_cmd_q is not None else st.q.copy()
                    else:
                        interp_from = st.q.copy()
                    self.set_state('INTERP', f'to f0 of {clip.name}')
                    state, t_in = ('INTERP', 0.0)
                elif ev == 'policy':
                    # HOLD 满足接管条件后创建策略核心，以首帧静态参考进入 STANDBY_PRE，并启动目标混合。
                    if state == 'STANDBY_PRE':
                        self.event('INFO', f'{self.policy_button} ignored in STANDBY_PRE (the policy is already standing): press {self.play_button} to start the motion')
                        continue
                    if state != 'HOLD':
                        self.event('INFO', f'{self.policy_button} ignored in {state}; release and press again in HOLD')
                        continue
                    ok, why = self.policy_allowed(t_in, st, f0, clip)
                    if not ok:
                        self.event('REFUSED', f'{self.policy_button} (policy standby): {why}; release and press again to retry')
                        continue
                    self.core = DeployCore(c, self.policy, self.static_pre[self.clip_idx], self.safety)
                    self.core.reset(0)
                    self.policy_takeover = PolicyTakeover(self.last_cmd_q if self.last_cmd_q is not None else f0, a.policy_blend_s)
                    if isinstance(self.pose_receiver, SimPoseReceiver):
                        self.pose_receiver.reset_reference(clip)
                    self.sb_win.clear()
                    self.set_state('STANDBY_PRE', f'policy standing still at frame 0 of {clip.name}; {why}; crew: feet carry the weight, rope just taut (only give the slack the clip needs), gantry ready, then {self.play_button} (>= {a.standby_min_s:.0f} s, steady {a.standby_window_s:.0f} s)')
                    self.event('BLEND', f'fixed PD -> policy target over {a.policy_blend_s:g} s; readiness windows start after full-policy submission')
                    state, t_in = ('STANDBY_PRE', 0.0)
                elif ev == 'start':
                    # 策略待机通过稳定性检查后，切回完整动作并进入 POLICY；过早的播放请求不排队。
                    if state != 'STANDBY_PRE':
                        hint = f' (press {self.policy_button} first: policy standby)' if state == 'HOLD' else ''
                        self.event('INFO', f'{self.play_button} ignored in {state}{hint}; playback is not queued')
                        continue
                    ok, why = self.standby_allowed(t_in, st)
                    if not ok:
                        self.event('REFUSED', f'{self.play_button} (motion start): {why}')
                        continue
                    self.core.switch_clip(clip, 0)
                    self.set_state('POLICY', f'motion {clip.name} from frame 0; {why}')
                    state, t_in = ('POLICY', 0.0)
            t_c0 = time.monotonic()
            if state == 'PASSIVE':
                # 初始等待：kp=0、kd=DAMP_KD，仅提供速度阻尼。
                self.set_cmd(None, np.zeros(N), np.full(N, DAMP_KD), 'PASSIVE')
            elif state == 'INTERP':
                # 固定 PD 将目标从起始姿态平滑移动到动作首帧，到时进入 HOLD。
                q_t = interpolate_pose(interp_from, f0, t_in, a.interp_s)
                q_t = np.clip(q_t, np.minimum(c.soft_lower, interp_from), np.maximum(c.soft_upper, interp_from))
                self.set_cmd(q_t, self.stand_kp, self.stand_kd, 'INTERP')
                if t_in >= a.interp_s:
                    self.set_state('HOLD', f'max|q-f0|={np.abs(st.q - f0)[self.live].max():.3f}')
            elif state == 'HOLD':
                # 保持首帧关节目标，等待策略接管请求；此状态不执行策略推理。
                q_t = f0.copy()
                self.set_cmd(q_t, self.stand_kp, self.stand_kd, 'HOLD')
            elif state in POLICY_STATES:
                # STANDBY_PRE/POLICY/STANDBY_POST 都持续推理；只有 POLICY 推进动作参考帧。
                core = self.core
                tilt_err = _tilt_vs(st.quat, core.clip.proj_grav[min(core.frame, core.clip.num_frames - 1)])
                root_pos, pelvis_quat = (None, st.quat)
                if c.needs_root_pose:
                    # 需要外部骨盆反馈的模型先读取位姿，数据失效时立即转阻尼。
                    try:
                        root_pos, pelvis_quat = self.pose_receiver.read()
                        if isinstance(self.pose_receiver, SimPoseReceiver) and self.pose_receiver.alignment_event:
                            self.event('POSE_ALIGN', f'{clip.name}: {self.pose_receiver.alignment_event}')
                            self.pose_receiver.alignment_event = None
                        self.latest_anchor_pose = (root_pos.copy(), pelvis_quat.copy())
                    except ValueError as error:
                        self.damp(f'anchor feedback unavailable: {error}')
                        continue
                # Save the exact reference before step() advances/switches clips.
                root_ref_frame = min(core.frame, core.clip.num_frames - 1)
                root_ref_time_s = root_ref_frame * c.control_dt
                root_tracking_state = SID[state]
                root_ref_pos, root_pos_error = pelvis_tracking(core.clip, root_ref_frame, root_pos)
                r = core.step(st.q, st.dq, st.gyro, pelvis_quat, advance=state == 'POLICY', root_pos=root_pos)
                action, q_t, n_cl, frame = (r.action, r.q_target, r.n_clamped, r.frame)
                policy_q_target = q_t.copy()
                if state == 'STANDBY_PRE' and self.policy_takeover is not None and not r.abort:
                    # 接管初期把固定 HOLD 目标逐渐混合到策略目标，混合结果仍需满足裁剪约束。
                    q_t, blend_clamped = self.policy_takeover.target(q_t, t_in, st.q, st.dq, c, self.safety)
                    blend_alpha = self.policy_takeover.alpha
                if state == 'STANDBY_PRE' and self.standby_feedback_ready():
                    # 完全接管后收集倾角和速度的滑动窗口，用于判断是否允许播放动作。
                    self.sb_win.append((t_start, tilt_err, float(np.abs(st.dq)[self.live].max())))
                    while self.sb_win and t_start - self.sb_win[0][0] > a.standby_window_s:
                        self.sb_win.popleft()
                if state in STANDBY_STATES:
                    self.standby_status(state, t_in, t_start, st, tilt_err, age, root_pos_error)
                elif t_start - self.pose_status_t >= a.status_s:
                    self.pose_status_t = t_start
                    self.printer.q.put(f'[{t_start - self.t0:8.3f}] STATUS: POLICY {t_in:6.1f} s '
                                       f'ref frame {root_ref_frame}: {pelvis_error_text(root_pos_error)}\n')
                if a.log_obs and core.hist is not None:
                    obs = core.hist.copy()
                # 策略核心报告异常则切阻尼；否则安装新目标和策略 PD 增益，并立即发送。
                if r.abort:
                    self.damp(f'DeployCore abort in {state} at frame {r.frame}: {r.abort}')
                elif self.set_cmd(q_t, c.kp, c.kd, state):
                    sent = self.send()
                    if sent and state == 'STANDBY_PRE' and self.policy_takeover is not None and self.policy_takeover.submitted(time.monotonic()):
                        self.sb_win.clear()
                        self.event('BLEND', 'complete; full policy control; readiness samples require zero sim support' if a.sim_band_file else 'complete; full policy control; collecting fresh readiness samples')
                    # 仿真支撑仅在有效推理和命令提交成功后，才尝试按配置释放。
                    if sent and state == 'STANDBY_PRE' and t_in >= max(0.0, a.sim_band_release_standby_s):
                        self.release_band('after valid policy inference and command submission')
                    if state == 'POLICY' and r.done:
                        # 动作结束：旧模式转固定 PD；默认改用末帧静态参考，进入策略待机。
                        if a.end_hold_legacy:
                            self.end = dict(q0=q_t.copy(), f_end=self.fends[self.clip_idx], then_interp=False)
                            self.set_state('END_HOLD', f'clip {clip.name} done at frame {r.frame}; blending gains over {a.end_blend_s} s (--end-hold-legacy)')
                        else:
                            # STANDBY_POST 固定的是参考姿态，策略仍根据反馈不断调整关节目标。
                            core.switch_clip(self.static_post[self.clip_idx], 0)
                            next_hint = '; queued selection will start supported PD preparation, or choose a motion' if self.motion_selection is not None else ('; L2+up = next clip' if self.clip_idx + 1 < len(self.clips) else '')
                            self.set_state('STANDBY_POST', f'clip {clip.name} done at frame {r.frame}; policy keeps standing in the last-frame pose (re-hoist before L2+B' + next_hint + ')')
            elif state == 'END_HOLD':
                # 平滑混合策略增益与固定 PD 增益，同时过渡目标；完成后按标记保持或准备下一动作。
                T = a.end_blend_s
                s = 1.0 if T <= 0 else float(np.clip(t_in / T, 0.0, 1.0))
                kp = (1.0 - s) * c.kp + s * self.stand_kp
                kd = (1.0 - s) * c.kd + s * self.stand_kd
                q_t = self.end['f_end'].copy() if T <= 0 else interpolate_pose(self.end['q0'], self.end['f_end'], t_in, T)
                # 将目标限制在当前角度 ± eff_lim/kp 内，这里约束的是位置误差产生的比例力矩。
                q_t = np.clip(q_t, st.q - eff_lim / kp, st.q + eff_lim / kp)
                if self.set_cmd(q_t, kp, kd, 'END_HOLD') and t_in >= T:
                    if self.end.get('then_interp'):
                        self.clip_idx = self.end.get('target_idx', self.clip_idx + 1)
                        interp_from = q_t.copy()
                        self.set_state('INTERP', f'to f0 of {self.clips[self.clip_idx].name} (after the STANDBY_POST blend)')
                    elif self.end.get('then_hold'):
                        if float(np.abs(q_t - f0)[self.live].max()) < 1e-06:
                            self.set_state('HOLD', f'{clip.name} (after the STANDBY_PRE exit blend), max|q-f0|={np.abs(st.q - f0)[self.live].max():.3f}')
                        else:
                            interp_from = q_t.copy()
                            self.set_state('INTERP', f'to f0 of {clip.name} (after the STANDBY_PRE exit blend; target was clamped)')
            elif state == 'DAMP':
                # 阻尼命令由发送线程持续执行；仿真可在配置的等待时间后退出主循环。
                if auto_damp_t is None:
                    auto_damp_t = t_start
                if a.sim and a.sim_exit_after_damp is not None and (t_start - auto_damp_t >= a.sim_exit_after_damp):
                    self.event('SIM', f'exit {a.sim_exit_after_damp} s after DAMP (--sim-exit-after-damp)')
                    self.stop.set()
            # 记录本周期的控制计算耗时、状态、观测、动作与目标；最终由 save() 写入文件。
            compute_ms = (time.monotonic() - t_c0) * 1000.0
            R = self.rec
            R['t'].append(t_start - self.t0)
            R['deadline'].append(dl - self.t0)
            R['compute_ms'].append(compute_ms)
            R['state'].append(SID[self.state])
            R['clip'].append(self.clip_idx)
            R['frame'].append(frame)
            R.setdefault('root_ref_pos', []).append(root_ref_pos)
            R.setdefault('root_pos_error', []).append(root_pos_error)
            R.setdefault('root_ref_frame', []).append(root_ref_frame)
            R.setdefault('root_ref_time_s', []).append(root_ref_time_s)
            R.setdefault('root_tracking_state', []).append(root_tracking_state)
            R['root_pos'].append(self.latest_anchor_pose[0] if state in POLICY_STATES and self.latest_anchor_pose is not None else np.full(3, np.nan))
            R['anchor_quat'].append(self.latest_anchor_pose[1] if state in POLICY_STATES and self.latest_anchor_pose is not None else np.full(4, np.nan))
            R['q'].append(st.q)
            R['dq'].append(st.dq)
            R['gyro'].append(st.gyro)
            R['quat'].append(st.quat)
            R['tick'].append(st.tick)
            R['state_age_ms'].append(age * 1000.0)
            R['action'].append(action)
            R['q_target'].append(q_t)
            R.setdefault('policy_q_target', []).append(policy_q_target)
            R.setdefault('policy_blend_alpha', []).append(blend_alpha)
            R.setdefault('blend_n_clamped', []).append(blend_clamped)
            R['n_clamped'].append(n_cl)
            R['cmd_kp_mean'].append(float(np.mean(self.cmd[1])))
            R['tilt_err'].append(tilt_err)
            # 可选 UDP 输出状态和动作进度，供背景视频等外部程序同步。
            if self.sync_sock is not None:
                try:
                    self.sync_sock.sendto(f'{self.state} {self.clip_idx} {frame} {t_start - self.t0:.3f}'.encode(), self.sync_addr)
                except OSError:
                    pass
            # 完整历史观测体积较大，只在启用 log_obs 且未达到记录上限时保存。
            if a.log_obs:
                if len(R['t']) <= self.rec_cap:
                    R['obs_hist'].append(obs.reshape(-1).astype(np.float32) if obs is not None else np.zeros(c.input_shapes[0], np.float32))
                elif not self.rec_warned:
                    self.rec_warned = True
                    self.event('WARN', f'--rec-max-s {a.rec_max_s} reached: obs_hist logging stopped (other logs continue)')
            # 严重落后时重置调度基准，避免连续追赶大量已经错过的周期。
            if time.monotonic() - dl > dt * 10:
                dl = time.monotonic()

    def release_band(self, when: str):
        if not self.a.sim or not self.a.sim_band_file:
            return
        # Serialize with DAMP so a concurrent stop cannot be followed by release.
        with self.lock:
            if (self.state != 'STANDBY_PRE' or self.band_off_t is not None
                    or self.damp_req is not None or self.sig is not None or self.stop.is_set()):
                return
            now = time.monotonic()
            if self.band_release_t is None:
                write_band_command(self.a.sim_band_file, release_start=now,
                                   release_s=self.a.sim_band_ramp_s)
                self.band_release_t = now
                self.event('SIM', f'elastic band RELEASING over {self.a.sim_band_ramp_s:g} s ({when})')
            if now - self.band_release_t < self.a.sim_band_ramp_s:
                return
            try:
                os.remove(self.a.sim_band_file)
            except FileNotFoundError:
                pass
            self.band_off_t = now
            self.sb_win.clear()
            self.event('SIM', 'elastic band OFF (smooth release complete; collecting unsupported readiness samples after full policy takeover)')

    def sim_band_on(self, why: str):
        if not self.a.sim or not self.a.sim_band_file:
            return
        with self.lock:
            write_band_command(self.a.sim_band_file)
            self.band_release_t = None
            self.band_off_t = None
            self.event('SIM', f'elastic band ON ({why})')

    def standby_feedback_ready(self):
        return ((self.policy_takeover is None or self.policy_takeover.completed_at is not None)
                and (not (self.a.sim and self.a.sim_band_file) or self.band_off_t is not None))

    def standby_allowed(self, t_in: float, st) -> tuple[bool, str]:
        """R1+X: STANDBY_PRE long enough and stable (tilt error vs the static reference, joint speed) over the last window."""
        a = self.a
        if self.policy_takeover is not None:
            elapsed = self.policy_takeover.standby_elapsed(time.monotonic())
            if elapsed is None:
                return (False, f'policy target blend {100 * self.policy_takeover.alpha:.0f}%: wait for completion')
            t_in = elapsed
        if a.sim and a.sim_band_file:
            if self.band_off_t is None:
                return (False, 'sim elastic band still supporting / releasing; wait for zero support')
            unsupported_s = time.monotonic() - self.band_off_t
            if unsupported_s < a.sim_after_band_s:
                return (False, f'band fully released {unsupported_s:.2f} s ago < {a.sim_after_band_s} s')
            t_in = min(t_in, unsupported_s)
        if t_in < a.standby_min_s:
            return (False, f'full-policy standby for {t_in:.2f} s < {a.standby_min_s} s')
        w = list(self.sb_win)
        need = int(0.8 * a.standby_window_s / self.cfg.control_dt)
        if len(w) < max(need, 1):
            return (False, f'only {len(w)} standby samples in the last {a.standby_window_s} s (need {need})')
        te = np.array([x[1] for x in w])
        dqm = np.array([x[2] for x in w])
        if not np.all(np.isfinite(te)):
            return (False, 'non-finite tilt error in the standby window')
        te_max = float(np.degrees(te.max()))
        if te_max >= a.standby_max_tilt_deg:
            return (False, f'standby tilt error max {te_max:.1f} deg over the last {a.standby_window_s} s >= {a.standby_max_tilt_deg} deg')
        if float(dqm.max()) >= a.standby_max_dq:
            adq = np.where(self.live, np.abs(st.dq), 0.0)
            j = int(np.argmax(adq))
            return (False, f'standby max|dq| {dqm.max():.2f} rad/s over the last {a.standby_window_s} s >= {a.standby_max_dq} (now: {self.cfg.joint_names[j]} {adq[j]:.2f})')
        tm = float(st.temp[self.live].max())
        if tm > a.warn_temp:
            return (False, f'max motor temperature {tm:.0f} C > --warn-temp {a.warn_temp} (UNVERIFIED field semantics)')
        return (True, f'standby {t_in:.1f} s, last {a.standby_window_s} s: tilt err max {te_max:.1f} deg, max|dq| {dqm.max():.2f} rad/s')


    def confirmed(self, kind: str, t: float, prompt: str) -> bool:
        """Double-press confirmation: True on the 2nd press of `kind` within --confirm-s in the same state entry."""
        c = self.confirm
        if c is not None and c['kind'] == kind and (c['state_t'] == self.state_t) and (t - c['t'] <= self.a.confirm_s):
            self.confirm = None
            return True
        self.confirm = {'kind': kind, 't': t, 'state_t': self.state_t}
        self.event('CONFIRM', prompt)
        return False

    def standby_status(self, state: str, t_in: float, t: float, st, tilt_err: float, age: float, root_error=None):
        """STANDBY_*: status line every --status-s and edge-triggered WARNs. Only enqueues to the printer (never blocks)."""
        a = self.a
        s = self.sbs
        yaw = _yaw(st.quat)
        leg = float(np.abs(st.dq[LEG_IDX]).max())
        tmax = float(np.where(self.live[:, None], st.temp, -np.inf).max())
        if s is None or s['state_t'] != self.state_t:
            s = self.sbs = {'state_t': self.state_t, 'yaw_prev': yaw, 'dyaw': 0.0, 't_print': t, 'leg': collections.deque(), 'te': collections.deque(), 'leg_over_t': None, 'warned': {}, 'age_max': 0.0, 'dyaw_max': 0.0}
        d = yaw - s['yaw_prev']
        s['dyaw'] += (d + np.pi) % (2 * np.pi) - np.pi
        s['yaw_prev'] = yaw
        s['dyaw_max'] = max(s['dyaw_max'], abs(s['dyaw']))
        s['age_max'] = max(s['age_max'], age)
        s['leg'].append((t, leg))
        s['te'].append((t, tilt_err))
        while s['leg'] and t - s['leg'][0][0] > 3.0:
            s['leg'].popleft()
        while s['te'] and t - s['te'][0][0] > 3.0:
            s['te'].popleft()
        if leg > a.warn_leg_dq:
            s['leg_over_t'] = t if s['leg_over_t'] is None else s['leg_over_t']
        else:
            s['leg_over_t'] = None
        dyaw_deg = float(np.degrees(s['dyaw']))
        conds = {'yaw': (abs(dyaw_deg) > a.warn_yaw_deg, f'yaw changed {dyaw_deg:+.0f} deg since entering {state} (> {a.warn_yaw_deg:.0f}): the motion runs relative to the current heading -> re-aim the gantry or re-hoist'), 'leg': (s['leg_over_t'] is not None and t - s['leg_over_t'] > a.warn_leg_dq_s, f'leg |dq| > {a.warn_leg_dq} rad/s for > {a.warn_leg_dq_s} s (stepping?) -> re-hoist (gantry over the robot, take up slack only), then L2+B'), 'temp': (tmax > a.warn_temp, f'max motor temperature {tmax:.0f} C > --warn-temp {a.warn_temp} (DAMP at {a.max_temp})'), 'long': (t_in > a.standby_warn_s, f'{state} for {t_in:.0f} s > {a.standby_warn_s:.0f} s (documented standby cap: long unsupported standby is unmeasured on the robot) -> start the motion or re-hoist')}
        warn_now = []
        for k, (on, msg) in conds.items():
            if on:
                warn_now.append(k)
                if k not in s['warned']:
                    self.event('WARN', msg)
                s['warned'][k] = t
            elif k in s['warned'] and t - s['warned'][k] > a.status_s:
                del s['warned'][k]
        if t - s['t_print'] >= a.status_s:
            s['t_print'] = t
            te = np.array([x[1] for x in s['te']])
            te_s = f'{np.degrees(tilt_err):4.1f} (3 s max {np.degrees(np.nanmax(te)):4.1f})' if np.isfinite(te).any() else 'n/a'
            line = f"{state} {t_in:6.1f} s: yaw {dyaw_deg:+6.1f} deg, tilt err {te_s} deg, leg max|dq| 3 s {max((x[1] for x in s['leg'])):.2f} rad/s, temp max {tmax:.0f} C, lowstate age max {s['age_max'] * 1000.0:.1f} ms"
            if state == 'STANDBY_PRE':
                ok, why = self.standby_allowed(t_in, st)
                line += f'; {self.play_button}: ' + ('would be ACCEPTED' if ok else f'would be refused ({why})')
            if warn_now:
                line += '  WARN: ' + ','.join(warn_now)
            line += '; ' + pelvis_error_text(root_error)
            self.printer.q.put(f'[{t - self.t0:8.3f}] STATUS: {line}\n')

    def policy_allowed(self, t_in: float, st, f0, clip) -> tuple[bool, str]:
        a = self.a
        if a.no_policy:
            return (False, '--no-policy: this run only does INTERP/HOLD')
        e = np.where(self.live, np.abs(st.q - f0), 0.0)
        err = float(e.max())
        if t_in < a.hold_min_s:
            return (False, f'HOLD for {t_in:.2f} s < {a.hold_min_s} s')
        if err >= a.hold_tol:
            j = int(np.argmax(e))
            return (False, f'max|q - f0| = {err:.3f} rad ({self.cfg.joint_names[j]}) >= {a.hold_tol}')
        qn = float(np.linalg.norm(st.quat))
        if not np.isfinite(qn) or abs(qn - 1.0) > 0.05:
            return (False, f'bad IMU quaternion (norm {qn:.3f})')
        adq = np.where(self.live, np.abs(st.dq), 0.0)
        dqm = float(adq.max())
        if dqm >= a.start_max_dq:
            j = int(np.argmax(adq))
            return (False, f'max|dq| = {dqm:.2f} rad/s ({self.cfg.joint_names[j]}) >= {a.start_max_dq}')
        g = projected_gravity(st.quat / max(np.linalg.norm(st.quat), 1e-09))
        g0 = clip.proj_grav[0]
        tilt = float(np.degrees(np.arccos(np.clip(np.dot(g, g0) / (np.linalg.norm(g0) + 1e-09), -1, 1))))
        if tilt >= a.start_max_tilt_deg:
            return (False, f'tilt vs clip frame 0 = {tilt:.1f} deg >= {a.start_max_tilt_deg}')
        tm = float(st.temp[self.live].max())
        if tm > a.warn_temp:
            return (False, f'max motor temperature {tm:.0f} C > --warn-temp {a.warn_temp} (UNVERIFIED field semantics)')
        return (True, f'max|q - f0| = {err:.3f} rad, max|dq| = {dqm:.2f}, tilt vs clip {tilt:.1f} deg')

    def auto_events(self, state: str, t_in: float, st) -> list[str]:
        a = self.a
        if state == 'PASSIVE' and t_in >= a.auto_passive_s:
            return ['interp']
        if state == 'HOLD':
            ok, why = self.policy_allowed(t_in, st, self.f0s[self.clip_idx], self.clips[self.clip_idx])
            if ok:
                return ['policy']
            if t_in > a.auto_hold_timeout_s:
                self.damp(f'auto-sequence: standby start condition not met within {a.auto_hold_timeout_s} s ({why})')
            return []
        if state == 'STANDBY_PRE' and t_in >= a.auto_standby_s:
            ok, why = self.standby_allowed(t_in, st)
            if ok:
                return ['start']
            if t_in > a.auto_standby_s + a.auto_hold_timeout_s:
                self.damp(f'auto-sequence: motion start condition not met within {a.auto_hold_timeout_s} s ({why})')
            return []
        if state == 'STANDBY_POST':
            if self.clip_idx + 1 < len(self.clips):
                if t_in >= a.auto_between_s:
                    c = self.confirm
                    if c is None or c['kind'] != 'interp' or c['state_t'] != self.state_t:
                        return ['interp']
                    if time.monotonic() - c['t'] >= 0.5:
                        return ['interp']
            elif t_in >= a.auto_end_hold_s:
                if a.sim_band_file and self.band_off_t is not None:
                    self.sim_band_on('auto-sequence: re-hoist before L2+B')
                if t_in >= a.auto_end_hold_s + 1.0:
                    self.damp('auto-sequence finished')
            return []
        if state == 'END_HOLD':
            if self.end is not None and (self.end.get('then_interp') or self.end.get('then_hold')):
                return []
            if self.clip_idx + 1 < len(self.clips):
                if t_in >= max(a.auto_between_s, a.end_blend_s + 0.1):
                    return ['interp']
            elif t_in >= a.auto_end_hold_s:
                self.damp('auto-sequence finished')
        return []

    def save(self):
        a = self.a
        a.log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(self.wall0))
        path = a.log_dir / f'run_{stamp}.npz'
        R = {k: np.asarray(v) for k, v in self.rec.items() if len(v)}
        n = min(self.cl_n, self.cl_cap)
        idx = np.arange(self.cl_n - n, self.cl_n) % self.cl_cap
        meta = {'args': {k: str(v) for k, v in vars(a).items()}, 'clips': [c.name for c in self.clips], 'states': STATES, 'events': self.events, 'damp_reason': self.damp_reason, 'n_rx': self.n_rx, 'n_crc_bad': self.n_crc_bad, 'policy_calls': self.n_policy_calls, 'joint_names': self.cfg.joint_names, 'stage': a.stage, 'profile': self.profile, 'n_sent': self.n_sent, 'n_write_fail': self.n_write_fail, 'n_cmd_exc': self.n_cmd_exc, 'cb_errors': self.cb_errors, 'cb_last_err': self.cb_last_err, 'n_lowcmd_rx': self.watcher.count if self.watcher else None, 'self_delivery': self.self_delivery, 'rt_info': self.rt_info, 'printer_failures': self.printer.n_fail, 'cmd_log_wrapped': self.cl_n > self.cl_cap, 'joint_lower': self.cfg.joint_lower.tolist(), 'joint_upper': self.cfg.joint_upper.tolist(), 'lock_waist': False, 'locked_joints': np.nonzero(self.locked)[0].tolist(), 'reserved_mode': self.mm_latched, 't0_monotonic': self.t0, 'end_hold_legacy': bool(a.end_hold_legacy)}
        np.savez_compressed(path, **R, cmd_t=self.cl_t[idx], cmd_q=self.cl_q[idx], cmd_kp=self.cl_kp[idx], cmd_kd=self.cl_kd[idx], cmd_state=self.cl_s[idx], meta=json.dumps(meta))
        print(f"[log] {path}  ({len(self.rec['t'])} policy-loop records, {n} lowcmd records)", flush=True)
        summarize(path)
        return path

def _yaw(quat) -> float:
    n = float(np.linalg.norm(quat))
    if not np.isfinite(n) or n < 0.5:
        return 0.0
    w, x, y, z = np.asarray(quat, float) / n
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))

def _tilt(quat) -> float:
    g = projected_gravity(quat / max(np.linalg.norm(quat), 1e-09))
    return float(np.arccos(np.clip(-g[2], -1, 1)))

def _tilt_vs(quat, g_ref) -> float:
    n = float(np.linalg.norm(quat))
    if not np.isfinite(n) or n < 0.5:
        return np.nan
    g = projected_gravity(quat / n)
    return float(np.arccos(np.clip(np.dot(g, g_ref) / (np.linalg.norm(g_ref) + 1e-09), -1, 1)))

def _governor() -> str:
    p = Path('/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor')
    try:
        g = p.read_text().strip()
        return f"{g} ({('writable' if os.access(p, os.W_OK) else 'needs sudo to change')})"
    except Exception:
        return 'unknown'

def summarize(path):
    z = np.load(path, allow_pickle=False)
    meta = json.loads(str(z['meta']))
    t, st = (z['t'], z['state'])
    names = meta.get('states', STATES)
    SID = {n: i for i, n in enumerate(names)}
    print('[summary] events:')
    for e in meta['events']:
        print(f"    {e['t']:8.3f}  {e['kind']:<8} {e['msg']}")
    per = np.diff(t) * 1000.0
    if len(per) > 5:
        print(f'[summary] policy-loop period ms: mean {per.mean():.3f}  p1 {np.percentile(per, 1):.3f}  p99 {np.percentile(per, 99):.3f}  max {per.max():.3f}  |jitter| p99 {np.percentile(np.abs(per - 20.0), 99):.3f}')
        lat = (t - z['deadline']) * 1000.0
        print(f'[summary] wake-up lateness ms: p50 {np.median(lat):.3f}  p99 {np.percentile(lat, 99):.3f}  max {lat.max():.3f};  compute ms (STANDBY_*/POLICY): p50/p99/max ' + _pct(z['compute_ms'][np.isin(st, [SID[k] for k in POLICY_STATES if k in SID])]))
        print(f"[summary] state age at loop ms: p50 {np.median(z['state_age_ms']):.2f}  p99 {np.percentile(z['state_age_ms'], 99):.2f}  max {z['state_age_ms'].max():.2f}")
    ct = z['cmd_t']
    if len(ct) > 10:
        cp = np.diff(ct) * 1000.0
        print(f"[summary] lowcmd: {len(ct)} msgs in {ct[-1] - ct[0]:.2f} s = {(len(ct) - 1) / (ct[-1] - ct[0]):.1f} Hz (includes immediate sends after policy steps); gap p99 {np.percentile(cp, 99):.3f} max {cp.max():.3f} ms; write failures {meta.get('n_write_fail')}")
    pol = st == SID['POLICY']
    if pol.any():
        print(f"[summary] POLICY steps {pol.sum()}  max|action| {np.abs(z['action'][pol]).max():.2f}  clamped joints/step mean {z['n_clamped'][pol].mean():.2f}")
        g = np.array([projected_gravity(q / max(np.linalg.norm(q), 1e-09)) for q in z['quat'][pol]])
        msg = f'[summary] POLICY tilt (IMU) max {np.degrees(np.arccos(np.clip(-g[:, 2], -1, 1))).max():.1f} deg'
        if 'tilt_err' in z.files and np.isfinite(z['tilt_err'][pol]).any():
            te = np.degrees(np.nanmax(z['tilt_err'][pol]))
            lim = np.degrees(SafetyCfg().max_tilt_err)
            msg += f'; tilt error vs reference max {te:.1f} deg (DeployCore abort > {lim:.0f} deg, margin {lim - te:.1f} deg)'
        print(msg)
        if 'joint_lower' in meta:
            lo, hi = (np.asarray(meta['joint_lower']), np.asarray(meta['joint_upper']))
            qp = z['q'][pol]
            at = (qp < lo + 0.02) | (qp > hi - 0.02)
            frac = at.mean(axis=0)
            if frac.max() > 0:
                top = np.argsort(-frac)[:3]
                print('[summary] POLICY time within 0.02 rad of the hard stop: ' + ', '.join((f"{meta['joint_names'][j]} {100 * frac[j]:.1f}%" for j in top if frac[j] > 0)))
    _standby_summary(z, names, meta)
    summarize_pelvis(z, meta)
    lk = meta.get('locked_joints') or []
    if meta.get('lock_waist') and lk and len(t):
        cq = z['cmd_q'][:, lk] if len(z['cmd_t']) else np.zeros((1, len(lk)))
        ck = z['cmd_kp'][:, lk] if len(z['cmd_t']) else np.zeros((1, len(lk)))
        cd = z['cmd_kd'][:, lk] if len(z['cmd_t']) else np.zeros((1, len(lk)))
        lt = z['locked_temp'] if 'locked_temp' in z.files else np.zeros(1)
        lm = z['locked_mstat'] if 'locked_mstat' in z.files else np.zeros(1, np.int64)
        print(f"[summary] WAIST LOCKED (reserved_mode {meta.get('reserved_mode')}), motors {lk}: lowstate max|q| {np.abs(z['q'][:, lk]).max():.4f} max|dq| {np.abs(z['dq'][:, lk]).max():.4f} max temp {lt.max():.0f} mstat seen {sorted({int(x) for x in np.ravel(lm)})}; sent max|q|/kp/kd {np.abs(cq).max():.3g}/{np.abs(ck).max():.3g}/{np.abs(cd).max():.3g}")
        if pol.any():
            print(f"[summary] POLICY raw action on locked joints (not applied): max|a| {np.abs(z['action'][pol][:, lk]).max():.2f}")
    print(f"[summary] stage {meta.get('stage')} profile {meta.get('profile')}; rt {meta.get('rt_info')}; lowcmd self-delivery {meta.get('self_delivery')}; DDS callback errors {meta.get('cb_errors')}; stdout failures {meta.get('printer_failures')}")
    print(f"[summary] damp reason: {meta['damp_reason']}")

def _standby_summary(z, names, meta):
    """Per contiguous STANDBY_PRE / POLICY / STANDBY_POST segment: duration, tilt error vs reference, max|dq| (live joints), max|a|."""
    t, st = (z['t'], z['state'])
    if not len(t):
        return
    live = np.ones(z['dq'].shape[1], bool)
    live[meta.get('locked_joints') or []] = False
    lab = np.array([names[i] if 0 <= i < len(names) else '?' for i in st])
    cut = np.nonzero(lab[1:] != lab[:-1])[0] + 1
    tot = {}
    have_te = 'tilt_err' in z.files
    for a, b in zip(np.r_[0, cut], np.r_[cut, len(lab)]):
        name = lab[a]
        if name not in POLICY_STATES:
            continue
        dur = float(t[b - 1] - t[a]) + 0.02
        tot[name] = tot.get(name, 0.0) + dur
        dqm = np.abs(z['dq'][a:b][:, live]).max(1)
        te = np.degrees(z['tilt_err'][a:b]) if have_te else np.full(b - a, np.nan)
        te_s = f'tilt err max {np.nanmax(te):5.2f} mean {np.nanmean(te):5.2f} deg' if np.isfinite(te).any() else 'tilt err n/a'
        print(f"[summary] {name:<12} t {t[a]:8.2f} dur {dur:7.2f} s ({b - a} steps): {te_s}; max|dq| max {dqm.max():.2f} p99 {np.percentile(dqm, 99):.2f} rad/s; max|a| {np.abs(z['action'][a:b]).max():.2f}")
    if tot:
        print('[summary] totals: ' + ', '.join((f'{k} {v:.1f} s' for k, v in tot.items())))

def _pct(x):
    return 'n/a' if len(x) == 0 else f'{np.median(x):.2f}/{np.percentile(x, 99):.2f}/{x.max():.2f}'

def main():
    # 同一脚本也作为 lowcmd 监视子进程：只统计控制命令，辅助检测其他发布者。
    # watcher_child() 内部直接退出进程，不会继续执行下面的控制器启动流程。
    if len(sys.argv) >= 2 and sys.argv[1] == '--lowcmd-watcher-child':
        watcher_child(int(sys.argv[2]), sys.argv[3] if len(sys.argv) > 3 else '')

    # 解析命令行参数；deploy_config.json、模型和动作文件稍后由 Runner 加载。
    # iface/domain 决定 DDS 通信范围，stage 选择安全阈值档位，clip 可指定多个动作。
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--iface', required=True)
    ap.add_argument('--domain', type=int, required=True, help='0 real robot, 1 sim')
    ap.add_argument('--stage', choices=list(STAGES), required=True, help='safety profile: air/feet (vel 0.3, |a|<10, torque 0.5x effort, hold tilt 20 deg), slack (0.5, 15, 1.0x, 29 deg), ground (core defaults 0.8, 50, 1.5x, 46 deg)')
    ap.add_argument('--artifacts', type=Path, default=gc.DEFAULT_ARTIFACTS, help='dir with policy.onnx + deploy_config.json')
    ap.add_argument('--clip', type=Path, nargs='+', required=True, help='clip npz(s); default: sequential L2+up; --motion-select: three files in L1+A/X/Y order; legacy default.npz/walk.npz/dance.npz retain filename mapping in any order')
    ap.add_argument('--motion-select', action='store_true', help='L1+A/X/Y select three clips; fixed PD preparation waits in HOLD for R1+B takeover; Start plays after readiness checks')
    ap.add_argument('--log-dir', type=Path, default=gc.REPO / 'deploy_logs')
    ap.add_argument('--sim', action='store_true', help="simulation run (skips 'yes' prompt; allows --auto-sequence and sim-only test hooks)")
    ap.add_argument('--auto-sequence', action='store_true', help='SIM ONLY: PASSIVE -> INTERP -> HOLD -> STANDBY_PRE -> POLICY -> STANDBY_POST (-> blend -> INTERP next clip ...) -> band on -> DAMP -> exit')
    ap.add_argument('--no-policy', action='store_true', help='never start the policy (R1+B/R1+A always refused): INTERP/HOLD/END tests only, e.g. the first powered run')
    ap.add_argument('--allow-ssh', action='store_true', help='domain 0: allow running from an ssh session (NOT recommended)')
    # None 表示沿用 stage 的阈值；显式传值时覆盖对应项。
    ap.add_argument('--max-vel-frac', type=float, default=None)
    ap.add_argument('--max-abs-action', type=float, default=None)
    ap.add_argument('--torque-clamp-factor', type=float, default=None)
    ap.add_argument('--max-tilt-hold', type=float, default=None, help='rad, tilt -> DAMP in INTERP/HOLD/END_HOLD')
    # cmd-hz 是低层命令发送频率；策略循环频率由 deploy_config 的 control_dt 决定。
    ap.add_argument('--cmd-hz', type=float, default=500.0)
    ap.add_argument('--interp-s', type=float, default=4.0)
    ap.add_argument('--policy-blend-s', type=float, default=5.0, help='HOLD -> policy target blend inside STANDBY_PRE [s]; readiness windows start after completion; 0 restores the abrupt handoff')
    ap.add_argument('--end-blend-s', type=float, default=1.0, help='END_HOLD: gain/target blend duration')
    ap.add_argument('--end-hold-legacy', action='store_true', help='clip done -> END_HOLD (stand-gain blend, cannot balance) instead of STANDBY_POST (policy keeps standing)')
    # STANDBY_PRE 已由策略维持姿态；满足持续时间和稳定窗口要求后才允许播放动作。
    ap.add_argument('--standby-min-s', type=float, default=3.0, help='R1+X: minimum time in STANDBY_PRE')
    ap.add_argument('--standby-window-s', type=float, default=3.0, help='R1+X: stability window (end of STANDBY_PRE); 3 s catches a stepping robot that 0.5 s missed (review sim)')
    ap.add_argument('--standby-max-tilt-deg', type=float, default=8.0, help='R1+X: max tilt error vs the reference over the window')
    ap.add_argument('--standby-max-dq', type=float, default=1.0, help='R1+X: max |dq| [rad/s] over the window')
    ap.add_argument('--confirm-s', type=float, default=3.0, help='L2+up in STANDBY_PRE / STANDBY_POST (leaves the policy: stand gains) must be pressed twice within this time')
    ap.add_argument('--status-s', type=float, default=3.0, help='STANDBY_*: status line period')
    ap.add_argument('--warn-yaw-deg', type=float, default=15.0, help='STANDBY_*: WARN when the IMU yaw changed more than this since entering the state')
    ap.add_argument('--warn-leg-dq', type=float, default=1.0, help='STANDBY_*: WARN when leg max|dq| [rad/s] stays above this ...')
    ap.add_argument('--warn-leg-dq-s', type=float, default=0.5, help='... for longer than this [s]')
    ap.add_argument('--standby-warn-s', type=float, default=120.0, help='STANDBY_*: WARN after this long (documented standby cap until the lowstate age over a long standby is measured)')
    ap.add_argument('--sync-udp', default=None, metavar='HOST:PORT', help="send 'state clip frame t' over UDP every tick (background-video sync, scripts/deploy/bg_sync_server.py)")
    # HOLD -> STANDBY_PRE 的接管条件：保持时间、首帧姿态误差、关节速度和倾角。
    ap.add_argument('--hold-min-s', type=float, default=1.0)
    ap.add_argument('--hold-tol', type=float, default=0.30, help='max |q - f0| [rad] to allow the policy start')
    ap.add_argument('--start-max-dq', type=float, default=0.5, help='max |dq| [rad/s] to allow the policy start')
    ap.add_argument('--start-max-tilt-deg', type=float, default=10.0, help='max tilt vs clip proj_grav[0] to allow the policy start')
    # 运行保护：状态过期、循环超时、过温或连续发送失败时进入 DAMP 阻尼状态。
    ap.add_argument('--stale-ms', type=float, default=20.0)
    ap.add_argument('--max-overrun', type=float, default=2.0, help='policy cycles late before DAMP')
    ap.add_argument('--max-temp', type=float, default=80.0, help='motor temperature [C] -> DAMP (UNVERIFIED default)')
    ap.add_argument('--warn-temp', type=float, default=70.0, help='motor temperature [C] above which policy takeover is refused (UNVERIFIED)')
    ap.add_argument('--max-write-fail', type=int, default=10, help='consecutive failed rt/lowcmd writes -> DAMP')
    ap.add_argument('--stand-arm-kd', type=float, default=None, help='override fixed-PD kd for the 8 arm joints; default: each joint kd from deploy_config.json')
    ap.add_argument('--stand-waist-kd', type=float, default=None, help='override fixed-PD kd for torso yaw; default: its kd from deploy_config.json')
    # 通信与启动检查：状态 CRC、等待 lowstate、检测其他控制器及查询内置运动模式。
    ap.add_argument('--no-state-crc', action='store_true', help='do not drop lowstate messages with a bad CRC (domain 0)')
    ap.add_argument('--wait-state-s', type=float, default=5.0)
    ap.add_argument('--lowcmd-check-s', type=float, default=1.0)
    ap.add_argument('--lowcmd-recheck-s', type=float, default=0.5, help="foreign rt/lowcmd re-check right after 'yes'")
    ap.add_argument('--no-lowcmd-watch', dest='lowcmd_watch', action='store_false', help='disable the run-time foreign rt/lowcmd watch')
    ap.add_argument('--check-mode-timeout', type=float, default=2.0)
    ap.add_argument('--require-check-mode', action='store_true', help='domain 0: refuse if MotionSwitcher CheckMode does not answer')
    ap.add_argument('--button-stable-n', type=int, default=3)
    ap.add_argument('--button-release-s', type=float, default=0.1)
    ap.add_argument('--button-max-age-s', type=float, default=0.1)
    # 推理线程数、命令线程绑核和实时调度优先级，用于减少控制周期抖动。
    ap.add_argument('--ort-threads', type=int, default=1)
    ap.add_argument('--cmd-cpu', default='auto', help="pin the command thread: 'auto' (last allowed CPU), a CPU index, or 'none'")
    ap.add_argument('--rt-prio', type=int, default=50, help='try SCHED_FIFO for the command thread (needs CAP_SYS_NICE/rtprio; 0 = skip)')
    # lowcmd 日志使用环形缓冲区；log-obs 可额外保存策略输入的历史观测。
    ap.add_argument('--log-obs', action='store_true', help='also log DeployCore.hist (10 frames oldest->newest x obs_frame_dim (63 or 72), frame-major) every POLICY step')
    ap.add_argument('--cmd-log-max-s', type=float, default=900.0, help='lowcmd log ring-buffer length')
    ap.add_argument('--rec-max-s', type=float, default=1800.0, help='--log-obs stops after this many seconds of records')
    # 仿真支撑与测试钩子：弹力带控制、NaN/主循环卡顿注入，以及自动序列的等待时间。
    ap.add_argument('--sim-band-file', default=None, help='SIM: flag file read by sim_h1_mujoco.py --band-file')
    ap.add_argument('--sim-band-ramp-s', type=float, default=5.0, help='SIM: smooth support reduction from 100%% to 0%% over this duration [s]; 0 = immediate release; readiness samples start after full release AND policy blend')
    ap.add_argument('--sim-band-release-s', type=float, default=-1.0, help='deprecated: keep negative; releasing support in fixed PD is no longer allowed')
    ap.add_argument('--sim-after-band-s', type=float, default=1.0, help='SIM: minimum time after band release before Start may play the motion')
    ap.add_argument('--sim-band-release-standby-s', type=float, default=-1.0, help='SIM: earliest release time in STANDBY_PRE, always after valid policy inference and command submission; <0 = first valid command')
    ap.add_argument('--sim-no-band-end-hold', action='store_true', help='deprecated: rejected; support is restored at motion end and fixed PD transitions')
    ap.add_argument('--sim-exit-after-damp', type=float, default=None, help='SIM: exit this many s after entering DAMP')
    ap.add_argument('--sim-inject-nan-step', type=int, default=None, help='SIM: policy returns NaN from this call on')
    ap.add_argument('--sim-stall-at-s', type=float, default=None, help='SIM: block the main loop this long after POLICY starts')
    ap.add_argument('--sim-stall-s', type=float, default=0.5)
    ap.add_argument('--sim-ask-yes', action='store_true', help="SIM: ask the 'yes' confirmation like on the robot")
    ap.add_argument('--sim-try-check-mode', action='store_true', help='SIM: also call MotionSwitcher CheckMode (exercise the code path)')
    ap.add_argument('--auto-passive-s', type=float, default=1.0)
    ap.add_argument('--auto-between-s', type=float, default=1.0)
    ap.add_argument('--auto-standby-s', type=float, default=3.5, help='auto-sequence: R1+X after this long in STANDBY_PRE (if allowed)')
    ap.add_argument('--auto-end-hold-s', type=float, default=3.0)
    ap.add_argument('--auto-hold-timeout-s', type=float, default=10.0)
    # 部分策略需要参考坐标系下的外部骨盆位姿，通过 UDP 接收并检查数据时效。
    ap.add_argument('--pose-listen', default=None, metavar='HOST:PORT', help='UDP pelvis pose input in the reference frame, required for anchor_obs or BlinkMimic pelvis feedback')
    ap.add_argument('--pose-max-age', type=float, default=0.1)
    ap.add_argument('--max-motor-lost', type=int, default=None, help='optional GO per-motor packet-loss count threshold; not a fault bitmask')
    a = ap.parse_args()
    if not np.isfinite(a.sim_band_ramp_s) or a.sim_band_ramp_s < 0:
        ap.error('--sim-band-ramp-s must be finite and nonnegative')
    if not np.isfinite(a.policy_blend_s) or a.policy_blend_s < 0:
        ap.error('--policy-blend-s must be finite and nonnegative')

    # 拒绝废弃的支撑释放选项：固定 PD 阶段保持支撑，动作结束时恢复支撑。
    if a.sim_band_release_s >= 0:
        ap.error('--sim-band-release-s must be negative: support stays on throughout fixed PD')
    if a.sim_no_band_end_hold:
        ap.error('--sim-no-band-end-hold is no longer supported: motion end must restore support')
    # 动作选择模式要求三个不同文件，且不能与自动序列或旧结束模式混用。
    validate_selection_args(a, ap)
    # 补齐后续代码使用的可选字段；当前入口关闭锁腰和动作许可文件相关功能。
    a.lock_waist = False
    a.clip_allow_file = None
    a.i_know_clip = None
    a.lock_waist_standby_allow = None
    a.lock_waist_direct_start = False
    # 真机域 0 禁止仿真模式及以下测试钩子；这些钩子在其他域也必须显式启用 --sim。
    sim_only = [a.auto_sequence, a.sim_band_file, a.sim_band_release_standby_s >= 0, a.sim_exit_after_damp is not None, a.sim_inject_nan_step is not None, a.sim_stall_at_s is not None, a.sim_ask_yes, a.sim_try_check_mode]
    if a.domain == 0 and (a.sim or any(sim_only)):
        sys.exit('REFUSED: --sim / --auto-sequence / --sim-* hooks are not allowed on DDS domain 0 (real robot)')
    if any(sim_only) and (not a.sim):
        sys.exit('--auto-sequence and --sim-* options require --sim')
    # 真机默认要求本地终端运行，避免控制程序依赖 SSH 会话；allow-ssh 可显式覆盖。
    if a.domain == 0 and (os.environ.get('SSH_CONNECTION') or os.environ.get('SSH_TTY')) and (not a.allow_ssh):
        sys.exit("REFUSED: running from an ssh session. Run the runner on the deploy PC's local console (a dropped ssh/Wi-Fi link must not be able to take the controller down). --allow-ssh overrides (not recommended).")
    # 自动仿真流程进入 DAMP 后默认等待 1 秒，再退出 run()。
    if a.auto_sequence and a.sim_exit_after_damp is None:
        a.sim_exit_after_damp = 1.0

    # 创建打印线程和控制器；Runner 在此加载模型/动作、校验维度并初始化状态与缓冲区。
    printer = Printer()
    r = Runner(a, printer)
    # print 和异常输出先入队，再由打印线程写终端及 console.txt，降低 I/O 对控制的影响。
    sys.stdout = QueueStream(printer.q)
    sys.stderr = QueueStream(printer.q)
    printer.open_file(a.log_dir / f"run_{time.strftime('%Y%m%d_%H%M%S', time.localtime(r.wall0))}.console.txt")

    # 信号到达时向非阻塞管道写入通知，命令线程通过 _poll_signal() 读取并切换阻尼。
    rfd, wfd = os.pipe()
    os.set_blocking(rfd, False)
    os.set_blocking(wfd, False)
    signal.set_wakeup_fd(wfd, warn_on_full_buffer=False)
    r.sig_r = rfd

    def on_signal(signum, _frame):
        # 信号处理器只记录并通知，不在这里加控制锁或发送 DDS 命令。
        r.sig = signum
        r.stop.set()
        if not r.cmd_started:
            # 命令尚未启动时，直接打断启动等待或 yes 输入。
            raise KeyboardInterrupt(f'signal {signum}')
    # Ctrl+C、终止、终端断开和 Ctrl+Z 等均走退出流程，避免控制进程被默认挂起。
    for sg in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGQUIT, signal.SIGTSTP):
        signal.signal(sg, on_signal)
    # 忽略后台终端读写引发的作业控制暂停信号。
    for sg in (signal.SIGTTIN, signal.SIGTTOU):
        signal.signal(sg, signal.SIG_IGN)
    exit_code = 0
    try:
        # 初始化 DDS、检查状态/其他控制器/温度、确认后启动发送线程，初始为 PASSIVE 阻尼。
        r.startup()
        # 按 control_dt 运行状态机：处理手柄、插值、策略推理、安全检查和数据记录。
        r.run()
    except KeyboardInterrupt as e:
        # 已开始控制时先切阻尼；启动阶段中断则只报告失败，不发送命令。
        if r.cmd_started:
            r.damp(f'operator interrupt ({e})')
        else:
            print(f'[startup] interrupted ({e}); nothing was sent', flush=True)
            exit_code = 1
    except SystemExit as e:
        # startup() 也用 SystemExit 表示启动检查不通过或操作员未确认。
        if r.cmd_started:
            r.damp(f'exit: {e}')
        else:
            print(f'[startup] {e}', flush=True)
            exit_code = 1
    except Exception as e:
        # 普通运行异常保留堆栈，若已接管控制则请求阻尼，并返回非零退出码。
        traceback.print_exc()
        if r.cmd_started:
            r.damp(f'exception {e!r}')
        exit_code = 1
    finally:
        # 正常返回或上述异常均执行此收尾；只有发送线程已启动才执行阻尼和运行数据保存。
        if r.cmd_started:
            t_f = time.monotonic()
            reason = f'signal {signal.Signals(r.sig).name}' if r.sig is not None else 'shutdown'
            # damp() 将 kp 置 0、kd 置 DAMP_KD；已处于 DAMP 时重复调用会直接返回。
            r.damp(reason)
            r.event('SHUTDOWN', 'sending damping for 1 s')
            # 暂不停止发送线程，让它继续发送阻尼约 1 秒；单调时钟不受系统校时影响。
            t_end = time.monotonic() + 1.0
            while time.monotonic() < t_end:
                time.sleep(max(0.0, t_end - time.monotonic()))
            t_j = time.monotonic()
            # stop 控制主循环，cmd_stop 控制命令线程；分开停止才能保留上述阻尼发送窗口。
            r.stop.set()
            r.cmd_stop.set()
            # 最多等待 1 秒，是否真正结束由下面日志中的 is_alive() 给出。
            r.cmd_thread.join(timeout=1.0)
            r.event('SHUTDOWN', f'command thread stopped (alive={r.cmd_thread.is_alive()}); no more rt/lowcmd. damping {t_j - t_f:.3f} s, join {time.monotonic() - t_j:.3f} s')
            # DAMP 已恢复仿真支撑，保持到仿真器停止；保存状态、命令和事件到 npz。
            try:
                r.save()
            except Exception:
                traceback.print_exc()
        # 关闭监视子进程，最后让打印线程处理队列中的日志并关闭文件。
        if r.watcher is not None:
            r.watcher.close()
        printer.close()
    # 直接结束整个进程，跳过解释器常规退出清理，因此日志和线程收尾放在前面完成。
    os._exit(exit_code)
if __name__ == '__main__':
    main()
