"""Read-only H1 GO state, motor mapping, pelvis IMU and controller diagnostics."""
import argparse
import time
from pathlib import Path
import numpy as np
import h1_common as gc
from h1_deploy.deploy.core import DeployConfig


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--iface', required=True)
    ap.add_argument('--domain', type=int, default=0)
    ap.add_argument('--artifacts', type=Path, default=gc.DEFAULT_ARTIFACTS)
    ap.add_argument('--rate', type=float, default=5)
    ap.add_argument('--once', action='store_true')
    ap.add_argument('--seconds', type=float, default=0)
    ap.add_argument('--check-mode', action='store_true')
    ap.add_argument('--sign', help='joint name or policy index; show model sign hints')
    a = ap.parse_args()
    cfg = DeployConfig.load(a.artifacts / 'deploy_config.json')
    hints = gc.sign_hints(cfg) if a.sign is not None else {}
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
    from unitree_sdk2py.utils.crc import CRC
    gc.init_dds(a.domain, a.iface)
    if a.check_mode: print('MotionSwitcher:', gc.check_motion_mode())
    crc, latest, counters = CRC(), [None], [0, 0]
    def on_state(message):
        if message.crc != crc.Crc(message):
            counters[1] += 1
            return
        latest[0] = (time.monotonic(), message)
        counters[0] += 1
    sub = ChannelSubscriber('rt/lowstate', LowState_)
    sub.Init(on_state, 0)
    start = time.monotonic()
    try:
        while not a.seconds or time.monotonic() - start < a.seconds:
            if latest[0] is None:
                if time.monotonic() - start > 5:
                    raise RuntimeError('no CRC-valid H1 GO state within 5 seconds')
                time.sleep(0.02)
                continue
            received, message = latest[0]
            state = gc.extract_state(message)
            age = time.monotonic() - received
            print(f'GO H1 tick={state.tick} age={age*1000:.1f}ms received={counters[0]} CRC_bad={counters[1]} keys={gc.pressed(state.keys)}')
            print('pelvis quat:', np.round(state.quat, 4), 'pelvis gyro:', np.round(state.gyro, 4))
            indices = range(19) if a.sign is None else [int(a.sign) if a.sign.isdigit() else cfg.joint_names.index(a.sign)]
            for i in indices:
                print(f'{i:2d} -> motor {gc.JOINT_TO_MOTOR[i]:2d} {cfg.joint_names[i]:30s} q={state.q[i]:+.3f} dq={state.dq[i]:+.3f} temp={state.temp[i,0]:.0f} lost={state.motorstate[i]}')
                if a.sign is not None: print(hints.get(cfg.joint_names[i], 'no geometry hint'))
            if a.once: break
            time.sleep(1 / max(a.rate, 0.1))
    finally:
        sub.Close()


if __name__ == '__main__':
    main()
