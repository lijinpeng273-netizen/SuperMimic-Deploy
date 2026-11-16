"""只读 H1 定位校准候选与漂移检查；不会发布位姿、电机命令或切换模式。"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from xml.sax.saxutils import quoteattr

import numpy as np
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'src'))
import h1_common as gc
from h1_deploy.deploy.core import DeployConfig
from h1_deploy.deploy.pose_diagnostics import FootGeometry, SourceSequence, analyze_pose


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be positive and finite')
    return number


def finite(value):
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError('must be finite')
    return number


def valid_vector(values, width, *, quaternion=False):
    out = np.asarray(values, float)
    if out.shape != (width,) or not np.isfinite(out).all():
        raise ValueError('non-finite or incorrectly sized state vector')
    if quaternion and abs(np.linalg.norm(out) - 1) > 0.01:
        raise ValueError('invalid quaternion norm')
    return out


def collect_dds(a):
    # No ChannelFactory, publishers, robot clients or runner imports. Explicit XML
    # avoids SDK tracing files. DDS source stamps are used for sequence/pairing,
    # never subtracted from the PC clock as a transport-age measurement.
    from cyclonedds.domain import Domain, DomainParticipant
    from cyclonedds.qos import Qos, Policy
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, LowCmd_, SportModeState_
    from unitree_sdk2py.utils.crc import CRC

    crc = CRC()
    xml = ('<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name='
           + quoteattr(a.iface) + '/></Interfaces></General></Domain></CycloneDDS>')
    domain = Domain(a.domain, xml)
    participant = DomainParticipant(a.domain)
    kinds = {'rt/lowstate': LowState_, a.odom_topic: SportModeState_, 'rt/lowcmd': LowCmd_}
    topics = {n: Topic(participant, n, k) for n, k in kinds.items()}
    qos = Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(2048))
    readers = {n: DataReader(participant, t, qos=qos) for n, t in topics.items()}
    seq = {n: SourceSequence() for n in ('rt/lowstate', a.odom_topic)}
    rows = {n: [] for n in seq}
    latest = {}
    counters = dict(lowcmd_observed=0, crc_bad=0, invalid_lowstate=0, invalid_odometry=0, zero_payload_stamps=0)
    begin, started, next_print = time.monotonic(), None, 0.0
    print(f'[只读] domain={a.domain} iface={a.iface}; 等待 lowstate 和 {a.odom_topic}，没有命令发布者。', flush=True)
    try:
        while started is None or time.monotonic() - started < a.seconds:
            for name, reader in readers.items():
                received = time.monotonic()
                for message in reader.take(2048):
                    info = message.sample_info
                    if not info.valid_data:
                        continue
                    if name == 'rt/lowcmd':
                        counters['lowcmd_observed'] += 1
                        continue
                    stamp, publisher = int(info.source_timestamp), int(info.publication_handle)
                    row = dict(received=received, stamp=stamp)
                    tick = None
                    try:
                        if name == 'rt/lowstate':
                            if message.crc != crc.Crc(message):
                                counters['crc_bad'] += 1
                                continue
                            if len(message.motor_state) != 20:
                                raise ValueError('expected H1 GO motor layout')
                            q = valid_vector([message.motor_state[i].q for i in gc.JOINT_TO_MOTOR], 19)
                            dq = valid_vector([message.motor_state[i].dq for i in gc.JOINT_TO_MOTOR], 19)
                            quat = valid_vector(message.imu_state.quaternion, 4, quaternion=True)
                            gyro = valid_vector(message.imu_state.gyroscope, 3)
                            pelvis_quat, pelvis_gyro = gc.pelvis_imu(quat, gyro, q[10], dq[10])
                            row.update(q=q, pelvis_quat=pelvis_quat, gyro=pelvis_gyro)
                            tick = int(message.tick)
                        else:
                            if message.error_code != 0:
                                raise ValueError('odometry error_code is nonzero')
                            row.update(odom_pos=valid_vector(message.position, 3),
                                       odom_quat=valid_vector(message.imu_state.quaternion, 4, quaternion=True))
                            if message.stamp.sec == 0 and message.stamp.nanosec == 0:
                                counters['zero_payload_stamps'] += 1
                    except (ValueError, TypeError, IndexError):
                        counters['invalid_lowstate' if name == 'rt/lowstate' else 'invalid_odometry'] += 1
                        continue
                    if not seq[name].accept(stamp, publisher, tick):
                        continue
                    latest[name] = row
                    if started is not None:
                        rows[name].append(row)
            now = time.monotonic()
            if started is None:
                if len(latest) == 2 and all(now - r['received'] <= 0.1 for r in latest.values()):
                    started = now
                    print(f'[采样] 开始 {a.seconds:g} 秒记录；请保持已声明的支撑和静止条件。', flush=True)
                elif now - begin > a.connect_timeout:
                    raise RuntimeError(f'等待有效数据超时：{counters}；已收到 {list(latest)}')
            elif now >= next_print:
                z = latest[a.odom_topic]['odom_pos'][2]
                print(f'[采样] {now - started:5.1f}/{a.seconds:g}s  odom.z={z:.4f}m  '
                      f'lowstate={len(rows["rt/lowstate"])} odom={len(rows[a.odom_topic])}', flush=True)
                next_print = now + a.status_s
            time.sleep(0.005)
    finally:
        # These are subscription-only entities; destroying them cannot send lowcmd.
        readers.clear()
        topics.clear()
    problems = []
    quality = dict(counters=counters, streams={}, pairing_clock='DDS source timestamps; requires compatible clocks',
                   freshness='publication sequence + local receive progress; not estimator acquisition age')
    for name, data in rows.items():
        if len(data) < 10:
            raise RuntimeError(f'{name}: 有效样本不足，只有 {len(data)} 个')
        source_t = np.array([r['stamp'] for r in data], np.int64)
        arrival = np.array([r['received'] for r in data])
        source_gap = float(np.diff(source_t).max() / 1e9)
        receive_gap = float(np.diff(arrival).max())
        problems.extend(f'{name}: {p}' for p in seq[name].problems)
        if max(source_gap, receive_gap) > a.max_gap:
            problems.append(f'{name}: source/receive gap exceeds {a.max_gap:g}s')
        if time.monotonic() - data[-1]['received'] > a.max_gap:
            problems.append(f'{name}: feedback stopped before collection ended')
        if arrival[-1] - arrival[0] < a.seconds * 0.8:
            problems.append(f'{name}: less than 80% duration coverage')
        quality['streams'][name] = dict(samples=len(data), sequence_counts=dict(seq[name].counts),
                                        source_gap_max_s=source_gap, receive_gap_max_s=receive_gap,
                                        source_span_s=float((source_t[-1] - source_t[0]) / 1e9))
    samples, pairing = pair_samples(rows['rt/lowstate'], rows[a.odom_topic], a.max_pair_ms)
    quality['pairing'] = pairing
    if pairing['coverage'] < 0.8:
        problems.append('less than 80% of selected odometry samples paired; verify source clocks/latency')
    quality['interface'], quality['domain'], quality['odom_topic'] = a.iface, a.domain, a.odom_topic
    return samples, quality, problems


def pair_samples(low, odom, max_pair_ms):
    """Pair at <=50 Hz using source stamps; no receive-time fallback."""
    stamps = np.asarray([r['stamp'] for r in low], np.int64)
    values = {k: [] for k in ('t', 'q', 'pelvis_quat', 'odom_quat', 'odom_pos')}
    selected, skews, prev = 0, [], None
    origin = odom[0]['stamp']
    for row in odom:
        if prev is not None and row['stamp'] - prev < 20_000_000:
            continue
        prev = row['stamp']
        selected += 1
        i = int(np.searchsorted(stamps, row['stamp']))
        nearby = [j for j in (i - 1, i) if 0 <= j < len(stamps)]
        j = min(nearby, key=lambda x: abs(int(stamps[x]) - row['stamp']))
        skew_ms = (int(stamps[j]) - row['stamp']) / 1e6
        if abs(skew_ms) > max_pair_ms:
            continue
        skews.append(skew_ms)
        values['t'].append((row['stamp'] - origin) / 1e9)
        for key in ('q', 'pelvis_quat'):
            values[key].append(low[j][key])
        for key in ('odom_pos', 'odom_quat'):
            values[key].append(row[key])
    if len(skews) < 10:
        raise RuntimeError('同步样本不足：两路 DDS 源时钟/数据间隔不满足配对条件；没有改用接收时刻凑数')
    return values, dict(selected=selected, paired=len(skews), coverage=len(skews) / selected,
                        max_abs_skew_ms=float(np.max(np.abs(skews))), limit_ms=max_pair_ms)


def simulate(a, cfg, geometry):
    """MuJoCo geometric truth with known sensor errors, no DDS or policy control."""
    q = cfg.default_joint_pos.copy()
    quat = np.array([1., 0., 0., 0.])
    sole, _ = geometry.heights(q, quat)
    height = float(sole.max())
    t = np.arange(0, a.seconds, cfg.control_dt)
    rotation = Rotation.from_euler('z', (a.sim_yaw_drift * t)[:, None], degrees=True).as_quat()[:, [3, 0, 1, 2]]
    pos = np.column_stack((a.sim_xy_drift[0] * t, a.sim_xy_drift[1] * t, np.full(len(t), height + a.sim_z_bias)))
    samples = dict(t=t, q=np.repeat(q[None], len(t), axis=0), pelvis_quat=rotation.copy(),
                   odom_quat=rotation, odom_pos=pos)
    quality = dict(source='MuJoCo forward kinematics with injected sensor errors; no dynamics or policy validation',
                   truth_pelvis_height_m=height, injected_odom_z_bias_m=a.sim_z_bias,
                   injected_xy_drift_m_s=list(a.sim_xy_drift), injected_yaw_drift_deg_s=a.sim_yaw_drift)
    return samples, quality, height


def show(report):
    f, g, o = report['fit'], report['geometry'], report['odometry']
    print(f'\n[结果] 配对 {report["samples"]} 帧，覆盖 {report["span_s"]:.2f}s；只读候选，deployment_ready=False')
    print(f'  条件声明：静止={report["stationary_asserted"]}，接地={report["contact_asserted"]}，实测高度={report["measured_pelvis_height_m"]} m')
    quality = report.get('quality', {})
    if 'pairing' in quality:
        p, c = quality['pairing'], quality['counters']
        print(f'  数据配对：覆盖 {p["coverage"]:.1%}，最大时间差 {p["max_abs_skew_ms"]:.3f} ms；'
              f'CRC 丢弃 {c["crc_bad"]}，无效 lowstate/odom {c["invalid_lowstate"]}/{c["invalid_odometry"]}')
        print(f'  观察到其他发布者的 lowcmd：{c["lowcmd_observed"]} 条（本工具没有发布者）')
    print(f'  里程计平均高度       : {o["position_mean_m"][2]:.6f} m')
    print(f'  脚底接地所需骨盆高度 : 左 {g["sole_required_height_mean_m"][0]:.6f} / 右 {g["sole_required_height_mean_m"][1]:.6f} m')
    print(f'  脚踝参考平面高度     : 左 {g["ankle_required_height_mean_m"][0]:.6f} / 右 {g["ankle_required_height_mean_m"][1]:.6f} m')
    print(f'  odom.z − 平均脚踝距离: {g["odom_minus_mean_ankle_height_mean_m"] * 1000:+.3f} mm（计算一致性，不是绝对精度）')
    for name, key in (('实测高度拟合偏移', 'manual_world_z_offset_m'), ('接地几何拟合偏移', 'contact_world_z_offset_m')):
        value = f[key]
        print(f'  {name}: ' + ('未提供对应条件' if value is None else f'{value:+.6f} m'))
    if f['candidate_available']:
        c = f['corrected']
        print(f'  候选世界 z 偏移      : {f["candidate_world_z_offset_m"]:+.6f} m（首 {report["calibration_window_s"]:g}s 拟合后固定）')
        print(f'  候选修正后平均高度   : {c["mean_pelvis_height_m"]:.6f} m')
        print(f'  后续窗口脚底 z 均值  : 左 {c["holdout_sole_z_mean_m"][0] * 1000:+.2f} / 右 {c["holdout_sole_z_mean_m"][1] * 1000:+.2f} mm')
    else:
        print('  候选不可用：')
        for issue in report['issues']:
            print('   -', issue)
    label = '静止条件下报告的漂移' if report['stationary_asserted'] else '观测变化（未声明静止，不能解释成漂移）'
    print(f'  {label}:')
    print(f'    odom 水平首尾窗口差 {o["xy_window_displacement_m"] * 1000:.3f} mm')
    print(f'    odom 航向窗口差 {o["yaw"]["change_deg"]:+.4f}°；斜率 {o["yaw"]["slope_deg_s"]:+.6f}°/s')
    print(f'    pelvis IMU 航向斜率 {report["pelvis_imu"]["slope_deg_s"]:+.6f}°/s')
    print('  固定 z 偏移不消除水平/航向漂移；本报告没有确认 robot_center 原点或参考动作对齐。')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source', choices=('dds', 'mujoco'), default='dds', help='mujoco: geometry and injected-error verification, no network')
    ap.add_argument('--iface', help='DDS network interface, e.g. enp12s0')
    ap.add_argument('--domain', type=int, default=0)
    ap.add_argument('--odom-topic', default='rt/odommodestate')
    ap.add_argument('--artifacts', type=Path, default=REPO / 'deploy_artifacts/last', help='directory containing deploy_config.json; no policy is loaded')
    ap.add_argument('--model', type=Path, default=gc.DEFAULT_MJCF)
    ap.add_argument('--seconds', type=positive, default=20.0)
    ap.add_argument('--pelvis-height', type=positive, help='measured floor-to-model-pelvis-origin height [m], NOT shell-bottom height')
    ap.add_argument('--contact', choices=('unknown', 'left', 'right', 'both'), default='unknown', help='manually confirmed feet touching the same flat floor')
    ap.add_argument('--stationary', action='store_true', help='operator confirms the whole robot stays stationary throughout sampling')
    ap.add_argument('--calibration-s', type=positive, default=1.0)
    ap.add_argument('--connect-timeout', type=positive, default=5.0)
    ap.add_argument('--max-pair-ms', type=positive, default=5.0)
    ap.add_argument('--max-gap', type=positive, default=0.2, help='maximum source/receive gap [s] for a calibration candidate')
    ap.add_argument('--max-joint-span', type=positive, default=0.02, help='maximum recorded joint position span [rad] for stationary fitting')
    ap.add_argument('--max-contact-gap', type=positive, default=0.02, help='maximum disagreement of contact/measurement-implied pelvis heights [m]')
    ap.add_argument('--status-s', type=positive, default=3.0)
    ap.add_argument('--report', type=Path, help='optional diagnostic JSON; creates a new file, never overwrites; not a runner config')
    ap.add_argument('--sim-z-bias', type=finite, default=-0.08, help='mujoco: injected odometry z minus true z [m]')
    ap.add_argument('--sim-yaw-drift', type=finite, default=0.025, help='mujoco: injected yaw drift [deg/s]')
    ap.add_argument('--sim-xy-drift', nargs=2, type=finite, default=(0.00007, -0.00002), metavar=('VX', 'VY'), help='mujoco: injected xy drift [m/s]')
    a = ap.parse_args()
    if a.seconds < max(3.0, a.calibration_s + 1.1):
        ap.error('--seconds must be >=3 and leave >1 second after --calibration-s')
    if not 0 <= a.domain <= 232:
        ap.error('--domain must be in [0, 232]')
    if a.source == 'dds' and not a.iface:
        ap.error('--iface is required for DDS')
    if a.odom_topic in ('rt/lowstate', 'rt/lowcmd'):
        ap.error('--odom-topic must name a SportModeState topic')
    if a.source == 'mujoco' and a.pelvis_height is not None:
        ap.error('mujoco uses its known ground-truth height; omit --pelvis-height')
    if a.report and a.report.exists():
        ap.error('--report already exists; choose a new filename')
    try:
        cfg = DeployConfig.load(a.artifacts / 'deploy_config.json')
        geometry = FootGeometry(a.model, cfg.joint_names)
        if a.source == 'dds':
            samples, quality, problems = collect_dds(a)
            height, stationary, contact = a.pelvis_height, a.stationary, a.contact
        else:
            samples, quality, height = simulate(a, cfg, geometry)
            problems, stationary, contact = [], True, 'both'
        report = analyze_pose(samples, geometry, measured_height=height, contact=contact,
                              stationary=stationary, calibration_s=a.calibration_s, problems=problems,
                              max_joint_span=a.max_joint_span, max_contact_gap=a.max_contact_gap)
        report.update(source=a.source, quality=quality, model=str(a.model.resolve()),
                      model_sha256=hashlib.sha256(a.model.read_bytes()).hexdigest(),
                      config=str((a.artifacts / 'deploy_config.json').resolve()))
        if a.source == 'mujoco':
            actual = report['fit']['candidate_world_z_offset_m']
            passed = actual is not None and abs(actual + a.sim_z_bias) < 1e-6
            passed = passed and abs(report['odometry']['yaw']['slope_deg_s'] - a.sim_yaw_drift) < 1e-6
            passed = passed and np.allclose(report['odometry']['position_slope_m_s'][:2], a.sim_xy_drift, atol=1e-9)
            report['simulation_verification'] = dict(passed=bool(passed), expected_offset_m=-a.sim_z_bias,
                                                     scope='geometry and diagnostics only; not policy stability')
            if not passed:
                raise RuntimeError('MuJoCo injected-error verification failed')
        show(report)
        if a.source == 'mujoco':
            print('[仿真验证] 已恢复注入的高度偏移、水平漂移率和航向漂移率；未验证策略闭环。')
        if a.report:
            a.report.parent.mkdir(parents=True, exist_ok=True)
            with a.report.open('x', encoding='utf-8') as f:
                json.dump(report, f, ensure_ascii=False, indent=2, allow_nan=False)
                f.write('\n')
            print('[报告]', a.report)
        else:
            print('[文件] 未指定 --report，本次没有写入报告或采样文件。')
        return 0 if report['fit']['candidate_available'] or not stationary else 2
    except KeyboardInterrupt:
        print('\n[结束] 只读采样已中断。')
        return 130
    except (ValueError, RuntimeError, OSError) as error:
        print(f'[失败] {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
