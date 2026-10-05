#!/usr/bin/env python3
"""pH 계획 실행기. 실기 실행은 사용자만 수행한다.

--dry-run은 JSON 계획만 검사하며 ROS를 import/초기화하지 않는다.
기존 RigView/Guard/Driver와 TCP 직렬화를 재사용하고 제어 설정은 변경하지 않는다.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import signal
import statistics
import subprocess
import time

from ph_profiles import ATM, digest, summary, validate

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / 'src/can_powerpack/config'
REGISTRY_SOURCE = 'src/can_powerpack/docs/액추에이터_개체_대장.md'
# 기존 대장의 숫자 ID 그대로 사용. A01 등의 별칭은 자동 추정하지 않는다.
ENCODERS = {'1': (28000., 8340.), '2': (29155., 8900.),
            '3': (29000., 9000.), '4': (29890., 9600.)}


def wall():
    return datetime.now(timezone.utc).isoformat()


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def save(path, value):
    # 임시 파일을 같은 폴더에 둔 뒤 교체해 중간 JSON을 남기지 않는다.
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(clean(value), indent=2, ensure_ascii=False, allow_nan=False))
    temporary.replace(path)


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('profile', type=Path, help='ph_profiles.py가 생성한 JSON')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--actuator', required=True, choices=ENCODERS)
    parser.add_argument('--mount', required=True, type=int)
    parser.add_argument('--axis', type=int, choices=[1], default=1)
    parser.add_argument('--d-max', type=float, default=72,
                        help='파일과 일치 여부만 검사. 변경하려면 계획을 다시 생성한다')
    parser.add_argument('--note', default='')
    parser.add_argument('--launch-command', default='', help='사용자가 실제 사용한 런치 명령 기록')
    parser.add_argument('--confirm-preflight', action='store_true',
                        help='개체/장착/물리 보정/다른 TCP 송신기 없음/초기 대기압 목표를 직접 확인')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2293)
    parser.add_argument('--connect-timeout', type=float, default=5.)
    parser.add_argument('--out', type=Path, help='새 결과 폴더. 기존 폴더 사용 불가')
    args = parser.parse_args()
    document = json.loads(args.profile.read_text())
    plan = document['plan']
    validate(plan)
    if digest(plan) != document['summary']['sha256']:
        parser.error('프로파일 해시 불일치')
    if not math.isfinite(args.d_max) or abs(args.d_max - plan['d_max']) > 1e-9:
        parser.error('--d-max와 파일이 다름. 실행 중 클리핑하지 않는다')
    if args.mount < 1:
        parser.error('--mount는 1 이상')
    print(json.dumps(summary(plan), indent=2))
    print(f'Axis 1, actuator {args.actuator}, mount {args.mount}, encoder {ENCODERS[args.actuator]}')
    if args.dry_run:
        return 0
    if not args.confirm_preflight or not args.launch_command:
        parser.error('실기는 --confirm-preflight와 --launch-command 기록이 필요하다')
    return run(args, plan)


def run(args, plan):
    # 하드웨어 의존 import는 반드시 dry-run 분기 뒤에만 둔다.
    import rclpy
    import yaml
    from rcl_interfaces.srv import GetParameters, ListParameters
    from rclpy.parameter import parameter_value_to_python
    from rclpy.signals import SignalHandlerOptions
    from actuator_map import RigView, Guard, Driver, Abort, POS_BOARD, NEG_BOARD, G
    from pressure_sweep_server import _connect
    from valve_deadzone import read_config

    class View(RigView):
        def __init__(self, cfg):
            self.pressure_received = None
            super().__init__(cfg, 0)

        def _on_sensors(self, msg):
            super()._on_sensors(msg)
            self.pressure_received = time.monotonic()

        def pressure_age(self):
            return math.inf if self.pressure_received is None else time.monotonic() - self.pressure_received

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output = args.out or Path.home() / 'result/ph' / args.actuator / f'{stamp}_{plan["id"]}'
    output.mkdir(parents=True, exist_ok=False)
    snapshots = {}
    file_encoder = {}
    for name in ('powerpack_config.yaml', 'valve_params.yaml', 'pump_params.yaml', 'encoder_params.yaml'):
        path = CONFIG / name
        snapshots[name] = (dict(path=str(path), text=path.read_text(),
                                sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                           if path.exists() else None)
        if path.exists():
            document = yaml.safe_load(snapshots[name]['text']) or {}
            channel = document.get('/pack2/can_bridge', {}).get('ros__parameters', {}).get(
                'TeensyEncoder', {}).get('channels', {}).get('0', {})
            file_encoder.update(channel)
    meta = dict(start_wall=wall(), actuator=args.actuator, mount=args.mount, axis=1,
                profile=plan, profile_sha256=digest(plan), commit=git('rev-parse', 'HEAD'),
                baseline_commit='f3cd4af', git_status=git('status', '--porcelain'),
                config_files=snapshots, launch_command=args.launch_command,
                launch_command_source='operator declaration; effective parameters read separately',
                encoder=dict(raw_0deg=ENCODERS[args.actuator][0], raw_90deg=ENCODERS[args.actuator][1],
                             source=REGISTRY_SOURCE, merged_file_channel0=file_encoder), note=args.note,
                pressure_timeout_s=1., send_hz=50, log_hz=100,
                point_mechanics=dict(mass_kg=2., link_length_m=.15, reel_radius_m=.025,
                                     gravity_m_s2=G, angle_offset_deg=0.),
                timing=dict(log_samples=0, max_log_gap_s=0., send_packets=0, max_send_gap_s=0.),
                freshness_scope='ROS message age, not per-board CAN frame age',
                release=dict(target_send_complete=False, error=None), status='preflight')
    save(output / 'meta.json', meta)
    cfg = read_config(str(CONFIG / 'powerpack_config.yaml'))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    rig = View(cfg)
    conn = drv = None
    guard = Guard(max_rate=25, ang_min=-5, ang_max=88, track_tol=4,
                  track_grace=5, enc_timeout=1, osc_window=4, osc_flips=6)
    began = time.monotonic()
    meta['start_mono_s'] = began
    label = 'preflight'
    rate, last_angle, last_angle_time = 0., None, None
    run_file = (output / 'run.csv').open('x', newline='', buffering=1)
    log = csv.writer(run_file)
    log.writerow(['t_mono_s', 'segment', 'ref_pos_kpa', 'ref_neg_kpa', 'p_pos_kpa',
                  'p_neg_kpa', 'angle_deg', 'angle_age_s', 'pressure_age_s',
                  'rail_pos_kpa', 'rail_neg_kpa', 'sensor_valid', 'rate_deg_s'])
    points_file = (output / 'points.csv').open('x', newline='', buffering=1)
    point_log = csv.writer(points_file)
    point_log.writerow(['idx', 'sweep', 'center_kpa', 'diff_kpa', 'p_pos_ref', 'p_neg_ref',
                        'p_pos_meas', 'p_neg_meas', 'angle_deg', 'angle_std_deg', 'settled',
                        'settle_s', 'n_sample', 'tau_grav_Nm', 'force_N',
                        'p_pos_std_kpa', 'p_neg_std_kpa', 'segment'])
    next_log = began
    last_log = last_send = None

    def state():
        return [rig.angle(), rig.kpa(POS_BOARD(0)), rig.kpa(NEG_BOARD(0)), rig.kpa(1), rig.kpa(2)]

    def sample(enforce=True):
        nonlocal next_log, rate, last_angle, last_angle_time, last_log
        rclpy.spin_once(rig, timeout_sec=0.)
        now = time.monotonic()
        angle, pos, neg, rail_pos, rail_neg = state()
        valid = (all(v is not None and math.isfinite(v) for v in (angle, pos, neg, rail_pos, rail_neg))
                 and rig.angle_age() <= 1 and rig.pressure_age() <= 1)
        if angle is not None and math.isfinite(angle) and rig._ang_t != last_angle_time:
            if last_angle_time is not None:
                dt = rig._ang_t - last_angle_time
                if dt > 0:
                    rate += min(1., dt / .3) * ((angle - last_angle) / dt - rate)
            last_angle, last_angle_time = angle, rig._ang_t
        refs = (drv.ref_pos, drv.ref_neg) if drv else (None, None)
        if now >= next_log:
            log.writerow([now - began, label, *refs, pos, neg, angle, rig.angle_age(),
                          rig.pressure_age(), rail_pos, rail_neg, int(valid), rate])
            meta['timing']['log_samples'] += 1
            if last_log is not None:
                meta['timing']['max_log_gap_s'] = max(meta['timing']['max_log_gap_s'], now-last_log)
            last_log = now
            next_log += .01
            if next_log < now:
                next_log = now + .01  # 늦어진 샘플을 과거 시각으로 복제하지 않는다.
        if enforce:
            if not valid:
                raise Abort('axis 1: missing/non-finite/stale sensor (ROS message timeout 1 s)')
            guard.check(rig, angle, rate, pos, neg, drv.ref_pos, drv.ref_neg)
        return valid, (angle, pos, neg)

    def params(node):
        def request(kind, suffix, request):
            client = rig.create_client(kind, f'{node}/{suffix}')
            try:
                if not client.wait_for_service(timeout_sec=5.):
                    raise Abort(f'Parameter service missing: {node}')
                future = client.call_async(request)
                deadline = time.monotonic() + 5
                while not future.done() and time.monotonic() < deadline:
                    sample(False)
                    time.sleep(.002)
                if not future.done() or future.result() is None:
                    raise Abort(f'Parameter read failed: {node}')
                return future.result()
            finally:
                rig.destroy_client(client)
        names = request(ListParameters, 'list_parameters', ListParameters.Request(depth=0)).result.names
        result = {}
        for start in range(0, len(names), 100):
            batch = names[start:start+100]
            values = request(GetParameters, 'get_parameters', GetParameters.Request(names=batch)).values
            result.update(zip(batch, map(parameter_value_to_python, values)))
        return result

    def execute(segment, enforce=True):
        nonlocal label, last_send
        label = segment['segment']
        started = time.monotonic()
        next_send = started
        quiet = None
        quiet_samples = []
        finished_send = False
        while True:
            try:
                valid, values = sample(enforce)
            except Exception as exc:
                if enforce:
                    raise
                # ROS/디스크 오류가 복귀 패킷 송신까지 막지 않도록 분리한다.
                meta['release']['telemetry_error'] = f'{type(exc).__name__}: {exc}'
                valid, values = False, (None, None, None)
            now = time.monotonic()
            elapsed = now - started
            if now >= next_send:
                # 동일 보간계수로 두 챔버를 이동: center 변경에서도 차압 영역 유지.
                u = min(1., elapsed / segment['duration_s'])
                if segment['mode'] == 'cosine':
                    u = (1 - math.cos(math.pi * u)) / 2
                targets = [a + (b - a) * u for a, b in zip(segment['start'], segment['end'])]
                drv.ref_pos, drv.ref_neg = targets
                drv.hold()
                meta['timing']['send_packets'] += 1
                if last_send is not None:
                    meta['timing']['max_send_gap_s'] = max(meta['timing']['max_send_gap_s'], now-last_send)
                last_send = now
                finished_send = u >= 1
                next_send = now + .02  # 지연 후 패킷 몰아 보내기 금지
            if segment['mode'] == 'settle':
                if valid and abs(rate) < .3:
                    if quiet is None:
                        quiet = now
                    quiet_samples.append(values)
                    if now - quiet >= 3:
                        return True, quiet_samples, elapsed
                else:
                    quiet, quiet_samples = None, []
            if elapsed >= segment['duration_s'] and finished_send:
                return segment['mode'] != 'settle', quiet_samples, elapsed
            time.sleep(.001)

    rc = 1
    try:
        actual = {node: params(node) for node in ('/pack2/can_bridge', '/pack2/pp_controller')}
        meta['effective_parameters'] = actual
        save(output / 'meta.json', meta)
        bridge, controller = actual['/pack2/can_bridge'], actual['/pack2/pp_controller']
        if bridge.get('encoder_source') != 'teensy' or bridge.get('teensy_enable') is not True:
            raise Abort('Requires enabled Teensy encoder source')
        for key, value in zip(('raw_0deg', 'raw_90deg'), ENCODERS[args.actuator]):
            if bridge.get(f'TeensyEncoder.channels.0.{key}') != value:
                raise Abort(f'Actuator {args.actuator}: active encoder calibration mismatch ({key})')
        if controller.get('control_mode') != 0 or controller.get('RefTcpServer.all_channels') is not True:
            raise Abort('Requires control_mode=0 and RefTcpServer.all_channels=true')
        if controller.get('RefTcpServer.enable') is not True:
            raise Abort('RefTcpServer is not enabled')
        expected = {'num_positive_channels': 6, 'num_total_channels': 12, 'num_actuators': 1,
                    'channel_board_offset': 4, 'line_pressure_boards.pos': 1,
                    'line_pressure_boards.neg': 2, 'PositionController.axis0.pos_gid': 0,
                    'PositionController.axis0.neg_gid': 6, 'PositionController.axis0.actuator_idx': 0,
                    'RefTcpServer.port': args.port}
        for key, value in expected.items():
            if controller.get(key) != value:
                raise Abort(f'Axis 1 interface mismatch: {key}, expected {value}')
        # 실제 제어기가 쓰는 센서 환산과 파일 기반 RigView 환산이 같아야 한다.
        for board in (1, 2, POS_BOARD(0), NEG_BOARD(0)):
            for suffix, value in (('offset', cfg['offs'][board]), ('gain', cfg['gains'][board])):
                if controller.get(f'Sensor_calibration.boards.{board}.{suffix}') != value:
                    raise Abort('Active pressure calibration differs from recorded base YAML')
        if controller.get('Sensor_calibration.atm_offset') != cfg['atm']:
            raise Abort('Active pressure atmosphere offset mismatch')
        valid, values = sample(False)
        if not valid or any(abs(p - ATM) > 4 for p in values[1:]):
            raise Abort('Before connecting, fresh sensors and chambers near atmosphere are required')
        if not -5 <= values[0] <= 88:
            raise Abort('Initial angle outside [-5, 88]')
        if abs(rig.kpa(1) - 200) > 8 or abs(rig.kpa(2) - 30) > 8:
            raise Abort('Rails not within 8 kPa of 200/30; no runtime profile clipping')
        conn = _connect(args)
        conn.settimeout(.1)
        drv = Driver(conn, [0], 50, 2)
        meta['status'] = 'running'
        meta['preflight_verified'] = True
        save(output / 'meta.json', meta)
        for i, segment in enumerate(plan['segments']):
            settled, values, duration = execute(segment)
            if segment['mode'] == 'settle':
                columns = list(zip(*values)) if values else [[], [], []]
                mean = [statistics.mean(v) if v else None for v in columns]
                std = [statistics.pstdev(v) if v else None for v in columns]
                p, n = segment['end']
                # sweep은 직전 이동의 차압 방향. 첫 점은 up으로 표시.
                ramp = plan['segments'][i-1]
                before = ramp['start'][0] - ramp['start'][1]
                direction = 'down' if p - n < before - 1e-9 else 'up'
                torque = 2 * G * .15 * math.sin(math.radians(mean[0])) if mean[0] is not None else None
                point_log.writerow([i, direction, (p+n)/2, p-n, p, n, mean[1], mean[2],
                                    mean[0], std[0], int(settled), duration, len(values),
                                    torque, torque/.025 if torque is not None else None,
                                    std[1], std[2], segment['segment']])
        meta['status'], rc = 'completed', 0
    except (Exception, KeyboardInterrupt) as exc:
        meta['status'] = 'aborted'
        meta['abort'] = dict(axis=1, segment=label, reason=f'{type(exc).__name__}: {exc}', wall=wall())
        print(f'ABORT: {meta["abort"]["reason"]}')
    finally:
        # 첫 Ctrl-C 후 복귀 동안 추가 Ctrl-C가 복귀를 끊지 않게 한다.
        previous_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if drv is not None:
                try:
                    if meta['status'] != 'completed':
                        start = [drv.ref_pos, drv.ref_neg]
                        duration = max(abs(v - ATM) for v in start) / 4
                        if duration > 0:
                            execute(dict(segment='shutdown/release', start=start, end=[ATM, ATM],
                                         duration_s=duration, mode='linear'), enforce=False)
                        execute(dict(segment='shutdown/atmosphere', start=[ATM, ATM], end=[ATM, ATM],
                                     duration_s=5., mode='hold'), enforce=False)
                    meta['release']['target_send_complete'] = True
                except Exception as exc:
                    meta['release']['error'] = f'{type(exc).__name__}: {exc}'
                    rc = 1
                meta['release']['final_measured'] = dict(zip(
                    ('angle_deg', 'p_pos_kpa', 'p_neg_kpa', 'rail_pos_kpa', 'rail_neg_kpa'), state()))
                meta['release']['pressure_age_s'] = rig.pressure_age()
                # 목표 송신과 실측 도달은 별개이며 성공이라고 추정하지 않는다.
            else:
                meta['release']['error'] = 'No TCP driver established; no release commanded'
            if conn is not None:
                conn.close()
            meta['end_wall'] = wall()
            save(output / 'meta.json', meta)
        finally:
            run_file.close()
            points_file.close()
            rig.destroy_node()
            rclpy.shutdown()
            signal.signal(signal.SIGINT, previous_handler)
    print(f'Results: {output}; release target sent: {meta["release"]["target_send_complete"]}')
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
