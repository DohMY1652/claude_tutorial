#!/usr/bin/env python3
"""오프라인 전용 압력 계획 생성기. ROS·소켓·하드웨어 접근 없음.

JSON segments가 정착/보간을 포함하는 원본 계획이다. CSV는 시간형 표본 또는
정착형 목표 목록이며, 기존 pressure_id_seq.py의 입력 파일 형식이 아니다.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import random

ATM = 101.325
NAMES = ('R0', 'S1a', 'S1b', 'S2', 'S3', 'S4', 'S5', 'S6')


def pair(center, diff):
    return [center + diff / 2, center - diff / 2]


def allowed(p, d_max):
    return (103 - 1e-9 <= p[0] <= 185 + 1e-9
            and 40 - 1e-9 <= p[1] <= 100 + 1e-9
            and -1e-9 <= p[0] - p[1] <= d_max + 1e-9)


def peak_slew(segment):
    delta = max(abs(b - a) for a, b in zip(segment['start'], segment['end']))
    factor = math.pi / 2 if segment['mode'] == 'cosine' else 1
    return delta * factor / segment['duration_s']


def generate(name, seed=0, d_max=72):
    if name not in NAMES or not math.isfinite(d_max) or not 0 < d_max <= 72:
        raise ValueError('Unknown profile or d_max outside (0, 72]')
    plan = dict(schema_version=1, id=name, seed=seed, d_max=d_max,
                limits=dict(pos=[103, 185], neg=[40, 100], rails=[200, 30],
                            rail_margin=10, atmosphere_kpa=ATM),
                excluded_points=[], segments=[],
                settle=dict(rate_deg_s=0.3, quiet_s=3, timeout_s=60, rate_tau_s=0.3))
    current = [ATM, ATM]

    def add(end, duration, mode, kind, label, slew=2):
        nonlocal current
        plan['segments'].append(dict(segment=f'{name}/{len(plan["segments"]):03d}/{label}',
                                     kind=kind, mode=mode, start=current[:], end=list(end),
                                     duration_s=duration, ramp_kpa_s=slew))
        current = list(end)

    def hold(seconds, label, kind='measurement', settle=False):
        add(current, seconds, 'settle' if settle else 'hold', kind, label)

    def move(end, label, rate=2, kind='measurement'):
        duration = max(abs(b - a) for a, b in zip(current, end)) / rate
        if duration > 1e-12:
            add(end, duration, 'linear', kind, label, rate)

    def enter(end, label='entry', rate=2):
        move(end, label, rate, 'entry')

    def release(rate=4):
        move([ATM, ATM], 'release', rate, 'release')

    def point(end, label):
        move(end, label)
        hold(60, label + '/settle', settle=True)

    hold(5, 'start', 'atmosphere')
    low = pair(101.3, 3.4)
    if name in ('R0', 'S3'):
        for rate in ([2] if name == 'R0' else [0.5, 1, 2, 3]):
            enter(low, rate=rate)
            move(pair(101.3, 60 if name == 'R0' else 66), f'up-{rate}', rate)
            move(low, f'down-{rate}', rate)
            release(rate)
    elif name in ('S1a', 'S1b'):
        values = ([103, 111, 119, 127, 135, 143, 151, 159, 167, 172]
                  if name == 'S1a' else [100, 92, 84, 76, 68, 60, 52, 44, 40])
        enter([103, 100])
        for i, value in enumerate(values + values[-2::-1]):
            point([value, 100] if name == 'S1a' else [103, value], f'point-{i}')
    elif name == 'S2':
        for center in (101.3, 108, 115):
            points = []
            for diff in (0, 8, 16, 24, 32, 40, 48, 54, 60, 66, 72):
                target = pair(center, diff)
                if allowed(target, d_max):
                    points.append(target)
                else:
                    plan['excluded_points'].append(dict(center_kpa=center, diff_kpa=diff,
                        reason='outside measurement pressure box or d_max'))
            if not points:
                raise ValueError(f'No valid S2 points at center={center}')
            if current == [ATM, ATM]:
                enter(points[0])
            for i, target in enumerate(points + points[-2::-1]):
                point(target, f'c{center}-point-{i}')
    elif name == 'S4':
        enter(low)
        for diff in (24, 48, 60):
            move(pair(101.3, diff), f'from-below-{diff}')
            hold(120, f'hold-below-{diff}')
            move(pair(101.3, diff + 12), f'overshoot-{diff}')
            move(pair(101.3, diff), f'from-above-{diff}')
            hold(120, f'hold-above-{diff}')
    elif name == 'S5':
        # 표에 외곽 정점이 없으므로 S3와 같은 66을 명시적 계획 파라미터로 사용.
        plan['outer_diff_kpa'] = 66
        enter(low, rate=1)
        for diff in (20, 40, 60):
            for value in (diff, diff - 6, diff):
                move(pair(101.3, value), f'up-loop-{diff}-{value}', 1)
        move(pair(101.3, 66), 'peak', 1)
        for diff in (60, 40, 20):
            for value in (diff, diff + 6, diff):
                move(pair(101.3, value), f'down-loop-{diff}-{value}', 1)
        move(low, 'down', 1)
    else:
        rng = random.Random(seed)
        anchor = pair(108, min(40, d_max))
        if not allowed(anchor, d_max):
            raise ValueError('S6 requires d_max >= 16')
        enter(anchor)
        remaining = 600.0
        while remaining > 60:
            duration = rng.uniform(8, 30)
            for _ in range(10000):
                center = rng.uniform(101.3, 115)
                minimum = max(2 * (103 - center), 2 * (center - 100))
                if minimum > d_max:
                    continue
                target = pair(center, rng.uniform(minimum, d_max))
                delta = max(abs(b - a) for a, b in zip(current, target))
                if allowed(target, d_max) and delta * math.pi / (2 * duration) <= 2:
                    break
            else:
                raise ValueError('Unable to sample a slew-limited random waypoint')
            add(target, duration, 'cosine', 'measurement', 'random')
            pause = rng.uniform(0, 15)
            if pause:
                hold(pause, 'random-hold')
            remaining -= duration + pause
        duration = max(8, max(abs(b - a) for a, b in zip(current, anchor)) * math.pi / 4)
        if duration > 30 or duration > remaining:
            raise ValueError('S6 return does not fit the time budget')
        add(anchor, duration, 'cosine', 'measurement', 'return')
        remaining -= duration
        while remaining > 1e-9:
            pause = min(15, remaining)
            hold(pause, 'budget-hold')
            remaining -= pause
        plan['measurement_duration_s'] = 600
    release()
    hold(5, 'end', 'atmosphere')
    validate(plan)
    return plan


def validate(plan):
    segments = plan['segments']
    d_max = plan['d_max']
    if not math.isfinite(d_max) or not 0 < d_max <= 72:
        raise ValueError('Invalid d_max')
    previous = [ATM, ATM]
    for s in segments:
        values = s['start'] + s['end'] + [s['duration_s'], s['ramp_kpa_s']]
        if not all(math.isfinite(v) for v in values) or s['duration_s'] <= 0:
            raise ValueError('Non-finite values or nonpositive duration')
        if s['start'] != previous:
            raise ValueError('Discontinuous segments')
        if s['mode'] not in ('linear', 'cosine', 'hold', 'settle'):
            raise ValueError('Unknown interpolation')
        if s['mode'] in ('hold', 'settle') and s['start'] != s['end']:
            raise ValueError('Hold must be constant')
        if s['kind'] == 'measurement':
            if not all(allowed(p, d_max) for p in (s['start'], s['end'])):
                raise ValueError('Measurement outside pressure/diff limits; no clipping')
        elif s['kind'] == 'atmosphere':
            if s['start'] != [ATM, ATM] or s['end'] != [ATM, ATM]:
                raise ValueError('Atmosphere must be exact')
        elif s['kind'] in ('entry', 'release'):
            atm_end, measurement_end = ((s['start'], s['end']) if s['kind'] == 'entry'
                                        else (s['end'], s['start']))
            if atm_end != [ATM, ATM] or not allowed(measurement_end, d_max):
                raise ValueError('Transition must connect atmosphere and measurement')
            if s['mode'] != 'linear':
                raise ValueError('Transition must be a coupled linear ramp')
        else:
            raise ValueError('Unknown segment kind')
        limit = 4 if s['kind'] == 'release' else (3 if plan['id'] == 'S3' else 2)
        if not 0 < s['ramp_kpa_s'] <= limit or peak_slew(s) > s['ramp_kpa_s'] + 1e-9:
            raise ValueError('Channel slew exceeded')
        previous = s['end']
    for s in (segments[0], segments[-1]):
        if s['kind'] != 'atmosphere' or s['duration_s'] != 5:
            raise ValueError('Requires initial/final five-second atmosphere holds')


def digest(plan):
    return hashlib.sha256(json.dumps(plan, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def summary(plan):
    segments = plan['segments']
    timed = sum(s['duration_s'] for s in segments if s['mode'] != 'settle')
    count = sum(s['mode'] == 'settle' for s in segments)
    maximum = [max(abs(s['end'][i]-s['start'][i]) *
                   (math.pi/2 if s['mode'] == 'cosine' else 1) / s['duration_s']
                   for s in segments) for i in (0, 1)]
    return dict(id=plan['id'], seed=plan['seed'], sha256=digest(plan),
                duration_min_s=timed + count * 3, duration_max_s=timed + count * 60,
                settling_points=count, excluded_points=len(plan['excluded_points']),
                pressure_bounds_kpa=dict(pos=[min(s['end'][0] for s in segments),
                                              max(s['end'][0] for s in segments)],
                                         neg=[min(s['end'][1] for s in segments),
                                              max(s['end'][1] for s in segments)]),
                max_pos_slew_kpa_s=maximum[0], max_neg_slew_kpa_s=maximum[1],
                max_channel_slew_kpa_s=max(map(peak_slew, segments)),
                max_diff_kpa=max(s['end'][0] - s['end'][1] for s in segments))


def samples(plan, hz=50):
    """정착형의 그림에서는 timeout을 사용한다. 실기 재생에 쓰지 않는다."""
    elapsed = 0.0
    yield elapsed, plan['segments'][0], [ATM, ATM]
    for s in plan['segments']:
        steps = max(1, math.ceil(s['duration_s'] * hz))
        for i in range(1, steps + 1):
            u = i / steps
            fraction = (1 - math.cos(math.pi * u)) / 2 if s['mode'] == 'cosine' else u
            p = [a + (b - a) * fraction for a, b in zip(s['start'], s['end'])]
            yield elapsed + s['duration_s'] * u, s, p
        elapsed += s['duration_s']


def export(plan, directory, plot=True):
    validate(plan)
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    stem = directory / f'{plan["id"]}_{plan["seed"]}'
    # 기존 계획을 덮어쓰지 않는다. 다른 설정·해시는 별도 폴더에 저장한다.
    paths = [stem.with_suffix(ext) for ext in ('.csv', '.json')]
    if plot:
        paths.append(stem.with_suffix('.png'))
    if any(p.exists() for p in paths):
        raise FileExistsError(f'Profile already exists: {stem}')
    settling = any(s['mode'] == 'settle' for s in plan['segments'])
    with paths[0].open('x', newline='') as stream:
        writer = csv.writer(stream)
        if settling:
            writer.writerow(['idx', 'segment', 'center_kpa', 'diff_kpa', 'p_pos_ref',
                             'p_neg_ref', 'ramp_kpa_s', 'kind', 'mode', 'duration_s'])
            for i, s in enumerate(plan['segments']):
                p, n = s['end']
                writer.writerow([i, s['segment'], (p+n)/2, p-n, p, n,
                                 s['ramp_kpa_s'], s['kind'], s['mode'], s['duration_s']])
        else:
            writer.writerow(['t_s', 'segment', 'center_kpa', 'diff_kpa', 'p_pos_ref',
                             'p_neg_ref', 'kind'])
            for t, s, (p, n) in samples(plan):
                writer.writerow([t, s['segment'], (p+n)/2, p-n, p, n, s['kind']])
    with paths[1].open('x') as stream:
        json.dump(dict(plan=plan, summary=summary(plan), csv_sha256=hashlib.sha256(
            paths[0].read_bytes()).hexdigest()), stream, indent=2, allow_nan=False)
    if plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        data = list(samples(plan, hz=2))
        fig, ax = plt.subplots(figsize=(12, 4))
        for i, label in enumerate(('P+', 'P-')):
            ax.plot([r[0] for r in data], [r[2][i] for r in data], label=label)
        ax.set(xlabel='Time [s] (settling: timeout envelope)', ylabel='Reference [kPa abs]',
               title=f'{plan["id"]} seed={plan["seed"]} — OFFLINE PLAN ONLY')
        ax.legend()
        fig.tight_layout()
        fig.savefig(paths[2])
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=(*NAMES, 'all'), default='all')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--suite', choices=['standard', 'full'], default='standard',
                        help='all: S6 standard=train 0,1 / validation 4; full=train 0..3 / validation 4,5. seed is offset')
    parser.add_argument('--d-max', type=float, default=72)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--no-plot', action='store_true')
    args = parser.parse_args()
    plans = [generate(n, args.seed, args.d_max) for n in
             (NAMES[:-1] if args.profile == 'all' else [args.profile])]
    if args.profile == 'all':
        offsets = (0, 1, 4) if args.suite == 'standard' else tuple(range(6))
        plans.extend(generate('S6', args.seed + i, args.d_max) for i in offsets)
        if (args.out / 'suite.json').exists():
            raise FileExistsError('suite.json already exists')
    for plan in plans:
        export(plan, args.out, plot=not args.no_plot)
        print(json.dumps(summary(plan), ensure_ascii=False))
    if args.profile == 'all':
        entries = {p['id']: f'{p["id"]}_{p["seed"]}.json' for p in plans if p['id'] != 'S6'}
        random_files = [f'S6_{args.seed + i}.json' for i in offsets]
        order = [entries[n] for n in ('R0', 'S2', 'S1a', 'S1b', 'S3', 'S5', 'S4')]
        manifest = dict(suite=args.suite, order=order + random_files + [entries['R0']],
                        random_roles={f'S6_{args.seed+i}.json': ('training' if i < 4 else 'validation')
                                      for i in offsets},
                        profile_sha256={f'{p["id"]}_{p["seed"]}.json': digest(p) for p in plans})
        with (args.out / 'suite.json').open('x') as stream:
            json.dump(manifest, stream, indent=2)


if __name__ == '__main__':
    main()
