#!/usr/bin/env python3
"""
pp_results.py — ~/result 실행 결과 정리·조회

pp_logger 의 CSV 형식은 시간이 지나며 바뀌었다. 과거 결과를 지우지 않고
그대로 두되, 각 실행이 어떤 형식인지·무엇이 들어 있는지 한눈에 보이게 한다.

  pp_results.py list                  실행 목록 + 형식(schema) + 요약
  pp_results.py show <ts>             한 실행의 상세
  pp_results.py index                 ~/result/INDEX.md 갱신 + meta.json 백필
  pp_results.py prune [--apply]       빈/짧은 실행 정리 (기본은 미리보기)
"""
import csv
import json
import os
import sys
from datetime import datetime

RESULT_DIR = os.path.expanduser('~/result')
CURRENT_SCHEMA = 2

# 형식 판별: 해당 열이 있으면 그 스키마다. 위에서부터 먼저 맞는 것.
SCHEMA_MARKERS = [
    (2, 'pwm_pct_pos_bd5_v1micro_axis0'),   # 밸브 지령·전류·부피 포함
    (1, 'mpc_ref_gid0_kpa'),                # 압력 레퍼런스 포함
    (0, 'angle_deg_axis0'),                 # 각도만
]


def _runs():
    if not os.path.isdir(RESULT_DIR):
        return []
    out = []
    for name in sorted(os.listdir(RESULT_DIR)):
        d = os.path.join(RESULT_DIR, name)
        if not os.path.isdir(d):
            continue
        csvs = sorted(f for f in os.listdir(d) if f.endswith('.csv'))
        out.append((name, d, csvs))
    return out


def _probe(path):
    """CSV 를 한 번만 훑어 형식과 요약을 낸다. 전체를 메모리에 올리지 않는다."""
    info = {'schema': None, 'rows': 0, 'dur_s': 0.0, 'cols': 0,
            'err_mean': None, 'err_max': None,
            'p_ref': None, 'p_min': None, 'p_max': None,
            'pwm_max': None, 'cur_max': None}
    try:
        with open(path, newline='') as f:
            r = csv.reader(f)
            try:
                header = next(r)
            except StopIteration:
                return info
            info['cols'] = len(header)
            idx = {h: i for i, h in enumerate(header)}
            for ver, marker in SCHEMA_MARKERS:
                if marker in idx:
                    info['schema'] = ver
                    break

            def col(name):
                return idx.get(name)

            c_t   = col('time_sec')
            c_a   = col('angle_deg_axis0')
            c_tg  = col('target_deg_axis0')
            c_ref = col('mpc_ref_gid0_kpa')
            c_act = col('p_pos_actual_kpa_axis0')
            c_pwm = [col(f'pwm_pct_pos_bd5_{v}_axis0')
                     for v in ('v1micro', 'v2atm', 'v3macro')]
            c_cur = [col(f'cur_mA_pos_bd5_{v}_axis0')
                     for v in ('v1micro', 'v2atm', 'v3macro')]

            n = 0
            esum = 0.0; emax = 0.0
            refs = []; pmin = 1e9; pmax = -1e9
            pwm_max = [0.0, 0.0, 0.0]; cur_max = [0.0, 0.0, 0.0]
            last_t = 0.0
            for row in r:
                if len(row) < info['cols']:
                    continue
                n += 1
                try:
                    if c_t is not None:
                        last_t = float(row[c_t])
                    if c_a is not None and c_tg is not None:
                        e = abs(float(row[c_tg]) - float(row[c_a]))
                        esum += e; emax = max(emax, e)
                    if c_act is not None:
                        v = float(row[c_act]); pmin = min(pmin, v); pmax = max(pmax, v)
                    if c_ref is not None:
                        refs.append(float(row[c_ref]))
                    for k in range(3):
                        if c_pwm[k] is not None:
                            pwm_max[k] = max(pwm_max[k], float(row[c_pwm[k]]))
                        if c_cur[k] is not None:
                            cur_max[k] = max(cur_max[k], float(row[c_cur[k]]))
                except ValueError:
                    continue
            info['rows'] = n
            info['dur_s'] = last_t
            if n and c_a is not None:
                info['err_mean'] = esum / n
                info['err_max'] = emax
            if pmin < 1e9:
                info['p_min'] = pmin; info['p_max'] = pmax
            if refs:
                info['p_ref'] = (min(refs), max(refs))
            if any(c is not None for c in c_pwm):
                info['pwm_max'] = pwm_max
            if any(c is not None for c in c_cur):
                info['cur_max'] = cur_max
    except OSError:
        pass
    return info


def _fmt_ts(name):
    try:
        return datetime.strptime(name, '%Y%m%d_%H%M%S').strftime('%m-%d %H:%M')
    except ValueError:
        return name


def cmd_list(argv):
    runs = _runs()
    if not runs:
        print(f'{RESULT_DIR} 에 실행 결과가 없다'); return
    print(f'{"실행":17s} {"때":12s} {"형":>2s} {"행":>6s} {"초":>6s} '
          f'{"각도오차 평균/최대":>18s}  압력 kPa')
    print('-' * 96)
    counts = {}
    for name, d, csvs in runs:
        if not csvs:
            print(f'{name:17s} {_fmt_ts(name):12s}  -      0      0   (CSV 없음)')
            counts['빈 실행'] = counts.get('빈 실행', 0) + 1
            continue
        i = _probe(os.path.join(d, csvs[0]))
        sc = i['schema']
        counts[f'schema {sc}'] = counts.get(f'schema {sc}', 0) + 1
        err = (f"{i['err_mean']:7.2f}/{i['err_max']:7.2f}°"
               if i['err_mean'] is not None else f"{'-':>17s}")
        pr = ''
        if i['p_min'] is not None:
            pr = f"실측 {i['p_min']:6.1f}~{i['p_max']:6.1f}"
            if i['p_ref']:
                pr += f"  ref {i['p_ref'][0]:6.1f}~{i['p_ref'][1]:6.1f}"
        print(f"{name:17s} {_fmt_ts(name):12s} {sc if sc is not None else '?':>2} "
              f"{i['rows']:6d} {i['dur_s']:6.1f} {err}  {pr}")
    print('-' * 96)
    print('형식별: ' + ', '.join(f'{k}={v}' for k, v in sorted(counts.items()))
          + f'   (현재 형식 = schema {CURRENT_SCHEMA})')


def cmd_show(argv):
    if not argv:
        print('사용법: pp_results.py show <타임스탬프>'); return 2
    name = argv[0]
    d = os.path.join(RESULT_DIR, name)
    if not os.path.isdir(d):
        print(f'없다: {d}'); return 2
    csvs = sorted(f for f in os.listdir(d) if f.endswith('.csv'))
    print(f'경로 : {d}')
    for f in sorted(os.listdir(d)):
        print(f'  {f}  ({os.path.getsize(os.path.join(d, f)) / 1e6:.2f} MB)')
    mp = os.path.join(d, 'meta.json')
    if os.path.exists(mp):
        m = json.load(open(mp))
        print(f"\nschema {m.get('schema_version')}  git {m.get('git_commit')}  "
              f"{m.get('log_hz')} Hz  {m.get('num_axes')}축")
    if not csvs:
        return 0
    i = _probe(os.path.join(d, csvs[0]))
    print(f"\n형식 schema {i['schema']}   {i['rows']}행 {i['cols']}열   {i['dur_s']:.1f}초")
    if i['err_mean'] is not None:
        print(f"  각도 오차   평균 {i['err_mean']:.2f}°  최대 {i['err_max']:.2f}°")
    if i['p_min'] is not None:
        print(f"  압력 실측   {i['p_min']:.1f} ~ {i['p_max']:.1f} kPa")
    if i['p_ref']:
        print(f"  압력 레퍼런스 {i['p_ref'][0]:.1f} ~ {i['p_ref'][1]:.1f} kPa")
    if i['pwm_max']:
        print('  밸브 지령 최대  ' + '  '.join(
            f'{n} {v:.1f}%' for n, v in zip(('v1micro', 'v2atm', 'v3macro'), i['pwm_max'])))
    if i['cur_max']:
        print('  실측 전류 최대  ' + '  '.join(
            f'{n} {v:.1f}mA' for n, v in zip(('v1micro', 'v2atm', 'v3macro'), i['cur_max'])))
    return 0


def cmd_index(argv):
    """INDEX.md 를 만들고, meta.json 이 없는 과거 실행에 형식만이라도 백필한다."""
    runs = _runs()
    lines = ['# 실행 결과 목록', '',
             f'생성: {datetime.now():%Y-%m-%d %H:%M}   현재 형식 = schema {CURRENT_SCHEMA}', '',
             '| 실행 | 형식 | 행 | 길이 [s] | 각도오차 평균/최대 [°] | 압력 실측 [kPa] |',
             '|---|---|---|---|---|---|']
    backfilled = 0
    for name, d, csvs in runs:
        if not csvs:
            lines.append(f'| {name} | – | 0 | 0 | – | CSV 없음 |')
            continue
        i = _probe(os.path.join(d, csvs[0]))
        err = (f"{i['err_mean']:.2f} / {i['err_max']:.2f}"
               if i['err_mean'] is not None else '–')
        pr = (f"{i['p_min']:.1f} ~ {i['p_max']:.1f}" if i['p_min'] is not None else '–')
        lines.append(f"| {name} | {i['schema']} | {i['rows']} | {i['dur_s']:.1f} | {err} | {pr} |")
        mp = os.path.join(d, 'meta.json')
        if not os.path.exists(mp):
            try:
                json.dump({'schema_version': i['schema'], 'timestamp': name,
                           'csv': csvs[0], 'backfilled': True},
                          open(mp, 'w'), indent=2, ensure_ascii=False)
                backfilled += 1
            except OSError:
                pass
    p = os.path.join(RESULT_DIR, 'INDEX.md')
    open(p, 'w').write('\n'.join(lines) + '\n')
    print(f'{p} 갱신 ({len(runs)}건, meta.json 백필 {backfilled}건)')


def cmd_prune(argv):
    """CSV 가 없거나 2초 미만인 실행을 정리 대상으로 본다. --apply 없이는 지우지 않는다."""
    apply = '--apply' in argv
    victims = []
    for name, d, csvs in _runs():
        if not csvs:
            victims.append((name, d, 'CSV 없음')); continue
        i = _probe(os.path.join(d, csvs[0]))
        if i['rows'] < 200 or i['dur_s'] < 2.0:
            victims.append((name, d, f"{i['rows']}행 {i['dur_s']:.1f}초"))
    if not victims:
        print('정리할 실행이 없다'); return
    for name, d, why in victims:
        print(f'  {name}  ({why})')
    if not apply:
        print(f'\n{len(victims)}건이 대상이다. 실제로 지우려면 --apply 를 붙여라.')
        return
    import shutil
    for _n, d, _w in victims:
        shutil.rmtree(d, ignore_errors=True)
    print(f'\n{len(victims)}건 삭제')


def main():
    cmds = {'list': cmd_list, 'show': cmd_show, 'index': cmd_index, 'prune': cmd_prune}
    argv = sys.argv[1:]
    if not argv or argv[0] not in cmds:
        print(__doc__); return 0
    return cmds[argv[0]](argv[1:]) or 0


if __name__ == '__main__':
    sys.exit(main())
