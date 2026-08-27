#!/usr/bin/env python3
"""피팅 산출물을 운영 config 로 안전하게 병합한다.

`valve_fit_solve.py` 가 낸 valve_params.yaml 을 그대로 config/ 에 넣으면 두 가지로
노드가 죽는다 (HANDOFF 2-2 에 기록된 사고):

  1. 파이썬이 같은 dict 를 참조로 재사용하면 PyYAML 이 anchor/alias(&id001/*id001)로
     압축해 저장하는데, ROS 2 의 rcl_yaml_param_parser 는 alias 를 지원하지 않는다
     → "Will not support aliasing at line N", 두 노드 동시 SIGABRT.
  2. 빈 리스트(`weak_params: []`)는 타입을 추론할 수 없어 **노드 생성 중**
     InvalidParameterValueException 으로 죽는다 (사용자 코드가 돌기도 전이다).

그래서 병합할 때마다 사람이 손으로 지우는 대신, 여기서 검사·정리하고 넣는다.
덤으로 밸브가 운전 상류압에서 닫히는지(제어권)도 확인한다 — 그게 안 되면 챔버가
명령과 무관하게 레일·탱크 압력까지 끌려 올라간다.

사용:
  scripts/valve_params_install.py results_fit/refit_pos_micro/valve_params.yaml [...]
  scripts/valve_params_install.py --check config/valve_params.yaml     # 검사만
"""
import argparse
import copy
import math
import os
import shutil
import sys
from datetime import datetime

import yaml

NODE_KEY = '/pack2/pp_controller'
PARAMS = ('A_max', 'k_shape', 'C_k', 'C_p', 'C_z', 'A_bw', 'beta_bw', 'gamma_bw',
          'alpha_shape', 'wn_up', 'zeta_up', 'wn_down', 'zeta_down', 'I_MAX')


def strip_meta(node):
    """_fit 같은 진단 키를 지우고, 빈 컨테이너를 없앤다. 새 객체로 만들어 alias 도 끊는다."""
    if isinstance(node, dict):
        out = {}
        for k, v in node.items():
            if k == '_fit':
                continue
            v2 = strip_meta(v)
            if isinstance(v2, (list, dict)) and len(v2) == 0:
                continue                      # 빈 컨테이너는 ROS 가 타입을 못 정한다
            out[k] = v2
        return out
    if isinstance(node, list):
        return [strip_meta(v) for v in node]
    return node


def closable_limit(q):
    """u=0 에서도 열리기 시작하는 상류압 [kPa abs]. 낮으면 그 위에서 밸브를 닫을 수 없다."""
    try:
        k, C_k, C_p, alpha = (float(q['k_shape']), float(q['C_k']),
                              float(q['C_p']), float(q['alpha_shape']))
    except (KeyError, TypeError, ValueError):
        return None
    if C_p <= 0:
        return float('inf')
    F_open = -math.log(math.expm1(90.0 / alpha)) / k
    return (C_k + F_open) / C_p


def audit(doc, rail_max_abs, chamber_max_abs):
    """반환: (문제 리스트, 정보 리스트)"""
    bad, info = [], []
    ch = (doc.get(NODE_KEY, {}).get('ros__parameters', {})
             .get('channel_config', {}) or {})
    for gid in sorted(ch):
        entry = ch[gid]
        if not isinstance(entry, dict):
            continue
        for role in ('micro', 'atm', 'macro'):
            q = entry.get(role)
            if not isinstance(q, dict):
                continue
            missing = [p for p in PARAMS if p not in q]
            if missing:
                bad.append(f'{gid}.{role}: 파라미터 누락 {missing}')
                continue
            lim = closable_limit(q)
            seen = rail_max_abs if (role == 'micro' and not gid.startswith('ch1')) \
                   else chamber_max_abs
            if lim is not None and lim < seen:
                bad.append(f'{gid}.{role}: 상류 {lim:.0f} kPa abs 이상에서 u=0 에도 열린다 '
                           f'(이 밸브가 겪는 최대 {seen:.0f}). C_p={float(q["C_p"]):.3e} 재피팅 필요')
            else:
                info.append(f'{gid}.{role}: 닫힘 한계 {lim:.0f} kPa abs — 정상')
    return bad, info


def scan_containers(node, path=''):
    """빈 리스트/딕트가 남아 있는지 훑는다 (ROS 파서를 죽이는 형태)."""
    out = []
    if isinstance(node, dict):
        if not node:
            out.append(path or '<root>')
        for k, v in node.items():
            out += scan_containers(v, f'{path}.{k}' if path else str(k))
    elif isinstance(node, list):
        if not node:
            out.append(path or '<root>')
        for i, v in enumerate(node):
            out += scan_containers(v, f'{path}[{i}]')
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sources', nargs='+', help='피팅 산출 valve_params.yaml (여러 개 가능)')
    ap.add_argument('--out', default=None, help='기본: config/valve_params.yaml')
    ap.add_argument('--check', action='store_true', help='병합하지 않고 검사만 한다')
    ap.add_argument('--rail-max-kpa', type=float, default=351.325,
                    help='양압 micro 가 겪는 최대 상류압 [kPa abs]. 기본 = 대기압 + '
                         'PressureRefGen.rail.pos_sp_max_kpa(250)')
    ap.add_argument('--chamber-max-kpa', type=float, default=190.0,
                    help='챔버 쪽 밸브의 최대 상류압 [kPa abs]. 기본 = 과압 트립')
    a = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    out = a.out or os.path.join(here, '..', 'config', 'valve_params.yaml')
    out = os.path.abspath(out)

    if a.check:
        doc = yaml.safe_load(open(a.sources[0])) or {}
        bad, info = audit(doc, a.rail_max_kpa, a.chamber_max_kpa)
        empt = scan_containers(doc)
        for s in info:
            print(f'  OK   {s}')
        for s in bad:
            print(f'  문제 {s}')
        if empt:
            print(f'  문제 빈 컨테이너 {len(empt)}개: {empt[:5]} — ROS 파서가 죽는다')
        return 1 if (bad or empt) else 0

    merged = {NODE_KEY: {'ros__parameters': {'channel_config': {}}}}
    tgt = merged[NODE_KEY]['ros__parameters']['channel_config']
    for src in a.sources:
        doc = yaml.safe_load(open(src)) or {}
        ch = (doc.get(NODE_KEY, {}).get('ros__parameters', {})
                 .get('channel_config', {}) or {})
        n = 0
        for gid, entry in ch.items():
            clean = strip_meta(copy.deepcopy(entry))     # deepcopy = alias 원천 차단
            tgt.setdefault(gid, {})
            tgt[gid].update(clean)
            n += 1
        print(f'  읽음: {src} — 채널 {n}개')

    bad, info = audit(merged, a.rail_max_kpa, a.chamber_max_kpa)
    empt = scan_containers(merged)
    print()
    for s in info:
        print(f'  OK   {s}')
    for s in bad:
        print(f'  경고 {s}')
    if empt:
        print(f'  중단: 빈 컨테이너가 남았다 {empt[:5]}')
        return 2

    if os.path.exists(out):
        bak = out + '.bak.' + datetime.now().strftime('%Y%m%d_%H%M%S')
        shutil.copy2(out, bak)
        print(f'\n  기존 파일 백업: {bak}')

    header = (f'# valve_params.yaml — valve_params_install.py 병합본\n'
              f'# 생성: {datetime.now().isoformat(timespec="seconds")}\n'
              f'# 입력: {", ".join(os.path.abspath(s) for s in a.sources)}\n'
              f'#\n'
              f'# _fit 진단 블록은 제거했다 (빈 리스트가 ROS 파서를 죽인다).\n'
              f'# 없는 밸브/채널은 powerpack_config.yaml 의 기본값으로 폴백한다.\n')
    with open(out, 'w') as f:
        f.write(header)
        # default_flow_style=False + deepcopy 로 anchor/alias 가 생기지 않는다
        yaml.safe_dump(merged, f, allow_unicode=True, default_flow_style=False,
                       sort_keys=False)
    print(f'  기록: {out}')
    print('\n  colcon build 후 기동 로그의 "밸브별 13-parameter: N/M 채널" 로 확인할 것')
    return 0


if __name__ == '__main__':
    sys.exit(main())
