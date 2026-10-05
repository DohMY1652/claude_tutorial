#!/usr/bin/env python3
"""시스템 식별용 압력 레퍼런스 시퀀스 — 시나리오 1~10.

무엇을 하는가
-------------
(P+, P−) 목표를 정해진 계획대로 pp_controller 에 보내고, 그동안을 CSV 한 장에
적는다. 밸브 지령은 전적으로 pp_controller 가 낸다 — 이 스크립트는 목표를 보내고
지켜볼 뿐이다.

CSV 열 (축 하나면 이 여섯 개가 전부다):

    time_s, p_pos_ref, p_pos_meas, p_neg_ref, p_neg_meas, angle_deg

축을 여럿 고르면 축마다 뒤 다섯 열이 `_ax<번호>` 접미사로 반복된다.
`time_s` 는 스크립트가 첫 목표를 보내기 시작한 시점이 0 이다.

구간을 나중에 자르려면 `p_pos_ref`/`p_neg_ref` 가 **바뀌는 지점**을 쓴다
(`np.diff`). 어떤 계획이었는지는 CSV 옆에 같이 남는 `_meta.json` 에 단 단위로
들어 있다. 밸브 개도나 PID 내부까지 보려면 `pp_logger` 의
`~/result/<ts>/<ts>.csv` 를 붙인다 — 그쪽 `mpc_ref_gid<축-1>_kpa` 와 여기
`p_pos_ref` 를 상호상관으로 맞추면 된다 (계단열이라 뾰족하게 잡힌다).

시나리오
--------
  1  정적 격자      P+ 101.3~156.3 × P− 101.3~46.3, 단 ΔP ≤ 55        5 s/점
  2  대칭 계단      ΔP 0→10→20→30→40→50→55→0                          5 s/단
  3  양압만 계단    P− 고정, P+ 101.3→156.3→101.3                     5 s/단
  4  음압만 계단    P+ 고정, P− 101.3→46.3→101.3                      5 s/단
  5  등차압 배분    ΔP [20,30,40,50,55] × 배분 변경                    5 s/점
  6  큰 계단        ΔP 10 ↔ 50/55, 양압만/음압만/대칭/배분전환    3 s 전 + 7 s 후
  7  작은 계단      동작점 ΔP 40·50 에서 ±5                       3 s 전 + 5 s 후
  8  2입력 APRBS    유효 (P+,P−) 풀에서 무작위                    dwell 0.5/1/2/3 s
  9  차압 APRBS     ΔP [10..55] + 배분 변동                       dwell 0.5/1/2/3 s
 10  등차압 무작위배분  ΔP [30,40,50,55] 고정, 배분만 무작위            2 s/단

배분(allocation) α 의 정의:  P+ = 대기압 + α·ΔP,  P− = 대기압 − (1−α)·ΔP
  α=1 양압만 · α=0 음압만 · α=0.5 대칭.

★ 레일을 먼저 올려야 한다
-------------------------
P+ 최대 156.3 kPa abs 다. 레일 기본 160 으로는 차압이 4 kPa 뿐이라 채널이 목표를
**못 만든다** (실측 20260914_100318 에서 P+ 가 −4.6 kPa 모자랐다). 별도 터미널에서

    python3 rail_ref.py --hold-ref 200 30      # 측정 내내 유지

를 띄우고, 이 스크립트에 `--pos-rail 200 --neg-rail 30` 을 준다. 안 주면 yaml 의
레일값으로 도달 가능 범위를 잡아 위쪽 점들을 **빼 버린다** (뭘 뺐는지 찍는다).

★ 올림은 계단, 큰 내림은 기울인다
---------------------------------
시나리오 6~10 은 계단·APRBS 가 목적이므로 기본값(`--slew 0`)은 목표를 한 번에
바꾼다. **단 차압이 `--step-down-max`(12 kPa) 넘게 떨어지는 전이만은 예외로**
`--slew-down`(6 kPa/s) 으로 기울인다.

올릴 때는 중력이 저항이라 팔이 지령을 앞지르지 않는다. 내릴 때는 중력이 같은
방향이라 차압을 빼는 순간 **팔이 먼저 떨어진다** — 20260914_175359 에서 시나리오
2 의 마지막 ΔP 55→0 을 계단으로 냈다가 −44.2 °/s 로 각속도 한계에 걸렸다.

전이는 두 챔버를 **하나의 보간 변수로 묶어** 같은 시간에 끝낸다. 채널마다 따로
기울이면 델타가 작은 쪽이 먼저 도착해 그 사이 차압이 두 끝점 밖을 지나간다.

팔 전체가 걱정되면 `--slew` 에 값을 주면 전 구간이 그 기울기로 나간다 — 대신
0.5 s dwell 은 그만큼 의미가 없어진다. 반대로 팔을 떼고 채널만 볼 때는
`--step-down-max 999` 로 전부 계단으로 낼 수 있다.

사용 예
-------
    # 계획만 (하드웨어 안 건드림)
    python3 pressure_id_seq.py --axes 1 --scenario all --dry-run

    # 1축, 준정적 구간만
    python3 pressure_id_seq.py --axes 1 --scenario 1,2,3,4,5 --pos-rail 200 --neg-rail 30

    # 1축, APRBS
    python3 pressure_id_seq.py --axes 1 --scenario 8 --aprbs-main-s 600 --seed 7
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import random
import sys
import time
from dataclasses import dataclass
from datetime import datetime

import rclpy

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from pressure_sweep_server import ATM_KPA, NUM_AXES, _connect   # noqa: E402
from actuator_map import (                                      # noqa: E402
    Abort, Driver, Guard, RigView, POS_BOARD, NEG_BOARD, _spin,
)
from valve_deadzone import read_config                          # noqa: E402

RAIL_POS_BOARD, RAIL_NEG_BOARD = 1, 2      # rail_ref.py 와 같은 보드 번호


# ════════════════════════════════════════════════════════════════════════════
#  여러 축의 각도를 같이 본다
# ════════════════════════════════════════════════════════════════════════════
class Rig(RigView):
    """`RigView` 는 축 하나만 본다. 여러 축을 동시에 지령할 수 있으므로
    인덱스를 받는 접근자를 더한다. 발행은 여전히 하지 않는다."""

    def angle_at(self, idx: int) -> float | None:
        if self._ang is None or len(self._ang) <= idx:
            return None
        return self._ang[idx]


# ════════════════════════════════════════════════════════════════════════════
#  계획
# ════════════════════════════════════════════════════════════════════════════
@dataclass
class Step:
    scen: str          # 시나리오 번호
    seg: str           # 사람이 읽는 구간 이름 (CSV 에 그대로 들어간다)
    p_pos: float
    p_neg: float
    dwell: float
    ramp_in: bool = False   # 대기압/시나리오 경계에서 기울여 들어간다


def alloc_pair(dp: float, alpha: float) -> tuple[float, float]:
    """차압과 배분에서 (P+, P−) 를 만든다."""
    return ATM_KPA + alpha * dp, ATM_KPA - (1.0 - alpha) * dp


def dp_of(p_pos: float, p_neg: float) -> float:
    return p_pos - p_neg


def alpha_of(p_pos: float, p_neg: float) -> float:
    dp = p_pos - p_neg
    return (p_pos - ATM_KPA) / dp if abs(dp) > 1e-9 else math.nan


def _levels(start: float, stop: float, step: float) -> list[float]:
    """start 에서 stop 까지 step 간격. 끝값이 격자에 안 맞으면 **끝값을 더한다**
    (156.325 / 46.325 처럼 ΔP 55 한계에 딱 붙는 점을 잃지 않기 위해)."""
    vals: list[float] = []
    v = start
    if step > 0:
        while v <= stop + 1e-9:
            vals.append(round(v, 3)); v += step
    else:
        while v >= stop - 1e-9:
            vals.append(round(v, 3)); v += step
    if not vals or abs(vals[-1] - stop) > 1e-6:
        vals.append(round(stop, 3))
    return vals


# ── 시나리오별 생성기 ────────────────────────────────────────────────────────
def scen1(a) -> list[Step]:
    """정적 격자. 인접 점 사이 이동을 줄이려고 안쪽 루프를 **뱀처럼** 뒤집는다."""
    pos = _levels(ATM_KPA, ATM_KPA + a.dp_max, a.grid_step)
    neg = _levels(ATM_KPA, ATM_KPA - a.dp_max, -a.grid_step)
    out: list[Step] = []
    for rep in range(a.rep1):
        for i, pp in enumerate(pos):
            row = neg if i % 2 == 0 else list(reversed(neg))
            for pn in row:
                if pp - pn <= a.dp_max + 1e-9:
                    out.append(Step("1", f"static-grid r{rep+1}", pp, pn, a.dwell_grid))
    return out


def scen2(a) -> list[Step]:
    dps = [0, 10, 20, 30, 40, 50, 55, 0]
    out: list[Step] = []
    for rep in range(a.rep2):
        for dp in dps:
            pp, pn = alloc_pair(float(dp), 0.5)
            out.append(Step("2", f"sym-stair r{rep+1}", pp, pn, a.dwell_stair))
    return out


def _ladder(levels: list[float]) -> list[float]:
    """올라갔다 내려온다. 정점과 바닥은 한 번씩만."""
    return levels + list(reversed(levels[:-1]))


def scen3(a) -> list[Step]:
    lv = _ladder(_levels(ATM_KPA, ATM_KPA + a.dp_max, a.grid_step))
    return [Step("3", f"pos-stair r{r+1}", pp, ATM_KPA, a.dwell_stair)
            for r in range(a.rep3) for pp in lv]


def scen4(a) -> list[Step]:
    lv = _ladder(_levels(ATM_KPA, ATM_KPA - a.dp_max, -a.grid_step))
    return [Step("4", f"neg-stair r{r+1}", ATM_KPA, pn, a.dwell_stair)
            for r in range(a.rep4) for pn in lv]


def scen5(a) -> list[Step]:
    out: list[Step] = []
    for rep in range(a.rep5):
        for dp in a.dp_list5:
            for al in a.allocs:
                pp, pn = alloc_pair(dp, al)
                out.append(Step("5", f"const-dp {dp:g} r{rep+1}", pp, pn, a.dwell_grid))
    return out


def scen6(a) -> list[Step]:
    """큰 계단. 낮은 쪽을 3 s 물고 있다가 높은 쪽으로 던지고 7 s 본다.
    되돌아오는 계단은 **다음 반복의 3 s 선행 구간**이 그대로 잰다."""
    lo, hi = a.big_lo, a.big_hi
    modes = (
        ("pos-only", alloc_pair(lo, 1.0), alloc_pair(hi, 1.0)),
        ("neg-only", alloc_pair(lo, 0.0), alloc_pair(hi, 0.0)),
        ("sym", alloc_pair(lo, 0.5), alloc_pair(hi, 0.5)),
        # 배분 전환: ΔP 는 그대로 두고 양압만 ↔ 음압만 으로 던진다
        ("alloc-flip", alloc_pair(a.big_alloc_dp, 1.0), alloc_pair(a.big_alloc_dp, 0.0)),
    )
    out: list[Step] = []
    for name, low, high in modes:
        if a.big_modes and name not in a.big_modes:
            continue
        for rep in range(a.rep6):
            out.append(Step("6", f"big-step {name}", low[0], low[1], a.pre6))
            out.append(Step("6", f"big-step {name}", high[0], high[1], a.post6))
    return out


def scen7(a) -> list[Step]:
    out: list[Step] = []
    for op in a.op7:
        for sign in (+1.0, -1.0):
            base = alloc_pair(op, 0.5)
            pert = alloc_pair(op + sign * a.small7, 0.5)
            name = f"small-step dp{op:g}{'+' if sign > 0 else '-'}{a.small7:g}"
            for rep in range(a.rep7):
                out.append(Step("7", name, base[0], base[1], a.pre7))
                out.append(Step("7", name, pert[0], pert[1], a.post7))
    return out


def _pool(a) -> list[tuple[float, float]]:
    """유효 (P+, P−) 풀. ΔP ≤ 55 이고 챔버가 갈 수 있는 쪽만."""
    pos = _levels(ATM_KPA, ATM_KPA + a.dp_max, a.pool_step)
    neg = _levels(ATM_KPA, ATM_KPA - a.dp_max, -a.pool_step)
    return [(pp, pn) for pp in pos for pn in neg if pp - pn <= a.dp_max + 1e-9]


def _aprbs(rng, scen, seg, picks, dwells, duration) -> list[Step]:
    """APRBS: 진폭도 유지시간도 무작위. 같은 점을 연속으로 뽑지 않는다 —
    연속되면 그 구간은 계단이 아니라 그냥 긴 유지가 되어 정보가 없다."""
    out: list[Step] = []
    total = 0.0
    prev = None
    while total < duration:
        for _ in range(20):
            cand = rng.choice(picks)
            if cand != prev:
                break
        d = rng.choice(dwells)
        out.append(Step(scen, seg, cand[0], cand[1], d))
        total += d
        prev = cand
    return out


def scen8(a, rng) -> list[Step]:
    return _aprbs(rng, "8", "aprbs-2in", _pool(a), a.aprbs_dwells, a.aprbs_main_s)


def scen9(a, rng) -> list[Step]:
    picks = [alloc_pair(dp, al) for dp in a.dp_list9 for al in a.allocs9]
    return _aprbs(rng, "9", "aprbs-diff", picks, a.aprbs_dwells, a.aprbs_diff_s)


def scen10(a, rng) -> list[Step]:
    out: list[Step] = []
    for dp in a.dp_list10:
        prev = None
        for _ in range(a.n10):
            for _ in range(20):
                al = rng.uniform(a.alloc_min, a.alloc_max)
                if prev is None or abs(al - prev) > 0.05:
                    break
            prev = al
            pp, pn = alloc_pair(dp, al)
            out.append(Step("10", f"rand-alloc dp{dp:g}", pp, pn, a.dwell10))
    return out


# 무작위를 쓰는 것(8·9·10)만 rng 를 받는다
BUILDERS = {"1": scen1, "2": scen2, "3": scen3, "4": scen4, "5": scen5,
            "6": scen6, "7": scen7}
BUILDERS_RNG = {"8": scen8, "9": scen9, "10": scen10}
TITLES = {
    "1": "정적 격자", "2": "대칭 계단", "3": "양압만 계단", "4": "음압만 계단",
    "5": "등차압 배분", "6": "큰 계단", "7": "작은 계단",
    "8": "2입력 APRBS", "9": "차압 APRBS", "10": "등차압 무작위배분",
}


def build_plan(a, rng) -> tuple[list[Step], list[tuple[Step, str]]]:
    """계획과, 도달 불가로 뺀 점들을 같이 준다."""
    raw: list[Step] = []
    for s in a.scen_list:
        raw += (BUILDERS_RNG[s](a, rng) if s in BUILDERS_RNG
                else BUILDERS[s](a))

    ok: list[Step] = []
    bad: list[tuple[Step, str]] = []
    for st in raw:
        why = unreachable(st.p_pos, st.p_neg, a)
        if why is None:
            ok.append(st)
        else:
            bad.append((st, why))
    # 시나리오가 바뀌는 첫 점은 기울여 들어간다 (대기압 → 첫 점 포함)
    prev = None
    for st in ok:
        if st.scen != prev:
            st.ramp_in = True
            prev = st.scen
    return ok, bad


def vent_move(p_pos0: float, p_neg0: float, st: Step) -> float:
    """이 전이에서 챔버가 **대기압 쪽으로** 움직이는 양 [kPa].

    챔버를 대기로 빼는 것은 레일에서 채우거나 진공으로 뽑는 것보다 훨씬 빠르다
    (20260914_182338 실측: 대기 쪽 200 kPa/s vs 레일 쪽 82 kPa/s, 2.4 배).
    그래서 두 챔버에 같은 계단을 줘도 **대기 쪽이 먼저 도착해 실제 차압이
    꺼진다** — 그 배분 전환에서 지령 차압은 50 고정인데 실제는 23.5 까지
    무너졌고 팔이 −28 °/s 로 떨어졌다.

    꺼지는 깊이는 대략 이 이동량의 절반이다 (50 kPa 이동 → 26.5 kPa 함몰).
    """
    return max(p_pos0 - st.p_pos,      # 양압 챔버가 내려간다 = 대기로 뺀다
               st.p_neg - p_neg0,      # 음압 챔버가 올라간다 = 대기를 넣는다
               0.0)


def transition_s(p_pos0: float, p_neg0: float, st: Step, a) -> float:
    """앞 목표에서 이 단으로 넘어가는 데 쓸 시간 [s]. 0 이면 계단이다.

    두 가지를 같이 잡는다. 어느 쪽이든 걸리면 그쪽 속도로 기울인다.

      · **지령 차압이 떨어지는 양** — 팔이 준정적으로 따라 내려온다.
        올릴 때는 중력이 저항이라 팔이 지령을 앞지르지 않지만, 내릴 때는
        중력이 같은 방향이라 차압을 빼는 순간 팔이 먼저 떨어진다.
      · **대기 쪽으로 움직이는 양** — 위 vent_move 참고. 지령 차압이
        그대로여도(배분 전환) 실제 차압이 꺼진다.
    """
    d_pos = abs(st.p_pos - p_pos0)
    d_neg = abs(st.p_neg - p_neg0)
    if st.ramp_in:
        return max(d_pos, d_neg) / a.entry_slew
    drop = (p_pos0 - p_neg0) - (st.p_pos - st.p_neg)
    worst = max(drop, vent_move(p_pos0, p_neg0, st))
    thr = step_down_limit(st.scen, a)
    t_down = worst / a.slew_down if worst > thr and a.slew_down > 0 else 0.0
    # --slew 를 줬어도 내림 규칙보다 빠르게는 안 간다 (--slew 20 같은 값으로
    # 위의 두 함정을 우회하게 두면 안 된다).
    t_all = max(d_pos, d_neg) / a.slew if a.slew > 0 else 0.0
    return max(t_down, t_all)


def unreachable(p_pos: float, p_neg: float, a) -> str | None:
    """챔버는 한쪽으로만 간다. 그리고 레일에 붙으면 차압이 없어 유량이 안 난다."""
    if p_pos < a.pos_min - 1e-6:
        return f"P+ {p_pos:.1f} < {a.pos_min:.1f} (양압 챔버는 대기압 아래로 못 간다)"
    if p_pos > a.pos_max + 1e-6:
        return f"P+ {p_pos:.1f} > {a.pos_max:.1f} (양압 레일 여유 부족 — rail_ref 로 올릴 것)"
    if p_neg > a.neg_max + 1e-6:
        return f"P− {p_neg:.1f} > {a.neg_max:.1f} (음압 챔버는 대기압 위로 못 간다)"
    if p_neg < a.neg_min - 1e-6:
        return f"P− {p_neg:.1f} < {a.neg_min:.1f} (진공 레일 여유 부족)"
    return None


# ════════════════════════════════════════════════════════════════════════════
#  인자
# ════════════════════════════════════════════════════════════════════════════
def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="시스템 식별용 압력 레퍼런스 시퀀스 (시나리오 1~10)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    ap.add_argument("--axes", default="1",
                    help="지령할 축 '1,2' 또는 'all'. 선택 축은 **같은 목표를 동시에** "
                         "받고, 나머지는 계획 내내 대기압이다")
    ap.add_argument("--scenario", default="all",
                    help="돌릴 시나리오 '1,2,8' 또는 'all' (1~10)")
    ap.add_argument("--seed", type=int, default=1,
                    help="APRBS·무작위 배분의 난수 씨앗. 같은 씨앗이면 같은 계획이 나온다")

    g = ap.add_argument_group("격자·계단 값")
    g.add_argument("--dp-max", type=float, default=55.0, help="차압 상한 [kPa]")
    g.add_argument("--grid-step", type=float, default=10.0,
                    help="시나리오 1·3·4 의 격자 간격 [kPa]")
    g.add_argument("--pool-step", type=float, default=5.0,
                    help="시나리오 8 의 (P+,P−) 풀 격자 [kPa]")
    g.add_argument("--allocs", default="0,0.25,0.5,0.75,1.0",
                    help="시나리오 5 의 배분 α 목록")
    g.add_argument("--dwell-grid", type=float, default=5.0, help="시나리오 1·5 유지 [s]")
    g.add_argument("--dwell-stair", type=float, default=5.0, help="시나리오 2·3·4 유지 [s]")

    b = ap.add_argument_group("계단 시나리오 6·7")
    b.add_argument("--big-lo", type=float, default=10.0, help="큰 계단 낮은 쪽 ΔP [kPa]")
    b.add_argument("--big-hi", type=float, default=55.0, help="큰 계단 높은 쪽 ΔP [kPa]")
    b.add_argument("--big-alloc-dp", type=float, default=50.0,
                    help="배분 전환 계단의 고정 ΔP [kPa]")
    b.add_argument("--big-modes", default="",
                   help="시나리오 6 에서 돌릴 모드만 고른다 (쉼표). 비우면 전부. "
                        "pos-only · neg-only · sym · alloc-flip — 한 채널만 떼어 "
                        "게인을 확인할 때 쓴다")
    b.add_argument("--pre6", type=float, default=3.0, help="계단 전 유지 [s]")
    b.add_argument("--post6", type=float, default=7.0, help="계단 후 유지 [s]")
    b.add_argument("--op7", default="40,50", help="작은 계단 동작점 ΔP 목록 [kPa]")
    b.add_argument("--small7", type=float, default=5.0, help="작은 계단 크기 [kPa]")
    b.add_argument("--pre7", type=float, default=3.0, help="작은 계단 전 유지 [s]")
    b.add_argument("--post7", type=float, default=5.0, help="작은 계단 후 유지 [s]")

    r = ap.add_argument_group("APRBS 8·9·10")
    r.add_argument("--aprbs-dwells", default="0.5,1,2,3", help="무작위 유지시간 목록 [s]")
    r.add_argument("--aprbs-main-s", type=float, default=300.0, help="시나리오 8 길이 [s]")
    r.add_argument("--aprbs-diff-s", type=float, default=180.0, help="시나리오 9 길이 [s]")
    r.add_argument("--dp-list9", default="10,20,30,40,50,55", help="시나리오 9 의 ΔP 목록")
    r.add_argument("--allocs9", default="0,0.25,0.5,0.75,1.0", help="시나리오 9 의 배분 목록")
    r.add_argument("--dp-list10", default="30,40,50,55", help="시나리오 10 의 ΔP 목록")
    r.add_argument("--n10", type=int, default=100, help="시나리오 10 의 ΔP 당 단 수")
    r.add_argument("--dwell10", type=float, default=2.0, help="시나리오 10 유지 [s]")
    r.add_argument("--alloc-min", type=float, default=0.0, help="무작위 배분 하한")
    r.add_argument("--alloc-max", type=float, default=1.0, help="무작위 배분 상한")

    rep = ap.add_argument_group("반복 횟수")
    rep.add_argument("--rep1", type=int, default=1)
    rep.add_argument("--rep2", type=int, default=3)
    rep.add_argument("--rep3", type=int, default=3)
    rep.add_argument("--rep4", type=int, default=3)
    rep.add_argument("--rep5", type=int, default=3)
    rep.add_argument("--rep6", type=int, default=5)
    rep.add_argument("--rep7", type=int, default=5)

    sl = ap.add_argument_group("변화율 — 계단을 계단으로 낼지 정한다")
    sl.add_argument("--slew", type=float, default=0.0,
                    help="0 이면 목표를 **한 번에** 바꾼다 (계단·APRBS 가 목적이므로 기본). "
                         "값을 주면 전 구간이 그 기울기 [kPa/s] 로 나간다")
    sl.add_argument("--entry-slew", type=float, default=4.0,
                    help="대기압에서 첫 점으로, 그리고 시나리오가 바뀔 때의 기울기 [kPa/s]")
    sl.add_argument("--step-down-max", type=float, default=20.0,
                    help="계단으로 낼 수 있는 한계 [kPa]. **지령 차압이 떨어지는 양**과 "
                         "**챔버가 대기압 쪽으로 움직이는 양** 중 큰 쪽이 이보다 크면 "
                         "기울인다. 앞엣것은 팔이 중력으로 따라 내려오는 것이고 "
                         "(실측 ΔP 55→0 계단에서 −44 °/s), 뒤엣것은 대기 쪽 채널이 "
                         "2.4 배 빨라 실제 차압이 꺼지는 것이다 (실측 배분 전환에서 "
                         "지령 50 고정인데 실제 23.5, −28 °/s). "
                         "**준정적(1~5) 에만 걸린다** — 계단 쪽은 "
                         "--step-down-max-fast 를 본다. 20 을 고른 근거: 실측에서 "
                         "차압 하락 55 kPa → −44 °/s (0.8 °/s per kPa), 대기쪽 이동 "
                         "50 kPa → −28 °/s (0.56). 20 이면 각각 16·11 °/s 로 "
                         "준정적 한계 25 안에 든다")
    sl.add_argument("--step-down-max-fast", type=float, default=999.0,
                    help="계단·APRBS(6~10) 의 같은 문턱 [kPa]. 기본 999 = 끔 — 그쪽은 "
                         "각속도 한계가 지령 변화량에 따라 열리므로 지령을 "
                         "기울일 이유가 없다. 팔을 아끼려면 12 쯤을 준다")
    sl.add_argument("--slew-down", type=float, default=6.0,
                    help="그 내림 전이의 기울기 [kPa/s]. 1축 플랜트 기울기 0.86 kPa/° "
                         "기준으로 준정적 하강 약 7 °/s 다 (한계 25)")
    sl.add_argument("--release-slew", type=float, default=4.0,
                    help="끝내거나 중단할 때 대기압으로 내리는 기울기 [kPa/s]")

    s = ap.add_argument_group("안전 한계 — 걸리면 즉시 대기압 램프")
    s.add_argument("--rate-base", type=float, default=20.0,
                   help="각속도 상한의 **바닥값** [°/s]. 지령이 가만히 있을 때 허용치다 "
                        "— 아무것도 안 시켰는데 이만큼 움직이면 그건 사고다")
    s.add_argument("--rate-gain", type=float, default=2.5,
                   help="상한이 지령 변화량에 붙는 기울기 [°/s per kPa]. "
                        "허용 = base + gain × (그 단에서 흔든 kPa). "
                        "실측 최대 1.58 °/s per kPa (s6 대기쪽 45 kPa → −71.1 °/s) "
                        "이므로 2.5 는 1.6 배 여유다")
    s.add_argument("--rate-cap", type=float, default=180.0,
                   help="적응 상한의 천장 [°/s]")
    s.add_argument("--rate-relax", type=float, default=3.0,
                   help="올라간 상한이 바닥값으로 되돌아오는 시정수 [s]. 팔은 계단 뒤 "
                        "한동안 계속 움직이므로 즉시 조이면 정상 응답에서 걸린다")
    s.add_argument("--ang-min", type=float, default=-5.0, help="각도 하한 [°]")
    s.add_argument("--ang-max", type=float, default=88.0,
                    help="각도 상한 [°]. 90° 위는 정적으로 불안정하다 — 올리지 말 것")
    s.add_argument("--track-tol", type=float, default=20.0,
                    help="압력 추종 오차 허용 [kPa]. 계단 직후에는 당연히 벌어지므로 "
                         "**죽은 채널만 잡을 만큼** 느슨하다")
    s.add_argument("--track-grace", type=float, default=10.0, help="그 오차 지속 한계 [s]")
    s.add_argument("--enc-timeout", type=float, default=1.0, help="엔코더 무갱신 한계 [s]")
    s.add_argument("--no-angle", action="store_true",
                   help="**액추에이터를 안 달고** 밸브·압력만 볼 때. 각도에서 나오는 감시를 "
                        "전부 끈다 (엔코더 갱신·각도 범위·각속도·진동). "
                        "**압력 추종 감시는 그대로 살아 있다** — 챔버가 안 잡히는 것은 "
                        "액추에이터가 없어도 위험하다. 로그의 각도 열은 빈칸이 된다. "
                        "팔을 다시 달면 반드시 빼고 --ang-max 88 로 돌아갈 것")
    s.add_argument("--osc-window", type=float, default=4.0, help="진동 판정 창 [s]")
    s.add_argument("--osc-flips", default="auto",
                    help="그 창 안 각속도 부호 반전 한계. 'auto' = 준정적(1~5)만 켜고 "
                         "계단·APRBS(6~10)는 끈다 — **레퍼런스가 스스로 뒤집으므로** "
                         "켜 두면 정상 데이터에서 걸린다. 0 = 항상 끔")
    s.add_argument("--osc-auto", type=int, default=6,
                    help="'auto' 일 때 준정적 구간에 쓸 반전 한계")
    s.add_argument("--rate-tau", type=float, default=0.3, help="각속도 LPF 시정수 [s]")

    lim = ap.add_argument_group("도달 범위")
    lim.add_argument("--pos-rail", type=float, default=None,
                    help="양압 레일 실제값 [kPa abs]. **이 값은 레일을 올리지 않는다** — "
                         "도달 범위를 계산하는 데만 쓴다. 올리려면 별도 터미널에서 "
                         "`rail_ref.py --hold-ref 200 30` 을 띄워야 한다. "
                         "기동 직후 실측 레일과 대조해서 다르면 멈춘다 "
                         "(기본 = yaml LinePID.pos.ref)")
    lim.add_argument("--neg-rail", type=float, default=None,
                    help="음압 레일 실제값 [kPa abs]. 위와 같다 — 레일을 올리지 않는다 "
                         "(기본 = yaml LinePID.neg.ref)")
    lim.add_argument("--rail-wait", type=float, default=120.0,
                    help="레일이 --pos-rail/--neg-rail 에 들어올 때까지 기다릴 시간 [s]. "
                         "0 = 안 기다리고 바로 잰다. **기본 120** — 펌프가 200 kPa 까지 "
                         "올리는 데 수 분이 걸리는데, 기동 직후 1 초만 재면 아직 낮아서 "
                         "멀쩡한 단을 잘라낸다 (20260917: 실제로는 런 내내 200 이었는데 "
                         "시작 1 초에 165.6 으로 읽혀 23 단이 잘렸다)")
    lim.add_argument("--rail-margin", type=float, default=10.0,
                    help="레일에서 이만큼 떨어져야 밸브가 유량을 낸다 [kPa]")
    lim.add_argument("--pos-min", type=float, default=ATM_KPA)
    lim.add_argument("--neg-max", type=float, default=ATM_KPA)

    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2293)
    ap.add_argument("--connect-timeout", type=float, default=30.0)
    ap.add_argument("--send-hz", type=float, default=50.0,
                    help="송신·샘플 주기 [Hz]. 0.5 s dwell 을 쓰므로 20 보다 높다")
    ap.add_argument("--out", default=None,
                    help="CSV 경로 (기본 ~/result/pressure_id_<ts>.csv)")
    ap.add_argument("--progress-every", type=int, default=25, help="진행 표시 간격 [단]")

    rs = ap.add_argument_group("중단 / 이어 하기")
    rs.add_argument("--resume", default=None,
                    help="끊긴 런의 CSV 나 _meta.json 경로. 거기서 **다음 단부터** 잇는다. "
                         "계획 지문을 대조해 다르면 멈춘다 — 다른 계획으로 이어 붙이면 "
                         "두 조각이 같은 실험이 아니게 된다. 기록은 새 CSV 로 가고, "
                         "새 _meta.json 에 resumed_from 으로 연결된다")
    rs.add_argument("--start-step", type=int, default=None,
                    help="이 단부터 시작한다 (1 기준). --resume 없이 손으로 지정할 때만 쓴다")
    ap.add_argument("--dry-run", action="store_true", help="계획만 찍고 끝낸다")
    ap.add_argument("--list-steps", action="store_true", help="--dry-run 에서 전 단을 찍는다")
    ap.add_argument("--rail-tol", type=float, default=8.0,
                    help="기동 시 실측 레일이 --pos-rail/--neg-rail 과 이보다 벌어지면 "
                         "**경고하고 실측값으로 도달 범위를 다시 잡는다** [kPa]. "
                         "0 = 확인 안 함. 레일 루프는 200 을 지령해도 185 부근에서 "
                         "머무는 일이 잦다 — 그래서 기본은 중단이 아니라 경고다")
    ap.add_argument("--rail-strict", action="store_true",
                    help="레일이 안 맞으면 경고 대신 중단한다")
    ap.add_argument("--yes", action="store_true", help="확인 프롬프트를 건너뛴다")
    a = ap.parse_args()

    # ── 목록 파싱 ────────────────────────────────────────────────────────
    def _flist(text, name):
        try:
            return [float(v) for v in text.replace(" ", "").split(",") if v]
        except ValueError:
            ap.error(f"{name} 는 쉼표로 구분한 숫자다")

    if a.axes.strip().lower() == "all":
        a.axis_list = list(range(NUM_AXES))
    else:
        try:
            a.axis_list = sorted({int(v) - 1 for v in a.axes.replace(" ", "").split(",") if v})
        except ValueError:
            ap.error("--axes 는 '1,2' 또는 'all' 이다")
        if not a.axis_list or not all(0 <= x < NUM_AXES for x in a.axis_list):
            ap.error(f"--axes 는 1~{NUM_AXES}")

    if a.scenario.strip().lower() == "all":
        a.scen_list = [str(i) for i in range(1, 11)]
    else:
        a.scen_list = [v for v in a.scenario.replace(" ", "").split(",") if v]
        bad = [v for v in a.scen_list if v not in TITLES]
        if bad:
            ap.error(f"모르는 시나리오: {', '.join(bad)} (1~10)")

    a.allocs = _flist(a.allocs, "--allocs")
    a.allocs9 = _flist(a.allocs9, "--allocs9")
    a.dp_list5 = [20.0, 30.0, 40.0, 50.0, 55.0]
    a.dp_list9 = _flist(a.dp_list9, "--dp-list9")
    a.dp_list10 = _flist(a.dp_list10, "--dp-list10")
    a.op7 = _flist(a.op7, "--op7")
    a.big_modes = [v for v in a.big_modes.replace(" ", "").split(",") if v]
    known = ("pos-only", "neg-only", "sym", "alloc-flip")
    if any(v not in known for v in a.big_modes):
        ap.error(f"--big-modes 는 {', '.join(known)} 중에서 고른다")
    a.aprbs_dwells = _flist(a.aprbs_dwells, "--aprbs-dwells")
    if min(a.aprbs_dwells) * a.send_hz < 5:
        ap.error(f"--send-hz {a.send_hz:g} 로는 dwell {min(a.aprbs_dwells):g} s 에 "
                 f"패킷이 {min(a.aprbs_dwells)*a.send_hz:.0f} 개뿐이다 — send-hz 를 올릴 것")

    # ── 레일에서 도달 범위 ───────────────────────────────────────────────
    if a.pos_rail is None or a.neg_rail is None:
        import yaml
        cfg_path = os.path.join(_HERE, "..", "config", "powerpack_config.yaml")
        with open(cfg_path, encoding="utf-8") as fh:
            lp = yaml.safe_load(fh)["/pack2/pp_controller"]["ros__parameters"]["LinePID"]
        if a.pos_rail is None:
            a.pos_rail = float(lp["pos"]["ref"])
        if a.neg_rail is None:
            a.neg_rail = float(lp["neg"]["ref"])
    a.pos_max = a.pos_rail - a.rail_margin
    a.neg_min = a.neg_rail + a.rail_margin
    return a


# ════════════════════════════════════════════════════════════════════════════
#  실행
# ════════════════════════════════════════════════════════════════════════════
def print_plan(a, steps: list[Step], dropped: list[tuple[Step, str]]) -> None:
    print("\n=== 압력 레퍼런스 시퀀스 계획 ===")
    print(f"축 {', '.join(str(x+1) for x in a.axis_list)} "
          f"(양압 gid {', '.join(str(x) for x in a.axis_list)} / "
          f"음압 gid {', '.join(str(x+NUM_AXES) for x in a.axis_list)}) — 같은 목표 동시 지령")
    print(f"나머지 축: 계획 내내 대기압 {ATM_KPA:g} kPa abs")
    print(f"레일 양압 {a.pos_rail:g} / 음압 {a.neg_rail:g} kPa abs "
          f"→ 도달 범위 P+ {a.pos_min:.1f}~{a.pos_max:.1f} / P− {a.neg_min:.1f}~{a.neg_max:.1f}")
    print("      ↑ 이 값은 **주장일 뿐 레일을 올리지 않는다.** 올리려면 별도 터미널에서")
    print(f"        `rail_ref.py --hold-ref {a.pos_rail:g} {a.neg_rail:g}` 를 띄운다."
          f" 기동 시 실측과 대조한다 (±{a.rail_tol:g} kPa)")

    # 전이 시간은 실행부와 **같은 transition_s** 로 낸다 (계획과 실제가 갈라지면
    # 44 분짜리 계획이 한 시간을 넘어도 모른다).
    ramp_s: dict[str, float] = {}
    n_ramp: dict[str, int] = {}
    p0, n0 = ATM_KPA, ATM_KPA
    for st in steps:
        t = transition_s(p0, n0, st, a)
        ramp_s[st.scen] = ramp_s.get(st.scen, 0.0) + t
        n_ramp[st.scen] = n_ramp.get(st.scen, 0) + (1 if t > 0 else 0)
        p0, n0 = st.p_pos, st.p_neg

    print(f"\n{'시나리오':<22} {'단':>6} {'유지':>8} {'전이':>8} {'합':>8}  기울인 전이")
    total = 0.0
    for s in a.scen_list:
        sel = [x for x in steps if x.scen == s]
        hold = sum(x.dwell for x in sel)
        rmp = ramp_s.get(s, 0.0)
        total += hold + rmp
        print(f"{s:>2}  {TITLES[s]:<17} {len(sel):>6} {hold/60.0:>6.1f} 분 "
              f"{rmp/60.0:>6.1f} 분 {(hold+rmp)/60.0:>6.1f} 분   "
              f"{n_ramp.get(s, 0):>3d}/{len(sel)} 단")
    print(f"{'합계':<22} {len(steps):>6} {'':>8} {'':>8} {total/60.0:>6.1f} 분")

    if dropped:
        print(f"\n도달 불가로 **{len(dropped)} 단을 뺐다**:")
        seen: dict[str, int] = {}
        for st, why in dropped:
            seen[why] = seen.get(why, 0) + 1
        for why, n in list(seen.items())[:6]:
            print(f"   ×{n:<4d} {why}")
        if a.pos_rail < ATM_KPA + a.dp_max + a.rail_margin:
            print(f"   → 양압 레일이 {a.pos_rail:g} 라 P+ 를 {a.pos_max:.1f} 까지밖에 못 만든다. "
                  f"별도 터미널에서 `rail_ref.py --hold-ref 200 30` 을 띄우고 "
                  f"`--pos-rail 200 --neg-rail 30` 을 줄 것")

    print("\n변화율: " + ("계단은 **한 번에**" if a.slew <= 0 else f"전 구간 {a.slew:g} kPa/s") +
          f" · 문턱을 넘는 내림은 {a.slew_down:g} kPa/s"
          f" · 진입/시나리오 경계 {a.entry_slew:g} kPa/s · 복귀 {a.release_slew:g} kPa/s")
    print("        내림 문턱은 '지령 차압이 떨어지는 양' 과 '챔버가 대기 쪽으로 "
          "움직이는 양' 중 큰 쪽으로 본다.")
    print(f"송신 {a.send_hz:g} Hz · 난수 씨앗 {a.seed}")
    if a.no_angle:
        print("안전: **--no-angle — 각도 감시 전부 꺼짐** "
              "(엔코더 갱신·각도 범위·각속도·진동)")
        print(f"      살아 있는 것은 압력 추종뿐: {a.track_tol:g} kPa 가 "
              f"{a.track_grace:g} s 지속되면 중단")
        print("      ⚠ 액추에이터를 달면 이 옵션을 빼고 --ang-max 88 로 돌아갈 것")
    else:
        print(f"안전: 각도 [{a.ang_min:g}, {a.ang_max:g}]° · 엔코더 {a.enc_timeout:g} s "
              f"— 이 둘은 항상 켜짐")
        print(f"      각속도 상한은 **그 단에서 흔든 kPa 에 붙어 움직인다**: "
              f"{a.rate_base:g} + {a.rate_gain:g}×kPa, 천장 {a.rate_cap:g} °/s, "
              f"이완 {a.rate_relax:g} s")
    # 계획 안에서 실제로 나올 허용치 폭을 보여 준다
    ex_by: dict[str, list[float]] = {}
    p0, n0 = ATM_KPA, ATM_KPA
    for st in steps:
        ex_by.setdefault(st.scen, []).append(excite_kpa(p0, n0, st))
        p0, n0 = st.p_pos, st.p_neg
    for sc in a.scen_list:
        osc, trk = guard_mode(sc, a)
        o = "끔" if osc >= OFF else f"{osc}회/{a.osc_window:g}s"
        t = "끔" if trk >= OFF else f"{trk:g} kPa/{a.track_grace:g}s"
        d = step_down_limit(sc, a)
        ds = "계단 자유" if d >= 999 else f"내림 ≤{d:g} kPa"
        ex = ex_by.get(sc, [0.0])
        print(f"      s{sc:<2s} 허용 {rate_allow(min(ex), a):3.0f}~"
              f"{rate_allow(max(ex), a):3.0f} °/s (흔듦 {min(ex):.0f}~{max(ex):.0f} kPa) · "
              f"{ds:<13s} · 진동 {o:<9s} · 추종 {t}")
    print(f"중단되면 **양 챔버를 {a.release_slew:g} kPa/s 로 대기압까지 내린다** (끊지 않는다).")


# Guard 는 '한계 이상' 으로 판정하므로 0 을 주면 즉시 걸린다. 끄려면 크게 준다.
OFF = 10 ** 9
# Driver.step_toward 에 주면 한 틱에 목표로 간다 = 계단. 기울기는 우리가 직접 만든다.
JUMP = 1e9

# 진동·추종 감시는 **시나리오마다 의미가 다르다.**
#   · 진동  — 레퍼런스가 스스로 부호를 뒤집는 구간(6~10)에서는 정상 데이터가 걸린다
#   · 추종  — dwell 이 유예시간보다 짧으면(8·9·10) 못 따라가는 게 당연하다.
#             6·7 은 계단 뒤 5~7 s 를 물고 있으므로 살려 둔다 — 여기서 걸리면
#             레일이 모자란 것이고, 그건 실제로 중단해야 할 상황이다.
QUASI_STATIC = ("1", "2", "3", "4", "5")
FAST_RANDOM = ("8", "9", "10")


def step_down_limit(scen: str, a) -> float:
    """계단으로 낼 수 있는 한계 [kPa]. 각속도 한계가 열려 있는 계단 구간에서는
    지령을 기울일 이유가 없으므로 기본이 사실상 무한이다."""
    return a.step_down_max if scen in QUASI_STATIC else a.step_down_max_fast


def rate_allow(excite_kpa: float, a) -> float:
    """그 단에서 허용할 각속도 [°/s]. **지령을 얼마나 흔들었는지에 붙는다.**

    고정 상한은 원리적으로 못 맞춘다 — 준정적 구간에서 25 는 느슨하고, 큰 계단에서
    70 은 여전히 빡빡하다(실측 −71.1). 팔이 얼마나 빨리 가는지는 그 단에서 지령을
    몇 kPa 흔들었는지로 정해지므로, 상한도 거기에 비례해야 한다. 그러면 "큰 계단은
    빨라도 된다" 와 "가만히 있는데 움직이면 사고다" 를 하나로 잡는다.
    """
    return min(a.rate_cap, a.rate_base + a.rate_gain * max(0.0, excite_kpa))


def excite_kpa(p_pos0: float, p_neg0: float, st: Step) -> float:
    """그 단이 팔을 흔드는 크기 [kPa]. 지령 차압의 변화와, 실제 차압을 꺼뜨리는
    대기쪽 이동 중 큰 쪽이다 (vent_move 참고)."""
    return max(abs((st.p_pos - st.p_neg) - (p_pos0 - p_neg0)),
               vent_move(p_pos0, p_neg0, st))


def guard_mode(scen: str, a) -> tuple[int, float]:
    """(진동 반전 한계, 추종 허용 오차). OFF 면 그 감시를 끈 것이다.
    각속도 상한은 여기 없다 — rate_allow 가 매 단 다시 낸다."""
    if str(a.osc_flips).strip().lower() == "auto":
        osc = a.osc_auto if scen in QUASI_STATIC else OFF
    else:
        osc = int(a.osc_flips) or OFF
        if osc < 0:
            osc = OFF
    trk = OFF if scen in FAST_RANDOM else a.track_tol
    return osc, trk


def _rail_avg(rig: Rig, drv: Driver, hold_s: float) -> tuple[float | None, float | None]:
    """두 레일 압력을 hold_s 동안 평균한다. 한 샘플로 보면 리플에 걸려 오판한다."""
    acc: dict[int, list[float]] = {RAIL_POS_BOARD: [], RAIL_NEG_BOARD: []}
    t_end = time.monotonic() + max(0.0, hold_s)
    while True:
        for b in acc:
            v = rig.kpa(b)
            if v is not None:
                acc[b].append(v)
        if time.monotonic() >= t_end:
            break
        drv.hold()
        _spin(rig, drv.dt)
    return tuple(sum(v) / len(v) if v else None for v in acc.values())  # type: ignore


def _write_meta(out: str, a, steps: list[Step], dropped: list) -> None:
    """어떤 계획으로 돈 결과인지 CSV 옆에 남긴다. 씨앗과 인자가 있어야 같은
    시퀀스를 다시 만들 수 있고, 뺀 점을 알아야 격자가 왜 비었는지 안다."""
    import json
    meta = {
        "script": os.path.basename(__file__),
        "started": datetime.now().isoformat(timespec="seconds"),
        "csv": os.path.basename(out),
        "axes": [x + 1 for x in a.axis_list],
        "scenarios": a.scen_list,
        "seed": a.seed,
        "rails": {"pos": a.pos_rail, "neg": a.neg_rail, "margin": a.rail_margin},
        "reach": {"pos_min": a.pos_min, "pos_max": a.pos_max,
                  "neg_min": a.neg_min, "neg_max": a.neg_max},
        "n_steps": len(steps),
        "n_dropped": len(dropped),
        "plan_sig": _plan_sig(steps),
        "start_step": getattr(a, "start_step", 1),
        "resumed_from": getattr(a, "resumed_from", None),
        "duration_s_nominal": sum(x.dwell for x in steps),
        "args": {k: v for k, v in sorted(vars(a).items())
                 if isinstance(v, (int, float, str, bool, list, type(None)))},
        "plan": [{"i": i, "scen": x.scen, "seg": x.seg,
                  "p_pos": round(x.p_pos, 3), "p_neg": round(x.p_neg, 3),
                  "dwell": x.dwell, "ramp_in": x.ramp_in}
                 for i, x in enumerate(steps, 1)],
    }
    path = out.replace(".csv", "_meta.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(meta, fh, ensure_ascii=False, indent=1)
    print(f"[계획] {path}")


def _plan_sig(steps: list[Step]) -> str:
    """계획의 지문. 씨앗·인자가 같아도 값이 같은지 **직접** 확인한다."""
    import hashlib
    h = hashlib.sha256()
    for x in steps:
        h.update(f"{x.scen}|{x.seg}|{x.p_pos:.3f}|{x.p_neg:.3f}|{x.dwell:.3f}\n".encode())
    return h.hexdigest()[:16]


def _progress_path(out: str) -> str:
    return out.replace(".csv", "_progress.json")


def _write_progress(out: str, done: int, total: int, t0: float) -> None:
    """어디까지 갔는지 **매 단** 남긴다. Ctrl-C 든 정전이든 여기가 있어야 잇는다.
    수백 바이트라 매 단 써도 부담이 없다 (meta 는 143 KB 라 그러면 안 된다)."""
    import json
    try:
        with open(_progress_path(out), "w", encoding="utf-8") as fh:
            json.dump({"csv": os.path.basename(out), "last_done": done,
                       "n_steps": total, "elapsed_s": round(time.monotonic() - t0, 1),
                       "updated": datetime.now().isoformat(timespec="seconds")}, fh,
                      ensure_ascii=False, indent=1)
    except OSError:
        pass          # 기록 실패가 실험을 멈출 이유는 없다


def _resume_point(path: str, steps: list[Step]) -> tuple[int, str]:
    """끊긴 런에서 다시 시작할 단 번호를 찾는다. (start_step, meta 경로)"""
    import json
    path = os.path.expanduser(path)
    if path.endswith("_meta.json"):
        meta_p = path
    elif path.endswith("_progress.json"):
        meta_p = path.replace("_progress.json", "_meta.json")
    elif path.endswith(".csv"):
        meta_p = path.replace(".csv", "_meta.json")
    else:
        raise SystemExit(f"[중단] --resume 에는 CSV 나 _meta.json 을 준다: {path}")
    if not os.path.exists(meta_p):
        raise SystemExit(f"[중단] 계획 파일이 없다: {meta_p}")
    with open(meta_p, encoding="utf-8") as fh:
        meta = json.load(fh)

    # ── 같은 계획인가 — 이게 틀리면 이어 붙인 데이터가 한 실험이 아니다 ──
    old_plan = meta.get("plan") or []
    h = __import__("hashlib").sha256()
    for x in old_plan:
        h.update(f"{x['scen']}|{x['seg']}|{x['p_pos']:.3f}|{x['p_neg']:.3f}|"
                 f"{float(x['dwell']):.3f}\n".encode())
    if h.hexdigest()[:16] != _plan_sig(steps):
        raise SystemExit(
            f"[중단] 계획이 다르다 — 이어 붙일 수 없다.\n"
            f"        끊긴 런 {len(old_plan)} 단 / 지금 인자 {len(steps)} 단\n"
            f"        `--seed`, `--scenario`, `--dp-max`, `--rep*`, 레일을 원래 런과\n"
            f"        똑같이 주어야 한다. 원래 값은 {meta_p} 의 args 에 있다.")

    prog_p = meta_p.replace("_meta.json", "_progress.json")
    if os.path.exists(prog_p):
        with open(prog_p, encoding="utf-8") as fh:
            done = int(json.load(fh).get("last_done", 0))
        return done + 1, meta_p

    # 진행 기록이 없는 판으로 돈 런 — CSV 의 레퍼런스 계단을 계획과 맞춰 복원한다.
    csv_p = meta_p.replace("_meta.json", ".csv")
    if not os.path.exists(csv_p):
        raise SystemExit(
            f"[중단] 진행 기록도 CSV 도 없다 ({prog_p}).\n"
            f"        `--start-step N` 으로 손으로 줄 것.")
    done = _steps_done_from_csv(csv_p, old_plan)
    if done <= 0:
        raise SystemExit(f"[중단] CSV 에서 진행을 복원하지 못했다: {csv_p}")
    print(f"[이어] 진행 기록이 없어 CSV 에서 복원했다 — {done} 단까지 확인됨")
    # 마지막 단은 유지가 덜 끝난 채 끊겼을 수 있으므로 **그 단부터 다시** 한다.
    return done, meta_p


def _steps_done_from_csv(csv_p: str, plan: list[dict]) -> int:
    """CSV 의 레퍼런스가 바뀌는 지점을 계획과 순서대로 맞춰 몇 단까지 갔는지 센다.

    진행 기록 파일이 생기기 전에 돈 런을 위한 복원 경로다. 레퍼런스가 유지되는
    구간 하나가 계획의 한 단이다 (전이 중에는 매 틱 바뀌므로 짧은 조각은 버린다).
    """
    MIN_HOLD = 10          # 샘플. 이보다 짧으면 전이 조각이다
    holds: list[tuple[float, float]] = []
    prev, run = None, 0
    try:
        with open(csv_p, newline="", encoding="utf-8") as fh:
            rd = csv.reader(fh)
            head = next(rd)
            ip, inn = head.index("p_pos_ref"), head.index("p_neg_ref")
            for row in rd:
                try:
                    key = (round(float(row[ip]), 3), round(float(row[inn]), 3))
                except (ValueError, IndexError):
                    continue
                if key == prev:
                    run += 1
                    continue
                if prev is not None and run >= MIN_HOLD:
                    holds.append(prev)
                prev, run = key, 1
            if prev is not None and run >= MIN_HOLD:
                holds.append(prev)
    except (OSError, StopIteration, ValueError) as exc:
        raise SystemExit(f"[중단] CSV 를 읽지 못했다: {csv_p} ({exc})")

    pi = 0
    for hp, hn in holds:
        while pi < len(plan) and not (abs(plan[pi]["p_pos"] - hp) < 0.01
                                      and abs(plan[pi]["p_neg"] - hn) < 0.01):
            pi += 1
        if pi >= len(plan):
            break
        pi += 1
    return pi


def main() -> int:
    a = parse_args()
    rng = random.Random(a.seed)
    steps, dropped = build_plan(a, rng)
    if not steps:
        print("[중단] 도달 가능한 단이 하나도 없다. 레일과 --dp-max 를 확인할 것.",
              file=sys.stderr)
        for st, why in dropped[:5]:
            print(f"   {why}", file=sys.stderr)
        return 2

    # ── 이어 하기 ────────────────────────────────────────────────────────
    a.start_step = a.start_step or 1
    a.resumed_from = None
    if a.resume:
        a.start_step, a.resumed_from = _resume_point(a.resume, steps)
    if not 1 <= a.start_step <= len(steps):
        print(f"[중단] --start-step {a.start_step} 이 1~{len(steps)} 범위 밖이다.",
              file=sys.stderr)
        return 2
    if a.start_step > 1:
        st0 = steps[a.start_step - 1]
        # 끊긴 뒤 챔버는 **대기압에 있다** (종료 시 항상 대기압까지 내린다).
        # 이어 하는 첫 단이 ΔP 50 이면 거기로 바로 뛰는 것이라 팔을 때린다.
        # 계획상 어떤 단이었든 기울여 들어간다.
        st0.ramp_in = True
        rest = steps[a.start_step - 1:]
        print(f"\n[이어] {a.start_step} 단부터 — 남은 {len(rest)} 단, "
              f"약 {sum(x.dwell for x in rest)/60.0:.1f} 분")
        if a.resumed_from:
            print(f"       끊긴 런: {a.resumed_from}")
        print(f"       첫 단 s{st0.scen} {st0.seg}  "
              f"P+ {st0.p_pos:.1f} / P− {st0.p_neg:.1f} — 기울여 진입한다")

    print_plan(a, steps, dropped)
    if a.dry_run:
        if a.list_steps:
            print()
            for i, st in enumerate(steps, 1):
                mark = " ⟵ 기울여 진입" if st.ramp_in else ""
                print(f"  {i:4d} s{st.scen:<2s} {st.seg:<22s} "
                      f"P+ {st.p_pos:7.2f} / P− {st.p_neg:7.2f}  "
                      f"ΔP {dp_of(st.p_pos, st.p_neg):5.1f}  α {alpha_of(st.p_pos, st.p_neg):5.2f}  "
                      f"{st.dwell:4.1f} s{mark}")
        return 0
    if not a.yes:
        if input("\n비상정지를 확인했으면 RUN 을 입력: ").strip() != "RUN":
            print("취소했다.")
            return 2

    cfg = read_config(os.path.join(_HERE, "..", "config", "powerpack_config.yaml"))
    out = a.out or os.path.expanduser(
        f"~/result/pressure_id_{datetime.now():%Y%m%d_%H%M%S}.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    _write_meta(out, a, steps, dropped)

    rclpy.init()
    rig = Rig(cfg, a.axis_list[0])
    group = tuple(a.axis_list)

    # 실제 값은 시나리오가 바뀔 때마다 guard_mode 로 갈아 끼운다.
    guards = {ax: Guard(a.rate_base, a.ang_min, a.ang_max, a.track_tol,
                        a.track_grace, a.enc_timeout, a.osc_window, OFF,
                        use_angle=not a.no_angle)
              for ax in a.axis_list}
    allow = a.rate_base          # 지금 허용 중인 각속도 [°/s]
    t_allow = time.monotonic()   # 마지막으로 이완시킨 시각
    prev_ang: dict[int, float | None] = {ax: None for ax in a.axis_list}
    prev_t: dict[int, float] = {ax: time.monotonic() for ax in a.axis_list}
    rate: dict[int, float] = {ax: 0.0 for ax in a.axis_list}

    # 시간 · 목표/실제 양압 · 목표/실제 음압 · 각도. 축이 하나면 접미사 없이 6열이다.
    if len(a.axis_list) == 1:
        cols = ["time_s", "p_pos_ref", "p_pos_meas",
                "p_neg_ref", "p_neg_meas", "angle_deg"]
    else:
        cols = ["time_s"]
        for ax in a.axis_list:
            cols += [f"p_pos_ref_ax{ax+1}", f"p_pos_meas_ax{ax+1}",
                     f"p_neg_ref_ax{ax+1}", f"p_neg_meas_ax{ax+1}",
                     f"angle_deg_ax{ax+1}"]

    conn = None
    drv = None
    cur_step: tuple[int, Step] | None = None
    rc = 0
    n_row = 0
    f = open(out, "w", newline="", buffering=1)
    wr = csv.writer(f)
    wr.writerow(cols)
    t0 = time.monotonic()

    def sample() -> None:
        """한 틱 분을 적고 축마다 안전을 본다. 목표는 **지금 실제로 보내고 있는
        값**이다 (기울여 들어가는 구간에서는 최종 목표가 아니라 그 순간 값).

        감시는 **행을 적은 뒤에** 본다. 먼저 보면 한계를 넘긴 바로 그 표본이
        CSV 에 안 남아, 나중에 왜 걸렸는지 볼 수가 없다 (20260914_180329 은
        마지막 행이 +22.8 °/s 인데 중단 메시지는 +25.9 였다).
        """
        nonlocal n_row, allow, t_allow
        now = time.monotonic()
        # 올라간 상한을 바닥값으로 되돌린다. 계단 뒤에도 팔은 한동안 움직이므로
        # 즉시 조이면 그 정상 응답에서 걸린다.
        w = min(1.0, (now - t_allow) / max(1e-3, a.rate_relax))
        allow += w * (a.rate_base - allow)
        t_allow = now
        for gd in guards.values():
            gd.max_rate = allow
        row = [f"{now - t0:.3f}"]
        pend: list[tuple[int, float, float, float | None, float | None]] = []
        for ax in a.axis_list:
            p_pos = rig.kpa(POS_BOARD(ax))
            p_neg = rig.kpa(NEG_BOARD(ax))
            ang = rig.angle_at(ax)
            if ang is not None:
                if prev_ang[ax] is not None and now > prev_t[ax]:
                    raw = (ang - prev_ang[ax]) / (now - prev_t[ax])
                    w = min(1.0, (now - prev_t[ax]) / max(1e-3, a.rate_tau))
                    rate[ax] += w * (raw - rate[ax])
                prev_ang[ax], prev_t[ax] = ang, now
                pend.append((ax, ang, rate[ax], p_pos, p_neg))
            elif a.no_angle:
                # 각도가 없어도 **압력 추종 감시는 돌려야 한다.** 각도·각속도는
                # 쓰이지 않으므로(use_angle=False) 0 을 넣는다.
                pend.append((ax, 0.0, 0.0, p_pos, p_neg))
            row += [f"{drv.ref_pos:.3f}",
                    f"{p_pos:.3f}" if p_pos is not None else "",
                    f"{drv.ref_neg:.3f}",
                    f"{p_neg:.3f}" if p_neg is not None else "",
                    f"{ang:.3f}" if ang is not None else ""]
        wr.writerow(row)
        n_row += 1

        for ax, ang, rt, p_pos, p_neg in pend:
            guards[ax].check(rig, ang, rt, p_pos, p_neg,
                             drv.ref_pos, drv.ref_neg)

    try:
        conn = _connect(a)
        drv = Driver(conn, group, a.send_hz, a.entry_slew)
        # 대기압을 잠깐 유지해 컨트롤러 적분을 안정시킨다.
        t_end = time.monotonic() + 2.0
        while time.monotonic() < t_end:
            drv.hold(); _spin(rig, drv.dt)
        if a.no_angle:
            print("[각도] --no-angle — 각도 감시를 끈다 (압력 추종 감시는 유지). "
                  "액추에이터를 달면 반드시 빼고 --ang-max 88 로 돌아갈 것")
        elif rig.angle_at(a.axis_list[0]) is None:
            raise Abort("각도를 한 번도 못 받았다 — 엔코더와 런치를 확인할 것.\n"
                        "       액추에이터 없이 밸브·압력만 볼 생각이면 --no-angle 을 준다")

        # ── 레일을 **실측**으로 확인한다 ──────────────────────────────────
        # --pos-rail 은 주장일 뿐 레일을 올리지 않는다. 20260914_180303 에서
        # --pos-rail 200 으로 돌렸는데 rail_ref 를 안 띄워 실제는 160 이었고,
        # P+ 156 목표에서 차압이 4 kPa 밖에 안 남았다.
        # 다만 레일 루프는 200 을 지령해도 185 부근에서 머무는 일이 잦으므로
        # (20260915: 184.3), 기본은 중단이 아니라 **경고 + 실측값으로 재계산**이다.
        if a.rail_tol > 0:
            # 레일이 목표에 들어올 때까지 기다린다. 펌프는 켜자마자 200 이 아니다 —
            # 20260917 런은 300 초쯤부터 200 을 유지했는데, 시작 1 초만 재고
            # 165.6 이라 판단해 **멀쩡히 돌 수 있었던 23 단을 잘랐다.**
            # 여기서 기다리지 않으면 재측정에서도 같은 일이 난다.
            rp, rn = _rail_avg(rig, drv, 1.0)
            t_wait0 = time.monotonic()
            if a.rail_wait > 0:
                ok = lambda v, tgt: (v is None) or abs(v - tgt) <= a.rail_tol
                last = 0.0
                while not (ok(rp, a.pos_rail) and ok(rn, a.neg_rail)):
                    el = time.monotonic() - t_wait0
                    if el >= a.rail_wait:
                        break
                    if el - last >= 10.0 or last == 0.0:
                        print(f"  [레일 대기] {el:4.0f}/{a.rail_wait:.0f} s  "
                              f"P+ {rp if rp is None else round(rp,1)} → {a.pos_rail:g}  "
                              f"P− {rn if rn is None else round(rn,1)} → {a.neg_rail:g}",
                              flush=True)
                        last = el
                    rp, rn = _rail_avg(rig, drv, 2.0)
                if ok(rp, a.pos_rail) and ok(rn, a.neg_rail):
                    print(f"  [레일 대기] {time.monotonic()-t_wait0:.0f} s 만에 들어왔다.")
                else:
                    print(f"  [레일 대기] {a.rail_wait:.0f} s 를 기다렸지만 못 들어왔다 — "
                          f"실측값으로 진행한다.")
            bad = []
            if rp is not None and abs(rp - a.pos_rail) > a.rail_tol:
                bad.append(f"양압 레일 실측 {rp:.1f} ≠ --pos-rail {a.pos_rail:g}")
            if rn is not None and abs(rn - a.neg_rail) > a.rail_tol:
                bad.append(f"음압 레일 실측 {rn:.1f} ≠ --neg-rail {a.neg_rail:g}")
            if bad and a.rail_strict:
                raise Abort("레일이 계획과 다르다 (±%g kPa, --rail-strict):\n       %s"
                            % (a.rail_tol, "\n       ".join(bad)))
            if bad:
                print("\n[경고] " + "\n       ".join(bad))
                if rp is not None:
                    a.pos_rail, a.pos_max = rp, rp - a.rail_margin
                if rn is not None:
                    a.neg_rail, a.neg_min = rn, rn + a.rail_margin
                keep = [st for st in steps
                        if unreachable(st.p_pos, st.p_neg, a) is None]
                cut = len(steps) - len(keep)
                if cut:
                    # 값 비교(x not in keep)로 뽑으면 안 된다 — Step 은 dataclass 라
                    # 값이 같은 다른 단까지 같다고 본다. 동일성으로 가른다.
                    kept = {id(x) for x in keep}
                    cut_list = [x for x in steps if id(x) not in kept]
                    prev_scen = None
                    for st in keep:
                        st.ramp_in = (st.scen != prev_scen)
                        prev_scen = st.scen
                    steps = keep
                    print(f"       → 실측 레일로 도달 범위를 다시 잡았다: "
                          f"P+ ≤ {a.pos_max:.1f} / P− ≥ {a.neg_min:.1f} kPa")
                    print(f"       → **{cut} 단을 뺐다.** 남은 {len(steps)} 단으로 진행한다.")
                else:
                    cut_list = []
                    print(f"       → 도달 범위 P+ ≤ {a.pos_max:.1f} / P− ≥ {a.neg_min:.1f} kPa "
                          f"안에 계획이 전부 들어온다. 그대로 진행한다.")
                # 계획 파일을 실측 레일 기준으로 갱신. **빌드 때 뺀 것(dropped)을
                # 그대로 들고 간다** — 여기서 [] 로 덮어쓰면 n_dropped 가 항상 0 이
                # 되어, "안 잘렸다" 를 확인할 방법이 사라진다. 20260916 에 실제로
                # --pos-rail 없이 돌면 yaml 기본 160 때문에 61 단이 조용히 잘리는데
                # 메타에는 n_dropped: 0 으로 남았다.
                _write_meta(out, a, steps, dropped + [(x, "실측 레일") for x in cut_list])
                print()
            else:
                print(f"[레일] 실측 P+ {rp:.1f} / P− {rn:.1f} kPa — 계획과 일치")

        seg_now = None
        for idx, st in enumerate(steps, 1):
            if idx < a.start_step:
                continue
            if (st.scen, st.seg) != seg_now:
                if seg_now is None or st.scen != seg_now[0]:
                    osc, trk = guard_mode(st.scen, a)
                    for gd in guards.values():
                        gd.osc_flips, gd.track_tol = osc, trk
                        gd._flips.clear(); gd._bad_since = None
                    print(f"\n[감시] 진동 "
                          f"{'끔' if osc >= OFF else f'{osc}회/{a.osc_window:g}s'}"
                          f" · 추종 {'끔' if trk >= OFF else f'{trk:g} kPa/{a.track_grace:g}s'}"
                          f"  (각도 한계는 항상 켜짐)")
                seg_now = (st.scen, st.seg)
                print(f"── s{st.scen} {TITLES[st.scen]} / {st.seg}", flush=True)
            # 이 단이 흔드는 크기만큼 각속도 상한을 올린다. 이완만 하고
            # 내려 잡지 않는다 (max) — 앞 단의 응답이 아직 남아 있다.
            allow = max(allow, rate_allow(excite_kpa(drv.ref_pos, drv.ref_neg, st), a))
            cur_step = (idx, st)

            # ── 전이 — 두 챔버를 **같은 시간에** 끝나게 함께 민다 ──────────
            # 채널마다 따로 기울이면 델타가 작은 쪽이 먼저 도착해, 그 사이 차압이
            # 계획에 없던 값을 지나간다 (배분 전환이면 50 이 아니라 100 kPa 를
            # 지나가 팔을 위로 때린다). 하나의 보간 변수로 묶어야 차압이 두 끝점
            # 사이를 단조롭게 지난다.
            T = transition_s(drv.ref_pos, drv.ref_neg, st, a)
            if T > 0.0:
                p0, n0 = drv.ref_pos, drv.ref_neg
                t_ramp = time.monotonic()
                while True:
                    u = min(1.0, (time.monotonic() - t_ramp) / T)
                    drv.step_toward(p0 + u * (st.p_pos - p0),
                                    n0 + u * (st.p_neg - n0), slew=JUMP)
                    _spin(rig, drv.dt)
                    sample()
                    if u >= 1.0:
                        break
            else:
                drv.step_toward(st.p_pos, st.p_neg, slew=JUMP)
                _spin(rig, drv.dt)
                sample()

            deadline = time.monotonic() + st.dwell
            while time.monotonic() < deadline:  # 유지 — 매 틱 다시 보낸다
                drv.step_toward(st.p_pos, st.p_neg, slew=JUMP)
                _spin(rig, drv.dt)
                sample()

            _write_progress(out, idx, len(steps), t0)
            if idx % a.progress_every == 0 or idx == len(steps):
                angs = " ".join(f"ax{ax+1} {rig.angle_at(ax):6.2f}°"
                                if rig.angle_at(ax) is not None else f"ax{ax+1} --"
                                for ax in a.axis_list)
                print(f"  [{idx}/{len(steps)}] {(time.monotonic()-t0)/60.0:5.1f} 분  "
                      f"P+ {st.p_pos:6.1f} / P− {st.p_neg:6.1f}  {angs}  "
                      f"허용 {allow:.0f} °/s", flush=True)
        print(f"\n[완료] {a.start_step}~{len(steps)} 단, {n_row} 샘플.")
    except Abort as exc:
        print(f"\n[중단] 안전 감시: {exc}", file=sys.stderr)
        if cur_step is not None:
            i_, st_ = cur_step
            print(f"        {i_}/{len(steps)} 단  s{st_.scen} {st_.seg}  "
                  f"P+ {st_.p_pos:.1f} / P− {st_.p_neg:.1f}  "
                  f"(허용은 {allow:.0f} °/s 까지 열려 있었다)", file=sys.stderr)
        rc = 3
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C", file=sys.stderr)
        rc = 130
    except (ConnectionError, OSError, TimeoutError) as exc:
        print(f"\n[위험] 통신 실패: {exc}", file=sys.stderr)
        rc = 1
    finally:
        # ── 어떤 경로로 끝나든 대기압까지 **기울여** 내린다 ──────────────────
        # 끊으면 컨트롤러에 마지막 목표가 남아 압력이 유지된다.
        if drv is not None:
            print(f"[복귀] 양 챔버를 {a.release_slew:g} kPa/s 로 대기압까지 내린다...")
            t_end = time.monotonic() + 120.0
            try:
                while time.monotonic() < t_end:
                    if drv.step_toward(ATM_KPA, ATM_KPA, slew=a.release_slew):
                        break
                    _spin(rig, drv.dt)
                for _ in range(int(1.0 * a.send_hz)):
                    drv.hold(); _spin(rig, drv.dt)
                print("[복귀] 대기압 도달.")
            except OSError as exc:
                print(f"[위험] 복귀 송신 실패: {exc} — 펌프를 즉시 정지할 것.",
                      file=sys.stderr)
                rc = rc or 1
        f.close()
        if conn is not None:
            conn.close()
        rig.destroy_node()
        rclpy.shutdown()
        if n_row:
            print(f"[기록] {out}  ({n_row} 샘플)")
        # ── 못 끝냈으면 **이어 할 명령을 그대로** 찍는다 ────────────────────
        # 47 분짜리를 중간에 끊고 나서 몇 단까지 갔는지 세어 보게 두면 안 된다.
        done = cur_step[0] if cur_step is not None else 0
        if done and done < len(steps):
            # 이전 --resume / --start-step 은 걷어낸다 (값이 따로 오는 형태 포함)
            argv, skip = [], False
            for x in sys.argv[1:]:
                if skip:
                    skip = False; continue
                if x in ("--resume", "--start-step"):
                    skip = True; continue
                if x.startswith(("--resume=", "--start-step=")):
                    continue
                argv.append(x)
            print(f"\n[이어 하기] {done}/{len(steps)} 단에서 멈췄다. "
                  f"남은 {len(steps)-done} 단을 이으려면:\n\n"
                  f"  python3 {os.path.relpath(__file__, os.getcwd())} "
                  f"{' '.join(argv)} \\\n      --resume {out}\n\n"
                  f"  (인자를 그대로 두어야 계획 지문이 맞는다. 다르면 멈춘다)")
    return rc


if __name__ == "__main__":
    sys.exit(main())
