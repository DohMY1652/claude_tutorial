#!/usr/bin/env python3
"""6축(12채널) 무액추에이터 압력 스윕 레퍼런스 전송기.

이 스크립트가 TCP 클라이언트로 동작해 pp_controller의 ``RefTcpServer``에 접속한다.
기존 ``pressure_ref_client.py``는 양/음압 한 쌍만 보내지만, 이 스크립트는 12개
목표를 한 패킷으로 보내므로 여러 축을 동시에 시험할 수 있다. 파일명에 server가
남아 있는 것은 최초 버전의 흔적이며, 현재 네트워크 역할은 명확히 클라이언트다.

패킷 순서와 단위
------------------
  [axis1 P+, axis2 P+, ..., axis6 P+,
   axis1 P-, axis2 P-, ..., axis6 P-]  [kPa absolute]

전송 형식은 기존 RefTcpServer와 같은 little-endian double 12개(96바이트)다.
컨트롤러는 ``control_mode=0``과 ``RefTcpServer.all_channels=true``로 실행해야 한다.

축 선택
--------
  시작할 때 돌릴 축을 받는다. ``--axes 1,3`` 처럼 주거나, 생략하면 실행 직후
  물어본다 (``all`` = 6축 전부). 선택한 축의 채널들은 **같은 목표를 동시에**
  받고, 선택하지 않은 축은 계획 내내 대기압을 유지한다.

시험 순서
---------
  스윕 종류를 ``--sweep`` 으로 고르거나 실행 직후 물어본다.

  · 양압 : 대기압 -> 180 kPa (P- 는 대기압 유지)
  · 음압 : 대기압 -> 20 kPa  (P+ 는 대기압 유지)
  · 동시 : P+ 를 올리면서 P- 를 같이 내린다 (한 챔버쌍에 차압을 동시에 건다)
  · 전체 : 양압 -> 음압 -> 동시 (기본)

  정점(되돌아올 지점)은 실행 직후 물어본다 (``--pos-max`` / ``--neg-min`` 으로 줄 수도
있다). 각 종류를 10/20/30/40/50 kPa 간격으로 훑고, **올라간 계단을 같은 스텝으로
되짚어 내려와** 대기압에서 끝나므로 하강 방향 스텝 응답도 같은 조건으로 잰다:

    101.3 → 111.3 → ... → 171.3 → 180.0 → 171.3 → ... → 111.3 → 101.3

  (--one-way 를 주면 예전처럼 정점에서 대기압으로 한 번에 떨어진다.)
  모든 목표는 --dwell초 동안 --send-hz 주기로 반복 전송된다.

Ctrl-C나 정상 종료 시에는 가능한 경우 12채널 모두 대기압을 여러 번 보낸다.
TCP가 끊기면 컨트롤러에는 마지막 목표가 남을 수 있으므로 제어기/펌프를 즉시
정지하고 실제 압력을 확인해야 한다.
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import time
from dataclasses import dataclass
from typing import Sequence


NUM_AXES = 6
ATM_KPA = 101.325
# 스윕 끝값 [kPa absolute]. --pos-max / --neg-min 으로 실행마다 바꿀 수 있다.
POS_MAX_KPA = 180.0
# 20.0 → 35.0: 진공 레일이 실측 31 kPa abs 인데 목표가 20 이면 **물리적으로 도달
# 불가**다 (챔버는 레일보다 낮아질 수 없다). 그 구간에서 5 초씩 밸브를 활짝 열고
# 진공 펌프만 소모했고, 지표에는 도달 실패(오버슛 −1.9 kPa)로 찍혔다
# (실기 20260907_213052). 레일보다 4 kPa 위에서 끝낸다 — 레일이 더 깊어지면
# --neg-min 으로 내릴 것.
NEG_MIN_KPA = 35.0
STEP_SIZES_KPA = (10.0, 20.0, 30.0, 40.0, 50.0)

# 내부 축 인덱스는 0부터, 사용자에게 보이는 번호는 1부터다.
ALL_AXES = tuple(range(NUM_AXES))

# 스윕 종류. "all" 은 양압 -> 음압 -> 동시 순서로 전부 돈다.
# 계단 스윕
SWEEP_KINDS = ("pos", "neg", "both", "all")
# 사인파 — 계단 대신 연속 파형을 보낸다 (아래 run_sine)
SINE_KINDS = ("sine-pos", "sine-neg", "sine-same", "sine-diff")
# 튜닝용 — 동작점(중앙 압력)마다 작은 양방향 계단을 준다. 게인을 정하려면 계단
# 응답이 필요하고, **동작점마다** 필요하다 (밸브 개도와 차압에 따라 플랜트 이득이
# 변하므로 한 점에서 맞춘 게인이 다른 점에서 진동한다).
TUNE_KINDS = ("tune-pos", "tune-neg")

# 여러 동작 범위를 한 번에 훑는 시나리오. 한 실행으로 진폭·주기·동작점을 모두 바꿔
# 보고, 어느 조건에서 무너지는지 한 로그 안에서 비교한다.
#   (이름, 진폭 p-p, 주기 s, 반복, 양압 중앙값, 음압 중앙값)
# 설계 근거:
#   · 기준       지금까지 튜닝한 조건. 다른 구간과 비교할 기준선이 된다.
#   · 큰 진폭    유량 요구를 2배로. 레일이 버티는지 본다.
#   · 빠른 주기  기울기 1.57 → 4.19 kPa/s. 대역폭 한계가 드러난다.
#   · 작고 빠름  기울기 3.14 kPa/s 인데 진폭이 작아 데드존 분해능이 시험된다.
#   · 높은 동작점 양압 챔버 122~142 → micro 차압이 3~23 으로 줄어든다.
#                 차압 보정(gain_dp_ref)이 실제로 먹는지 보는 구간.
#   · 낮은 동작점 양압 챔버 105~125 → atm 차압이 4~24 로 줄어 배기가 약해진다.
SCAN_SEGMENTS = (
    ("기준  진폭20 주기40", 20.0, 40.0, 2, 120.0, 60.0),
    ("큰진폭 진폭40 주기40", 40.0, 40.0, 2, 125.0, 60.0),
    ("빠름  진폭20 주기15", 20.0, 15.0, 4, 120.0, 60.0),
    ("작고빠름 진폭10 주기10", 10.0, 10.0, 5, 120.0, 60.0),
    ("높은동작점 진폭20 주기40", 20.0, 40.0, 2, 132.0, 45.0),
    ("낮은동작점 진폭20 주기40", 20.0, 40.0, 2, 115.0, 75.0),
)
# ── 주기 스캔 (sine-freq) ───────────────────────────────────────────────────
# 진폭·동작점을 **고정**하고 주기만 줄여 간다. 진폭이 고정이므로 주기를 반으로
# 줄이면 요구 기울기가 두 배가 된다 (max|dP/dt| = pi*amp/T).
# 마지막에 첫 주기를 한 번 더 돈다 — 데드존이 운전 중에 밀려서(9분에 3~5 %p)
# 느린 구간과 빠른 구간을 그냥 비교하면 표류와 주파수 영향이 섞인다.
# 처음과 끝의 같은 주기끼리 비교하면 그 표류분을 따로 떼어 볼 수 있다.
FREQ_PERIODS_S = (80.0, 60.0, 40.0, 25.0, 16.0, 10.0)

def freq_cycles(period: float) -> int:
    """주기마다 반복 횟수. 느린 구간에서 시간을 낭비하지 않으면서도
    과도 1주기 + 정상 1주기 이상은 확보한다."""
    return int(max(2, min(5, round(40.0 / period))))

TUNE_CENTERS_POS = (120.0, 140.0, 160.0)
TUNE_CENTERS_NEG = (80.0, 60.0, 40.0)
TUNE_STEPS_KPA = (2.0, 5.0, 10.0)
ALL_KINDS = (*SWEEP_KINDS, *SINE_KINDS, *TUNE_KINDS, "sine-scan", "sine-freq")
SWEEP_LABEL = {
    "pos": "양압만", "neg": "음압만",
    "both": "양·음압 동시", "all": "양압 → 음압 → 동시",
    "sine-pos":  "사인파 — 양압만",
    "sine-neg":  "사인파 — 음압만",
    "sine-same": "사인파 — 양·음압 같은 방향 (차압 유지한 채 함께 오르내림)",
    "sine-diff": "사인파 — 양·음압 반대 방향 (벌어졌다 가까워짐)",
    "tune-pos":  "튜닝용 계단 — 양압, 동작점 3곳 × 계단 ±2/±5/±10 kPa",
    "tune-neg":  "튜닝용 계단 — 음압, 동작점 3곳 × 계단 ±2/±5/±10 kPa",
    "sine-scan": "사인 시나리오 — 진폭·주기·동작점을 바꿔가며 한 번에 6구간",
    "sine-freq": "주기 스캔 — 진폭·동작점 고정, 주기만 점점 빠르게 (끊김 없이 연속)",
}

SINE_AMP_KPA = 40.0      # 대기압에서의 최대 편차 [kPa]
SINE_PERIOD_S = 10.0     # 한 주기 [s]
SINE_CYCLES = 3          # 반복 주기 수
SINE_LEAD_S = 3.0        # 파형 시작점까지 올라가는 진입 램프 [s]


@dataclass(frozen=True)
class Stage:
    """5초 동안 유지할 한 목표와 사람이 읽을 설명."""

    group: tuple[int, ...]
    phase: str
    step_kpa: float | None
    target_kpa: float
    refs_kpa: tuple[float, ...]
    # 양·음압 동시 구간에서만 쓴다. target_kpa 가 P+, 이것이 P- 목표다.
    target_neg_kpa: float | None = None


def _axis_label(group: Sequence[int]) -> str:
    return "+".join(str(axis + 1) for axis in group) + "축"


def parse_axes(text: str) -> tuple[int, ...]:
    """"1,3" / "1 3" / "all" 을 내부 축 인덱스 튜플로 바꾼다 (1-based 입력).

    중복은 제거하고 오름차순으로 정렬한다. 잘못된 입력은 ValueError 다 —
    실기를 움직이는 스크립트이므로 조용히 넘기지 않는다.
    """

    cleaned = text.strip().lower()
    if not cleaned:
        raise ValueError("축을 하나 이상 지정해야 한다")
    if cleaned in ("all", "a", "*"):
        return ALL_AXES

    axes: set[int] = set()
    for token in cleaned.replace(",", " ").split():
        try:
            number = int(token)
        except ValueError as exc:
            raise ValueError(f"축 번호가 아니다: {token!r}") from exc
        if not 1 <= number <= NUM_AXES:
            raise ValueError(f"축 번호는 1~{NUM_AXES} 여야 한다: {number}")
        axes.add(number - 1)
    return tuple(sorted(axes))


def prompt_axes() -> tuple[int, ...]:
    """--axes 를 주지 않았을 때 실행 직후 물어본다."""

    print(f"\n돌릴 축을 고른다 (1~{NUM_AXES}). 쉼표나 공백으로 여러 개, all = 전체.")
    print("  예) 1        → 축1 만 (양압 ch0 / 음압 ch6)")
    print("  예) 1,3,5    → 세 축에 같은 목표를 동시에")
    print("  예) all      → 6축 동시")
    while True:
        try:
            answer = input("축: ")
        except EOFError:
            raise ValueError("축 입력이 없다 (--axes 로 지정할 것)") from None
        try:
            return parse_axes(answer)
        except ValueError as exc:
            print(f"  {exc}. 다시 입력한다.")


def prompt_sweep() -> str:
    """--sweep 을 주지 않았을 때 스윕 종류를 물어본다 (엔터 = 전체)."""

    print("\n종류를 고른다.")
    print("  ── 계단 스윕 ──")
    print("  1) 양압 → 음압 → 동시 (기본)")
    print("  2) 양압만")
    print("  3) 음압만")
    print("  4) 양·음압 동시만")
    print("  ── 사인파 ──")
    print("  5) 양압만 오르내림")
    print("  6) 음압만 오르내림")
    print("  7) 같은 방향  — 차압을 유지한 채 둘이 함께 오르내림")
    print("  8) 반대 방향  — 벌어졌다 가까워짐")
    print("  ── 게인 튜닝용 계단 ──")
    print("  9) 양압  — 동작점 3곳 × 계단 ±2/±5/±10 kPa 양방향")
    print(" 10) 음압  — 동작점 3곳 × 계단 ±2/±5/±10 kPa 양방향")
    print("  ── 시나리오 ──")
    print(" 11) 사인 스캔 — 진폭·주기·동작점을 바꿔가며 6구간을 한 번에 (약 9분)")
    print(" 12) 주기 스캔 — 진폭·동작점 고정, 주기만 점점 빠르게 (연속, + 표류 확인)")
    table = {"": "all", "1": "all", "2": "pos", "3": "neg", "4": "both",
             "5": "sine-pos", "6": "sine-neg", "7": "sine-same", "8": "sine-diff",
             "9": "tune-pos", "10": "tune-neg", "11": "sine-scan", "12": "sine-freq"}
    table.update({k: k for k in ALL_KINDS})
    while True:
        try:
            answer = input("종류 (엔터=1): ").strip().lower()
        except EOFError:
            return "all"
        if answer in table:
            return table[answer]
        print("  1~12 중에서 고른다.")


def prompt_endpoint(label: str, default: float, lo: float, hi: float,
                    note: str = "", unit: str = "kPa abs") -> float:
    """수치 하나를 물어본다 (정점·진폭·주기 공용). 엔터면 기본값."""

    if note:
        print(f"  {note}")
    suffix = f" [{unit}]" if unit else ""
    while True:
        try:
            raw = input(f"{label}{suffix} (엔터={default:g}): ").strip()
        except EOFError:
            return default
        if not raw:
            return default
        try:
            value = float(raw)
        except ValueError:
            print("  숫자를 입력한다.")
            continue
        if not lo < value <= hi:
            print(f"  {lo:g} 보다 크고 {hi:g} 이하여야 한다.")
            continue
        return value


def _ramp_values(start: float, stop: float, increment: float) -> list[float]:
    """start는 제외하고 stop은 반드시 포함하는 단조 목표열을 만든다.

    예: 101.325 -> 180, increment=30이면 [131.325, 161.325, 180.0].
    마지막 간격이 increment보다 작더라도 사용자가 지정한 끝값을 정확히 넣는다.

    단 격자 마지막 값이 끝값과 **거의 같으면** 끝값으로 덮어쓴다. 그러지 않으면
    0.3 kPa 차이로 같은 목표가 두 스테이지가 된다 (정점 51 을 고르면 격자가
    51.325 에서 끝나 51.325 → 51.0 이 연달아 나왔다). 센서 분해능이 0.25 kPa 이므로
    0.5 kPa 미만 차이는 의미가 없다.
    """

    if increment <= 0.0:
        raise ValueError("increment는 0보다 커야 한다")
    direction = 1.0 if stop > start else -1.0
    values: list[float] = []
    value = start
    while True:
        candidate = value + direction * increment
        if (direction > 0.0 and candidate >= stop) or (
                direction < 0.0 and candidate <= stop):
            break
        values.append(candidate)
        value = candidate
    eps = max(0.5, 0.05 * increment)
    if values and abs(values[-1] - stop) < eps:
        values[-1] = stop
    else:
        values.append(stop)
    return values


def _paired_ramps(increment: float, pos_max: float = POS_MAX_KPA,
                  neg_min: float = NEG_MIN_KPA) -> list[tuple[float, float]]:
    """양·음압 동시 램프. 두 목표가 **같은 스테이지 수**로 끝값에 닿게 만든다.

    두 방향의 스팬이 다르다 (P+ 78.675 kPa, P- 81.325 kPa). 같은 절대 간격을
    쓰면 스텝 수가 어긋나 한쪽이 먼저 끝에 닿고, 그 뒤 구간은 "동시" 가 아니다.
    그래서 양압을 기준으로 스텝 수를 정하고 음압은 같은 비율로 배분한다 —
    음압의 실제 간격은 increment × (81.325 / 78.675) ≈ increment × 1.034 다.
    """

    pos_values = _ramp_values(ATM_KPA, pos_max, increment)
    count = len(pos_values)
    neg_values = [ATM_KPA + (neg_min - ATM_KPA) * (i + 1) / count
                  for i in range(count)]
    return list(zip(pos_values, neg_values))


def _round_trip(values: Sequence[float], one_way: bool = False) -> list[float]:
    """편도 목표열에 **같은 스텝으로 되돌아오는 복귀열**을 붙인다.

    예 (양압 10 kPa 스텝):
      111.3 121.3 131.3 141.3 151.3 161.3 171.3 180.0
                                              ↓ 정점은 한 번만 지난다
      171.3 161.3 151.3 141.3 131.3 121.3 111.3 101.3

    복귀를 계단으로 내려오면 **하강 방향의 스텝 응답**도 같은 조건으로 잴 수 있다.
    예전에는 정점에서 대기압으로 한 번에 떨어뜨려서(78 kPa 계단) 하강 지표가
    큰 스텝 하나뿐이었다.
    끝에 대기압을 넣어 마무리하므로 다음 스텝 크기 블록은 항상 대기압에서 시작한다.
    one_way=True 면 예전 동작(편도 + 대기압 복귀 1스텝)과 같다.
    """

    seq = list(values)
    if one_way:
        return seq + [ATM_KPA]
    return seq + list(reversed(seq[:-1])) + [ATM_KPA]


def _refs(group: Sequence[int], positive: float = ATM_KPA,
          negative: float = ATM_KPA) -> tuple[float, ...]:
    """선택 축만 바꾸고 나머지 12채널은 대기압인 배열을 만든다."""

    pos = [ATM_KPA] * NUM_AXES
    neg = [ATM_KPA] * NUM_AXES
    for axis in group:
        pos[axis] = positive
        neg[axis] = negative
    return tuple(pos + neg)


def build_tune_stages(group: Sequence[int], positive: bool,
                      centers: Sequence[float]) -> list[Stage]:
    """게인 튜닝용 계단열.

    한 동작점에서만 맞춘 게인은 다른 동작점에서 진동한다. 밸브 개도가 달라지면
    플랜트 이득 dP/du 가 같이 변하기 때문이다 (유량 ∝ 유효면적 × 차압 함수).
    그래서 **동작점 3곳 × 계단 3크기 × 양방향** 으로 응답을 모은다:

        중앙 → 중앙+Δ → 중앙 → 중앙−Δ → 중앙   (Δ = 2, 5, 10 kPa)

    각 계단에서 오버슛·정착시간·정상상태오차를 뽑으면 kp/ki/kd 를 근거로 정할 수
    있다. 작은 계단(±2)은 데드존 보상과 분해능을, 큰 계단(±10)은 유량 한계와
    적분 windup 을 드러낸다. 양방향인 이유는 채우는 밸브와 빼는 밸브가 다르기
    때문이다 (게인도 방향별로 따로 있다).
    """
    stages: list[Stage] = []
    stages.append(Stage(group, "시작/대기", None, ATM_KPA, _refs(group)))
    kw = "positive" if positive else "negative"
    side = "양압" if positive else "음압"
    for c in centers:
        stages.append(Stage(group, f"{side} 동작점 {c:g} 안정화", None, c,
                            _refs(group, **{kw: c})))
        for dv in TUNE_STEPS_KPA:
            for tgt, lab in ((c + dv, f"+{dv:g}"), (c, "복귀"),
                             (c - dv, f"−{dv:g}"), (c, "복귀")):
                stages.append(Stage(group, f"{side} {c:g} {lab}", dv, tgt,
                                    _refs(group, **{kw: tgt})))
    stages.append(Stage(group, "대기 복귀", None, ATM_KPA, _refs(group)))
    return stages


def build_stages(group: Sequence[int], sweep: str = "all",
                 pos_max: float = POS_MAX_KPA,
                 neg_min: float = NEG_MIN_KPA,
                 one_way: bool = False) -> list[Stage]:
    """선택한 축들에 **같은 목표를 동시에** 주는 스윕 순서를 만든다."""

    group = tuple(group)
    if not group:
        raise ValueError("축을 하나 이상 지정해야 한다")
    if sweep not in SWEEP_KINDS:
        raise ValueError(f"스윕 종류는 {SWEEP_KINDS} 중 하나여야 한다: {sweep!r}")

    stages: list[Stage] = []
    # 시작 직후 모든 챔버가 대기압 목표인지 먼저 1스텝 확인한다.
    stages.append(Stage(group, "시작/대기", None, ATM_KPA, _refs(group)))

    # P+만 상승한다. P-와 선택되지 않은 모든 축은 계속 대기압이다.
    if sweep in ("pos", "all"):
        for step in STEP_SIZES_KPA:
            prev = ATM_KPA
            for target in _round_trip(_ramp_values(ATM_KPA, pos_max, step), one_way):
                phase = "양압 상승" if target > prev else "양압 하강"
                stages.append(Stage(group, phase, step, target,
                                    _refs(group, positive=target)))
                prev = target

    # P-만 하강한다. P+와 선택되지 않은 모든 축은 계속 대기압이다.
    if sweep in ("neg", "all"):
        for step in STEP_SIZES_KPA:
            prev = ATM_KPA
            for target in _round_trip(_ramp_values(ATM_KPA, neg_min, step), one_way):
                phase = "음압 하강" if target < prev else "음압 상승"
                stages.append(Stage(group, phase, step, target,
                                    _refs(group, negative=target)))
                prev = target

    # 양·음압 동시. 한 축의 두 챔버에 차압을 동시에 걸어 레일 두 개를 같이
    # 부하한다 (양압 펌프와 진공 펌프가 동시에 수요를 받는 조건).
    if sweep in ("both", "all"):
        for step in STEP_SIZES_KPA:
            pairs = _paired_ramps(step, pos_max, neg_min)
            # 두 목표를 함께 미러링한다 — 복귀도 동시에 계단으로 올라온다/내려온다.
            if one_way:
                seq = pairs + [(ATM_KPA, ATM_KPA)]
            else:
                seq = pairs + list(reversed(pairs[:-1])) + [(ATM_KPA, ATM_KPA)]
            prev = ATM_KPA
            for pos_target, neg_target in seq:
                phase = "동시 확대" if pos_target > prev else "동시 축소"
                stages.append(Stage(group, phase, step, pos_target,
                                    _refs(group, positive=pos_target,
                                          negative=neg_target),
                                    target_neg_kpa=neg_target))
                prev = pos_target
    return stages


def sine_center(mode: str, amp: float, c_pos: float | None,
                c_neg: float | None) -> tuple[float, float]:
    """중앙값 기본값을 모드에 맞게 정한다.

    **흔들리는 쪽**은 대기압에서 amp/2 만큼 떨어진 곳이 기본이다 — 그래야 파형이
    대기압에서 출발해 amp 만큼 갔다 돌아온다 (중앙값 옵션이 없던 시절과 같은 거동).
    **고정된 쪽**은 대기압이 기본이다.
    """
    swing_pos = mode in ("sine-pos", "sine-same", "sine-diff")
    swing_neg = mode in ("sine-neg", "sine-same", "sine-diff")
    if c_pos is None:
        c_pos = ATM_KPA + amp * 0.5 if swing_pos else ATM_KPA
    if c_neg is None:
        c_neg = ATM_KPA - amp * 0.5 if swing_neg else ATM_KPA
    return c_pos, c_neg


def sine_targets(mode: str, amp: float, phase: float,
                 c_pos: float, c_neg: float) -> tuple[float, float]:
    """정규화 위상 s ∈ [0,1] 에서 (P+ 목표, P− 목표) 를 만든다.

    파형은 (1 − cos)/2 다 — 시작·끝에서 기울기가 0 이라 계단 없이 출발하고 멈춘다.

    **amp 는 진폭(peak-to-peak), c_* 는 중앙값**이다:

        값 = 중앙값 − amp/2 + amp·s        (s: 0 → 1 → 0)

    즉 중앙값에서 ±amp/2 로 흔들린다. 중앙값을 안 주면 대기압에서 출발하도록
    잡히므로(sine_center 참조) 예전과 같은 파형이 된다.

    모드:
      sine-pos   양압만 오르내린다   P− 는 c_neg 에 고정
      sine-neg   음압만 오르내린다   P+ 는 c_pos 에 고정
      sine-same  둘이 **같은 방향**으로 (차압을 유지한 채 함께 오르내림)
      sine-diff  둘이 **반대 방향**으로 (벌어졌다 가까워짐)
    """

    s = (1.0 - math.cos(2.0 * math.pi * phase)) * 0.5      # 0 → 1 → 0
    half = amp * 0.5
    up = -half + amp * s          # −amp/2 → +amp/2 → −amp/2
    if mode == "sine-pos":
        return c_pos + up, c_neg
    if mode == "sine-neg":
        return c_pos, c_neg + up
    if mode == "sine-same":
        return c_pos + up, c_neg + up
    if mode == "sine-diff":
        return c_pos + up, c_neg - up
    raise ValueError(f"사인 모드가 아니다: {mode!r}")


def run_sine_edge(conn: socket.socket | None, group: Sequence[int], mode: str,
                  amp: float, send_hz: float, c_pos: float, c_neg: float,
                  lead: float, entering: bool) -> None:
    """대기압 ↔ 파형 시작점 사이의 램프 **한 번**.

    주기 스캔처럼 여러 구간을 끊지 않고 이어 붙일 때 쓴다. 램프는 전체의 맨
    앞·맨 뒤에 한 번씩만 두고, 구간 사이에는 아무것도 넣지 않는다.
    파형이 (1−cos)/2 라 시작점과 끝점이 둘 다 `중앙값 − 진폭/2` 이고 그 지점의
    기울기가 0 이므로, 구간끼리는 값도 기울기도 그대로 이어진다.
    """
    if lead <= 0.0:
        return
    start = sine_targets(mode, amp, 0.0, c_pos, c_neg)
    a_pt = (ATM_KPA, ATM_KPA) if entering else start
    b_pt = start if entering else (ATM_KPA, ATM_KPA)
    period_s = 1.0 / send_hz
    t0 = time.monotonic()
    next_log = 0.0
    while True:
        t = time.monotonic() - t0
        if t >= lead:
            break
        k = t / lead
        pos = a_pt[0] + (b_pt[0] - a_pt[0]) * k
        neg = a_pt[1] + (b_pt[1] - a_pt[1]) * k
        if conn is not None:
            conn.sendall(encode_refs(_refs(group, positive=pos, negative=neg)))
        if t >= next_log:
            print(f"  [{t:5.1f}/{lead:.0f} s] {'진입' if entering else '복귀':>8s}  "
                  f"P+={pos:7.2f}  P−={neg:7.2f} kPa abs", flush=True)
            next_log = t + 1.0
        time.sleep(period_s)
    if conn is not None:
        conn.sendall(encode_refs(_refs(group, positive=b_pt[0], negative=b_pt[1])))


def run_sine(conn: socket.socket | None, group: Sequence[int], mode: str,
             amp: float, period: float, cycles: int, send_hz: float,
             c_pos: float, c_neg: float, lead: float = SINE_LEAD_S) -> None:
    """사인파 목표를 send_hz 로 연속 전송한다.

    계단 스윕과 달리 스테이지가 없다 — 매 전송마다 목표를 다시 계산한다.
    시작·끝에는 진입/복귀 램프를 둔다: sine-same 은 P− 가 대기압이 아닌
    (대기압 − amp) 에서 시작하므로 그냥 틀면 그 자체가 큰 계단이 된다.
    """

    period_s = 1.0 / send_hz
    t0 = time.monotonic()
    start_pos, start_neg = sine_targets(mode, amp, 0.0, c_pos, c_neg)
    total = lead + cycles * period + lead
    next_log = 0.0

    while True:
        t = time.monotonic() - t0
        if t >= total:
            break
        if t < lead:                     # 진입 램프: 대기압 → 파형 시작점
            k = t / lead
            pos = ATM_KPA + (start_pos - ATM_KPA) * k
            neg = ATM_KPA + (start_neg - ATM_KPA) * k
            phase_txt = "진입"
        elif t < lead + cycles * period:  # 파형
            ph = (t - lead) / period
            pos, neg = sine_targets(mode, amp, ph, c_pos, c_neg)
            phase_txt = f"{ph:4.2f}주기"
        else:                            # 복귀 램프: 파형 끝점 → 대기압
            k = (t - lead - cycles * period) / lead
            pos = start_pos + (ATM_KPA - start_pos) * k
            neg = start_neg + (ATM_KPA - start_neg) * k
            phase_txt = "복귀"

        refs = _refs(group, positive=pos, negative=neg)
        if conn is not None:
            conn.sendall(encode_refs(refs))
        if t >= next_log:
            print(f"  [{t:6.1f}/{total:.0f} s] {phase_txt:>8s}  "
                  f"P+={pos:7.2f}  P−={neg:7.2f} kPa abs", flush=True)
            next_log = t + 1.0
        time.sleep(period_s)


def encode_refs(refs_kpa: Sequence[float]) -> bytes:
    """kPa 목표 12개를 RefTcpServer의 little-endian double 패킷으로 만든다."""

    if len(refs_kpa) != 2 * NUM_AXES:
        raise ValueError(f"압력 목표는 {2 * NUM_AXES}개여야 한다")
    for pressure in refs_kpa:
        if not 0.0 <= pressure <= 1000.0:
            raise ValueError(f"비정상 압력 목표: {pressure} kPa absolute")
    return struct.pack(f"<{len(refs_kpa)}d", *refs_kpa)


def _fmt_target(stage: Stage) -> str:
    """동시 구간은 P+/P- 를 함께 보여준다."""

    if stage.target_neg_kpa is None:
        return f"{stage.target_kpa:7.3f}"
    return f"{stage.target_kpa:7.3f}/{stage.target_neg_kpa:7.3f}"


def _fmt_refs(refs: Sequence[float]) -> str:
    pos = ", ".join(f"{v:7.3f}" for v in refs[:NUM_AXES])
    neg = ", ".join(f"{v:7.3f}" for v in refs[NUM_AXES:])
    return f"P+=[{pos}]  P-=[{neg}]"


def print_plan(stages: Sequence[Stage], dwell: float, verbose: bool,
               group: Sequence[int], sweep: str = "all",
               pos_max: float = POS_MAX_KPA, neg_min: float = NEG_MIN_KPA,
               one_way: bool = False) -> None:
    duration = len(stages) * dwell
    print("\n=== 압력 스윕 계획 ===")
    print(f"스테이지 {len(stages)}개 x {dwell:g}초 = "
          f"{duration / 60.0:.1f}분")
    print("채널 순서: [1P+,2P+,3P+,4P+,5P+,6P+,1P-,2P-,3P-,4P-,5P-,6P-]")
    print(f"선택한 축: {_axis_label(group)} "
          f"(양압 gid {', '.join(str(a) for a in group)} / "
          f"음압 gid {', '.join(str(a + NUM_AXES) for a in group)}) — 동시 지령")
    print(f"나머지 축: 계획 내내 대기압 {ATM_KPA:g} kPa abs 유지")
    print(f"스윕 종류: {SWEEP_LABEL[sweep]}")
    print(f"끝값: 양압 최대 {pos_max:g} / 음압 최소 {neg_min:g} kPa abs "
          f"(진공 레일보다 낮은 음압 목표는 도달 불가)")
    print("복귀: " + ("정점에서 대기압으로 한 번에 (--one-way)" if one_way else
                      "같은 스텝으로 계단 복귀 — 하강 방향 스텝 응답도 같이 잰다"))
    if verbose:
        for index, stage in enumerate(stages, 1):
            step = "-" if stage.step_kpa is None else f"{stage.step_kpa:g}"
            print(f"{index:03d} {_axis_label(stage.group):9s} "
                  f"{stage.phase:13s} step={step:>2s} "
                  f"target={_fmt_target(stage):>15s}  {_fmt_refs(stage.refs_kpa)}")


def _send_for_dwell(conn: socket.socket, payload: bytes,
                    dwell: float, send_hz: float) -> None:
    """한 목표를 dwell 동안 반복 전송해 순간적인 패킷 하나에 의존하지 않는다."""

    period = 1.0 / send_hz
    deadline = time.monotonic() + dwell
    while True:
        conn.sendall(payload)
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break
        time.sleep(min(period, remaining))


def _connect(args) -> socket.socket:
    """RefTcpServer 에 붙는다. --connect-timeout 동안 재시도한다."""

    deadline = time.monotonic() + args.connect_timeout
    while True:
        try:
            conn = socket.create_connection((args.host, args.port), timeout=2.0)
            break
        except OSError as exc:
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"{args.host}:{args.port}에 {args.connect_timeout:g}초 동안 "
                    f"접속하지 못했다: {exc}") from exc
            print(f"[접속 재시도] {args.host}:{args.port} — {exc}")
            time.sleep(1.0)
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    print(f"[접속] pp_controller RefTcpServer: {args.host}:{args.port}")
    return conn


def _send_atmosphere(conn: socket.socket | None) -> None:
    """종료 전에 전 채널 대기압 목표를 반복 전송한다."""

    if conn is None:
        return
    refs = (ATM_KPA,) * (2 * NUM_AXES)
    payload = encode_refs(refs)
    try:
        for _ in range(10):
            conn.sendall(payload)
            time.sleep(0.05)
        print(f"\n[안전 복귀] 전 채널 대기압 목표 전송: {_fmt_refs(refs)}")
    except OSError as exc:
        print(f"\n[위험] 대기압 목표 전송 실패: {exc}", file=sys.stderr)
        print("       컨트롤러에는 마지막 목표가 남을 수 있다. 펌프/제어기를 즉시 정지할 것.",
              file=sys.stderr)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--one-way", action="store_true",
                        help="복귀를 계단으로 내려오지 않고 정점에서 대기압으로 한 번에 "
                             "떨어뜨린다 (예전 동작)")
    parser.add_argument("--pos-max", type=float, default=None,
                        help=f"양압 정점 [kPa abs] — 여기까지 올라갔다 내려온다 "
                             f"(생략하면 실행 직후 물어본다, 기본 {POS_MAX_KPA:g})")
    parser.add_argument("--neg-min", type=float, default=None,
                        help=f"음압 정점 [kPa abs] — 여기까지 내려갔다 올라온다 "
                             f"(생략하면 물어본다, 기본 {NEG_MIN_KPA:g}). "
                             "진공 레일보다 낮게 주면 도달할 수 없다")
    parser.add_argument("--axes", default="",
                        help="돌릴 축 1~6 (예: 1,3,5 / all). 생략하면 실행 직후 물어본다")
    parser.add_argument("--sweep", default="", choices=("", *ALL_KINDS),
                        help="계단: pos | neg | both | all / "
                             "튜닝: tune-pos | tune-neg  |  "
                             "사인파: sine-pos | sine-neg | sine-same | sine-diff "
                             "(생략하면 실행 직후 물어본다)")
    parser.add_argument("--sine-amp", type=float, default=None,
                        help=f"사인 진폭 = 대기압에서의 최대 편차 [kPa] "
                             f"(생략하면 물어본다, 기본 {SINE_AMP_KPA:g})")
    parser.add_argument("--sine-period", type=float, default=None,
                        help=f"사인 한 주기 [s] (생략하면 물어본다, 기본 {SINE_PERIOD_S:g})")
    parser.add_argument("--scan-gap", type=float, default=8.0,
                        help="사인 스캔(sine-scan)에서 구간 사이 대기압 유지 시간 [s]. "
                             "로그에서 구간 경계를 찾기 쉽고, 앞 구간의 적분 상태가 "
                             "다음 구간으로 새지 않는다. "
                             "주기 스캔(sine-freq)에는 적용되지 않는다 — 거기서는 "
                             "구간을 끊지 않고 이어 붙인다")
    parser.add_argument("--freq-periods", default=None,
                        help="주기 스캔(sine-freq)에서 쓸 주기 목록 [s], 쉼표. "
                             f"기본 {','.join('%g' % v for v in FREQ_PERIODS_S)}. "
                             "진폭이 고정이라 주기를 반으로 줄이면 요구 기울기가 "
                             "두 배가 된다 (max|dP/dt| = pi*진폭/주기)")
    parser.add_argument("--no-freq-recheck", action="store_true",
                        help="주기 스캔 끝에 첫 주기를 다시 돌지 않는다. "
                             "기본은 다시 도는 것이다 — 데드존이 운전 중에 밀리므로 "
                             "그 표류분을 주파수 영향과 분리하려면 필요하다")
    parser.add_argument("--tune-centers", default=None,
                        help="튜닝 계단의 동작점 [kPa abs], 쉼표. 기본 양압 "
                             f"{','.join('%g' % v for v in TUNE_CENTERS_POS)} / 음압 "
                             f"{','.join('%g' % v for v in TUNE_CENTERS_NEG)}")
    parser.add_argument("--sine-center-pos", type=float, default=None,
                        help="양압 사인파의 **중앙값** [kPa abs]. 파형은 여기서 "
                             "±진폭/2 로 흔들린다. 생략하면 대기압에서 출발하도록 "
                             "자동으로 잡는다 (= 대기압 + 진폭/2)")
    parser.add_argument("--sine-center-neg", type=float, default=None,
                        help="음압 사인파의 중앙값 [kPa abs]. 생략하면 대기압 − 진폭/2")
    parser.add_argument("--sine-cycles", type=int, default=None,
                        help=f"반복 주기 수 (생략하면 물어본다, 기본 {SINE_CYCLES})")
    parser.add_argument("--sine-lead", type=float, default=SINE_LEAD_S,
                        help=f"파형 시작점까지의 진입/복귀 램프 [s] (기본 {SINE_LEAD_S:g})")
    parser.add_argument("--host", default="127.0.0.1",
                        help="pp_controller가 실행 중인 주소 (기본: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=2293,
                        help="RefTcpServer.port와 같은 포트 (기본: 2293)")
    parser.add_argument("--connect-timeout", type=float, default=30.0,
                        help="controller 접속을 재시도할 최대 시간 [s] (기본: 30)")
    parser.add_argument("--dwell", type=float, default=5.0,
                        help="각 스테이지 유지 시간 [s] (기본: 5)")
    parser.add_argument("--send-hz", type=float, default=10.0,
                        help="같은 목표 반복 전송률 [Hz] (기본: 10)")
    parser.add_argument("--dry-run", action="store_true",
                        help="TCP를 열지 않고 모든 스테이지와 12채널 값을 출력")
    parser.add_argument("--yes", action="store_true",
                        help="실기 시작 전 RUN 확인 문구를 생략")
    args = parser.parse_args()
    if args.dwell <= 0.0:
        parser.error("--dwell은 0보다 커야 한다")
    if args.send_hz <= 0.0:
        parser.error("--send-hz는 0보다 커야 한다")
    if args.connect_timeout <= 0.0:
        parser.error("--connect-timeout은 0보다 커야 한다")
    if not 1 <= args.port <= 65535:
        parser.error("--port는 1..65535여야 한다")
    if args.sine_amp is not None and not 0.0 < args.sine_amp <= 90.0:
        parser.error("--sine-amp 은 0 보다 크고 90 이하여야 한다 "
                     "(음압은 대기압 아래 101 kPa 밖에 못 간다)")
    if args.sine_period is not None and args.sine_period <= 0.5:
        parser.error("--sine-period 는 0.5 s 보다 커야 한다")
    if args.sine_cycles is not None and args.sine_cycles < 1:
        parser.error("--sine-cycles 는 1 이상이어야 한다")
    if args.sine_lead < 0.0:
        parser.error("--sine-lead 는 0 이상이어야 한다")
    if args.pos_max is not None and not ATM_KPA < args.pos_max <= 250.0:
        parser.error(f"--pos-max 는 {ATM_KPA:g} 보다 크고 250 이하여야 한다")
    if args.neg_min is not None and not 0.0 < args.neg_min < ATM_KPA:
        parser.error(f"--neg-min 은 0 보다 크고 {ATM_KPA:g} 보다 작아야 한다")
    if args.axes:
        try:
            parse_axes(args.axes)
        except ValueError as exc:
            parser.error(f"--axes: {exc}")
    return args


def run_sine_plan(args, group: Sequence[int], mode: str,
                  amp: float, period: float, cycles: int,
                  c_pos: float, c_neg: float) -> int:
    """사인파 계획을 출력하고 (dry-run 이 아니면) 전송한다."""

    total = args.sine_lead * 2 + cycles * period
    pk_pos, pk_neg = sine_targets(mode, amp, 0.5, c_pos, c_neg)   # 위상 0.5 = 정점
    st_pos, st_neg = sine_targets(mode, amp, 0.0, c_pos, c_neg)
    print("\n=== 사인파 계획 ===")
    print(f"종류: {SWEEP_LABEL[mode]}")
    print(f"선택한 축: {_axis_label(group)} "
          f"(양압 gid {', '.join(str(a) for a in group)} / "
          f"음압 gid {', '.join(str(a + NUM_AXES) for a in group)}) — 동시 지령")
    print(f"나머지 축: 계획 내내 대기압 {ATM_KPA:g} kPa abs 유지")
    print(f"중앙값 P+ {c_pos:.1f} / P− {c_neg:.1f} kPa abs (여기서 ±{amp/2:g} 로 흔든다)")
    print(f"진폭 {amp:g} kPa(p-p) · 주기 {period:g} s × {cycles} 회 · "
          f"진입/복귀 램프 {args.sine_lead:g} s → 총 {total / 60.0:.1f} 분")
    print(f"P+ {st_pos:.1f} ↔ {pk_pos:.1f} kPa abs   |   P− {st_neg:.1f} ↔ {pk_neg:.1f} kPa abs")
    print(f"전송 {args.send_hz:g} Hz, 파형은 (1−cos)/2 라 시작·끝 기울기가 0 이다")
    if args.dry_run:
        print(f"\n{'위상':>6} {'P+':>9} {'P−':>9}")
        for i in range(13):
            ph = i / 12.0
            pp, nn = sine_targets(mode, amp, ph, c_pos, c_neg)
            print(f"{ph:6.2f} {pp:9.2f} {nn:9.2f}")
        return 0

    print(f"\n주의: {_axis_label(group)}에 동시 지령한다. 채널 과압 세이프티는 190 kPa 다.")
    print("TCP 단절 시 마지막 압력 목표가 컨트롤러에 남을 수 있다.")
    if not args.yes:
        if input("계획과 비상정지를 확인했으면 RUN을 입력: ").strip() != "RUN":
            print("취소했다.")
            return 2

    conn: socket.socket | None = None
    try:
        conn = _connect(args)
        _send_atmosphere(conn)
        run_sine(conn, group, mode, amp, period, cycles, args.send_hz,
                 c_pos, c_neg, args.sine_lead)
        print("\n[완료] 사인파를 마쳤다.")
        return 0
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C를 받았다.")
        return 130
    except (ConnectionError, OSError) as exc:
        print(f"\n[위험] TCP 통신 실패: {exc}", file=sys.stderr)
        print("       마지막 목표가 유지될 수 있다. 펌프/제어기를 즉시 정지할 것.",
              file=sys.stderr)
        return 1
    finally:
        _send_atmosphere(conn)
        if conn is not None:
            conn.close()


def execute_stages(args, stages: list[Stage]) -> int:
    """계획된 Stage 열을 순서대로 전송한다 (계단 스윕·튜닝 공용)."""

    conn: socket.socket | None = None
    try:
        conn = _connect(args)
        # 접속 직후에는 계획 시작 전에도 먼저 안전한 대기압 패킷을 보낸다.
        _send_atmosphere(conn)

        for index, stage in enumerate(stages, 1):
            payload = encode_refs(stage.refs_kpa)
            step = "-" if stage.step_kpa is None else f"{stage.step_kpa:g}"
            print(f"[{index:03d}/{len(stages):03d}] {_axis_label(stage.group):9s} "
                  f"{stage.phase:24s} step={step:>2s} "
                  f"target={_fmt_target(stage)} kPa abs")
            print(f"             {_fmt_refs(stage.refs_kpa)}", flush=True)
            _send_for_dwell(conn, payload, args.dwell, args.send_hz)

        print("\n[완료] 전체 계획을 마쳤다.")
        return 0
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C를 받았다.")
        return 130
    except (ConnectionError, OSError) as exc:
        print(f"\n[위험] TCP 통신 실패: {exc}", file=sys.stderr)
        print("       마지막 목표가 유지될 수 있다. 펌프/제어기를 즉시 정지할 것.",
              file=sys.stderr)
        return 1
    finally:
        _send_atmosphere(conn)
        if conn is not None:
            conn.close()


def run_sine_freq(args, group, amp: float, c_pos: float, c_neg: float) -> int:
    """진폭·동작점을 고정하고 주기만 줄여 간다 — 구간을 **끊지 않고 이어 붙인다**.

    같은 압력 범위를 점점 빠르게 왕복시키므로, 어느 기울기에서 추종이 무너지는지가
    한 번에 보인다. 진폭이 고정이라 요구 기울기는 주기에 반비례한다:

        max|dP/dt| = pi * 진폭 / 주기

    구간 사이에 대기압을 넣지 않는다. 파형이 (1−cos)/2 라 모든 구간이
    `중앙값 − 진폭/2` 에서 시작해 같은 점에서 끝나고 그 지점의 기울기가 0 이므로,
    값도 기울기도 이어진다 — 경계에서 계단이 생기지 않는다. 진입/복귀 램프는
    전체의 맨 앞과 맨 뒤에 한 번씩만 둔다.

    끊지 않으므로 앞 구간의 적분·열 상태가 다음 구간으로 넘어간다. 그게 실제
    운전에 가까운 조건이고, 대신 구간 경계는 로그에서 눈으로 못 찾으니 아래에서
    **각 구간의 시작·끝 시각을 출력**한다 (진입 램프 시작을 t=0 으로 본 값).

    마지막에 첫 주기를 한 번 더 돈다. 데드존이 운전 중에 밀려서(9분에 3~5 %p)
    앞뒤를 그냥 비교하면 표류와 주파수 영향이 섞인다. 처음과 끝의 같은 주기끼리
    비교하면 그 표류분만 따로 떼어 볼 수 있다.
    """
    periods = list(args.freq_periods_list)
    labels  = [f"{i+1}) 주기 {p:g} s" for i, p in enumerate(periods)]
    if not args.no_freq_recheck:
        periods.append(periods[0])
        labels.append(f"{len(periods)}) 주기 {periods[0]:g} s 재확인(표류)")

    segs = [(lab, per, freq_cycles(per)) for lab, per in zip(labels, periods)]
    wave = sum(c * per for _, per, c in segs)
    total = args.sine_lead + wave + args.sine_lead

    print("\n=== 주기 스캔 계획 ===")
    print(f"선택한 축: {_axis_label(group)}")
    print(f"진폭 {amp:g} kPa(p-p) 고정 · 중앙값 P+ {c_pos:.1f} / P− {c_neg:.1f} kPa abs")
    print(f"P+ {c_pos-amp/2:.1f} ↔ {c_pos+amp/2:.1f}   |   "
          f"P− {c_neg-amp/2:.1f} ↔ {c_neg+amp/2:.1f} kPa abs (전 구간 동일)")
    print(f"\n{'구간':>28} {'주기':>7} {'반복':>5} {'최대기울기':>11} {'로그 구간':>16}")
    t = args.sine_lead
    bounds = []
    for lab, per, cyc in segs:
        rate = math.pi * amp / per
        t1 = t + cyc * per
        bounds.append((lab, t, t1))
        print(f"{lab:>28} {per:6.0f}s {cyc:5d} {rate:8.2f} kPa/s "
              f"{t:7.0f}~{t1:<8.0f}s")
        t = t1
    print(f"\n진입 램프 {args.sine_lead:g} s + 파형 {wave:.0f} s + 복귀 램프 "
          f"{args.sine_lead:g} s → 총 {total/60.0:.1f} 분")
    print("구간 사이에 대기압을 넣지 않는다 — 값·기울기가 그대로 이어진다.")
    if args.dry_run:
        return 0

    print(f"\n주의: {_axis_label(group)}에 동시 지령한다. 채널 과압 세이프티는 190 kPa 다.")
    print("빠른 구간은 밸브가 못 따라가 진폭이 줄어든다 — 그것을 재는 것이 목적이다.")
    if not args.yes:
        if input("계획과 비상정지를 확인했으면 RUN을 입력: ").strip() != "RUN":
            print("취소했다.")
            return 2

    conn = None
    try:
        conn = _connect(args)
        _send_atmosphere(conn)
        print(f"\n{'='*66}\n  진입 램프 {args.sine_lead:g} s  "
              f"대기압 → P+ {c_pos-amp/2:.1f} / P− {c_neg-amp/2:.1f}\n{'='*66}", flush=True)
        run_sine_edge(conn, group, "sine-diff", amp, args.send_hz,
                      c_pos, c_neg, args.sine_lead, entering=True)
        for k, ((lab, per, cyc), (_, t0b, t1b)) in enumerate(zip(segs, bounds), 1):
            rate = math.pi * amp / per
            print(f"\n{'='*66}\n  [{k}/{len(segs)}] {lab}  × {cyc}회  "
                  f"최대기울기 {rate:.2f} kPa/s   로그 {t0b:.0f}~{t1b:.0f} s\n{'='*66}",
                  flush=True)
            # lead=0 → 램프 없이 파형만. 구간이 바로 이어진다.
            run_sine(conn, group, "sine-diff", amp, per, cyc, args.send_hz,
                     c_pos, c_neg, lead=0.0)
        print(f"\n{'='*66}\n  복귀 램프 {args.sine_lead:g} s → 대기압\n{'='*66}", flush=True)
        run_sine_edge(conn, group, "sine-diff", amp, args.send_hz,
                      c_pos, c_neg, args.sine_lead, entering=False)
        print("\n[완료] 주기 스캔을 마쳤다.")
        print("구간 경계 (진입 램프 시작 = t 0):")
        for lab, t0b, t1b in bounds:
            print(f"  {lab:>28}  {t0b:7.1f} ~ {t1b:7.1f} s")
        return 0
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C를 받았다.")
        return 130
    except (ConnectionError, OSError) as exc:
        print(f"\n[위험] TCP 통신 실패: {exc}", file=sys.stderr)
        return 1
    finally:
        _send_atmosphere(conn)
        if conn is not None:
            conn.close()


def run_sine_scan(args, group) -> int:
    """SCAN_SEGMENTS 를 순서대로 이어서 보낸다 — 한 로그에 전 구간이 들어간다.

    구간 사이에 --scan-gap 초 동안 대기압으로 내려 둔다. 그래야 로그에서 구간 경계를
    눈으로 찾기 쉽고, 앞 구간의 적분·열 상태가 다음 구간으로 새지 않는다.
    """
    total = sum(args.sine_lead * 2 + c * per for _, _, per, c, _, _ in SCAN_SEGMENTS)
    total += args.scan_gap * len(SCAN_SEGMENTS)
    print("\n=== 사인 스캔 계획 ===")
    print(f"선택한 축: {_axis_label(group)}")
    print(f"{'구간':>22} {'진폭':>6} {'주기':>6} {'반복':>4} {'P+ 범위':>16} {'P− 범위':>16} {'길이':>6}")
    for name, amp, per, cyc, cp, cn in SCAN_SEGMENTS:
        dur = args.sine_lead * 2 + cyc * per + args.scan_gap
        print(f"{name:>22} {amp:6.0f} {per:6.0f} {cyc:4d} "
              f"{cp-amp/2:7.1f}~{cp+amp/2:<7.1f} {cn-amp/2:7.1f}~{cn+amp/2:<7.1f} {dur:5.0f}s")
    print(f"구간 사이 대기압 {args.scan_gap:g} s → 총 {total/60.0:.1f} 분")
    if args.dry_run:
        return 0
    print(f"\n주의: {_axis_label(group)}에 동시 지령한다. 채널 과압 세이프티는 190 kPa 다.")
    if not args.yes:
        if input("계획과 비상정지를 확인했으면 RUN을 입력: ").strip() != "RUN":
            print("취소했다.")
            return 2
    conn = None
    try:
        conn = _connect(args)
        _send_atmosphere(conn)
        for k, (name, amp, per, cyc, cp, cn) in enumerate(SCAN_SEGMENTS, 1):
            print(f"\n{'='*66}\n  [{k}/{len(SCAN_SEGMENTS)}] {name}  "
                  f"P+ {cp-amp/2:.0f}~{cp+amp/2:.0f}  P− {cn-amp/2:.0f}~{cn+amp/2:.0f}\n{'='*66}",
                  flush=True)
            run_sine(conn, group, "sine-diff", amp, per, cyc, args.send_hz,
                     cp, cn, args.sine_lead)
            print(f"  구간 사이 대기압 {args.scan_gap:g} s", flush=True)
            _send_for_dwell(conn, encode_refs((ATM_KPA,) * (2 * NUM_AXES)),
                            args.scan_gap, args.send_hz)
        print("\n[완료] 사인 스캔을 마쳤다.")
        return 0
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C를 받았다.")
        return 130
    except (ConnectionError, OSError) as exc:
        print(f"\n[위험] TCP 통신 실패: {exc}", file=sys.stderr)
        return 1
    finally:
        _send_atmosphere(conn)
        if conn is not None:
            conn.close()


def main() -> int:
    args = parse_args()
    try:
        group = parse_axes(args.axes) if args.axes else prompt_axes()
    except ValueError as exc:
        print(f"[중단] {exc}", file=sys.stderr)
        return 2
    sweep = args.sweep or prompt_sweep()

    if sweep == "sine-scan":
        return run_sine_scan(args, group)

    # ── 주기 스캔: 진폭·동작점 고정, 주기만 줄인다 ─────────────────────────
    if sweep == "sine-freq":
        amp = args.sine_amp
        if amp is None:
            print("\n진폭 = 파형의 peak-to-peak. 전 구간 이 값으로 고정한다.")
            amp = prompt_endpoint("진폭", SINE_AMP_KPA, 0.0, 90.0, unit="kPa",
                                  note="양압 최대가 과압 세이프티 190 kPa 를 넘지 않게 할 것.")
        if args.freq_periods:
            try:
                periods = [float(v) for v in args.freq_periods.replace(" ", "").split(",") if v]
            except ValueError:
                print("[중단] --freq-periods 는 쉼표로 구분한 숫자다.", file=sys.stderr)
                return 2
        else:
            periods = list(FREQ_PERIODS_S)
        if not periods or any(p <= 0.5 for p in periods):
            print("[중단] 주기는 0.5 s 보다 커야 한다.", file=sys.stderr)
            return 2
        args.freq_periods_list = periods

        c_pos, c_neg = sine_center("sine-diff", amp, args.sine_center_pos, args.sine_center_neg)
        lo_p, hi_p = c_pos - amp / 2.0, c_pos + amp / 2.0
        lo_n, hi_n = c_neg - amp / 2.0, c_neg + amp / 2.0
        if lo_p < ATM_KPA - 0.01:
            print(f"  [경고] 양압 최저 {lo_p:.1f} kPa 는 대기압 아래다 — 중앙값을 "
                  f"{ATM_KPA + amp / 2.0:.1f} 이상으로 올려라.")
        if hi_p > 190.0:
            print(f"  [경고] 양압 최고 {hi_p:.1f} kPa 는 과압 세이프티 190 위다.")
        if hi_n > ATM_KPA + 0.01:
            print(f"  [경고] 음압 최고 {hi_n:.1f} kPa 는 대기압 위다.")
        if lo_n < 35.0:
            print(f"  [경고] 음압 최저 {lo_n:.1f} kPa 는 진공 레일에 가깝다.")
        return run_sine_freq(args, group, amp, c_pos, c_neg)

    # ── 튜닝 계단: 동작점마다 양방향 소계단 ────────────────────────────────
    if sweep in TUNE_KINDS:
        positive = sweep == "tune-pos"
        if args.tune_centers:
            centers = [float(v) for v in args.tune_centers.replace(" ", "").split(",") if v]
        else:
            centers = list(TUNE_CENTERS_POS if positive else TUNE_CENTERS_NEG)
        bad = [c for c in centers
               if (positive and not ATM_KPA < c <= 185.0)
               or (not positive and not 20.0 <= c < ATM_KPA)]
        if bad:
            print(f"[중단] 동작점이 범위 밖이다: {bad}", file=sys.stderr)
            return 2
        stages = build_tune_stages(group, positive, centers)
        print("\n=== 튜닝 계단 계획 ===")
        print(f"종류: {SWEEP_LABEL[sweep]}")
        print(f"선택한 축: {_axis_label(group)}")
        print(f"동작점 {', '.join('%g' % c for c in centers)} kPa abs "
              f"× 계단 ±{'/±'.join('%g' % v for v in TUNE_STEPS_KPA)} kPa × 양방향")
        print(f"단계 {len(stages)}개 × 유지 {args.dwell:g} s "
              f"→ 총 {len(stages) * args.dwell / 60.0:.1f} 분")
        print("각 계단에서 오버슛·정착시간·정상상태오차를 뽑아 게인을 정한다.")
        if args.dry_run:
            for k, st in enumerate(stages, 1):
                print(f"  {k:03d} {st.phase:26s} target={st.target_kpa:7.2f}")
            return 0
        print(f"\n주의: {_axis_label(group)}에 동시 지령한다. "
              f"채널 과압 세이프티는 190 kPa 다.")
        if not args.yes:
            if input("계획과 비상정지를 확인했으면 RUN을 입력: ").strip() != "RUN":
                print("취소했다.")
                return 2
        return execute_stages(args, stages)

    # ── 사인파: 진폭·주기·반복 수·중앙값을 받고 전용 경로로 간다 ───────────
    if sweep in SINE_KINDS:
        amp, period, cycles = args.sine_amp, args.sine_period, args.sine_cycles
        if amp is None:
            print("\n진폭 = 파형의 peak-to-peak. 40 이면 중앙값 ±20 으로 흔든다.")
            amp = prompt_endpoint("진폭", SINE_AMP_KPA, 0.0, 90.0, unit="kPa",
                                  note="양압 최대가 과압 세이프티 190 kPa 를 넘지 않게 할 것.")
        if period is None:
            period = prompt_endpoint("한 주기", SINE_PERIOD_S, 0.5, 600.0, unit="s")
        if cycles is None:
            cycles = int(prompt_endpoint("반복 주기 수", float(SINE_CYCLES), 0.0, 1000.0,
                                         unit="회"))
        # 중앙값 — 안 주면 대기압에서 출발하는 예전 파형이 된다.
        c_pos, c_neg = sine_center(sweep, amp, args.sine_center_pos, args.sine_center_neg)
        if args.sine_center_pos is None and args.sine_center_neg is None and not args.yes:
            print(f"\n중앙값(파형의 한가운데). 기본은 대기압에서 출발하는 값이다: "
                  f"P+ {c_pos:.1f} / P− {c_neg:.1f} kPa abs")
            print("  레일을 200 으로 올렸으면 양압 중앙값을 140~150 쯤으로 올려 "
                  "챔버가 대기압에 붙지 않게 하는 편이 추종이 낫다.")
            if input("  중앙값을 직접 넣을까? [y/N] ").strip().lower() in ("y", "yes"):
                c_pos = prompt_endpoint("양압 중앙값", c_pos, ATM_KPA, 190.0, unit="kPa")
                c_neg = prompt_endpoint("음압 중앙값", c_neg, 5.0, ATM_KPA, unit="kPa")

        # 도달 가능 범위 검사 — 양압 챔버는 대기압 아래로, 음압은 위로 못 간다.
        lo_p, hi_p = c_pos - amp / 2.0, c_pos + amp / 2.0
        lo_n, hi_n = c_neg - amp / 2.0, c_neg + amp / 2.0
        if sweep in ("sine-pos", "sine-same", "sine-diff"):
            if lo_p < ATM_KPA - 0.01:
                print(f"  [경고] 양압 최저 {lo_p:.1f} kPa 는 대기압 아래다 — 양압 챔버는 "
                      f"대기압까지만 내려간다. 중앙값을 {ATM_KPA + amp / 2.0:.1f} "
                      f"이상으로 올려라.")
            if hi_p > 190.0:
                print(f"  [경고] 양압 최고 {hi_p:.1f} kPa 는 과압 세이프티 190 위다.")
        if sweep in ("sine-neg", "sine-same", "sine-diff"):
            if hi_n > ATM_KPA + 0.01:
                print(f"  [경고] 음압 최고 {hi_n:.1f} kPa 는 대기압 위다 — 음압 챔버는 "
                      f"대기압까지만 올라간다.")
            if lo_n < 35.0:
                print(f"  [경고] 음압 최저 {lo_n:.1f} kPa 는 진공 레일에 가깝다 "
                      "— 도달하지 못할 수 있다.")
        return run_sine_plan(args, group, sweep, amp, period, cycles, c_pos, c_neg)

    # ── 계단 스윕 ───────────────────────────────────────────────────────────
    turn = "최대" if args.one_way else "정점"
    pos_max, neg_min = args.pos_max, args.neg_min
    if sweep in ("pos", "both", "all") and pos_max is None:
        print(f"\n양압 {turn}을 고른다. 여기까지 올라갔다가 같은 계단으로 내려온다.")
        pos_max = prompt_endpoint(f"양압 {turn}", POS_MAX_KPA, ATM_KPA, 250.0, unit="kPa",
                                  note="채널 과압 세이프티는 190 kPa 다.")
    if sweep in ("neg", "both", "all") and neg_min is None:
        print(f"\n음압 {turn}을 고른다. 여기까지 내려갔다가 같은 계단으로 올라온다.")
        neg_min = prompt_endpoint(f"음압 {turn}", NEG_MIN_KPA, 0.0, ATM_KPA, unit="kPa",
                                  note="진공 레일(≈30)보다 낮으면 도달하지 못한다.")
    pos_max = POS_MAX_KPA if pos_max is None else pos_max
    neg_min = NEG_MIN_KPA if neg_min is None else neg_min

    stages = build_stages(group, sweep, pos_max, neg_min, args.one_way)
    print_plan(stages, args.dwell, verbose=args.dry_run, group=group, sweep=sweep,
               pos_max=pos_max, neg_min=neg_min, one_way=args.one_way)
    if args.dry_run:
        return 0

    print(f"\n주의: {_axis_label(group)}에 동시 지령한다 ({SWEEP_LABEL[sweep]}). "
          f"양압 {turn} {pos_max:g} / 음압 {turn} {neg_min:g} kPa abs, "
          f"채널 과압 세이프티는 190 kPa 다.")
    print("TCP 단절 시 마지막 압력 목표가 컨트롤러에 남을 수 있다.")
    if not args.yes:
        if input("계획과 비상정지를 확인했으면 RUN을 입력: ").strip() != "RUN":
            print("취소했다.")
            return 2
    return execute_stages(args, stages)


if __name__ == "__main__":
    raise SystemExit(main())
