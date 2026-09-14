#!/usr/bin/env python3
"""valve_deadzone.py — 밸브가 열리기 시작하는 지령(데드존)을 **차압별로 실측**한다.

밸브 모델(13-parameter)을 쓰지 않는다. 채널 압력을 원하는 차압이 되는 곳에 세워 두고
밸브를 조금씩 열어, 압력이 **유효하게** 움직이기 시작하는 지령을 그 차압의 데드존으로
기록한다. 결과는 컨트롤러가 `channel_config.chN.deadzone.*` 로 그대로 읽는 표다.

왜 차압별인가
-------------
데드존은 상수가 아니다. 상류압이 스풀을 여는 방향으로 밀기 때문에 차압이 크면 더 낮은
지령에서 열린다. 실측(20260908_161244)에서 챔버압 10 kPa 변화가 임계를 0.5 %p 옮겼다 —
사인파 한 주기 안에서 데드존이 계속 움직인다는 뜻이다. 상수 하나로 보상하면 반주기는
덜 열고 반주기는 더 열게 되므로, 표로 재서 차압으로 보간해야 한다.

무엇을 재나
-----------
밸브별로 **차압(상류−하류)** 을 정의한다. 부호가 항상 양수가 되는 방향이다.

    양압 micro : 레일 → 챔버      dp = P_rail_pos − P_chamber
    양압 atm   : 챔버 → 대기      dp = P_chamber  − P_atm
    음압 micro : 챔버 → 진공레일  dp = P_chamber  − P_rail_neg
    음압 atm   : 대기 → 챔버      dp = P_atm      − P_chamber

원하는 dp 에 맞는 챔버 목표압을 거꾸로 계산해 거기에 세운다. 레일 180 / 대기 101.3 이면

    양압 micro, dp=20/40/60 → 챔버 160/140/120  (레일에서 dp 만큼 아래)
    양압 atm,   dp=20/40/60 → 챔버 121/141/161  (대기에서 dp 만큼 위)
    음압 micro, dp=20/40/60 → 챔버 50/70/90     (레일 30 에서 dp 만큼 위)
    음압 atm,   dp=20/40/60 → 챔버 81/61/41     (대기에서 dp 만큼 아래)

측정 절차 (채널마다 독립, 선택한 채널을 **동시에** 진행한다)
--------------------------------------------------------
  PREPOS   목표압까지 몰아간다 (--drive-pct 로 활짝 열어 bang-bang)
  SETTLE   전 밸브를 닫고 압력이 멈출 때까지 기다린다
  BASELINE 닫힌 상태의 자연 드리프트(누설)를 --baseline 초 동안 잰다.
           이것이 귀무가설이다 — 누설보다 확실히 빨라야 "열렸다" 고 본다.
  COARSE   --u-coarse (%p) 씩 올리며 굵게 훑어 구간을 잡는다
  RESETTLE 닫고 다시 안정시킨다 (굵은 훑기로 압력이 움직였으므로)
  FINE     굵게 잡은 구간의 한 칸 아래에서 --u-step 씩 곱게 올린다.
           변화율이 (드리프트 + --rate-thresh) 를 --confirm 스텝 연속 넘으면,
           그 **첫 스텝의 지령** 을 데드존으로 확정한다.

전제
----
  · can_bridge 가 돌고 있어야 한다
  · pp_controller 는 **꺼져 있어야 한다** — board/pwm_cmd 발행자가 둘이면 명령이
    번갈아 나간다. 레일은 이 스크립트가 직접 잡는다 (LinePID 와 같은 밸브·같은 부호).
  · 펌프는 **켜져 있어야 한다** (소프트웨어 제어가 없어 작업자가 켠다)

사용
----
    # 전 채널, 차압 20/40/60 (기본)
    python3 valve_deadzone.py

    # 1·2축만, 양압만, 차압 10 kPa 간격
    python3 valve_deadzone.py --axes 1,2 --side pos --dp 10,20,30,40,50,60

    # 재고 나서 yaml 에 바로 반영
    python3 valve_deadzone.py --write
"""
from __future__ import annotations

import argparse
import datetime
import math
import re
import os
import shutil
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray

NAMESPACE = "/pack2"
PWM_TOTAL = 25 * 3
FULL = 4095
PCT_TO_PWM = 40.95

V_MICRO, V_ATM, V_MACRO = 0, 1, 2      # 보드 슬롯 v1/v2/v3
ROLE = {V_MICRO: "micro", V_ATM: "atm"}

N_AXES = 6
POS_BOARD0, NEG_BOARD0 = 5, 11         # 축 i → 양압 board 5+i, 음압 board 11+i
RAIL_POS_BOARD, RAIL_NEG_BOARD = 1, 2
RAIL_POS_SLOT, RAIL_NEG_SLOT = 0, 3    # board1 v1, board2 v1 (LinePID 와 동일)

HERE = os.path.dirname(os.path.abspath(__file__))
# valve_deadzone 매핑의 스칼라 기본값. yaml 에 이미 있으면 그 값을 보존한다.
# valve_deadzone 매핑의 스칼라 기본값. yaml 에 이미 있으면 그 값을 보존한다.
# **이 목록에 없는 키는 --write 때 사라진다** — 20260911 에 park_enable/park_below_pct 가
# 그렇게 날아가 파킹이 조용히 꺼졌다. 컨트롤러에 스칼라를 추가하면 여기도 추가할 것.
DEFAULT_DZ_OPT = {"enable": True, "margin_pct": 3.0,
                  "micro_pct": 0.0, "atm_pct": 0.0, "macro_pct": 0.0,
                  "park_enable": False, "park_below_pct": 5.0}

DEFAULT_YAML = os.path.join(os.path.dirname(HERE), "config", "powerpack_config.yaml")
RESULT_ROOT = os.path.expanduser("~/result")


# ════════════════════════════════════════════════════════════════════════════
# 하드웨어 접점
# ════════════════════════════════════════════════════════════════════════════
class Rig(Node):
    """board/sensors 를 kPa 로 읽고 board/pwm_cmd 를 % 로 쓴다. 레일도 여기서 잡는다."""

    def __init__(self, cfg: dict, rail_pos_sp: float, rail_neg_sp: float,
                 p_max: float, rail_ramp: float, lead: float,
                 idle_open_pct: float) -> None:
        super().__init__("valve_deadzone")
        self._raw: list[int] | None = None
        self._n_msg = 0
        self.offs, self.gains, self.atm = cfg["offs"], cfg["gains"], cfg["atm"]
        self.rail_pos_sp, self.rail_neg_sp = rail_pos_sp, rail_neg_sp
        self.p_max = p_max
        # 레일 PID 는 컨트롤러의 LinePID 와 **같은 게인·같은 구조**를 쓴다. 여기서
        # 다르게 잡으면 측정할 때의 레일 거동이 실제 운전과 달라진다.
        self.gpos, self.gneg = cfg["line_pos"], cfg["line_neg"]
        self._ipos = self._ineg = 0.0
        self._epos = self._eneg = None
        self._t_rail = None
        # 목표를 한 번에 주지 않고 **램프**로 끌고 간다. 대기압에서 180 을 바로 주면
        # 오차 79 kPa 로 적분이 상한까지 차고, 목표를 지나는 순간 u=100 (방출 완전
        # 닫힘) 에서 출발해 풀리는 데만 십수 초가 걸린다 — 그 사이 펌프가 계속 채운다
        # (실측 20260909_144612: 179.8 → 196 kPa). 오차를 작게 유지하면 적분이 애초에
        # 넘치지 않는다.
        self.rail_ramp = rail_ramp
        self.lead = lead
        self._sp_pos_now: float | None = None
        self._sp_neg_now: float | None = None
        # 지금 재고 있는 쪽. 쉬는 쪽 레일은 목표 없이 밸브를 활짝 연다 (publish 참조).
        self.priority: str | None = None      # 'pos' | 'neg' | None(양쪽 다)
        self.idle_open_pct = idle_open_pct
        self._last_pos_pwm = 0.0
        self._last_neg_pwm = 0.0
        self.pct = [0.0] * PWM_TOTAL          # 채널 밸브 명령 [%] — 상태 머신이 쓴다
        self.create_subscription(UInt16MultiArray, f"{NAMESPACE}/board/sensors",
                                 self._on_sensors, 20)
        self._pub = self.create_publisher(UInt16MultiArray,
                                          f"{NAMESPACE}/board/pwm_cmd", 5)

    def _on_sensors(self, msg: UInt16MultiArray) -> None:
        self._raw = list(msg.data)
        self._n_msg += 1

    @property
    def n_msg(self) -> int:
        return self._n_msg

    def kpa(self, board: int) -> float | None:
        """보드의 절대압 [kPa]. 컨트롤러와 같은 식: (raw_mV − offset)·gain + atm."""
        if self._raw is None or len(self._raw) < board:
            return None
        raw = self._raw[board - 1]
        if raw == 0:                       # 그 보드 프레임을 아직 못 받았다
            return None
        return (float(raw) - self.offs[board]) * self.gains[board] + self.atm

    def rival_publishers(self) -> list[str]:
        """board/pwm_cmd 를 발행하는 다른 노드 이름. 이름으로 봐야 누군지 알 수 있다."""
        mine = self.get_name()
        out = []
        for info in self.get_publishers_info_by_topic(f"{NAMESPACE}/board/pwm_cmd"):
            if info.node_name != mine:
                out.append(f"{info.node_namespace.rstrip('/')}/{info.node_name}")
        return out

    # ── 매 틱: 레일 조절 + 안전 + 발행 ──────────────────────────────────────
    def publish(self) -> None:
        data = [0] * PWM_TOTAL

        # ── 레일 ────────────────────────────────────────────────────────
        # 두 밸브는 펌프 닫힌 회로의 **유일한 출구/입구**다 (흡입=음압 라인,
        # 토출=양압 라인). 둘 다 정극성(PWM 그대로 개도)이고, LinePID 는 지령을
        # **(100 − u) 로 반전해서** 내보낸다:
        #
        #   board1 v1 : 양압 라인 → 대기 방출.  레일을 **올리려면 닫는다**.
        #   board2 v1 : 대기 → 음압 라인 유입.  레일을 **내리려면 닫는다**.
        #
        # 즉 목표에서 멀수록 u 가 커지고 밸브는 더 닫힌다. 목표에 닿으면 u=0,
        # 밸브가 활짝 열려 펌프 토출을 그대로 흘려보내며 균형을 잡는다.
        # 컨트롤러와 같은 게인·같은 anti-windup 을 쓴다.
        now = time.monotonic()
        dt = 0.0 if self._t_rail is None else min(0.2, now - self._t_rail)
        self._t_rail = now
        p_pos, p_neg = self.kpa(RAIL_POS_BOARD), self.kpa(RAIL_NEG_BOARD)

        lead = self.lead

        def ramp_sp(now_sp, target, meas, dt):
            """내부 목표를 --rail-ramp [kPa/s] 로 target 쪽으로 옮긴다.

            압력이 못 따라오면 목표도 앞서가지 않는다 (--rail-lead 안에 묶는다).
            안 묶으면 레일이 막혔을 때 오차가 무한히 벌어져 적분이 다시 넘친다.
            """
            if now_sp is None:
                return meas                      # 지금 압력에서 출발
            step = self.rail_ramp * dt
            if now_sp < target:
                return min(target, now_sp + step, meas + lead)
            return max(target, now_sp - step, meas - lead)

        def rail_pwm(err, gains, integ, prev, dt):
            """LinePID 와 같은 식 + **조건부 적분**. 반환 (pwm, 새 integ, 새 prev).

            컨트롤러 원본은 back-calculation 만 쓰는데, 목표가 높으면 (180) 올라가는
            동안 적분이 u=100 을 훨씬 넘게 차 버린다. back-calculation 은 u 가 한계를
            **넘을 때만** 되돌리므로, 목표를 지나 err 가 음수가 돼도 쌓인 적분이
            ki·|err| 속도로만 풀린다 — 그 사이 방출 밸브는 닫힌 채고 레일은 계속
            올라간다. 실측 20260909_144612 에서 179.8 → 196 kPa 이 이것이다.

            그래서 이미 한계에 붙은 방향으로는 쌓지 않는다 (PressureCtrl 과 같은 규칙).
            """
            deriv = 0.0 if (prev is None or dt <= 0.0) else (err - prev) / dt
            base = gains["kp"] * err + gains["kd"] * deriv
            u_test = base + gains["ki"] * integ
            sat_hi = (u_test >= 100.0) and (err > 0.0)
            sat_lo = (u_test <= 0.0) and (err < 0.0)
            if not sat_hi and not sat_lo:
                integ += err * dt
            # 적분 자체도 출력 범위 안으로 묶는다 — 한 번 넘치면 풀리는 데만 수십 초다.
            if abs(gains["ki"]) > 1e-6:
                lim = 100.0 / abs(gains["ki"])
                integ = min(lim, max(-lim, integ))
            u_c = min(100.0, max(0.0, base + gains["ki"] * integ))
            return int(round((100.0 - u_c) * PCT_TO_PWM)), integ, err

        # ── 어느 레일을 잡을 것인가 ──────────────────────────────────────
        # 펌프는 **한 대**다. 흡입(음압 라인)과 토출(양압 라인)이 같은 기계의 양끝이라
        # 두 목표를 동시에 잡으면 서로를 잡아당긴다 — 음압을 15 로 유지하려고 유입을
        # 조이면 펌프가 빨아들일 질량이 없어 양압이 안 오른다.
        #
        # 그래서 **지금 재는 쪽만 목표를 준다.** 쉬는 쪽은 목표를 버리고 밸브를 활짝
        # 열어 활성 쪽에 최선의 조건을 만든다:
        #   양압 측정 중 → board2 v1 활짝 (대기 → 음압 라인).
        #                  흡입압 = 대기압 → 흡입 질량유량 최대, 압축비 최소.
        #   음압 측정 중 → board1 v1 활짝 (양압 라인 → 대기).
        #                  토출압 = 대기압 → 펌프가 가장 깊은 진공을 뽑는다.
        # --parallel-sides 로 양쪽을 동시에 재면 둘 다 목표를 준다 (그만큼 얕아진다).
        idle_pwm = int(self.idle_open_pct * PCT_TO_PWM)
        act_pos = (self.priority != "neg")
        act_neg = (self.priority != "pos")

        # 쉬는 쪽은 목표를 버리고 활짝 연다. 그 루프의 적분·목표는 리셋해 둔다 —
        # 나중에 활성으로 돌아올 때 낡은 상태를 들고 오면 안 된다.
        #
        # 센서를 못 읽는 경우도 **활짝 연다**. PWM 0 은 안전이 아니다 — 양압 방출이
        # 닫히면 펌프가 켜져 있는 한 레일이 무한정 올라간다 (브리지 워치독과 같은 규약).
        if not act_pos:
            data[RAIL_POS_SLOT] = idle_pwm
            self._ipos, self._epos, self._sp_pos_now = 0.0, None, None
        elif p_pos is None:
            data[RAIL_POS_SLOT] = FULL
        else:
            self._sp_pos_now = ramp_sp(self._sp_pos_now, self.rail_pos_sp, p_pos, dt)
            data[RAIL_POS_SLOT], self._ipos, self._epos = rail_pwm(
                self._sp_pos_now - p_pos, self.gpos, self._ipos, self._epos, dt)

        if not act_neg:
            data[RAIL_NEG_SLOT] = idle_pwm
            self._ineg, self._eneg, self._sp_neg_now = 0.0, None, None
        elif p_neg is None:
            data[RAIL_NEG_SLOT] = FULL
        else:
            self._sp_neg_now = ramp_sp(self._sp_neg_now, self.rail_neg_sp, p_neg, dt)
            data[RAIL_NEG_SLOT], self._ineg, self._eneg = rail_pwm(
                p_neg - self._sp_neg_now, self.gneg, self._ineg, self._eneg, dt)

        for i, v in enumerate(self.pct):
            if v > 0.0:
                data[i] = int(min(FULL, v * PCT_TO_PWM))

        self._last_pos_pwm = data[RAIL_POS_SLOT]
        self._last_neg_pwm = data[RAIL_NEG_SLOT]
        msg = UInt16MultiArray()
        msg.data = data
        self._pub.publish(msg)

    def rail_status(self) -> str:
        """레일이 제대로 잡히고 있는지 한 줄로. 개도가 0 인데 압력이 목표에 못 가면
        펌프 한계이고, 개도가 크게 열린 채면 아직 제어기가 닫는 중이다."""
        p_pos, p_neg = self.kpa(RAIL_POS_BOARD), self.kpa(RAIL_NEG_BOARD)
        def f(v):
            return "  ?  " if v is None else f"{v:5.1f}"
        tp = f"→{self.rail_pos_sp:.0f}" if self.priority != "neg" else " 쉼"
        tn = f"→{self.rail_neg_sp:.0f}" if self.priority != "pos" else " 쉼"
        return (f"레일 양압 {f(p_pos)}{tp} (방출 {self._last_pos_pwm/PCT_TO_PWM:3.0f}%) | "
                f"음압 {f(p_neg)}{tn} (유입 {self._last_neg_pwm/PCT_TO_PWM:3.0f}%)")

    def close_all_channels(self) -> None:
        for board in range(POS_BOARD0, NEG_BOARD0 + N_AXES):
            for slot in (V_MICRO, V_ATM, V_MACRO):
                self.pct[(board - 1) * 3 + slot] = 0.0

    def set_valve(self, board: int, slot: int, pct: float) -> None:
        self.pct[(board - 1) * 3 + slot] = max(0.0, min(100.0, pct))


def spin(rig: Rig, seconds: float, hz: float = 50.0) -> None:
    """seconds 동안 스핀하며 명령을 hz 로 재전송한다 (브리지 워치독 200 ms)."""
    end = time.monotonic() + seconds
    nxt = 0.0
    while time.monotonic() < end:
        rclpy.spin_once(rig, timeout_sec=0.005)
        now = time.monotonic()
        if now >= nxt:
            rig.publish()
            nxt = now + 1.0 / hz


# ════════════════════════════════════════════════════════════════════════════
# 측정 대상 하나 = 채널 × 밸브
# ════════════════════════════════════════════════════════════════════════════
class Probe:
    """채널 하나의 밸브 하나. 목표 차압과 압력이 움직이는 방향을 안다.

    부호 규약을 한 곳에 모아 둔다. dir = 이 밸브를 열었을 때 챔버압이 가는 방향:

        양압 micro (레일→챔버)    +1        음압 micro (챔버→진공레일)  −1
        양압 atm   (챔버→대기)    −1        음압 atm   (대기→챔버)      +1

    차압도 이 하나로 정리된다. ref = 이 밸브의 **챔버 반대편** 압력
    (micro 는 레일, atm 은 대기):

        dp = dir·(ref − P_chamber)          ← 언제나 양수 (상류 − 하류)
        sp = ref − dir·dp                   ← 원하는 dp 를 만드는 챔버 목표압
    """

    def __init__(self, gid: int, axis: int, is_pos: bool, slot: int, dp: float) -> None:
        self.gid, self.axis, self.is_pos, self.slot = gid, axis, is_pos, slot
        self.board = (POS_BOARD0 if is_pos else NEG_BOARD0) + axis
        self.dp_target = dp
        self.dir = +1.0 if (is_pos == (slot == V_MICRO)) else -1.0
        # 압력을 올리는 밸브 / 내리는 밸브 (사전 위치잡기에 쓴다)
        self.slot_up = V_MICRO if is_pos else V_ATM
        self.slot_dn = V_ATM if is_pos else V_MICRO
        # 측정 결과 — 같은 조건을 --repeats 회 반복해 모으고 중앙값으로 확정한다.
        # 한 번만 재면 그 회차의 판정 오차가 그대로 표에 들어간다. 중앙값은 이상치
        # 하나에 흔들리지 않고, 퍼짐(최대−최소)이 곧 그 점의 신뢰도가 된다.
        self.samples: list[tuple[float, float, float]] = []   # (지령, 차압, 챔버압)
        self.hit: tuple[float, float, float] | None = None    # 이번 회차 결과
        self.spread = math.nan      # 회차 간 퍼짐 (최대−최소) [%p]
        self.sd = math.nan          # 회차 간 표준편차 [%p]
        self.sp = math.nan          # 챔버 목표압
        self.ref = math.nan         # 반대편 압력 (측정 시점)
        self.u_coarse = math.nan    # 굵은 훑기에서 처음 움직인 지령
        self.u_dead = math.nan      # 확정된 데드존 [%]
        self.dp_at = math.nan       # 확정 시점의 실제 차압
        self.p_at = math.nan        # 확정 시점의 챔버압
        self.drift = 0.0            # 밸브 닫힘 상태의 자연 변화율 [kPa/s]
        self.note = ""
        self.done = False
        # 상태 머신용
        self.u = 0.0
        self.hits = 0
        # 스텝마다 (지령, 열기 직전 압력, 열기 직전 차압) — 확정 시 거슬러 올라간다
        self.hist: list[tuple[float, float | None, float]] = []
        self.rep = 0                # 지금 몇 회차인가 (로그에 남긴다)

    def finalize(self) -> None:
        """회차 표본을 중앙값으로 합친다."""
        if not self.samples:
            return
        us = sorted(x[0] for x in self.samples)
        mid = len(us) // 2
        self.u_dead = us[mid] if len(us) % 2 else 0.5 * (us[mid - 1] + us[mid])
        self.dp_at = sorted(x[1] for x in self.samples)[len(us) // 2]
        self.p_at = sorted(x[2] for x in self.samples)[len(us) // 2]
        self.spread = us[-1] - us[0]
        if len(us) >= 2:
            m = sum(us) / len(us)
            self.sd = (sum((v - m) ** 2 for v in us) / (len(us) - 1)) ** 0.5

    @property
    def name(self) -> str:
        return f"ch{self.gid}({'양압' if self.is_pos else '음압'}) {ROLE[self.slot]}"

    def ref_kpa(self, rig: Rig) -> float | None:
        """이 밸브의 챔버 반대편 압력."""
        if self.slot == V_MICRO:
            return rig.kpa(RAIL_POS_BOARD if self.is_pos else RAIL_NEG_BOARD)
        return rig.atm

    def close(self, rig: Rig) -> None:
        rig.set_valve(self.board, V_MICRO, 0.0)
        rig.set_valve(self.board, V_ATM, 0.0)

    def open_probe(self, rig: Rig, u: float) -> None:
        self.close(rig)
        rig.set_valve(self.board, self.slot, u)


# ── 압력 변화율 추정 ────────────────────────────────────────────────────────
def slope_kpa_s(samples: list[tuple[float, float]], skip_frac: float = 0.4) -> float:
    """(t, P) 표본의 최소제곱 기울기 [kPa/s].

    앞쪽 skip_frac 은 버린다 — 밸브 1차 응답(τ≈25 ms)과 센서 LPF 가 섞인 구간이라
    기울기를 과대평가한다. 우리가 보려는 것은 정상 유량이 만드는 변화율이다.
    """
    if len(samples) < 6:
        return 0.0
    use = samples[int(len(samples) * skip_frac):]
    if len(use) < 4:
        use = samples
    n = float(len(use))
    tm = sum(t for t, _ in use) / n
    pm = sum(p for _, p in use) / n
    num = sum((t - tm) * (p - pm) for t, p in use)
    den = sum((t - tm) ** 2 for t, _ in use)
    return num / den if den > 1e-12 else 0.0


def hold_and_measure(rig: Rig, probes: list[Probe], seconds: float
                     ) -> dict[int, float]:
    """seconds 동안 현재 명령을 유지하며 각 probe 의 압력 변화율을 잰다."""
    acc: dict[int, list[tuple[float, float]]] = {id(p): [] for p in probes}
    t0 = time.monotonic()
    nxt = 0.0
    while True:
        now = time.monotonic()
        if now - t0 >= seconds:
            break
        rclpy.spin_once(rig, timeout_sec=0.005)
        if now >= nxt:
            rig.publish()
            nxt = now + 0.02
        for pr in probes:
            P = rig.kpa(pr.board)
            if P is not None:
                acc[id(pr)].append((now - t0, P))
    return {id(p): slope_kpa_s(acc[id(p)]) for p in probes}


# ════════════════════════════════════════════════════════════════════════════
# 단계 1: 사전 위치잡기 — 각 채널을 목표 차압이 되는 압력에 세운다
# ════════════════════════════════════════════════════════════════════════════
def preposition(rig: Rig, probes: list[Probe], args) -> None:
    """여러 채널을 --prepos-batch 개씩 나눠 세운다.

    레일이 무너지는 것은 **위치잡기 때문**이다 — 채널을 활짝 열어 채우는 단계라
    유량이 크다. 측정 단계는 데드존 근처라 유량이 거의 0 이므로 같이 해도 된다.
    나눠서 세우면 레일이 각 배치 사이에 회복한다.
    """
    n = max(1, args.prepos_batch)
    if len(probes) <= n:
        _preposition_one_batch(rig, probes, args)
        return
    for k in range(0, len(probes), n):
        _preposition_one_batch(rig, probes[k:k + n], args)


def _preposition_one_batch(rig: Rig, probes: list[Probe], args) -> None:
    """챔버를 목표압 근처로 몰아간다.

    **정확히 맞출 필요가 없다.** 데드존은 확정 시점의 *실제* 차압으로 기록하므로
    (Probe.dp_at → yaml 표의 x 좌표), 목표에서 몇 kPa 벗어나도 표의 점 위치가 조금
    옮겨질 뿐 값이 틀리지 않는다. 그래서 "가능한 만큼 가까이 가고, 멈추면 받아들인다".

    지령은 **항상 --drive-pct 로 활짝 연다.** 예전에는 오차가 작아지면 지령을 절반으로
    낮췄는데(taper), 데드존을 아직 모르는 상태에서 그러면 지령이 데드존 아래로 내려가
    유량이 0 이 되고 목표 몇 kPa 앞에서 그대로 멈춘다 — 실측에서 156 목표에 153 에서
    정지했다. 대신 **여는 시간**을 줄인다: --pulse-sec 주기 안에서 오차에 비례하는
    만큼만 열고 나머지는 닫는다. 밸브 시정수 25 ms 보다 충분히 긴 펄스라 유량이 나온다.
    """
    print("    위치잡기: " + ", ".join(f"ch{p.gid}→{p.sp:.0f}" for p in probes))
    st = {id(p): {"ok_since": None, "p_ref": None, "t_ref": 0.0, "done": False}
          for p in probes}
    t_end = time.monotonic() + args.prepos_timeout
    while time.monotonic() < t_end:
        rclpy.spin_once(rig, timeout_sec=0.005)
        rig.publish()
        now = time.monotonic()
        if all(v["done"] for v in st.values()):
            break
        for pr in probes:
            v = st[id(pr)]
            if v["done"]:
                continue
            P = rig.kpa(pr.board)
            if P is None:
                continue
            err = pr.sp - P

            # ① 목표에 들어왔나
            if abs(err) <= args.sp_tol:
                pr.close(rig)
                if v["ok_since"] is None:
                    v["ok_since"] = now
                elif now - v["ok_since"] >= args.sp_hold:
                    v["done"] = True
                continue
            v["ok_since"] = None

            # ② 못 가고 멈춰 있나 (데드존·누설 균형). 받아들일 만하면 그대로 간다.
            if v["p_ref"] is None or abs(P - v["p_ref"]) > args.stall_kpa:
                v["p_ref"], v["t_ref"] = P, now
            elif now - v["t_ref"] >= args.stall_sec:
                pr.close(rig)
                v["done"] = True
                if abs(err) <= args.accept_tol:
                    pr.note = f"목표 {pr.sp:.0f} → 실제 {P:.1f} kPa 에서 정지, 그대로 측정"
                else:
                    pr.note = f"위치잡기 정지 (목표 {pr.sp:.0f}, 실제 {P:.1f})"
                print(f"      ch{pr.gid} {pr.note}")
                continue

            # ③ 몰아간다 — 지령은 활짝, 여는 **시간**만 오차에 비례해서 줄인다
            duty = min(1.0, max(args.min_duty, abs(err) / max(0.1, args.full_duty_kpa)))
            phase = (now % args.pulse_sec) / args.pulse_sec
            pr.close(rig)
            if phase < duty:
                rig.set_valve(pr.board, pr.slot_up if err > 0 else pr.slot_dn,
                              args.drive_pct)
    for pr in probes:
        v = st[id(pr)]
        P = rig.kpa(pr.board)
        if not v["done"] and P is not None:
            pr.note = f"위치잡기 시간초과 (목표 {pr.sp:.0f}, 실제 {P:.1f})"
            print(f"      ch{pr.gid} {pr.note}")
        pr.close(rig)


def settle(rig: Rig, probes: list[Probe], args) -> None:
    """전 밸브를 닫고 압력 변화율이 --settle-rate 아래로 떨어질 때까지 기다린다."""
    for pr in probes:
        pr.close(rig)
    deadline = time.monotonic() + args.settle_timeout
    while time.monotonic() < deadline:
        rates = hold_and_measure(rig, probes, args.settle_window)
        if all(abs(rates[id(p)]) <= args.settle_rate for p in probes):
            return
    # 시간이 다 됐으면 그대로 간다 — 남은 드리프트는 baseline 이 흡수한다


# ════════════════════════════════════════════════════════════════════════════
# 단계 2: 램프 — 굵게 훑어 구간을 잡고, 곱게 올려 확정한다
# ════════════════════════════════════════════════════════════════════════════
def ramp(rig: Rig, probes: list[Probe], args, log, phase: str,
         step: float, hold: float, thresh: float, confirm: int) -> None:
    """활성 probe 들의 지령을 step 씩 올리며 '유효한 움직임' 을 찾는다.

    각 probe 는 자기 지령·자기 판정을 갖는다 (채널마다 데드존이 다르므로).
    판정: dir·기울기 > 자연 드리프트 + thresh 를 confirm 스텝 연속.
    """
    active = [p for p in probes if not p.done]
    for pr in active:
        pr.hits = 0
        pr.hist = []
    while active:
        # 이번 스텝의 지령을 올린다. 올리기 **직전**의 압력·차압을 먼저 남긴다 —
        # 확정 시점(홀드 끝, 게다가 confirm 스텝 뒤)의 차압은 이미 유량이 흐른 뒤라
        # 목표에서 10 kPa 넘게 밀려 있다. 데드존의 x 좌표는 "흐르기 직전" 이어야 한다.
        for pr in active:
            P0, ref0 = rig.kpa(pr.board), pr.ref_kpa(rig)
            dp0 = (pr.dir * (ref0 - P0)) if (P0 is not None and ref0 is not None) else math.nan
            pr.u += step
            pr.hist.append((pr.u, P0, dp0))
            pr.open_probe(rig, pr.u)
        rates = hold_and_measure(rig, active, hold)
        still = []
        for pr in active:
            P = rig.kpa(pr.board)
            ref = pr.ref_kpa(rig)
            rate = rates[id(pr)]
            eff = pr.dir * rate                      # 이 밸브가 흘리는 방향의 변화율
            moved = eff > pr.drift * pr.dir + thresh
            dp_now = (pr.dir * (ref - P)) if (P is not None and ref is not None) else math.nan
            log.row(pr, phase, pr.u, P, ref, dp_now, rate, eff, pr.drift, moved)
            if moved:
                pr.hits += 1
                if pr.hits >= confirm:
                    # confirm 스텝 연속으로 움직였다 → **처음** 움직인 스텝이 데드존.
                    # 지령·압력·차압 모두 그 스텝을 열기 직전의 값을 쓴다.
                    hit_u, hit_p, hit_dp = pr.hist[-confirm]
                    if phase == "coarse":
                        pr.u_coarse = hit_u
                    else:
                        pr.hit = (hit_u, hit_dp, hit_p)
                    pr.close(rig)
                    pr.done = True
                    print(f"      {pr.name}: {phase} {hit_u:.2f} % "
                          f"(흐르기 직전 dp {hit_dp:.1f} kPa / 확정 시점 {dp_now:.1f}, "
                          f"{eff:+.2f} kPa/s vs 드리프트 {pr.dir*pr.drift:+.2f})")
                    continue
            else:
                pr.hits = 0
            if pr.u >= args.u_max - 1e-9:
                pr.close(rig)
                pr.done = True
                pr.note = f"{args.u_max:.0f} % 까지 움직임 없음"
                print(f"      {pr.name}: {pr.note}")
                continue
            # 안전: 챔버가 위험 구간으로 가면 그 채널만 접는다
            if P is not None and (P > args.p_max or P < args.p_min):
                pr.close(rig)
                pr.done = True
                pr.note = f"챔버 {P:.0f} kPa — 측정 범위를 벗어나 중단"
                print(f"      {pr.name}: {pr.note}")
                continue
            still.append(pr)
        active = still


def measure_group(rig: Rig, probes: list[Probe], args, log) -> None:
    """(밸브, 차압) 하나를 선택한 **모든 채널에서 동시에**, --repeats 회 반복 측정한다.

    반복은 첫 회차만 굵은 훑기를 하고, 이후에는 지금까지의 중앙값 바로 아래에서
    정밀 램프만 다시 탄다 (--refine-span). 굵은 훑기가 회차당 20 초쯤 걸리므로
    이것만으로 5 회 반복 비용이 절반 아래로 내려간다.
    """
    # 목표 챔버압 = ref − dir·dp. 도달 불가능한 조합은 빼낸다.
    keep = []
    for pr in probes:
        ref = pr.ref_kpa(rig)
        if ref is None:
            pr.note = "반대편 압력을 못 읽었다"
            continue
        pr.ref, pr.sp = ref, ref - pr.dir * pr.dp_target
        # 챔버 목표압의 한계. **안전 마진이 아니라 물리적 도달 가능 범위**다 —
        # p_max/p_min 은 사용자가 준 값 그대로 쓴다 (숨은 여유를 더하지 않는다).
        #   양압: 아래는 대기압 (atm 밸브는 대기까지만 뺀다)
        #         위는 레일 (레일보다 높게 채울 수 없다. 레일 바로 밑은 차압 0 이라
        #         무한히 걸리므로 --fill-headroom 만큼 띄운다)
        #   음압: 아래는 진공 레일, 위는 대기압
        if pr.is_pos:
            rail = rig.kpa(RAIL_POS_BOARD)
            lo = rig.atm + 1.5
            hi = min(args.p_max,
                     (rail - args.fill_headroom) if rail is not None else args.p_max)
        else:
            rail = rig.kpa(RAIL_NEG_BOARD)
            lo = max(args.p_min,
                     (rail + args.fill_headroom) if rail is not None else args.p_min)
            hi = rig.atm - 1.5
        if not (lo <= pr.sp <= hi):
            pr.note = (f"차압 {pr.dp_target:.0f} 불가 — 챔버 목표 {pr.sp:.0f} 가 "
                       f"[{lo:.0f},{hi:.0f}] 밖")
            print(f"      {pr.name}: {pr.note}")
            continue
        keep.append(pr)
    if not keep:
        return

    def baseline(group, hold):
        """밸브 닫힘 상태의 자연 드리프트 = 귀무가설. 누설·온도·센서가 다 여기 들어간다."""
        rates = hold_and_measure(rig, group, args.baseline)
        for pr in group:
            pr.drift = rates[id(pr)]
            log.row(pr, "baseline", 0.0, rig.kpa(pr.board), pr.ref_kpa(rig),
                    math.nan, pr.drift, pr.dir * pr.drift, pr.drift, False)

    def reposition(group, force):
        """첫 회차는 무조건, 이후에는 목표에서 --accept-tol 넘게 벗어난 채널만.

        표는 확정 시점의 실제 차압으로 인덱싱하므로 몇 kPa 차이는 그대로 기록하면
        된다. 매번 다시 세우면 그게 측정 시간의 대부분을 먹는다.
        """
        need = group if force else [
            pr for pr in group
            if (rig.kpa(pr.board) is None
                or abs(pr.sp - rig.kpa(pr.board)) > args.accept_tol)]
        if need:
            preposition(rig, need, args)
        elif not force:
            print(f"    재위치잡기 생략 — 전 채널이 목표 ±{args.accept_tol:.0f} kPa 안")
        settle(rig, group, args)

    for rep in range(args.repeats):
        for pr in keep:
            pr.rep = rep + 1
        alive = [pr for pr in keep if rep == 0 or pr.samples or not math.isnan(pr.u_coarse)]
        if not alive:
            break
        print(f"    ── {rep+1}/{args.repeats} 회차 " + "─" * 40)
        reposition(alive, force=(rep == 0))

        if rep == 0:
            baseline(alive, args.u_coarse_hold)
            print("    드리프트: " + ", ".join(
                f"ch{p.gid} {p.dir*p.drift:+.2f}" for p in alive) + " kPa/s")
            for pr in alive:
                pr.u, pr.done = 0.0, False
            ramp(rig, alive, args, log, "coarse",
                 args.u_coarse, args.u_coarse_hold, args.rate_thresh_coarse, 1)
            alive = [pr for pr in alive if not math.isnan(pr.u_coarse)]
            if not alive:
                return
            reposition(alive, force=False)

        baseline(alive, args.u_hold)
        for pr in alive:
            # 출발점: 첫 회차는 굵은 훑기가 잡은 칸의 한 칸 아래, 이후 회차는
            # 지금까지의 중앙값에서 --refine-span 아래.
            if pr.samples:
                us = sorted(x[0] for x in pr.samples)
                est = us[len(us) // 2]
                pr.u = max(0.0, est - args.refine_span)
            else:
                pr.u = max(0.0, pr.u_coarse - args.u_coarse)
            pr.done, pr.hit = False, None
        ramp(rig, alive, args, log, "fine",
             args.u_step, args.u_hold, args.rate_thresh, args.confirm)
        for pr in alive:
            if pr.hit is not None:
                pr.samples.append(pr.hit)
                # 회차 확정값을 로그에 한 줄로 남긴다 — 나중에 회차 간 분산을
                # steps.csv 만 보고 계산할 수 있게 (phase=hit 로 걸러내면 된다).
                u, dpa, pa = pr.hit
                log.row(pr, "hit", u, pa, pr.ref_kpa(rig), dpa,
                        math.nan, math.nan, pr.drift, True)

    for pr in keep:
        pr.finalize()
        if pr.samples:
            print(f"      {pr.name} dp={pr.dp_target:.0f}: {len(pr.samples)}회 중앙값 "
                  f"{pr.u_dead:.2f} % @dp {pr.dp_at:.1f} (퍼짐 {pr.spread:.2f} %p)")
        elif not pr.note:
            pr.note = "전 회차 판정 실패"


# ════════════════════════════════════════════════════════════════════════════
# 로깅
# ════════════════════════════════════════════════════════════════════════════
class Log:
    """모든 홀드 구간을 한 줄씩 남긴다.

    확정된 데드존만 남기면 판정이 옳았는지 나중에 확인할 수 없다. 지령·압력·차압·
    변화율·드리프트·판정을 전부 남겨서, 임계값(--rate-thresh)을 바꿔 재판정하거나
    "여기서 정말 열린 게 맞나" 를 로그만으로 되짚을 수 있게 한다.
    """

    HEADER = ("t_sec,gid,axis,side,valve,rep,phase,dp_target_kpa,u_pct,p_chamber_kpa,"
              "p_ref_kpa,dp_actual_kpa,rate_kpa_s,rate_eff_kpa_s,drift_kpa_s,moved\n")

    def __init__(self, path: str) -> None:
        self.fp = open(path, "w", encoding="utf-8")
        self.fp.write(self.HEADER)
        self.t0 = time.monotonic()

    def row(self, pr: Probe, phase: str, u: float, P, ref, dp,
            rate: float, eff: float, drift: float, moved: bool) -> None:
        def f(v):
            return "" if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:.4f}"
        self.fp.write(
            f"{time.monotonic()-self.t0:.3f},{pr.gid},{pr.axis},"
            f"{'pos' if pr.is_pos else 'neg'},{ROLE[pr.slot]},{pr.rep},{phase},"
            f"{pr.dp_target:.1f},{u:.3f},{f(P)},{f(ref)},{f(dp)},"
            f"{rate:.4f},{eff:.4f},{drift:.4f},{int(moved)}\n")
        self.fp.flush()

    def close(self) -> None:
        self.fp.close()


# ════════════════════════════════════════════════════════════════════════════
# 결과 정리 — 표 출력과 yaml 블록 생성
# ════════════════════════════════════════════════════════════════════════════
def fit_curve(dp: list[float], u: list[float], order: int, step: float,
              span: tuple[float, float] | None = None):
    """데드존(차압) 곡선을 다항식으로 피팅하고, 조밀한 표로 되돌린다.

    반환 (dp_dense, u_dense, 차수, rmse, max_resid, 외삽구간, 차수비교표) 또는 None.

    **차수는 교차검증(LOO-CV)으로 고른다.** RMSE 로 고르면 차수를 올릴수록 항상
    좋아지므로 과적합을 못 잡는다. 실측 ch1 atm 20점에서:
        차수  RMSE   LOO-CV   0~105 단조   dp99 외삽
          1   0.435   0.477       예        50.20
          2   0.435   0.508       예        50.15
          3   0.398   0.497     아니오       54.25
          4   0.383   0.495     아니오       70.80   ← 측정 최대(62.25)보다 높다
    RMSE 는 계속 줄지만 CV 는 커진다 = 과적합. 이 구간의 데드존은 사실상 직선이다.

    단조성은 **표를 내보낼 전 구간**에서 본다. 데드존은 차압이 커질수록 낮아져야
    한다. 고차는 측정점을 정확히 통과하면서도 외삽에서 되돌아 올라간다.

    왜 다항식을 그대로 yaml 에 넣지 않고 표로 되돌리나: 컨트롤러가 표를 선형보간하고
    조밀한 표(기본 5 kPa)의 선형보간은 다항식과 수치적으로 같다. 피팅의 이득(회차
    잡음 평활화)은 그대로 남고, 컨트롤러에 다항식 평가를 넣지 않아도 된다.
    """
    try:
        import numpy as np
    except ImportError:
        return None
    x, y = np.asarray(dp, float), np.asarray(u, float)
    if len(x) < 3:
        return None
    m_lo, m_hi = float(x.min()), float(x.max())
    lo, hi = span if span else (m_lo, m_hi)
    lo, hi = min(lo, m_lo), max(hi, m_hi)      # 측정 범위는 항상 포함한다
    dense = np.arange(lo, hi + 0.5 * step, step)
    if dense[-1] < hi:
        dense = np.append(dense, hi)

    table, best = [], None
    for k in range(1, max(1, order) + 1):
        if k + 1 > len(x) - 2:                 # 자유도가 모자라면 과적합이다
            break
        c = np.polyfit(x, y, k)
        res = y - np.polyval(c, x)
        rmse = float(np.sqrt(np.mean(res ** 2)))
        # leave-one-out 교차검증 — 고차가 정당한지 판단하는 정직한 기준
        loo = []
        for t in range(len(x)):
            m = np.ones(len(x), bool)
            m[t] = False
            loo.append(y[t] - np.polyval(np.polyfit(x[m], y[m], k), x[t]))
        cv = float(np.sqrt(np.mean(np.asarray(loo) ** 2)))
        yd = np.polyval(c, dense)
        mono = bool(np.all(np.diff(yd) <= 1e-9))
        table.append((k, rmse, cv, mono))
        if mono and (best is None or cv < best[2]):
            best = (k, c, cv, rmse, float(np.abs(res).max()), yd)
    if best is None:
        return None
    k, _, _, rmse, mx, yd = best
    return (dense.tolist(), yd.tolist(), k, rmse, mx,
            (round(m_lo - lo, 1), round(hi - m_hi, 1)), table)


def report(results: list[Probe], atm: float) -> str:
    """확정된 데드존을 채널×밸브×차압 표로 찍는다.

    칸은 `지령@실제차압 sd회차표준편차×회차수` 다. 목표 차압을 정확히 맞추지 못해도
    (챔버가 데드존 때문에 몇 kPa 앞에서 멈추는 일이 흔하다) 실제 차압으로 기록하므로
    값 자체는 유효하다 — 다만 어느 차압에서 잰 값인지 보여야 판단할 수 있다.
    """
    lines = []
    ok = [p for p in results if not math.isnan(p.u_dead)]
    dps = sorted({p.dp_target for p in results})
    lines.append("")
    lines.append("=" * 78)
    lines.append("데드존 실측 결과 — `지령[%]@실제차압[kPa] sd회차표준편차×회차수`")
    lines.append("지령이 이 값을 넘으면 유량이 시작된다. sd 가 크면 그 점은 못 믿는다.")
    lines.append("=" * 78)
    hdr = f"{'채널':6} {'밸브':6} " + " ".join(f"목표dp={d:<5.0f}".ljust(24) for d in dps)
    lines.append(hdr)
    lines.append("-" * len(hdr))
    for gid in sorted({p.gid for p in results}):
        for slot in (V_MICRO, V_ATM):
            row = [p for p in results if p.gid == gid and p.slot == slot]
            if not row:
                continue
            cells = []
            for d in dps:
                m = [p for p in row if p.dp_target == d]
                if m and not math.isnan(m[0].u_dead):
                    q = m[0]
                    dp = q.dp_at if not math.isnan(q.dp_at) else d
                    sd = "  -  " if math.isnan(q.sd) else f"{q.sd:4.2f}"
                    cells.append(f"{q.u_dead:6.2f}@{dp:<5.1f} sd{sd}"
                                 f"×{len(q.samples)}".ljust(24))
                else:
                    cells.append("-".ljust(24))
            side = "양압" if row[0].is_pos else "음압"
            lines.append(f"ch{gid:<2} {side} {ROLE[slot]:6} " + " ".join(cells))
    lines.append("")
    lines.append(f"확정 {len(ok)}/{len(results)} 조합.")
    notes = [p for p in ok if p.note]
    if notes:
        lines.append("측정은 됐지만 목표 차압과 다른 곳에서 잰 것:")
        for p in notes:
            lines.append(f"  {p.name} 목표dp={p.dp_target:.0f}: {p.note}")
    bad = [p for p in results if math.isnan(p.u_dead)]
    if bad:
        lines.append("못 잰 조합:")
        for p in bad:
            lines.append(f"  {p.name} dp={p.dp_target:.0f}: {p.note or '판정 없음'}")
    return "\n".join(lines)


def compare_valves(results: list[Probe]) -> str:
    """같은 채널의 micro 와 atm 을 맞대어 본다.

    두 밸브가 같은 부품이면 같은 차압에서 같은 데드존이 나와야 한다. 차이가 회차
    표준편차 수준이면 두 결과를 합쳐 쓸 수 있고 (--valve-table merged), 한쪽으로
    치우친 **계통 오차**면 그 원인을 알기 전까지 합치면 안 된다 — 평균이 둘 다 틀린
    값이 된다.
    """
    lines = ["", "=" * 78, "같은 채널의 micro vs atm — 합쳐 써도 되나", "=" * 78]
    any_row = False
    for gid in sorted({p.gid for p in results}):
        rows = []
        for slot in (V_MICRO, V_ATM):
            for p in results:
                if p.gid == gid and p.slot == slot and not math.isnan(p.u_dead):
                    rows.append(p)
        mi = sorted((p for p in rows if p.slot == V_MICRO), key=lambda p: p.dp_at)
        at = sorted((p for p in rows if p.slot == V_ATM), key=lambda p: p.dp_at)
        if not mi or not at:
            continue
        any_row = True
        lines.append(f"\n[ch{gid} {'양압' if rows[0].is_pos else '음압'}]")
        lines.append(f"  {'차압':>6} {'micro':>7} {'atm':>7} {'차이':>7} "
                     f"{'sd(mi/at)':>11}")
        diffs = []
        for m in mi:
            # atm 표를 보간해 micro 와 같은 차압에서 비교한다
            if len(at) == 1:
                a = at[0].u_dead
            elif m.dp_at <= at[0].dp_at:
                a = at[0].u_dead
            elif m.dp_at >= at[-1].dp_at:
                a = at[-1].u_dead
            else:
                k = next(i for i in range(1, len(at)) if at[i].dp_at >= m.dp_at)
                x0, x1 = at[k - 1].dp_at, at[k].dp_at
                w = (m.dp_at - x0) / (x1 - x0) if x1 > x0 else 0.0
                a = at[k - 1].u_dead + w * (at[k].u_dead - at[k - 1].u_dead)
            d = m.u_dead - a
            diffs.append(d)
            sds = (("%.2f" % m.sd) if not math.isnan(m.sd) else "-") + "/" + \
                  (("%.2f" % at[0].sd) if not math.isnan(at[0].sd) else "-")
            lines.append(f"  {m.dp_at:6.1f} {m.u_dead:7.2f} {a:7.2f} {d:+7.2f} "
                         f"{sds:>11}")
        mean_d = sum(diffs) / len(diffs)
        span_d = max(diffs) - min(diffs)
        sd_pool = [p.sd for p in mi + at if not math.isnan(p.sd)]
        sd_typ = (sum(sd_pool) / len(sd_pool)) if sd_pool else float("nan")
        lines.append(f"  → micro − atm 평균 {mean_d:+.2f} %p (폭 {span_d:.2f}), "
                     f"회차 sd 평균 {sd_typ:.2f} %p")
        if not math.isnan(sd_typ) and abs(mean_d) <= 2.0 * sd_typ:
            lines.append("     ⇒ 차이가 회차 잡음 수준이다. 합쳐 쓸 수 있다 "
                         "(--valve-table merged)")
        else:
            lines.append("     ⇒ **한쪽으로 치우친 계통 차이**다 (잡음보다 크고 부호가 "
                         "일정). 평균을 쓰면 둘 다 틀린다 — 원인을 모르면 "
                         "--valve-table separate 를 유지하는 편이 안전하다")
    if not any_row:
        lines.append("\n(한 밸브만 재서 비교할 것이 없다)")
    return "\n".join(lines)


def compare_channels(results: list[Probe]) -> str:
    """채널 간 차이를 본다 — 한 표를 전 채널에 써도 되는지 판단하는 근거다.

    같은 (측, 밸브, 목표차압) 에서 채널별 데드존이 얼마나 벌어지는지 본다. 그 폭이
    회차 퍼짐(같은 채널을 반복해 잰 편차)과 비슷하면 채널 차이는 측정 잡음 수준이므로
    측별 표 하나로 충분하다. 반대로 폭이 훨씬 크면 채널마다 따로 재야 한다.
    """
    lines = ["", "=" * 78,
             "채널 간 차이 — 측별 표 하나로 갈 수 있나",
             "=" * 78]
    any_row = False
    for is_pos in (True, False):
        for slot in (V_MICRO, V_ATM):
            rows = []
            for dp in sorted({p.dp_target for p in results
                              if p.is_pos == is_pos and p.slot == slot}):
                pts = [p for p in results if p.is_pos == is_pos and p.slot == slot
                       and p.dp_target == dp and not math.isnan(p.u_dead)]
                if len(pts) < 2:
                    continue
                us = [p.u_dead for p in pts]
                span = max(us) - min(us)
                rep = max(p.spread for p in pts if not math.isnan(p.spread)) \
                    if any(not math.isnan(p.spread) for p in pts) else float("nan")
                rows.append((dp, len(us), sum(us) / len(us), min(us), max(us), span, rep))
            if not rows:
                continue
            any_row = True
            lines.append(f"\n[{'양압' if is_pos else '음압'} {ROLE[slot]}]  "
                         f"채널 {len({p.gid for p in results if p.is_pos == is_pos})}개")
            lines.append(f"  {'목표dp':>6} {'채널수':>5} {'평균':>7} {'최소':>7} {'최대':>7}"
                         f" {'채널폭':>7} {'회차퍼짐':>8}  판정")
            for dp, n, mean, lo, hi, span, rep in rows:
                verdict = ("측정잡음 수준" if not math.isnan(rep) and span <= max(rep, 0.5) * 1.5
                           else "채널차 뚜렷" if span > 2.0 else "경계")
                lines.append(f"  {dp:6.0f} {n:5d} {mean:7.2f} {lo:7.2f} {hi:7.2f}"
                             f" {span:7.2f} {rep:8.2f}  {verdict}")
            spans = [r[5] for r in rows]
            lines.append(f"  → 채널폭 최대 {max(spans):.2f} %p"
                         + ("  ⇒ 측별 표 하나로 충분해 보인다 (--write-scope side)"
                            if max(spans) <= 1.5 else
                            "  ⇒ 채널별로 재는 편이 낫다 (--write-scope channel)"))
    if not any_row:
        lines.append("\n(채널이 하나뿐이라 비교할 것이 없다)")
    return "\n".join(lines)


def read_existing(path: str) -> dict:
    """yaml 에 이미 적혀 있는 데드존 표를 읽는다.

    끊어서 측정할 때 **이번에 안 잰 채널을 지우지 않기 위해** 필요하다. 예전에는
    마커 사이를 통째로 덮어써서, 두 번째 실행이 첫 번째 결과를 날렸다.

    반환 {"ch": {gid: {role: (dp[], u[])}}, "side": {"pos"/"neg": {role: (dp[], u[])}}}
    """
    out = {"ch": {}, "side": {}, "opt": {}, "ch_extra": {}, "ch_margin": {}}
    try:
        import yaml
        with open(path, encoding="utf-8") as fp:
            prm = yaml.safe_load(fp)["/pack2/pp_controller"]["ros__parameters"]
    except Exception:
        return out

    def grab_margin(dz):
        """deadzone 안의 margin_* 키. 표와 같은 매핑에 살기 때문에 여기서 안 챙기면
        표를 새로 쓸 때 통째로 사라진다 (20260911 에 실제로 12채널이 날아갔다)."""
        if not isinstance(dz, dict):
            return {}
        return {k: v for k, v in dz.items() if k.startswith("margin_")}

    def grab(dz):
        got = {}
        if not isinstance(dz, dict):
            return got
        for role in ("micro", "atm", "macro"):
            dp, u = dz.get(f"{role}_dp_kpa"), dz.get(f"{role}_u_pct")
            if (isinstance(dp, list) and isinstance(u, list)
                    and dp and len(dp) == len(u)):
                got[role] = ([float(x) for x in dp], [float(x) for x in u])
        return got

    for key, val in (prm.get("channel_config") or {}).items():
        m = re.fullmatch(r"ch(\d+)", str(key))
        if not (m and isinstance(val, dict)):
            continue
        gid = int(m.group(1))
        got = grab(val.get("deadzone"))
        if got:
            out["ch"][gid] = got
        mg = grab_margin(val.get("deadzone"))
        if mg:
            out["ch_margin"][gid] = mg
        # deadzone 말고 이 스크립트가 모르는 키(pid, volume_ml, ...)는 그대로 보존한다.
        # channel_config 매핑을 이 블록이 통째로 소유하므로, 안 적어 주면 사라진다.
        extra = {k: v for k, v in val.items() if k != "deadzone"}
        if extra:
            out["ch_extra"][gid] = extra
    vd = prm.get("valve_deadzone") or {}
    for side in ("pos", "neg"):
        got = grab(vd.get(side))
        if got:
            out["side"][side] = got
    # 스칼라 설정도 읽어 둔다 — 이 블록이 valve_deadzone 전체를 소유하므로
    # 다시 써 주지 않으면 사용자가 넣은 값이 사라진다 (예전에 정적 블록과 키가
    # 겹쳐 enable/*_pct 가 조용히 무시되던 버그가 이것이었다).
    for k in DEFAULT_DZ_OPT:
        if k in vd:
            out["opt"][k] = vd[k]
    return out


def _fit_or_raw(xs, ys, args, tag, rows):
    """측정점을 피팅해 조밀한 표로 바꾼다. 원측정값과 차수 비교를 주석으로 남긴다."""
    n_show = 12
    shown = ", ".join(f"{x:.1f}→{y:.2f}" for x, y in list(zip(xs, ys))[:n_show])
    rows.append(f"          # {tag} 측정 {len(xs)}점"
                + (f" (앞 {n_show}개만): " if len(xs) > n_show else ": ") + shown)
    got = fit_curve(xs, ys, args.fit_order, args.fit_step, args.table_range)
    if got:
        dx, dy, k, rmse, mx, (ex_lo, ex_hi), table = got
        rows.append(f"          # {tag} 차수선택(LOO-CV): "
                    + "  ".join(f"{t[0]}차 CV{t[2]:.3f}{'' if t[3] else '(비단조)'}"
                                for t in table)
                    + f"  → {k}차")
        note = ""
        if ex_lo > 0.05 or ex_hi > 0.05:
            note = (f", 외삽 아래 {ex_lo:.0f} + 위 {ex_hi:.0f} kPa "
                    f"(표 {min(dx):.0f}~{max(dx):.0f}, 측정 {min(xs):.0f}~{max(xs):.0f})")
        rows.append(f"          # {tag} {k}차: RMSE {rmse:.3f} %p, "
                    f"최대잔차 {mx:.3f} %p{note}")
        return dx, dy
    rows.append(f"          # {tag} 피팅 실패 — 측정점을 그대로 쓴다")
    return xs, ys


def _valve_points(results, keyfn, args):
    """(키, 역할) → 측정점 목록. --valve-table 정책을 여기서 적용한다.

    **차압별 중앙값이 아니라 회차 표본을 전부 넘긴다.** 회차마다 실제 차압이 조금씩
    다르므로 (레일·챔버가 흔들린다) 각 (지령, 실제차압) 쌍이 독립적인 정보다.
    중앙값 4개로 줄이면 자유도가 2차까지밖에 안 남아 차수 검증 자체가 불가능하고,
    회차 잡음을 평활화할 기회도 버린다.

    micro 와 atm 은 같은 부품이므로, 두 결과를 합쳐 쓰거나 한쪽만 믿을 수 있다.
    무엇이 맞는지는 compare_valves() 의 판정을 보고 사용자가 정한다.
    """
    raw = {}
    for pr in results:
        if not pr.samples:
            continue
        key = (keyfn(pr), ROLE[pr.slot])
        for u, dpa, _ in pr.samples:
            raw.setdefault(key, []).append((dpa, u))

    mode = args.valve_table
    if mode == "separate":
        return {k: sorted(v) for k, v in raw.items()}
    out = {}
    for key in {k for k, _ in raw}:
        mi, at = raw.get((key, "micro"), []), raw.get((key, "atm"), [])
        if mode == "merged":
            pooled = sorted(mi + at)
        elif mode == "atm":
            pooled = sorted(at) or sorted(mi)
        else:                                  # "micro"
            pooled = sorted(mi) or sorted(at)
        if not pooled:
            continue
        for role in ("micro", "atm"):
            if raw.get((key, role)):            # 그 역할을 실제로 쟀을 때만 쓴다
                out[(key, role)] = pooled
    return out


def _dump_extra(extra: dict) -> list[str]:
    """channel_config.chN 아래의 비-deadzone 키를 8칸 들여쓰기로 되살린다."""
    if not extra:
        return []
    import yaml as _y
    text = _y.safe_dump(extra, allow_unicode=True, default_flow_style=False,
                        sort_keys=True)
    return ["        " + ln for ln in text.rstrip("\n").split("\n")]


def yaml_block(results: list[Probe], args, existing: dict) -> str:
    """컨트롤러가 읽는 블록. **이번 측정과 기존 표를 병합한다.**

    --write-scope channel : 이번에 잰 채널의 표만 갱신하고 나머지는 그대로 둔다.
    --write-scope side    : 이번에 잰 채널들을 **측별로 묶어 하나로 피팅**하고
                            valve_deadzone.{pos,neg} 에 쓴다. 컨트롤러는 채널별 표가
                            없는 채널에 이 표를 쓴다 — 채널 간 차이가 작을 때
                            한두 채널만 재서 전 채널에 적용하는 길이다.
    """
    ch = {g: dict(v) for g, v in existing["ch"].items()}
    side = {k: dict(v) for k, v in existing["side"].items()}
    notes = []
    if args.valve_table != "separate":
        notes.append(f"    # --valve-table {args.valve_table}: micro 와 atm 을 "
                     f"{'묶어 하나로' if args.valve_table == 'merged' else args.valve_table + ' 쪽으로'} "
                     f"피팅해 두 밸브에 같은 표를 쓴다 (같은 부품)")

    by_side = args.write_scope == "side"
    keyfn = (lambda p: ("pos" if p.is_pos else "neg")) if by_side else (lambda p: p.gid)
    pts_map = _valve_points(results, keyfn, args)

    for (key, role), pts in sorted(pts_map.items(), key=lambda kv: (str(kv[0][0]), kv[0][1])):
        tag = f"{key} {role}" if by_side else f"ch{key} {role}"
        rows = []
        dx, dy = _fit_or_raw([x for x, _ in pts], [y for _, y in pts], args, tag, rows)
        notes += rows
        if by_side:
            side.setdefault(key, {})[role] = (dx, dy)
        else:
            ch.setdefault(key, {})[role] = (dx, dy)

    out = [BEGIN_MARK,
           f"    # 갱신 {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} — "
           f"scripts/valve_deadzone.py (회차 {args.repeats}, 피팅 {args.fit_order}차, "
           f"표 간격 {args.fit_step:.0f} kPa, 범위 {args.write_scope})",
           "    # dp_kpa = 차압(상류−하류) [kPa], u_pct = 그 차압에서 열리기 시작하는 지령 [%]",
           "    # 컨트롤러는 매 틱 실제 차압으로 이 표를 선형보간한다 (양 끝은 클램프).",
           "    # 우선순위: channel_config.chN.deadzone > valve_deadzone.{pos,neg} > 상수",
           "    # 이번에 안 잰 채널/밸브는 예전 값을 그대로 둔다 (끊어서 재도 안전).",
           ]
    out += notes
    opt = dict(DEFAULT_DZ_OPT)
    opt.update(existing.get("opt") or {})
    out.append("    valve_deadzone:")
    out.append("      # 이 매핑은 스크립트가 통째로 관리한다 (표와 같은 키라 나눠 둘 수 없다).")
    out.append("      # 아래 스칼라는 손으로 고쳐도 다음 --write 때 그대로 보존된다.")
    out.append(f"      enable: {str(bool(opt['enable'])).lower()}")
    out.append(f"      park_enable: {str(bool(opt['park_enable'])).lower()}"
               "     # 쉬는 밸브를 (표 최솟값 − park_below_pct) 에 걸어 둔다")
    out.append(f"      park_below_pct: {float(opt['park_below_pct']):.1f}")
    # 소수점을 반드시 찍는다 — `3` 으로 쓰면 ROS 가 정수 파라미터로 읽어 double 로
    # 받는 쪽에서 조용히 기본값이 쓰인다 (실제로 margin_pct 가 그렇게 무시됐다).
    out.append(f"      margin_pct: {float(opt['margin_pct']):.1f}"
               "   # 표에서 빼는 안전 여유 [%p]. 과보상은 릴레이 진동을 낳는다")
    for role in ("micro", "atm", "macro"):
        out.append(f"      {role}_pct: {float(opt[role + '_pct']):.1f}"
                   "        # 표가 없는 밸브에 쓰는 상수")
    if side:
        for tag_side in ("pos", "neg"):
            if tag_side not in side or not side[tag_side]:
                continue
            out.append(f"      {tag_side}:")
            for role in ("micro", "atm", "macro"):
                if role not in side[tag_side]:
                    continue
                dp, u = side[tag_side][role]
                out.append(f"        {role}_dp_kpa: [{', '.join(f'{v:.1f}' for v in dp)}]")
                out.append(f"        {role}_u_pct:  [{', '.join(f'{v:.2f}' for v in u)}]")
    extra = existing.get("ch_extra") or {}
    if ch or extra:
        out.append("    channel_config:")
        out.append("      # deadzone 은 스크립트가 관리한다. 그 밖의 키(pid 등)는")
        out.append("      # 손으로 적어도 다음 --write 때 그대로 보존된다.")
        for gid in sorted(set(ch) | set(extra)):
            if not ch.get(gid) and not extra.get(gid):
                continue
            out.append(f"      ch{gid}:")
            for line in _dump_extra(extra.get(gid) or {}):
                out.append(line)
            if not ch.get(gid):
                continue
            out.append("        deadzone:")
            for mk, mv in sorted((existing.get("ch_margin") or {}).get(gid, {}).items()):
                out.append(f"          {mk}: {float(mv):g}")
            for role in ("micro", "atm", "macro"):
                if role not in ch[gid]:
                    continue
                dp, u = ch[gid][role]
                out.append(f"          {role}_dp_kpa: [{', '.join(f'{v:.1f}' for v in dp)}]")
                out.append(f"          {role}_u_pct:  [{', '.join(f'{v:.2f}' for v in u)}]")
    out.append(END_MARK)
    return "\n".join(out) + "\n"


BEGIN_MARK = "    # ===== valve_deadzone.py 측정 결과 (자동 생성) ====="
END_MARK = "    # ===== 측정 결과 끝 ====="


def write_yaml(path: str, block: str) -> str:
    """yaml 의 측정 블록만 바꿔 넣는다. 마커가 없으면 어디에 붙일지 알려주고 멈춘다."""
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    with open(path, encoding="utf-8") as fp:
        text = fp.read()
    if BEGIN_MARK not in text or END_MARK not in text:
        raise RuntimeError(
            f"yaml 에 측정 블록 마커가 없다. 아래 두 줄 사이를 이 스크립트가 관리한다 —\n"
            f"{BEGIN_MARK}\n{END_MARK}\n"
            f"수동으로 아래 블록을 붙여 넣어라:\n\n{block}")
    i = text.index(BEGIN_MARK)
    j = text.index(END_MARK) + len(END_MARK) + 1
    backup = f"{path}.bak.{stamp}"
    shutil.copy2(path, backup)
    with open(path, "w", encoding="utf-8") as fp:
        fp.write(text[:i] + block + text[j:])
    return backup


def read_config(path: str) -> dict:
    """센서 보정과 LinePID 게인을 yaml 에서 읽는다.

    레일 게인을 스크립트가 따로 정하지 않는 이유: 측정할 때의 레일 거동이 실제 운전과
    같아야 한다. 컨트롤러가 쓰는 값을 그대로 가져온다.
    """
    import yaml
    with open(path, encoding="utf-8") as fp:
        doc = yaml.safe_load(fp)
    prm = doc["/pack2/pp_controller"]["ros__parameters"]
    sc = prm["Sensor_calibration"]
    lp = prm.get("LinePID", {})

    def gains(side, kp, ki, kd):
        d = lp.get(side, {})
        return {"kp": float(d.get("kp", kp)), "ki": float(d.get("ki", ki)),
                "kd": float(d.get("kd", kd)), "ref": float(d.get("ref", 0.0))}

    return {"offs": {int(k): float(v["offset"]) for k, v in sc["boards"].items()},
            "gains": {int(k): float(v["gain"]) for k, v in sc["boards"].items()},
            "atm": float(sc.get("atm_offset", 101.325)),
            "line_pos": gains("pos", 0.18, 0.3, 0.05),
            "line_neg": gains("neg", 0.8, 0.5, 0.01)}


# ════════════════════════════════════════════════════════════════════════════
def load_samples(dirs: list[str]) -> list[Probe]:
    """여러 실행의 samples.csv 를 읽어 Probe 로 되살린다.

    축을 따로따로 돌렸을 때 결과를 묶어 비교·피팅하려면 필요하다. 원본은 회차별
    확정값이므로 중앙값·표준편차를 그대로 다시 계산할 수 있다.

    dirs 는 ~/result/deadzone_<시각> 디렉터리 또는 samples.csv 경로.
    """
    import csv
    slot_of = {"micro": V_MICRO, "atm": V_ATM, "macro": V_MACRO}
    acc: dict[tuple, Probe] = {}
    n_row = 0
    for d in dirs:
        path = d if d.endswith(".csv") else os.path.join(d, "samples.csv")
        if not os.path.exists(path):
            print(f"  건너뜀 — {path} 가 없다")
            continue
        with open(path, encoding="utf-8") as fp:
            for row in csv.DictReader(fp):
                gid = int(row["gid"])
                slot = slot_of[row["valve"]]
                dp_t = float(row["dp_target_kpa"])
                key = (gid, slot, dp_t)
                if key not in acc:
                    is_pos = row["side"] == "pos"
                    acc[key] = Probe(gid, gid % N_AXES, is_pos, slot, dp_t)
                acc[key].samples.append((float(row["u_pct"]),
                                         float(row["dp_actual_kpa"]),
                                         float(row["p_chamber_kpa"])))
                n_row += 1
        print(f"  읽음 — {path}")
    out = list(acc.values())
    for pr in out:
        pr.finalize()
    print(f"  회차 표본 {n_row} 개 → 조합 {len(out)} 개 "
          f"(채널 {len({p.gid for p in out})}개)")
    return out


def parse_axes(text: str) -> list[int]:
    """"1,2,5" → [0,1,4] (사용자는 1-based 축 번호로 말한다)."""
    out = set()
    for tok in text.replace(" ", "").split(","):
        if not tok:
            continue
        if "-" in tok:
            a, _, b = tok.partition("-")
            out.update(range(int(a), int(b) + 1))
        else:
            out.add(int(tok))
    bad = [a for a in out if not 1 <= a <= N_AXES]
    if bad:
        raise ValueError(f"축은 1~{N_AXES} 다: {sorted(bad)}")
    return sorted(a - 1 for a in out)


def parse_args():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_argument_group("오프라인 비교 (하드웨어 없이)")
    g.add_argument("--compare", nargs="+", metavar="DIR",
                   help="이미 측정한 결과 디렉터리들(~/result/deadzone_*)의 samples.csv 를 "
                        "읽어 묶어서 비교·피팅한다. 축을 따로 돌렸을 때 쓴다. "
                        "--write 를 같이 주면 묶은 결과로 yaml 을 갱신한다. "
                        "이 모드는 하드웨어에 접속하지 않는다")
    g.add_argument("--only-gid", default=None,
                   help="비교·피팅에 쓸 채널만 고른다 (쉼표, gid). 예: 한 채널의 곡선을 "
                        "기준으로 삼아 --write-scope side 로 그 측 전체에 적용할 때")

    g = ap.add_argument_group("측정 범위")
    g.add_argument("--axes", default="1-6", help="측정할 축 (1-based). 기본 1-6")
    g.add_argument("--side", default="both", choices=("pos", "neg", "both"),
                   help="양압/음압/양쪽. 기본 both — 양압을 전부 끝내고 음압으로 넘어간다")
    g.add_argument("--channel-batch", type=int, default=1,
                   help="한 번에 측정할 채널 수. 기본 1 = **한 채널씩 끝내고 다음으로**. "
                        "채널을 여럿 동시에 세우면 레일이 무너져 측정이 깨진다. "
                        "레일에 여유가 있으면 2~3 으로 올려 시간을 줄인다")
    g.add_argument("--prewarm-sec", type=float, default=0.0,
                   help="각 채널 묶음을 재기 **직전**에 그 채널의 두 밸브를 활짝 열어 "
                        "이 시간만큼 데운다 [s]. 0 = 끔. "
                        "데드존은 열 상태에 따라 움직인다 — 20260911 실측에서 예열 전후로 "
                        "양압 micro 가 4.7~7.2 %%p 이동했다. 전 채널 측정은 40 분이 넘어 "
                        "처음 한 번 데운 것만으로는 뒤쪽 채널이 식는다. "
                        "두 밸브를 함께 열면 챔버는 레일과 대기 **사이**에 갇히므로 "
                        "과압 위험은 없다 (valve_warmup.py 의 full 방식과 같다)")
    g.add_argument("--prewarm-pct", type=float, default=85.0,
                   help="예열 지령 [%%]. 100 %% = 250 mA, 발열은 I²R 이다")
    g.add_argument("--prepos-batch", type=int, default=1,
                   help="위치잡기를 한 번에 몇 채널씩 하나. --channel-batch 를 올려도 "
                        "이건 1 로 두면 채우는 단계만 직렬화된다 (레일 보호)")
    g.add_argument("--parallel-sides", action="store_true",
                   help="양압과 음압을 동시에 측정한다 (시간 절반). 기본은 순차 — "
                        "중간에 끊겨도 한쪽은 완결된 표가 남는다")
    g.add_argument("--valves", default="micro,atm",
                   help="측정할 밸브. 기본 micro,atm (macro 는 안 쓴다)")
    g.add_argument("--dp", default="15,25,35,45,55,65,75",
                   help="측정할 차압 [kPa], 쉼표. 기본값은 레일 180/15 · 챔버 상한 188 "
                        "에서 양압·음압 네 밸브가 **모두** 닿는 범위다 (양압 micro 는 "
                        "레일 180 에서 dp 2~77, 음압 micro 는 레일 15 에서 dp 2~85)")
    g.add_argument("--dp-micro", default=None,
                   help="micro 만 다른 차압으로 (없으면 --dp). 레일을 높이면 micro 는 "
                        "높은 차압까지 가는데 atm 은 챔버 상한에 묶여 못 간다 — "
                        "레일 300 이면 micro 는 dp 100~197, atm 은 최대 94")
    g.add_argument("--dp-atm", default=None,
                   help="atm 만 다른 차압으로 (없으면 --dp)")

    g = ap.add_argument_group("레일")
    g.add_argument("--rail-pos", type=float, default=180.0,
                   help="양압 레일 목표 [kPa]. 측정 가능한 최대 차압을 정한다 "
                        "(양압 micro 는 레일−대기 = dp 상한). 운전값(yaml 130)이 아니라 "
                        "높게 잡아야 큰 차압까지 잴 수 있다")
    g.add_argument("--rail-neg", type=float, default=15.0,
                   help="음압 레일 목표 [kPa]. 낮출수록 음압 micro 의 차압 상한이 올라간다 "
                        "(음압은 대기가 유일한 고압측이라 어느 밸브든 dp < 101 이다)")
    g.add_argument("--rail-kp", type=float, default=None,
                   help="레일 PID 비례이득. 기본은 yaml LinePID 값 — 레일이 목표까지 "
                        "굼뜨면 여기서 올린다")
    g.add_argument("--rail-ki", type=float, default=None,
                   help="레일 PID 적분이득. 방출 밸브가 닫히는 속도를 지배한다 "
                        "(yaml 값 0.3 이면 오차 15 에서 22 초)")
    g.add_argument("--rail-kd", type=float, default=None, help="레일 PID 미분이득")
    g.add_argument("--rail-ramp", type=float, default=6.0,
                   help="레일 목표를 끌어올리는 속도 [kPa/s]. 낮을수록 오버슛이 적고 "
                        "도달이 느리다. 대기압→180 이면 약 %.0f s" % (79.0 / 6.0))
    g.add_argument("--rail-lead", type=float, default=1.0e6,
                   help="내부 목표가 실제 압력보다 앞설 수 있는 최대치 [kPa]. "
                        "기본은 사실상 **해제**다 — 15 로 묶었더니 오차가 그만큼으로 "
                        "제한돼 적분이 방출 밸브를 닫는 데 22 초가 걸렸다 (ki=0.3 에 "
                        "필요한 적분 330 ÷ 오차 15). 적분 폭주는 조건부 적분이 이미 "
                        "막으므로 이중으로 걸 이유가 없다")
    g.add_argument("--idle-open-pct", type=float, default=100.0,
                   help="쉬는 쪽 레일 밸브의 개도 [%%]. 기본 100 = 활짝. 양압을 잴 때 "
                        "음압 유입을 활짝 열면 펌프 흡입압이 대기압이라 토출이 최대가 "
                        "되고, 음압을 잴 때 양압 방출을 활짝 열면 압축비가 최소라 "
                        "가장 깊은 진공이 나온다")
    g.add_argument("--rail-tol", type=float, default=5.0,
                   help="레일이 목표의 이 안에 들어오면 측정을 시작한다 [kPa]")
    g.add_argument("--rail-wait", type=float, default=120.0,
                   help="레일이 목표에 오를 때까지 기다리는 **최대** 시간 [s]. "
                        "먼저 들어오면 바로 시작한다")

    g = ap.add_argument_group("램프")
    g.add_argument("--u-coarse", type=float, default=2.0, help="굵은 훑기 간격 [%%p]")
    g.add_argument("--u-coarse-hold", type=float, default=0.30, help="굵은 훑기 홀드 [s]")
    g.add_argument("--u-step", type=float, default=0.25, help="정밀 램프 간격 [%%p]")
    g.add_argument("--u-hold", type=float, default=0.50, help="정밀 램프 홀드 [s]")
    g.add_argument("--u-max", type=float, default=80.0, help="여기까지 안 열리면 포기 [%%]")
    g.add_argument("--rate-thresh", type=float, default=0.40,
                   help="'열렸다' 판정 변화율 [kPa/s] — 자연 드리프트에 더해서 본다")
    g.add_argument("--rate-thresh-coarse", type=float, default=1.20,
                   help="굵은 훑기의 판정 변화율 [kPa/s]")
    g.add_argument("--repeats", type=int, default=5,
                   help="차압마다 몇 번 반복 측정하나. 중앙값으로 확정하고 퍼짐을 함께 "
                        "보고한다. 2회차부터는 굵은 훑기를 건너뛴다")
    g.add_argument("--refine-span", type=float, default=3.0,
                   help="반복 회차의 정밀 램프 출발점 [%%p]. 지금까지 중앙값에서 "
                        "이만큼 아래에서 다시 올린다")
    g.add_argument("--confirm", type=int, default=2,
                   help="정밀 램프에서 연속 몇 스텝 넘어야 확정하나. 기본 2")

    g = ap.add_argument_group("위치잡기·안정화")
    g.add_argument("--drive-pct", type=float, default=70.0,
                   help="위치잡기에 쓰는 밸브 지령 [%%]")
    g.add_argument("--full-duty-kpa", type=float, default=6.0,
                   help="이 오차 이상이면 밸브를 계속 연다 [kPa]. 그보다 작으면 "
                        "오차에 비례해 **여는 시간**을 줄인다 (지령은 안 낮춘다 — "
                        "데드존 아래로 내려가면 아예 안 움직인다)")
    g.add_argument("--pulse-sec", type=float, default=0.25,
                   help="위치잡기 펄스 주기 [s]. 밸브 시정수(25 ms)보다 충분히 커야 한다")
    g.add_argument("--min-duty", type=float, default=0.06,
                   help="최소 duty. 이보다 짧게 열면 밸브가 다 열리기 전에 닫힌다")
    g.add_argument("--accept-tol", type=float, default=4.0,
                   help="목표에 못 가고 멈췄을 때, 이 안이면 **그대로 측정한다** [kPa]. "
                        "표는 실제 차압으로 인덱싱하므로 정확한 목표압이 필요없다")
    g.add_argument("--stall-kpa", type=float, default=0.4,
                   help="이보다 작게 움직이면 '멈췄다' 로 본다 [kPa]")
    g.add_argument("--stall-sec", type=float, default=3.0,
                   help="이 시간 동안 안 움직이면 위치잡기를 접는다 [s]")
    g.add_argument("--sp-tol", type=float, default=1.5, help="위치잡기 허용 오차 [kPa]")
    g.add_argument("--sp-hold", type=float, default=0.5, help="허용 오차 안에 머물 시간 [s]")
    g.add_argument("--prepos-timeout", type=float, default=45.0)
    g.add_argument("--settle-rate", type=float, default=0.30,
                   help="이 변화율 아래면 '멈췄다' [kPa/s]")
    g.add_argument("--settle-window", type=float, default=0.8)
    g.add_argument("--settle-timeout", type=float, default=20.0)
    g.add_argument("--baseline", type=float, default=2.0,
                   help="자연 드리프트를 재는 시간 [s]")
    g.add_argument("--dp-tol", type=float, default=5.0,
                   help="이보다 더 벗어나 있으면 위치잡기 실패로 표시 [kPa]")

    g = ap.add_argument_group("피팅")
    g.add_argument("--fit-order", type=int, default=4,
                   help="검토할 **최대** 차수. 1차부터 이 차수까지 전부 피팅해 "
                        "leave-one-out 교차검증 오차가 가장 작고 표 전 구간에서 단조 "
                        "감소인 차수를 고른다 (RMSE 로 고르면 과적합을 못 잡는다). "
                        "자유도가 모자란 차수는 건너뛴다")
    g.add_argument("--fit-step", type=float, default=5.0,
                   help="피팅 곡선을 yaml 표로 되돌릴 때의 차압 간격 [kPa]")
    g.add_argument("--table-range", default=None, metavar="LO,HI",
                   help="표를 내보낼 차압 범위 [kPa] — 측정 범위 밖까지 **외삽**한다. "
                        "예 0,105 (레일 200 이면 양압 micro 차압이 99 까지 간다). "
                        "생략하면 측정 범위까지만 만들고 그 밖은 컨트롤러가 클램프한다. "
                        "단조성은 이 전 구간에서 검사하므로 외삽에서 되돌아 올라가는 "
                        "차수는 자동으로 탈락한다")

    g = ap.add_argument_group("안전·출력")
    g.add_argument("--p-max", type=float, default=188.0,
                   help="챔버 **측정 범위** 상한 [kPa abs] — 과압 트립이 아니다. "
                        "챔버 목표압을 이 안에서 고르고, 램프가 넘어가면 그 조합만 접는다")
    g.add_argument("--p-min", type=float, default=15.0, help="챔버 측정 범위 하한 [kPa abs]")
    g.add_argument("--fill-headroom", type=float, default=2.0,
                   help="챔버 목표압을 레일에서 이만큼 띄운다 [kPa]. 안전 마진이 아니라 "
                        "도달 시간 문제다 — 레일 바로 밑은 차압이 0 이라 채우는 데 "
                        "무한히 걸린다")
    g.add_argument("--yaml", default=DEFAULT_YAML)
    g.add_argument("--write", action="store_true",
                   help="측정 결과를 yaml 에 반영한다 (원본은 백업). 이번에 **안 잰** "
                        "채널의 표는 그대로 둔다 — 끊어서 여러 번 돌려도 안전하다")
    g.add_argument("--valve-table", default="separate",
                   choices=("separate", "merged", "atm", "micro"),
                   help="micro 와 atm 은 같은 부품이다. separate = 따로 (기본), "
                        "merged = 두 결과를 묶어 하나로 피팅해 둘 다에 쓴다, "
                        "atm/micro = 그 한쪽의 곡선을 둘 다에 쓴다. "
                        "결과의 'micro vs atm' 절에서 차이가 회차 잡음 수준으로 나오면 "
                        "merged 가 낫고, 계통 차이면 separate 를 유지한다")
    g.add_argument("--write-scope", default="channel", choices=("channel", "side"),
                   help="channel = 잰 채널의 표만 갱신. "
                        "side = 잰 채널들을 측(양압/음압)별로 묶어 하나로 피팅해 "
                        "valve_deadzone.{pos,neg} 에 쓴다 — 채널별 표가 없는 채널이 "
                        "그 표를 쓰므로, 채널 차이가 작으면 한두 채널만 재고 끝낼 수 있다")
    g.add_argument("--yes", action="store_true", help="확인 프롬프트를 건너뛴다")
    return ap.parse_args()


def wait_rails(rig: Rig, args, cfg) -> None:
    """활성 레일이 목표에 들어올 때까지 기다린다.

    쉬는 쪽은 목표가 없으므로 **기다리지 않는다** — 활짝 열려 대기압으로 가 있는 게
    정상이다. 측정 단계가 바뀔 때(양압→음압)마다 다시 부른다: 음압 레일은 그때까지
    대기압에 있었으므로 새로 끌어내려야 한다.
    """
    want_pos = rig.priority != "neg"
    want_neg = rig.priority != "pos"
    who = " / ".join(([f"양압 {args.rail_pos:.0f}"] if want_pos else [])
                     + ([f"음압 {args.rail_neg:.0f}"] if want_neg else []))
    print(f"\n레일을 목표로 잡는다 ({who} kPa, 최대 {args.rail_wait:.0f} s)")
    print(f"  게인 (yaml LinePID + 인자): 양압 {cfg['line_pos']}, 음압 {cfg['line_neg']}")
    rig.close_all_channels()
    deadline = time.monotonic() + args.rail_wait
    last = 0.0
    while time.monotonic() < deadline:
        spin(rig, 0.5)
        p_pos, p_neg = rig.kpa(RAIL_POS_BOARD), rig.kpa(RAIL_NEG_BOARD)
        if p_pos is None or p_neg is None:
            continue
        if time.monotonic() - last > 3.0:
            print("    " + rig.rail_status())
            last = time.monotonic()
        ok_p = (not want_pos) or abs(p_pos - args.rail_pos) <= args.rail_tol
        ok_n = (not want_neg) or abs(p_neg - args.rail_neg) <= args.rail_tol
        if ok_p and ok_n:
            break
    p_pos, p_neg = rig.kpa(RAIL_POS_BOARD), rig.kpa(RAIL_NEG_BOARD)
    print("  " + rig.rail_status())
    # 목표에 못 갔으면 잴 수 있는 차압이 줄어든다. 멈추지는 않는다 — 불가능한 조합은
    # measure_group 이 이유를 찍고 건너뛴다.
    if want_pos and (p_pos is None or p_pos < args.rail_pos - args.rail_tol):
        print(f"  경고: 양압 레일이 목표에 못 미친다 (펌프 한계). "
              f"양압 micro 는 최대 차압 {(p_pos or 0) - rig.atm:.0f} kPa 까지만 측정된다.")
    if want_neg and (p_neg is None or p_neg > args.rail_neg + args.rail_tol):
        print(f"  경고: 음압 레일이 목표에 못 미친다. "
              f"음압 밸브는 최대 차압 {rig.atm - (p_neg or 0):.0f} kPa 까지만 측정된다.")


def main() -> int:
    args = parse_args()

    if isinstance(args.table_range, str):
        lo, hi = (float(v) for v in args.table_range.replace(" ", "").split(","))
        args.table_range = (lo, hi)

    # ── 오프라인 비교 모드 ──────────────────────────────────────────────────
    if args.compare:
        print(f"기존 결과 {len(args.compare)} 개를 묶는다")
        results = load_samples(args.compare)
        if args.only_gid:
            keep = {int(x) for x in args.only_gid.replace(" ", "").split(",") if x}
            results = [p for p in results if p.gid in keep]
            print(f"  gid {sorted(keep)} 만 사용 → 조합 {len(results)} 개")
        if not results:
            print("읽을 표본이 없다")
            return 1
        txt = (report(results, 101.325) + "\n" + compare_valves(results)
               + "\n" + compare_channels(results))
        print(txt)
        block = yaml_block(results, args, read_existing(args.yaml))
        out = os.path.join(args.compare[0], "merged_deadzone.yaml")
        with open(out, "w", encoding="utf-8") as fp:
            fp.write(block)
        print(f"\n묶은 표  {out}")
        if args.write:
            try:
                backup = write_yaml(args.yaml, block)
                print(f"{args.yaml} 갱신 (백업 {backup})")
            except RuntimeError as exc:
                print(exc)
                return 1
        else:
            print("yaml 에 반영하려면 --write")
        return 0

    axes = parse_axes(args.axes)
    def _dps(text):
        return [float(x) for x in text.replace(" ", "").split(",") if x]
    dps = _dps(args.dp)
    dps_for = {V_MICRO: _dps(args.dp_micro) if args.dp_micro else dps,
               V_ATM:   _dps(args.dp_atm)   if args.dp_atm   else dps}
    slots = [{"micro": V_MICRO, "atm": V_ATM}[v]
             for v in args.valves.replace(" ", "").split(",") if v]
    sides = {"pos": [True], "neg": [False], "both": [True, False]}[args.side]

    cfg = read_config(args.yaml)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir = os.path.join(RESULT_ROOT, f"deadzone_{stamp}")
    os.makedirs(outdir, exist_ok=True)

    n = len(axes) * len(sides) * sum(len(dps_for[sl]) for sl in slots)
    print(f"밸브 데드존 측정 — 축 {[a+1 for a in axes]}, "
          f"{'/'.join('양압' if s else '음압' for s in sides)}, "
          f"밸브 {args.valves}, 차압 "
          + " / ".join(f"{ROLE[sl]} {dps_for[sl]}" for sl in slots)
          + f" kPa → {n} 조합")
    print(f"레일 목표 {args.rail_pos:.0f} / {args.rail_neg:.0f} kPa, 로그 {outdir}")
    if not args.yes:
        print("\n확인: 펌프가 **켜져** 있고 pp_controller 는 **꺼져** 있어야 한다.")
        if input("진행할까? [y/N] ").strip().lower() not in ("y", "yes"):
            print("취소")
            return 1

    rclpy.init()
    # 레일 게인은 yaml LinePID 가 기본이고, 인자로 덮어쓸 수 있다.
    for side in ("line_pos", "line_neg"):
        for key, val in (("kp", args.rail_kp), ("ki", args.rail_ki), ("kd", args.rail_kd)):
            if val is not None:
                cfg[side][key] = float(val)
    rig = Rig(cfg, args.rail_pos, args.rail_neg, args.p_max, args.rail_ramp,
              args.rail_lead, args.idle_open_pct)
    log = Log(os.path.join(outdir, "steps.csv"))
    results: list[Probe] = []
    try:
        spin(rig, 1.5)
        if rig.n_msg == 0:
            print("board/sensors 를 못 받았다 — can_bridge 가 돌고 있나?")
            return 1
        rivals = rig.rival_publishers()
        if rivals:
            print(f"board/pwm_cmd 에 다른 발행자가 있다: {rivals}\n"
                  f"pp_controller 를 끄고 다시 실행해라 (명령이 번갈아 나간다).")
            return 1

        # 기본은 **양압 전부 → 음압 전부** 순서다. 중간에 끊기거나 문제가 생겨도
        # 한쪽은 완결된 표가 남는다. --parallel-sides 를 주면 둘을 같이 돌아 시간이
        # 절반이 된다 (레일이 따로고 챔버도 독립이라 간섭은 없다).
        side_groups = [sides] if args.parallel_sides else [[x] for x in sides]
        for grp in side_groups:
            label = "/".join("양압" if x else "음압" for x in grp)
            print(f"\n{'='*66}\n  {label} 시작\n{'='*66}")
            # 쉬는 쪽 레일은 목표를 버리고 밸브를 활짝 연다 (publish 참조).
            rig.priority = None if len(grp) != 1 else ("pos" if grp[0] else "neg")
            wait_rails(rig, args, cfg)
            chans = [(a, is_pos) for is_pos in grp for a in axes]
            nb = max(1, args.channel_batch)
            for k in range(0, len(chans), nb):
                batch = chans[k:k + nb]
                gids = [(a if is_pos else N_AXES + a) for a, is_pos in batch]
                print(f"\n{'─'*66}\n  채널 {', '.join('ch%d' % g for g in gids)} "
                      f"({k+1}~{k+len(batch)} / {len(chans)})\n{'─'*66}")
                if args.prewarm_sec > 0:
                    # 두 밸브를 함께 열어 코일만 데운다. 압력은 레일과 대기 사이에
                    # 갇히므로 위험하지 않고, 어차피 바로 위치잡기를 다시 한다.
                    print(f"    예열 {args.prewarm_sec:g} s @ {args.prewarm_pct:g} % "
                          f"({args.prewarm_pct*2.5:.0f} mA)", flush=True)
                    for a_, is_pos_ in batch:
                        bd = (POS_BOARD0 if is_pos_ else NEG_BOARD0) + a_
                        rig.set_valve(bd, V_MICRO, args.prewarm_pct)
                        rig.set_valve(bd, V_ATM, args.prewarm_pct)
                    spin(rig, args.prewarm_sec)
                    rig.close_all_channels()
                    spin(rig, 2.0)
                for slot in slots:
                    for dp in dps_for[slot]:
                        group = [Probe(a if is_pos else N_AXES + a, a, is_pos, slot, dp)
                                 for a, is_pos in batch]
                        print(f"\n[{label} {ROLE[slot]} dp={dp:.0f} kPa]  "
                              f"{rig.rail_status()}")
                        # 먼저 등록한다 — 이 그룹 도중에 Ctrl+C 로 끊겨도 그때까지
                        # 모은 회차 표본이 결과에 남는다.
                        results.extend(group)
                        measure_group(rig, group, args, log)

    except KeyboardInterrupt:
        print("\n중단 — 밸브를 닫고 결과를 저장한다")
    except RuntimeError as exc:
        print(f"\n{exc}")
    finally:
        rig.close_all_channels()
        spin(rig, 0.5)
        log.close()

    # 중간에 끊긴 그룹은 finalize 를 못 거쳤다 — 여기서 마저 확정한다.
    for pr in results:
        if pr.samples and math.isnan(pr.u_dead):
            pr.finalize()
    txt = (report(results, cfg["atm"]) + "\n" + compare_valves(results)
           + "\n" + compare_channels(results))
    print(txt)
    with open(os.path.join(outdir, "report.txt"), "w", encoding="utf-8") as fp:
        fp.write(txt + "\n")
    # 회차별 확정값을 따로 뽑아 둔다 — 분산을 바로 보려면 이 파일이 제일 편하다.
    with open(os.path.join(outdir, "samples.csv"), "w", encoding="utf-8") as fp:
        fp.write("gid,side,valve,dp_target_kpa,rep,u_pct,dp_actual_kpa,p_chamber_kpa\n")
        for pr in results:
            for k, (u, dpa, pa) in enumerate(pr.samples, 1):
                fp.write(f"{pr.gid},{'pos' if pr.is_pos else 'neg'},{ROLE[pr.slot]},"
                         f"{pr.dp_target:.1f},{k},{u:.3f},{dpa:.2f},{pa:.2f}\n")
    block = yaml_block(results, args, read_existing(args.yaml))
    with open(os.path.join(outdir, "deadzone.yaml"), "w", encoding="utf-8") as fp:
        fp.write(block)
    print(f"\n로그  {outdir}/steps.csv")
    print(f"결과  {outdir}/report.txt")
    print(f"yaml  {outdir}/deadzone.yaml")

    if args.write:
        try:
            backup = write_yaml(args.yaml, block)
            print(f"\n{args.yaml} 갱신 (백업 {backup})")
            print("컨트롤러를 다시 띄우면 반영된다 (재빌드 불필요 — config 는 런타임 로드).")
        except RuntimeError as exc:
            print(f"\n{exc}")
            return 1
    else:
        print("\nyaml 에 반영하려면 --write. 지금은 읽기 전용이다.")

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
