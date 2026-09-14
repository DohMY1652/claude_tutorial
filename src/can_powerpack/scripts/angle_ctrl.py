#!/usr/bin/env python3
"""각도 피드백 제어 — 측정 맵을 피드포워드로 쓰고 PID 는 잔차만 맡는다.

구조
----
    차압_ff  = g(목표각)                    측정 맵의 역함수 (올림·내림 중앙값)
    차압     = 차압_ff + kp·e + ki·∫e       e = 목표각 − 실측각
    P+ = 중앙 + 차압/2 ,  P− = 중앙 − 차압/2

압력 자체는 **기존 채널 PID** 가 잡는다 (TCP 로 목표압만 보낸다). 이 스크립트는
그 위에 각도 외부 루프를 얹을 뿐이라, 이미 튜닝된 압력 제어를 건드리지 않는다.

왜 느리게 짰나
--------------
액추에이터가 진동하면 고장난다고 했다. 그래서 **속도를 구조적으로 막는다**:

  1. 목표각을 계단으로 안 준다 — `--ang-slew` [°/s] 로 기울인다
  2. 출력 차압도 계단이 안 된다 — `--diff-slew` [kPa/s] 로 제한한다
  3. 게인이 낮다. 피드포워드가 동작점을 만들므로 PID 는 표 오차만 메우면 된다
  4. **적분 데드밴드** — 실측 마찰 밴드(차압 기준 7.5~12 kPa, 각도로 약 ±6°)
     안에서는 적분을 멈춘다. 마찰이 붙잡고 있는 오차를 적분이 밀면 stick-slip
     한계주기가 생긴다. 밸브가 만들 수 없는 정밀도를 요구하지 않는다.
  5. 미분항은 **없다.** 마찰이 있는 축의 각속도를 미분하면 잡음만 증폭된다.

안전
----
  · 각도 한계 — 기본 상한 85°. **90° 를 넘으면 중력 복원 토크가 줄어들어
    정적으로 불안정하다** (각도↑ → 복원토크↓ → 스톱까지 간다). 목표도 거기서 자른다.
  · 각속도 상한, 진동(부호 반전) 감시
  · 압력 추종 실패 감시 — 채널이 목표압을 못 잡으면 더 밀지 않는다
  · 어떤 경로로 끝나든 **양 챔버를 대기압까지 기울여 내린다**

사용
----
    # 맵을 읽어 피드포워드를 만들고, 목표각을 대화형으로 준다
    python3 angle_ctrl.py --axis 1 --map ~/result/actuator_map_A.csv,~/result/actuator_map_B.csv

    # 정해진 각도열을 자동으로
    python3 angle_ctrl.py --axis 1 --map ... --targets 20,40,60,75,60,40,20
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import socket
import sys
import threading
import time
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, UInt16MultiArray

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
from pressure_sweep_server import (ATM_KPA, NUM_AXES, _connect,  # noqa: E402
                                    encode_refs)
from valve_deadzone import read_config                                            # noqa: E402

NAMESPACE = "/pack2"
G = 9.81
POS_BOARD = lambda a: 5 + a       # noqa: E731
NEG_BOARD = lambda a: 11 + a      # noqa: E731


class Abort(Exception):
    pass


# ════════════════════════════════════════════════════════════════════════════
#  피드포워드 — 측정 맵의 역함수
# ════════════════════════════════════════════════════════════════════════════
def load_map(paths: str) -> tuple[list[float], list[float], dict]:
    """actuator_map CSV 들에서 **각도 → 차압** 곡선을 만든다.

    올림과 내림을 **각도 기준으로 평균**한다. 같은 각도를 만드는 데 필요한 차압이
    올릴 때와 내릴 때 다른데(실측 7.5~12 kPa), 그 차이가 곧 마찰이고 중앙값이
    마찰을 상쇄한 참값이다. 압력 기준으로 짝지으면 안 된다 — 같은 압력에서
    각도가 다르기 때문이다.
    """
    up: list[tuple[float, float]] = []   # (각도, 차압)
    dn: list[tuple[float, float]] = []
    for p in paths.split(","):
        p = os.path.expanduser(p.strip())
        if not p:
            continue
        with open(p, newline="", encoding="utf-8") as fh:
            for r in csv.DictReader(fh):
                if not int(r.get("settled", 1)):
                    continue
                # **지령이 아니라 실제로 걸린 차압**을 쓴다. 1축은 추종오차가
                # 0.3 kPa 안이라 차이가 없었지만, 2축은 P+ 가 지령보다 최대
                # 4.6 kPa 모자랐다(20260914_100329). 지령을 쓰면 맵이 "이만큼
                # 필요하다" 를 과대평가해 피드포워드가 통째로 틀어진다.
                try:
                    d_ = float(r["p_pos_meas"]) - float(r["p_neg_meas"])
                except (KeyError, TypeError, ValueError):
                    d_ = float(r["diff_kpa"])
                pair = (float(r["angle_deg"]), d_)
                (up if r["sweep"] == "up" else dn).append(pair)
    if len(up) < 2 or len(dn) < 2:
        raise SystemExit("[중단] 올림·내림 두 곡선이 다 있어야 마찰을 뗄 수 있다.")
    up.sort(); dn.sort()

    def itp(xy, x):
        xs = [a for a, _ in xy]; ys = [b for _, b in xy]
        if x <= xs[0]: return ys[0]
        if x >= xs[-1]: return ys[-1]
        for i in range(1, len(xs)):
            if xs[i] >= x:
                f = (x - xs[i-1]) / max(1e-9, xs[i] - xs[i-1])
                return ys[i-1] + f * (ys[i] - ys[i-1])
        return ys[-1]

    lo = max(up[0][0], dn[0][0]); hi = min(up[-1][0], dn[-1][0])
    ang = [lo + (hi - lo) * k / 40.0 for k in range(41)]
    diff = [(itp(up, a) + itp(dn, a)) / 2.0 for a in ang]
    # 각도별 마찰 **반폭**. 방향 피드포워드와 적분 데드밴드에 쓴다.
    fr = [abs(itp(up, a) - itp(dn, a)) / 2.0 for a in ang]
    info = {"lo": lo, "hi": hi, "fric_kpa": sum(fr) / len(fr),
            "n_up": len(up), "n_dn": len(dn)}
    return ang, diff, info, fr


def _refs_axes(per_axis: dict) -> tuple:
    """축마다 **다른** 목표압을 넣는다. 나머지 채널은 대기압.

    pressure_sweep_server 의 _refs 는 그룹 전체에 같은 값을 넣으므로 못 쓴다 —
    두 축이 같은 각도를 만들려면 서로 다른 차압이 든다 (맵이 다르다).
    """
    pos = [ATM_KPA] * NUM_AXES
    neg = [ATM_KPA] * NUM_AXES
    for ax0, (pp, nn) in per_axis.items():
        pos[ax0], neg[ax0] = pp, nn
    return tuple(pos + neg)


def interp(xs: list[float], ys: list[float], x: float) -> float:
    if x <= xs[0]: return ys[0]
    if x >= xs[-1]: return ys[-1]
    for i in range(1, len(xs)):
        if xs[i] >= x:
            f = (x - xs[i-1]) / max(1e-9, xs[i] - xs[i-1])
            return ys[i-1] + f * (ys[i] - ys[i-1])
    return ys[-1]


# ════════════════════════════════════════════════════════════════════════════
def _finish(sw, idx, tgt, arrived, arrive_s, chans, hold_buf):
    """한 목표를 끝낼 때 한 줄 요약. **추종했는지가 여기서 보인다.**"""
    row = [idx + 1, f"{tgt:.2f}", int(arrived), f"{arrive_s:.1f}"]
    flag = "" if arrived else "   ← **미도착**"
    print(f"      [결과] 목표 {tgt:5.1f}°{flag}", flush=True)
    for c in chans:
        buf = hold_buf.get(c.axis) or [c.ang if c.ang is not None else tgt]
        n = max(1, len(buf))
        m = sum(buf) / n
        sd = math.sqrt(sum((x - m) ** 2 for x in buf) / n)
        # 적분이 잔류 오차를 닫는 데 몇 초가 걸리므로 유지구간 **전체** 평균에는
        # 그 보정 과도가 섞인다. 실제로 수렴한 값은 뒤쪽 1/3 이다 — 둘을 같이 낸다.
        tail = buf[-max(1, n // 3):]
        me = sum(tail) / len(tail)
        tau = 2.0 * G * 0.15 * math.sin(math.radians(m))
        row += [f"{m:.3f}", f"{sd:.3f}", f"{m-tgt:+.3f}", f"{me-tgt:+.3f}",
                f"{c.peak_err:.2f}", f"{c.diff_cmd:.2f}", f"{tau:.4f}"]
        print(f"        [{c.name}] 유지평균 {m:6.2f}° (σ {sd:.3f})  "
              f"오차 {m-tgt:+5.2f}° (뒤1/3 {me-tgt:+5.2f}°)  "
              f"최대편차 {c.peak_err:5.2f}°  차압 {c.diff_cmd:5.1f}", flush=True)
    sw.writerow(row)


class Chan:
    """축 하나의 맵·게인·상태·제어.

    **레퍼런스(목표각과 그 슬루)는 바깥에서 하나만 만들어 모든 축에 같이 준다.**
    같은 타이밍에 같은 각도를 따라가게 하는 것이 목적이므로 램프는 공유하고,
    맵·마찰·적분처럼 축마다 다른 것만 여기에 둔다. 축이 서로 다른 맵을 갖는 것이
    정상이다 — 2축은 1축보다 뻣뻣해서 같은 각도에 더 큰 차압이 든다.
    """

    def __init__(self, axis: int, enc_idx: int, map_paths: str, args) -> None:
        self.axis, self.enc_idx = axis, enc_idx
        self.ang_tab, self.diff_tab, self.info, self.fric_tab = load_map(map_paths)
        # 마찰 폭(차압)을 각도로 환산한다. 국소 기울기 dθ/d차압 의 평균을 쓴다.
        self.slope = ((self.diff_tab[-1] - self.diff_tab[0])
                      / max(1e-9, self.ang_tab[-1] - self.ang_tab[0]))   # kPa/°
        self.i_deadband = (args.i_deadband if args.i_deadband is not None else
                           round(self.info["fric_kpa"] / max(1e-9, self.slope)
                                 * args.i_deadband_frac, 2))
        # ── 축 사이 게인 환산 ──────────────────────────────────────────
        # 게인이 전부 **kPa 당 도(度)** 로 정의돼 있어 플랜트 기울기가 다른 축에
        # 그대로 못 간다. 축2 는 1.26 kPa/° 로 축1(0.88)보다 43 % 뻣뻣하다.
        # 같은 값을 쓰면 축2 는 그만큼 굼뜨고, 축2 에 맞추면 축1 이 그만큼 세진다.
        #
        # 그래서 인자는 **--gain-ref-slope 에서 잰 값**으로 해석하고 축마다
        # 기울기 비로 환산한다. 각도로 본 거동(°/s, °)이 축에 무관하게 같아진다.
        k = self.slope / max(1e-9, args.gain_ref_slope)
        self.kp = args.kp * k
        self.ki = args.ki * k
        self.i_rate_max = args.i_rate_max * k
        self.kvel_down = args.kvel_down * k
        self.diff_slew = args.diff_slew * k
        self.diff_slew_down = args.diff_slew_down * k

        # 상태
        self.diff_cmd = 0.0
        self.integ = 0.0
        self.fric_q = 0.0        # 지금 적용 중인 마찰 피드포워드 비율 (부호 포함)
        self.move_dir = 0.0      # 지금 이동의 방향 (+1 올림 / −1 내림)
        self.move_done = False   # 이번 이동의 마찰 FF 를 이미 거뒀나 (재점화 금지)
        self.slew_sat = False    # 직전 틱에 차압 슬루가 포화였나 (적분 정지 조건)
        self.ang_ref_prev = 0.0
        self.ang_prev = 0.0
        self.t_prev = 0.0
        self.rate = 0.0          # 필터링한 **각속도** [°/s]
        self.flips: deque = deque()
        self.last_sign = 0
        self.bad_since = None
        self.peak_err = 0.0
        self.ang = None
        self.err = 0.0
        self.ff = 0.0
        self.p_pos = ATM_KPA
        self.p_neg = ATM_KPA
        self.pm = self.nm = None      # 마지막으로 읽은 챔버 실측압

    # ── 진단용 이름 ────────────────────────────────────────────────────
    @property
    def name(self) -> str:
        return f"축{self.axis}"

    def start(self, rig, args, now: float) -> float:
        """시작 각도를 읽고 **지금 챔버에 실제로 걸린 차압**에서 출발시킨다."""
        a0 = rig.angle(self.enc_idx)
        if a0 is None:
            raise Abort(f"{self.name}: 각도를 못 받았다 — 엔코더를 확인할 것")
        pm0 = rig.kpa(POS_BOARD(self.axis - 1))
        nm0 = rig.kpa(NEG_BOARD(self.axis - 1))
        if pm0 is None or nm0 is None:
            raise Abort(f"{self.name}: 챔버 압력을 못 읽는다 — 브리지/CAN 확인")
        # 피드포워드 값으로 초기화하면 첫 틱에 목표압이 통째로 계단이 된다.
        # 차압 슬루는 그 뒤의 *변화*만 제한하므로 초기값은 못 막는다.
        self.diff_cmd = max(0.0, pm0 - nm0)
        self.ang = self.ang_prev = a0
        self.t_prev = now
        return a0

    def read(self, rig, args, now: float) -> None:
        """각도를 읽고 안전을 본다. 한 축이라도 걸리면 전체가 중단된다."""
        ang = rig.angle(self.enc_idx)
        if ang is None or rig.age() > args.enc_timeout:
            raise Abort(f"{self.name}: 각도 갱신이 {rig.age():.1f} s 없다")
        if now > self.t_prev:
            a = min(1.0, (now - self.t_prev) / 0.3)
            self.rate += a * ((ang - self.ang_prev) / (now - self.t_prev) - self.rate)
        self.ang_prev, self.t_prev, self.ang = ang, now, ang

        if not (args.ang_min <= ang <= args.ang_max + 3.0):
            raise Abort(f"{self.name}: 각도 {ang:.1f}° 가 한계를 벗어났다")
        if abs(self.rate) > args.max_rate:
            raise Abort(f"{self.name}: 각속도 {self.rate:+.1f} °/s > {args.max_rate:g}")
        sgn = 0 if abs(self.rate) < 0.5 else (1 if self.rate > 0 else -1)
        if sgn and self.last_sign and sgn != self.last_sign:
            self.flips.append(now)
        if sgn:
            self.last_sign = sgn
        while self.flips and now - self.flips[0] > args.osc_window:
            self.flips.popleft()
        if len(self.flips) >= args.osc_flips:
            raise Abort(f"{self.name}: {args.osc_window:g} s 안에 각속도 부호가 "
                        f"{len(self.flips)}번 뒤집혔다 — 진동으로 본다")

    def control(self, ang_ref: float, tgt: float, dt: float, args) -> None:
        ang = self.ang
        self.err = err = ang_ref - ang
        # ── 방향 마찰 피드포워드 ────────────────────────────────────
        # 마찰 피드포워드는 **움직이라는 지령**이지 **서 있으라는 지령**이 아니다.
        # 한 값으로 둘 다 하려니 계속 한쪽이 깨졌다:
        #   0.8 → 잘 움직이는 대신 멈출 때 ±1.5° 지나침
        #   0.55 → 깔끔히 멈추는 대신 내림이 슬루를 못 따라감 (실측 지연 −4.4~−5.5°)
        # 국면으로 나눈다. 램프 중에는 실측 마찰 전폭을 쓰고, 목표에 선 뒤에는
        # 마찰 밴드 **중앙**으로 흘려보낸다.
        #
        # 중앙으로 되돌리는 것이 팔을 움직이지 않는 이유: 목적지 mid(ang_ref) 가
        # 지금 각도의 마찰 밴드(±f, 각도로 ±5.2°) 안에 있다. 밴드 안에서는 정지
        # 마찰이 안 깨진다. 예전에 "튄다"고 본 것은 방향이 아니라 **계단**이어서고,
        # 시간상수를 두면 사라진다. 밴드 가장자리가 아니라 중앙에서 출발하게 되므로
        # 적분이 양쪽으로 똑같은 권한을 갖는 효과도 같이 온다.
        d_ref = (ang_ref - self.ang_ref_prev) / dt
        self.ang_ref_prev = ang_ref
        ref_moving = abs(d_ref) > 0.05 * args.ang_slew
        if ref_moving:
            self.move_dir = 1.0 if d_ref > 0 else -1.0
            self.move_done = False      # 새 이동이 시작됐다 — 빗장을 푼다
        # 내림은 부피 외란(내려오며 양압 챔버가 줄고 음압이 는다)이 늘 지령을
        # 거스르므로 올림보다 더 든다 — 실측 램프 지연 4.9° vs 0.9°. 따로 준다.
        mag = args.fric_ff if self.move_dir > 0 else args.fric_ff_down

        # **이동이 끝났는지는 레퍼런스가 아니라 팔이 결정한다.**
        # ref 가 섰다고 회수하면 내림이 끊긴다: 내림은 램프 지연이 커서 그 시점에
        # 팔이 아직 3° 위에 있고, 지령이 f(4.86 kPa) 만큼 올라가며 하강이 멎는다.
        # 그러면 적분이 그 4.86 을 혼자 다시 끌어내려야 해서 상한 0.4 kPa/s 로
        # 12초가 더 든다 (실측 i −7.35, 도착 +41 s).
        #
        # 가던 방향으로 아직 --fric-hold-band 넘게 남았으면 이동으로 본다.
        # **가던 방향으로만** 연장하고 절대 뒤집지 않는다 — 부호를 뒤집게 두면
        # 지나칠 때마다 전폭 반전이 걸려 밴드 크기의 한계주기가 된다.
        #
        # 그리고 **한 번의 이동에 한 번만** 쓴다. 위 조건만으로 두면 내림에서 팔이
        # 목표보다 --fric-hold-band 만 떠올라도 전폭 내림 FF(−f)가 다시 걸린다.
        # 내려가면 꺼지고 떠오르면 켜지는 뱅뱅이 되어 밴드 크기의 한계주기가 된다 —
        # 20260914_104223 에서 유지 중 ff 가 21.8 ↔ 17.6 을 주기 10 s 로 왕복했고,
        # 내림 구간 ang_std 가 올림의 0.010 대비 0.46 이었다. 올림에서 안 나타난
        # 것은 past_target 이 먼저 잠가서다.
        # 이동이 끝난 뒤의 잔류는 적분의 몫이다. 다음 램프가 움직일 때 빗장이 풀린다.
        still_going = (not self.move_done
                       and self.move_dir != 0.0
                       and err * self.move_dir > args.fric_hold_band)
        # 팔이 **최종 목표**를 이미 지났으면 램프가 안 끝났어도 미는 것을 멈춘다.
        # 올림이 램프보다 빨라지면서(실측 1.74 vs 1.49 °/s) ref 가 목표에 닿기
        # 전에 팔이 1.07° 지나 있는데도 ref_moving 이라는 이유로 전폭을 계속
        # 밀고 있었다. 마찰 FF 는 목표까지 데려가는 지령이지 지나간 뒤에 쓸
        # 지령이 아니다. 미는 것을 거두기만 하고 뒤집지는 않는다.
        past_target = self.move_dir != 0.0 and (ang - tgt) * self.move_dir > 0.0
        moving = (ref_moving or still_going) and not past_target
        if not moving:
            self.move_done = True
        q_want = self.move_dir * (mag if moving else args.fric_ff_hold)
        # 어느 쪽이든 계단 없이 옮긴다 (램프 시작 때 걸리는 것도 포함).
        self.fric_q += (q_want - self.fric_q) * min(1.0, dt / args.fric_decay_tau)
        self.ff = (interp(self.ang_tab, self.diff_tab, ang_ref)
              + self.fric_q * interp(self.ang_tab, self.fric_tab, ang_ref))
        # 적분은 램프 중과 목표에 선 뒤를 완전히 다르게 다룬다. 램프 중 오차는
        # 예상된 추종 지연이고, 정상상태 오차는 마찰 밴드 안에 갇힌 잔류다 —
        # 같은 항으로 둘 다 처리하려 하면 한쪽이 반드시 망가진다.
        #
        # 데드밴드는 좁게 둔다. 넓히면 그 안의 오차를 **아무것도** 못 지운다.
        # P 항만 남는데 kp 0.3 kPa/° 이고 플랜트 기울기가 0.93 kPa/° 라
        # 원리적으로 닫히지 않는다. 진동은 --i-rate-max 가 막는다.
        if ref_moving:
            # ── 램프 중 ──
            # 이때의 오차는 팔이 램프를 쫓아가며 생기는 **예상된 지연**이다.
            # 이걸 적분하면 갈 곳 없이 쌓인다(실측 내림 −3.37 kPa → 도착 후
            # 1.2° 초과). 내림 슬루를 2.2 로 올린 뒤 d(self.ff)/dt = 1.4 kPa/s 가
            # 제한 안에 들어와 슬루 포화 정지가 더는 안 걸리므로 따로 막아야 한다.
            #
            # 0 으로 얼리는 대신 흘려보낸다. 직전 유지 구간에서 쌓인 값은 그
            # 각도·그 방향에서만 맞는 보정이라, 방향이 바뀌면 그대로 방해가 된다.
            self.integ -= self.integ * min(1.0, dt / args.i_leak_tau)
        elif abs(err) > self.i_deadband and not self.slew_sat:
            # ── 목표에 선 뒤 ──
            i_rate = self.ki * (err - math.copysign(self.i_deadband, err))
            # 적분이 만들 수 있는 지령 변화율 상한. 마찰 밴드를 넘는 순간 팔이
            # 튀는 stick-slip 을 막는 장치다 — ki 를 올려도 큰 오차에선 여기서 잘린다.
            i_rate = max(-self.i_rate_max, min(self.i_rate_max, i_rate))
            self.integ += i_rate * dt
            self.integ = max(-args.i_limit, min(args.i_limit, self.integ))
        # 내림 속도 피드포워드. 내림은 율 제한에 안 닿는데도(실측 −1.63 vs 제한
        # 2.2) 4.3° 뒤처진다 — 율이 아니라 플랜트가 느린 것이라 지령을 더 앞세워야
        # 한다. 레퍼런스만의 함수라 열려 있고(측정 안 씀) 진동할 수 없으며,
        # 램프가 서는 순간 사라지므로 정지 정확도에 손대지 않는다.
        # 올림은 율 제한이 원인이라 --diff-slew 로 풀었고 여기서 건드리지 않는다.
        v_ff = self.kvel_down * d_ref if (ref_moving and d_ref < 0.0) else 0.0
        want = self.ff + v_ff + self.kp * err + self.integ
        want = max(0.0, min(args.diff_max, want))
        # 출력도 계단이 안 되게 제한한다. 줄이는 쪽은 이동 거리가 길어 따로 준다.
        ds = (self.diff_slew if want >= self.diff_cmd else self.diff_slew_down) * dt
        self.slew_sat = abs(want - self.diff_cmd) > ds
        self.diff_cmd += max(-ds, min(ds, want - self.diff_cmd))

        self.p_pos = max(103.0, args.center + self.diff_cmd / 2.0)
        self.p_neg = min(100.0, args.center - self.diff_cmd / 2.0)

    def track_bad(self, rig, args, now: float) -> None:
        """채널이 목표압을 못 잡는 상태가 이어지면 중단한다."""
        pm = rig.kpa(POS_BOARD(self.axis - 1))
        nm = rig.kpa(NEG_BOARD(self.axis - 1))
        self.pm, self.nm = pm, nm
        bad = (pm is not None and abs(pm - self.p_pos) > args.track_tol) or \
              (nm is not None and abs(nm - self.p_neg) > args.track_tol)
        if bad:
            if self.bad_since is None:
                self.bad_since = now
            elif now - self.bad_since > args.track_grace:
                raise Abort(f"{self.name}: 압력 추종 실패가 계속된다 — "
                            "채널이 목표압을 못 잡는다")
        else:
            self.bad_since = None


class Rig(Node):
    """각도와 챔버 압력을 읽는다. 밸브 지령은 컨트롤러가 낸다."""

    def __init__(self, cfg: dict) -> None:
        super().__init__("angle_ctrl")
        self.offs, self.gains, self.atm = cfg["offs"], cfg["gains"], cfg["atm"]
        self._raw = None; self._ang = None; self._ang_t = None
        self.create_subscription(UInt16MultiArray, f"{NAMESPACE}/board/sensors",
                                 lambda m: setattr(self, "_raw", list(m.data)), 20)
        self.create_subscription(Float64MultiArray, f"{NAMESPACE}/board/analog",
                                 self._on_ang, 20)
        # 목표각을 토픽으로도 낸다. pp_monitor 의 Target 칸이 control_mode 0 에서는
        # 비어 있어(position_dbg 가 안 나온다) 목표를 화면으로 볼 방법이 없었다.
        #   controller/angle_ref_deg : [축0..5 목표각], 안 쓰는 축은 NaN
        self._pub_ref = self.create_publisher(
            Float64MultiArray, f"{NAMESPACE}/controller/angle_ref_deg", 5)

    def publish_ref(self, rows, tgt: float, slew: float) -> None:
        """rows 는 board/analog 인덱스들이다 — 모니터 표가 그 인덱스로 행을 만든다."""
        m = Float64MultiArray()
        d = [float("nan")] * NUM_AXES
        for row in rows:
            if 0 <= row < NUM_AXES:
                d[row] = tgt
        m.data = d + [slew]          # 마지막 = 슬루 중인 내부 목표
        self._pub_ref.publish(m)

    def _on_ang(self, m):
        self._ang = list(m.data); self._ang_t = time.monotonic()

    def kpa(self, b):
        if self._raw is None or len(self._raw) < b: return None
        raw = self._raw[b - 1]
        if raw == 0: return None
        return (float(raw) - self.offs[b]) * self.gains[b] + self.atm

    def angle(self, idx: int):
        if self._ang is None or len(self._ang) <= idx: return None
        return self._ang[idx]

    def age(self):
        return 1e9 if self._ang_t is None else time.monotonic() - self._ang_t


def main() -> int:
    ap = argparse.ArgumentParser(description="각도 피드백 (맵 피드포워드 + 느린 PID)",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--axes", default=None,
                    help="여러 축을 **같은 레퍼런스로 같은 타이밍에** 돌린다 (예: 1,2). "
                         "램프는 하나만 만들어 공유하고 맵·마찰·적분만 축마다 따로 "
                         "간다. 축마다 --map 을 **축 순서대로 하나씩** 준다")
    ap.add_argument("--axis", type=int, default=1)
    ap.add_argument("--enc-idx", type=int, default=None)
    ap.add_argument("--map", action="append", required=True,
                    help="actuator_map CSV 경로 (한 축의 것). 한 축에 여러 CSV 면 "
                         "쉼표로 잇고, 축이 여럿이면 --map 을 축 순서대로 반복한다")
    ap.add_argument("--targets", default=None,
                    help="목표각 열 [°], 쉼표. 생략하면 대화형(타이핑)")
    ap.add_argument("--tol", type=float, default=1.0,
                    help="도착 판정 [°]. |실측 − 목표| 가 이 안이면 도착으로 본다")
    ap.add_argument("--settle-hold", type=float, default=10.0,
                    help="도착한 뒤 **그대로 유지**하는 시간 [s]. 추종했는지 여기서 본다")
    ap.add_argument("--target-timeout", type=float, default=120.0,
                    help="이 시간 안에 못 도착하면 '미도착'으로 적고 넘어간다 [s]")
    ap.add_argument("--center", type=float, default=101.325,
                    help="P+/P− 의 공통압 [kPa abs]. 차압을 이 값 중심으로 나눈다")

    g = ap.add_argument_group("느리게 — 진동을 구조적으로 막는다")
    ap.add_argument("--ang-slew", type=float, default=1.5, help="목표각 변화율 [°/s]")
    ap.add_argument("--diff-slew-down", type=float, default=2.2,
                    help="[내림] 차압을 **줄이는** 쪽 변화율 상한 [kPa/s]. 내림이 더 "
                         "오래 걸리는 건 마찰항 부호가 뒤집혀 이동 거리가 2f 만큼 "
                         "길기 때문이다 (실측 19.7 vs 13.5 kPa). 거리가 기니 속도를 "
                         "따로 준다. 내림은 이 제한에 안 닿는다(실측 −1.63) — "
                         "내림이 느리면 여기가 아니라 --kvel-down 을 본다")
    ap.add_argument("--diff-slew", type=float, default=2.5,
                    help="[올림] 차압을 **늘리는** 쪽 변화율 상한 [kPa/s]. 계단을 "
                         "만들 수 없게 한다. **올림 지연의 병목은 게인이 아니라 "
                         "여기였다** — 램프 자체가 0.93 kPa/° × 1.49 °/s = "
                         "1.39 kPa/s 를 먹으므로 1.5 에서는 지연을 따라잡을 여유가 "
                         "0.11 kPa/s 뿐이다 (실측: 적분 0.00 인데 지령이 ff 보다 "
                         "6~9 kPa 아래, 상승률 정확히 1.50). 다만 2.5 는 자연 속도가 "
                         "램프를 넘어(1.74 vs 1.49 °/s) 오버슛으로 돌아온다 — "
                         "올림이 램프보다 빠르면 2.1 쪽으로 내린다")
    ap.add_argument("--fric-ff", type=float, default=0.45,
                    help="**방향 마찰 피드포워드** 비율 (0=끔, 1=측정 반폭 전부). "
                         "같은 각도를 만드는 데 필요한 차압이 올릴 때와 내릴 때 "
                         "다르다(실측 반폭 2.3~6.1 kPa). 중앙값만 쓰면 올릴 때는 "
                         "그만큼 모자라고 내릴 때는 그만큼 과하다 — "
                         "20260914 에 올라갈 때 느리고 60→45 는 48.3° 에서 멈췄다.\n"
                         "0.8 은 **과했다** — 5개 목표 전부 1.07~1.52° 씩 지나쳤다 "
                         "(올림 +, 내림 −, 대칭). 각 점에서 초과분을 차압으로 역산하니 "
                         "적정값이 0.51~0.59 로 일관돼 0.55 로 잡았다.\n"
                         "--diff-slew 를 2.5 로 올려 율 제한이 풀리자 이번엔 올림이 "
                         "램프보다 빨라졌다 (1.74 vs 1.49 °/s, 앞섬 0.24~0.58°). "
                         "초과분 0.5° × 0.93 kPa/° ÷ 4.86 ≈ 0.10 을 빼 0.45. "
                         "**올림 전용이다** — 내림은 --fric-ff-down 이 따로 쓴다")
    ap.add_argument("--kvel-down", type=float, default=1.5,
                    help="내림 램프의 속도 피드포워드 [kPa/(°/s)]. 내림은 율 제한에 "
                         "안 닿는데도 4.3° 뒤처진다 — 부피 외란(내려오며 양압 챔버가 "
                         "줄고 음압이 는다)이 늘 지령을 거스르기 때문이다. 실측 지연 "
                         "4.3° → 지령 부족 4.0 kPa, 그중 kp 가 1.3 을 대므로 "
                         "2.7 ÷ 1.49 ≈ 1.8 이 상한. 레퍼런스만의 함수라 진동 못 한다")
    ap.add_argument("--fric-ff-down", type=float, default=1.0,
                    help="내려가는 램프에서 쓸 마찰 비율. 내림은 부피 외란이 늘 지령을 "
                         "거슬러 올림보다 더 든다 (실측 램프 지연 올림 0.9° vs 내림 "
                         "4.9°, 각속도 1.65 vs 1.30 °/s). 1.0 = 실측 마찰 전폭")
    ap.add_argument("--fric-ff-hold", type=float, default=0.0,
                    help="목표에 선 뒤 남길 마찰 비율. 0 = 마찰 밴드 중앙. 도착 후에도 "
                         "미는 지령을 유지한 것이 +1° 크리프의 원인이었다 "
                         "(실측: ref 도달 시점엔 ±0.4° 인데 이후 4초에 걸쳐 +1.0~1.6° 기어갔다)")
    ap.add_argument("--fric-hold-band", type=float, default=0.3,
                    help="레퍼런스가 선 뒤에도 **가던 방향으로** 이만큼 넘게 남아 있으면 "
                         "이동이 안 끝난 것으로 보고 마찰 피드포워드를 유지한다 [°]. "
                         "착지 정확도를 정하는 손잡이다 — 회수에 지령이 f 만큼 움직여야 "
                         "하고 그동안 팔이 계속 가므로, 작으면 지나치고 크면 못 미친다. "
                         "1.5 는 내림을 1.3~1.5° 위에 세웠고(err_end +1.47/+1.24), "
                         "0.5 로 +0.79/+0.55 까지 왔다. 내림 err_end 가 양수면 줄인다")
    ap.add_argument("--fric-decay-tau", type=float, default=1.0,
                    help="목표에 선 뒤 마찰 피드포워드를 --fric-ff-hold 로 흘려보내는 "
                         "시간상수 [s]. 계단으로 떨어뜨리면 팔이 튄다 — 그게 예전에 "
                         "이 항을 얼려 두었던 이유다")
    ap.add_argument("--gain-ref-slope", type=float, default=0.88,
                    help="아래 게인들을 **어느 플랜트 기울기에서 잰 값으로 볼지** "
                         "[kPa/°]. 기본은 축1 실측 0.88. kp·ki·kvel-down·"
                         "i-rate-max·diff-slew 는 전부 kPa 단위라 기울기가 다른 축에 "
                         "그대로 못 간다 — 축마다 기울기 비로 자동 환산해서 "
                         "**각도로 본 거동**(°/s, °)이 축에 무관하게 같아지게 한다")
    ap.add_argument("--kp", type=float, default=0.30, help="[kPa/°]")
    ap.add_argument("--ki", type=float, default=0.30,
                    help="[kPa/(°·s)]. 0.03 은 데드밴드 밖에서도 0.066 kPa/s 라 "
                         "남은 1.8 kPa 를 모으는 데 27초가 걸렸다 — 정체로 보인 원인이다. "
                         "속도 자체는 --i-rate-max 가 따로 자르므로 여기를 올려도 "
                         "큰 오차에서 빨라지지 않는다")
    ap.add_argument("--i-deadband", type=float, default=None,
                    help="이 각도 오차 안에서는 적분을 멈춘다 [°]. 생략하면 맵의 "
                         "마찰 폭 × --i-deadband-frac 로 정한다. 마찰이 붙잡는 오차를 "
                         "적분이 밀면 stick-slip 한계주기가 되지만, 폭 전체를 죽이면 "
                         "그 안의 오차가 영구히 남는다 (실측 +3.3° 잔류)")
    ap.add_argument("--i-deadband-frac", type=float, default=0.05,
                    help="데드밴드를 마찰 폭의 몇 배로 둘지. 방향 피드포워드가 "
                         "마찰을 대부분 없애 주므로 전폭을 죽일 필요가 없다. "
                         "0.15(→0.7°)도 소프트 데드밴드라 err 0.95°에서 유효 오차를 "
                         "0.25°만 남겨 적분이 0.0375 kPa/s 로 기었다 (1° 닫는 데 24초). "
                         "유지 중 σ 가 0.014~0.50 이라 0.24° 는 잡음 위다")
    ap.add_argument("--i-limit", type=float, default=8.0, help="적분항 상한 [kPa]")
    ap.add_argument("--i-leak-tau", type=float, default=4.0,
                    help="램프 중 적분을 0 으로 흘려보내는 시간상수 [s]. 10초 이동이면 "
                         "92%% 가 빠진다. 이동 중 오차는 적분할 대상이 아니고, 직전 "
                         "유지에서 쌓인 값은 방향이 바뀌면 방해만 된다")
    ap.add_argument("--i-rate-max", type=float, default=0.40,
                    help="적분이 만들 수 있는 차압 변화율 상한 [kPa/s]. 진동을 막는 "
                         "장치가 이제 데드밴드가 아니라 이거다. 플랜트 기울기 "
                         "0.93 kPa/° 기준 0.40 은 최대 0.43 °/s — 튈 수 없는 속도다. "
                         "지령이 밴드 가장자리가 아니라 중앙에서 출발하게 되어 "
                         "보정 방향으로 갈 거리가 두 배가 됐으므로 같이 올린다")

    s = ap.add_argument_group("안전")
    ap.add_argument("--ang-max", type=float, default=85.0,
                    help="목표·실측 각도 상한 [°]. **90° 위는 중력 복원 토크가 "
                         "줄어 정적으로 불안정하다** — 넘기면 스톱까지 간다")
    ap.add_argument("--ang-min", type=float, default=-5.0)
    ap.add_argument("--max-rate", type=float, default=10.0,
                    help="각속도 상한 [°/s]. 느리게 도는 것이 목적이라 낮게 잡는다 — "
                         "걸려서 멈추는 쪽이 안전한 실패다. 실측 90%%값이 올림 5.1 / "
                         "내림 4.2 라 내림 슬루를 2.2 로 올려도 ~6 에 머문다")
    ap.add_argument("--osc-window", type=float, default=6.0, help="진동 판정 창 [s]")
    ap.add_argument("--osc-flips", type=int, default=6, help="그 창의 부호 반전 횟수")
    ap.add_argument("--track-tol", type=float, default=4.0, help="압력 추종 허용 [kPa]")
    ap.add_argument("--track-grace", type=float, default=6.0, help="[s]")
    ap.add_argument("--enc-timeout", type=float, default=1.0)
    ap.add_argument("--diff-max", type=float, default=75.0, help="차압 상한 [kPa]")

    ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=2293)
    ap.add_argument("--connect-timeout", type=float, default=30.0)
    ap.add_argument("--send-hz", type=float, default=20.0)
    ap.add_argument("--release-slew", type=float, default=4.0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--dry-run", action="store_true"); ap.add_argument("--yes", action="store_true")
    args = ap.parse_args()

    axes = ([int(v) for v in args.axes.replace(" ", "").split(",") if v]
            if args.axes else [args.axis])
    maps = args.map or []
    if len(maps) != len(axes):
        ap.error(f"--map 이 {len(maps)}개인데 축은 {len(axes)}개다. "
                 "축마다 맵이 다르다 — 축 순서대로 하나씩 줘라 "
                 "(2축은 1축보다 뻣뻣해서 같은 각도에 더 큰 차압이 든다)")
    if args.enc_idx is not None and len(axes) > 1:
        ap.error("--enc-idx 는 축이 하나일 때만 쓴다")

    chans = [Chan(ax, (args.enc_idx if args.enc_idx is not None else ax - 1),
                  mp, args) for ax, mp in zip(axes, maps)]

    print("\n=== 각도 피드백 ===")
    if len(chans) > 1:
        print(f"축 {', '.join(str(c.axis) for c in chans)} — "
              "**같은 레퍼런스를 같은 타이밍에** 따라간다 (램프 공유)")
    for c in chans:
        print(f"\n[{c.name}] board/analog[{c.enc_idx}]  "
              f"맵 올림 {c.info['n_up']}점 / 내림 {c.info['n_dn']}점, "
              f"각도 {c.info['lo']:.1f}~{c.info['hi']:.1f}°")
        print(f"  마찰 폭 ±{c.info['fric_kpa']:.2f} kPa · 기울기 {c.slope:.2f} kPa/° "
              f"(환산계수 ×{c.slope/max(1e-9,args.gain_ref_slope):.2f})")
        print(f"    kp {c.kp:.2f}  ki {c.ki:.2f}  내림속도FF {c.kvel_down:.2f}  "
              f"차압슬루 올림 {c.diff_slew:.2f} / 내림 {c.diff_slew_down:.2f} kPa/s")
        print(f"    적분 데드밴드 {c.i_deadband:g}° · 적분이 낼 수 있는 각속도 "
              f"{c.i_rate_max / max(1e-9, c.slope):.2f} °/s · "
              f"램프가 먹는 차압 {c.slope * args.ang_slew:.2f} kPa/s "
              f"(올림 여유 {c.diff_slew - c.slope * args.ang_slew:+.2f})")
        print(f"  {'각도':>5}{'중앙':>8}{'올릴 때':>9}{'내릴 때':>9}")
        for a in (30, 40, 50, 60, 70, 80, 84):
            if c.info["lo"] <= a <= c.info["hi"]:
                m = interp(c.ang_tab, c.diff_tab, a)
                fq = interp(c.ang_tab, c.fric_tab, a)
                print(f"  {a:5.0f}{m:8.1f}{m + args.fric_ff * fq:9.1f}"
                      f"{m - args.fric_ff_down * fq:9.1f}")
    lo_all = max(c.info["lo"] for c in chans)
    hi_all = min(c.info["hi"] for c in chans)
    if len(chans) > 1:
        print(f"\n공통으로 덮는 각도 {lo_all:.1f}~{hi_all:.1f}° — "
              "이 밖의 목표는 어느 한 축의 표 끝값으로 고정된다")
    print(f"피드포워드: 램프 중 마찰 비율 올림 {args.fric_ff:g} / "
          f"내림 {args.fric_ff_down:g}, 도착 후 {args.fric_ff_hold:g} "
          f"(τ {args.fric_decay_tau:g}s, 도착 판정 {args.fric_hold_band:g}°)")
    print(f"\n느리게: 목표각 {args.ang_slew:g} °/s (모든 축 공유). "
          f"아래는 기울기 {args.gain_ref_slope:g} kPa/° 기준값이고 축별 환산은 위에: "
          f"차압 올림 {args.diff_slew:g} / 내림 {args.diff_slew_down:g} kPa/s · "
          f"kp {args.kp:g} ki {args.ki:g} · 내림 속도FF {args.kvel_down:g} (미분항 없음)")
    print(f"  적분: 데드밴드 밖에서만, 슬루 포화 중 정지, 변화율 ≤ "
          f"{args.i_rate_max:g} kPa/s 기준")
    print(f"안전: 각도 [{args.ang_min:g}, {args.ang_max:g}]° · 각속도 {args.max_rate:g} °/s · "
          f"진동 {args.osc_flips}회/{args.osc_window:g}s · 차압 ≤ {args.diff_max:g}")
    print(f"  ⚠ 상한 {args.ang_max:g}° — 90° 를 넘으면 중력 복원 토크가 줄어 "
          "정적으로 불안정하다 (스톱까지 간다)")
    if args.dry_run:
        return 0
    if not args.yes:
        if input("\n비상정지를 확인했으면 RUN 을 입력: ").strip() != "RUN":
            print("취소했다."); return 2

    cfg = read_config(os.path.join(_HERE, "..", "config", "powerpack_config.yaml"))
    out = args.out or os.path.expanduser(
        f"~/result/angle_ctrl_{datetime.now():%Y%m%d_%H%M%S}.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    rclpy.init()
    rig = Rig(cfg)
    conn = None
    rc = 0
    f = open(out, "w", newline="", buffering=1)
    wr = csv.writer(f)
    # phase: move=목표로 가는 중, hold=도착 후 유지 중.
    # 축별 열은 뒤에 이어 붙인다 — 축이 하나면 예전과 같은 모양이다.
    cols = ["t", "idx", "phase", "ang_tgt", "ang_ref"]
    for c in chans:
        sfx = "" if len(chans) == 1 else f"_ax{c.axis}"
        cols += [f"{k}{sfx}" for k in
                 ("ang_meas", "err", "diff_ff", "diff_i", "diff_cmd",
                  "p_pos_ref", "p_neg_ref", "p_pos_meas", "p_neg_meas", "tau_Nm")]
    wr.writerow(cols)
    summ = open(out.replace(".csv", "_summary.csv"), "w", newline="", buffering=1)
    sw = csv.writer(summ)
    scols = ["idx", "ang_tgt", "arrived", "arrive_s"]
    for c in chans:
        sfx = "" if len(chans) == 1 else f"_ax{c.axis}"
        scols += [f"{k}{sfx}" for k in
                  ("ang_mean", "ang_std", "err_mean", "err_end",
                   "overshoot", "diff_final", "tau_Nm")]
    sw.writerow(scols)

    state = {"tgt": None, "msg": ""}

    def reader():
        while True:
            try:
                line = sys.stdin.readline()
            except Exception:
                return
            if not line: return
            t = line.strip()
            if t in ("q", "quit"): state["tgt"] = "QUIT"; return
            try:
                state["tgt"] = max(args.ang_min, min(args.ang_max, float(t)))
                state["msg"] = f"목표각 {state['tgt']:.1f}°"
            except ValueError:
                state["msg"] = f"못 읽었다: {t!r}"

    try:
        conn = _connect(args)
        dt = 1.0 / args.send_hz
        # 현재 각도에서 출발한다 — 목표가 튀면 차압도 튄다.
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            rclpy.spin_once(rig, timeout_sec=0.0); time.sleep(0.02)
        t_now = time.monotonic()
        # ── 출력은 **지금 챔버에 실제로 걸린 차압**에서 출발한다 ──────────
        # 피드포워드 값으로 초기화하면 첫 틱에 목표압이 통째로 계단이 된다.
        # 차압 슬루는 그 뒤의 *변화*만 제한하므로 초기값은 못 막는다 —
        # 20260914 에 대기압(차압 0)에서 12.4 kPa 가 한 번에 나가 팔이
        # 21.9 °/s 로 튀었다 (각속도 감시가 잡았다).
        a0s = [c.start(rig, args, t_now) for c in chans]
        # 램프는 **하나**다. 여러 축이면 현재 각도의 평균에서 출발한다 —
        # 어느 한 축에 맞추면 나머지 축이 첫 구간에서 계단을 받는다.
        a0 = sum(a0s) / len(a0s)
        ang_ref = max(args.ang_min, min(args.ang_max, a0))
        for c, a_ in zip(chans, a0s):
            ff0 = interp(c.ang_tab, c.diff_tab, ang_ref)
            print(f"[시작 {c.name}] 각도 {a_:.2f}°, 현재 차압 {c.diff_cmd:.1f} kPa "
                  f"에서 출발 (피드포워드 목표 {ff0:.1f} — "
                  f"{abs(ff0-c.diff_cmd)/max(1e-9,args.diff_slew):.0f} s 에 걸쳐 옮긴다)")
            if a_ < c.info["lo"] - 0.5:
                print(f"  ⚠ 현재 각도가 맵 하한({c.info['lo']:.1f}°) 아래다. "
                      "그 구간 피드포워드는 표 끝값으로 고정된다.")
        if len(chans) > 1 and max(a0s) - min(a0s) > 2.0:
            print(f"  ⚠ 축 사이 시작 각도가 {max(a0s)-min(a0s):.1f}° 벌어져 있다 — "
                  "첫 구간은 두 축이 서로 다른 거리를 간다")

        targets = None
        # 목표마다 **도착할 때까지 기다렸다가** settle_hold 만큼 유지한다.
        # 고정 시간으로 넘기면 도착 전에 다음 목표로 가 버려서 "추종했는가" 를
        # 판정할 수 없다 (20260914 실측: 25 s 로는 30°·45° 구간이 미도착이었다).
        ti = 0
        phase = "move"            # move → hold → 다음 목표
        t_phase = time.monotonic()
        in_band_since = None
        hold_buf: dict[int, list[float]] = {}
        if args.targets:
            targets = [max(args.ang_min, min(args.ang_max, float(v)))
                       for v in args.targets.replace(" ", "").split(",") if v]
            state["tgt"] = targets[0]
            print(f"\n[1/{len(targets)}] 목표 {targets[0]:.1f}° — 도착 후 "
                  f"{args.settle_hold:g} s 유지한다", flush=True)
        else:
            state["tgt"] = ang_ref
            threading.Thread(target=reader, daemon=True).start()
            print("목표각을 입력해라 (예: 40). q 로 종료.")

        t0 = time.monotonic()
        t_move0 = t0
        for c in chans:
            c.ang_ref_prev = ang_ref
        last_print = 0.0

        while True:
            if state["tgt"] == "QUIT":
                break

            # ── 목표각을 기울인다 ────────────────────────────────────────
            tgt = state["tgt"]
            step = args.ang_slew * dt
            ang_ref += max(-step, min(step, tgt - ang_ref))

            rclpy.spin_once(rig, timeout_sec=0.0)
            now = time.monotonic()
            # 안전은 축마다 본다. **한 축이라도 걸리면 전체가 중단되고**
            # 모든 축이 대기압으로 램프된다.
            for c in chans:
                c.read(rig, args, now)
                c.control(ang_ref, tgt, dt, args)
                c.peak_err = max(c.peak_err, abs(c.ang - tgt))

            conn.sendall(encode_refs(_refs_axes(
                {c.axis - 1: (c.p_pos, c.p_neg) for c in chans})))
            rig.publish_ref([c.enc_idx for c in chans], tgt, ang_ref)
            for c in chans:
                c.track_bad(rig, args, now)

            # ── 도착 / 유지 판정 ────────────────────────────────────────
            # 슬루가 아니라 **최종 목표** 기준이고, 여러 축이면 **전부** 들어와야
            # 도착이다. 한 축만 보고 넘어가면 나머지 축의 추종을 판정할 수 없다.
            worst = max(abs(c.ang - tgt) for c in chans)
            if targets is not None:
                def _next(arrived: bool, arrive_s: float) -> bool:
                    _finish(sw, ti, tgt, arrived, arrive_s, chans, hold_buf)
                    return True
                if phase == "move":
                    if worst <= args.tol and in_band_since is None:
                        in_band_since = now
                        phase = "hold"; t_phase = now
                        hold_buf = {c.axis: [] for c in chans}
                        print(f"      도착 ({now-t0:.1f}s, {now-t_move0:.1f}s 걸림) "
                              f"— {args.settle_hold:g} s 유지", flush=True)
                    if now - t_move0 > args.target_timeout:
                        _next(False, now - t_move0)
                        ti += 1
                        if ti >= len(targets): break
                        state["tgt"] = targets[ti]; phase = "move"
                        t_move0 = now; in_band_since = None
                        for c in chans: c.peak_err = 0.0
                        print(f"\n[{ti+1}/{len(targets)}] 목표 {targets[ti]:.1f}°",
                              flush=True)
                else:                        # hold
                    for c in chans:
                        hold_buf.setdefault(c.axis, []).append(c.ang)
                    if worst > args.tol * 2.5:
                        # 유지 중에 밴드를 크게 벗어났다 — 다시 접근 단계로
                        phase = "move"; in_band_since = None; t_move0 = now
                        print("      유지 중 이탈 — 다시 접근한다", flush=True)
                    elif now - t_phase >= args.settle_hold:
                        _next(True, t_phase - t_move0)
                        ti += 1
                        if ti >= len(targets): break
                        state["tgt"] = targets[ti]; phase = "move"
                        t_move0 = now; in_band_since = None
                        for c in chans: c.peak_err = 0.0
                        print(f"\n[{ti+1}/{len(targets)}] 목표 {targets[ti]:.1f}°",
                              flush=True)

            row = [f"{now-t0:.2f}", ti, phase, f"{tgt:.2f}", f"{ang_ref:.2f}"]
            for c in chans:
                tau = 2.0 * G * 0.15 * math.sin(math.radians(c.ang))
                row += [f"{c.ang:.3f}", f"{c.err:+.3f}", f"{c.ff:.2f}",
                        f"{c.integ:+.2f}", f"{c.diff_cmd:.2f}",
                        f"{c.p_pos:.2f}", f"{c.p_neg:.2f}",
                        "" if c.pm is None else f"{c.pm:.2f}",
                        "" if c.nm is None else f"{c.nm:.2f}", f"{tau:.4f}"]
            wr.writerow(row)
            if now - last_print >= 1.0:
                last_print = now
                m = state["msg"]; state["msg"] = ""
                head = f"  [{now-t0:6.1f}s] 목표 {tgt:6.2f}° 슬루 {ang_ref:6.2f}°"
                for c in chans:
                    head += (f"\n     [{c.name}] 실측 {c.ang:6.2f}° (e {c.err:+5.2f}) "
                             f" 차압 {c.diff_cmd:5.1f} (ff {c.ff:5.1f} i {c.integ:+5.2f}) "
                             f" 속도 {c.rate:+5.1f}°/s")
                print(head + (f"   {m}" if m else ""), flush=True)
            time.sleep(dt)
        print("\n[완료]")
    except Abort as exc:
        print(f"\n[중단] 안전: {exc}", file=sys.stderr); rc = 3
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C", file=sys.stderr); rc = 130
    except (ConnectionError, OSError, TimeoutError) as exc:
        print(f"\n[위험] 통신 실패: {exc}", file=sys.stderr); rc = 1
    finally:
        if conn is not None:
            print(f"[복귀] 양 챔버를 {args.release_slew:g} kPa/s 로 대기압까지 내린다...")
            try:
                dt = 1.0 / args.send_hz
                cur = {c.axis - 1: [args.center + c.diff_cmd / 2.0,
                                    args.center - c.diff_cmd / 2.0] for c in chans}
                t_end = time.monotonic() + 120.0
                while time.monotonic() < t_end:
                    s_ = args.release_slew * dt
                    done = True
                    for v in cur.values():
                        v[0] += max(-s_, min(s_, ATM_KPA - v[0]))
                        v[1] += max(-s_, min(s_, ATM_KPA - v[1]))
                        if abs(v[0] - ATM_KPA) >= 0.1 or abs(v[1] - ATM_KPA) >= 0.1:
                            done = False
                    conn.sendall(encode_refs(_refs_axes(
                        {k: (v[0], v[1]) for k, v in cur.items()})))
                    if done: break
                    rclpy.spin_once(rig, timeout_sec=0.0); time.sleep(dt)
                print("[복귀] 대기압 도달.")
            except OSError as exc:
                print(f"[위험] 복귀 실패: {exc} — 펌프를 즉시 정지할 것.", file=sys.stderr)
            conn.close()
        f.close()
        try: summ.close()
        except Exception: pass
        rig.destroy_node(); rclpy.shutdown()
        print(f"[기록] {out}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
