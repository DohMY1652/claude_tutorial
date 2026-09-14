#!/usr/bin/env python3
"""액추에이터 정특성 측정 — (P+, P−) → 조인트 각도 매핑.

무엇을 재는가
-------------
각 (양압, 음압) 조합에서 팔이 **멈추는 각도**를 잰다. 정지 상태에서는
액추에이터 토크와 중력 토크가 균형이므로, 그 각도 하나로 토크를 역산할 수 있다:

    tau_act(P+, P−) = m·g·L·sin(θ_meas)          [N·m]
    F_act           = tau_act / r_reel           [N]      (r = 릴 지름/2)

즉 이 실험의 산출물은 "각도표"가 아니라 **액추에이터 힘 모델**이다.
F = (P+ − P−)·A 가 실린더가 아니라서 안 맞는다고 했으니, 여기서 나온
(P+, P−) → F 표를 그대로 피드포워드로 쓰면 된다. CSV 에 tau·F 를 같이 적는다.

안전이 우선이다
---------------
액추에이터가 약하다고 했으므로 아래를 전부 건다. 하나라도 걸리면 **즉시
양 챔버를 대기압으로 되돌린다** (끊는 게 아니라 램프로 내린다 — 목표를 끊으면
컨트롤러에 마지막 값이 남는다).

  1. 목표를 절대 계단으로 주지 않는다. `--slew` [kPa/s] 로 기울여 올린다.
  2. 각속도 감시 — |dθ/dt| > `--max-rate` 면 중단 (폭주·발산)
  3. 진동 감시 — `--osc-window` 초 안에 각속도 부호가 `--osc-flips` 번 이상
     뒤집히면 중단. 진폭이 작아도 떨고 있으면 잡는다.
  4. 각도 한계 — [`--ang-min`, `--ang-max`] 밖이면 중단
  5. 추종 감시 — |P_실측 − P_목표| 가 `--track-tol` 을 `--track-grace` 초
     넘게 벗어나면 중단. 컨트롤러가 못 따라가는데 계속 밀면 안 된다.
  6. 엔코더 감시 — 각도 갱신이 `--enc-timeout` 초 없으면 중단
  7. 어떤 경로로 끝나든(정상·중단·Ctrl-C·예외) 반드시 대기압 램프를 거친다.

측정 순서
---------
공통압(center)마다 차압(diff)을 **올렸다 내린다.** 되돌아오는 쪽도 재는 이유는
이 종류의 액추에이터에 **히스테리시스**가 크기 때문이다. 왕복 두 값의 차이가
곧 마찰이고, 피드포워드를 쓸 때 그만큼이 오차 하한이 된다.

    P+ = center + diff/2
    P− = center − diff/2

사용 예
-------
    # 1축, 공통압 100·110, 차압 0→40 을 5 kPa 씩 왕복
    python3 actuator_map.py --axis 1 --centers 100,110 --diff-stop 40 --diff-step 5

    # 먼저 계획만 보기 (실기에 아무것도 안 보낸다)
    python3 actuator_map.py --axis 1 --dry-run
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import socket
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, UInt16MultiArray

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from pressure_sweep_server import (            # noqa: E402
    ATM_KPA, NUM_AXES, _connect, _refs, encode_refs,
)
from valve_deadzone import read_config         # noqa: E402

NAMESPACE = "/pack2"
G = 9.81
# 축 a(0-based) 의 챔버 압력 보드
POS_BOARD = lambda a: 5 + a      # noqa: E731
NEG_BOARD = lambda a: 11 + a     # noqa: E731


# ════════════════════════════════════════════════════════════════════════════
#  실기 상태 읽기
# ════════════════════════════════════════════════════════════════════════════
class RigView(Node):
    """각도와 챔버 압력만 읽는다. **아무것도 발행하지 않는다** —
    밸브 지령은 전적으로 pp_controller 가 낸다 (튜닝된 PID·데드존·FF 를 쓰려면
    그래야 한다). 이 노드는 감시자일 뿐이다."""

    def __init__(self, cfg: dict, enc_idx: int) -> None:
        super().__init__("actuator_map")
        self.offs, self.gains, self.atm = cfg["offs"], cfg["gains"], cfg["atm"]
        self.enc_idx = enc_idx
        self._raw: list[int] | None = None
        self._ang: list[float] | None = None
        self._ang_t: float | None = None
        self.create_subscription(UInt16MultiArray, f"{NAMESPACE}/board/sensors",
                                 self._on_sensors, 20)
        self.create_subscription(Float64MultiArray, f"{NAMESPACE}/board/analog",
                                 self._on_analog, 20)

    def _on_sensors(self, msg: UInt16MultiArray) -> None:
        self._raw = list(msg.data)

    def _on_analog(self, msg: Float64MultiArray) -> None:
        self._ang = list(msg.data)
        self._ang_t = time.monotonic()

    def kpa(self, board: int) -> float | None:
        if self._raw is None or len(self._raw) < board:
            return None
        raw = self._raw[board - 1]
        if raw == 0:                    # 그 보드 프레임을 아직 못 받았다
            return None
        return (float(raw) - self.offs[board]) * self.gains[board] + self.atm

    def angle(self) -> float | None:
        if self._ang is None or len(self._ang) <= self.enc_idx:
            return None
        return self._ang[self.enc_idx]

    def angle_age(self) -> float:
        return 1e9 if self._ang_t is None else (time.monotonic() - self._ang_t)


# ════════════════════════════════════════════════════════════════════════════
#  안전 감시
# ════════════════════════════════════════════════════════════════════════════
class Abort(Exception):
    """안전 감시가 걸렸다. 호출자는 반드시 대기압 램프를 거쳐야 한다."""


@dataclass
class Guard:
    max_rate: float
    ang_min: float
    ang_max: float
    track_tol: float
    track_grace: float
    enc_timeout: float
    osc_window: float
    osc_flips: int

    _bad_since: float | None = None
    _flips: list[float] = field(default_factory=list)
    _last_sign: int = 0

    def check(self, rig: RigView, ang: float, rate: float,
              p_pos: float | None, p_neg: float | None,
              ref_pos: float, ref_neg: float) -> None:
        now = time.monotonic()

        if rig.angle_age() > self.enc_timeout:
            raise Abort(f"엔코더 갱신이 {rig.angle_age():.1f} s 없다 "
                        f"(한계 {self.enc_timeout:g} s)")

        if not (self.ang_min <= ang <= self.ang_max):
            raise Abort(f"각도 {ang:.1f}° 가 한계 "
                        f"[{self.ang_min:g}, {self.ang_max:g}] 밖이다")

        if abs(rate) > self.max_rate:
            raise Abort(f"각속도 {rate:+.1f} °/s 가 한계 "
                        f"{self.max_rate:g} °/s 를 넘었다 — 폭주로 본다")

        # 진동: 각속도 부호 반전 횟수. 진폭이 작아도 떨고 있으면 잡는다.
        sign = 0 if abs(rate) < 0.5 else (1 if rate > 0 else -1)
        if sign and self._last_sign and sign != self._last_sign:
            self._flips.append(now)
        if sign:
            self._last_sign = sign
        self._flips = [t for t in self._flips if now - t <= self.osc_window]
        if len(self._flips) >= self.osc_flips:
            raise Abort(f"{self.osc_window:g} s 안에 각속도 부호가 "
                        f"{len(self._flips)} 번 뒤집혔다 — 진동으로 본다")

        # 추종: 컨트롤러가 목표를 못 잡고 있으면 더 밀지 않는다.
        bad = False
        if p_pos is not None and abs(p_pos - ref_pos) > self.track_tol:
            bad = True
        if p_neg is not None and abs(p_neg - ref_neg) > self.track_tol:
            bad = True
        if bad:
            if self._bad_since is None:
                self._bad_since = now
            elif now - self._bad_since > self.track_grace:
                raise Abort(
                    f"압력 추종 실패가 {self.track_grace:g} s 넘게 계속됐다 "
                    f"(목표 {ref_pos:.1f}/{ref_neg:.1f}, "
                    f"실측 {p_pos if p_pos is None else round(p_pos,1)}/"
                    f"{p_neg if p_neg is None else round(p_neg,1)})")
        else:
            self._bad_since = None


# ════════════════════════════════════════════════════════════════════════════
#  목표 송신 — 항상 기울여서 간다
# ════════════════════════════════════════════════════════════════════════════
class Driver:
    """현재 목표를 들고 있다가 매 틱 `slew` 만큼만 움직여 보낸다.

    **계단을 절대 만들지 않는다.** 대기압에서 목표를 한 번에 주면 오차가 커서
    적분이 차고, 목표를 지나는 순간 되돌리는 데 시간이 걸린다 — 그 과정이
    액추에이터를 때린다. 오차를 작게 유지하면 그 일이 애초에 없다.
    """

    def __init__(self, conn: socket.socket | None, group, send_hz: float,
                 slew: float) -> None:
        self.conn, self.group = conn, group
        self.dt = 1.0 / send_hz
        self.slew = slew
        self.ref_pos = ATM_KPA
        self.ref_neg = ATM_KPA

    def _send(self) -> None:
        if self.conn is not None:
            self.conn.sendall(encode_refs(
                _refs(self.group, positive=self.ref_pos, negative=self.ref_neg)))

    def step_toward(self, tgt_pos: float, tgt_neg: float, slew: float | None = None) -> bool:
        """한 틱만큼 목표에 다가간다. 도착했으면 True."""
        s = (self.slew if slew is None else slew) * self.dt
        done = True
        for name, tgt in (("ref_pos", tgt_pos), ("ref_neg", tgt_neg)):
            cur = getattr(self, name)
            if abs(tgt - cur) <= s:
                setattr(self, name, tgt)
            else:
                setattr(self, name, cur + math.copysign(s, tgt - cur))
                done = False
        self._send()
        return done

    def hold(self) -> None:
        self._send()


def _spin(rig: RigView, dt: float) -> None:
    rclpy.spin_once(rig, timeout_sec=0.0)
    time.sleep(dt)


# ════════════════════════════════════════════════════════════════════════════
#  한 점 측정
# ════════════════════════════════════════════════════════════════════════════
def goto_and_settle(rig: RigView, drv: Driver, guard: Guard,
                    tgt_pos: float, tgt_neg: float, args) -> dict:
    """목표까지 기울여 올린 뒤 각도가 멈출 때까지 기다린다."""
    ang_prev = rig.angle()
    t_prev = time.monotonic()
    rate = 0.0
    quiet_since: float | None = None
    t0 = time.monotonic()
    phase = "ramp"
    samples: list[tuple[float, float, float]] = []   # (angle, p_pos, p_neg)

    while True:
        arrived = drv.step_toward(tgt_pos, tgt_neg)
        _spin(rig, drv.dt)

        ang = rig.angle()
        if ang is None:
            if time.monotonic() - t0 > args.enc_timeout:
                raise Abort("각도를 한 번도 못 받았다 — 엔코더를 확인할 것")
            continue

        now = time.monotonic()
        if ang_prev is not None and now > t_prev:
            # 각속도는 1차 LPF 로 부드럽게 본다 (엔코더 LSB 잡음이 그대로 튄다)
            raw_rate = (ang - ang_prev) / (now - t_prev)
            a = min(1.0, (now - t_prev) / max(1e-3, args.rate_tau))
            rate += a * (raw_rate - rate)
        ang_prev, t_prev = ang, now

        p_pos = rig.kpa(POS_BOARD(args.axis - 1))
        p_neg = rig.kpa(NEG_BOARD(args.axis - 1))
        guard.check(rig, ang, rate, p_pos, p_neg, drv.ref_pos, drv.ref_neg)

        if not arrived:
            continue
        if phase == "ramp":
            phase = "settle"
            t_settle0 = now

        # ── 정착 판정: 각속도가 충분히 작은 상태가 연속으로 유지돼야 한다 ──
        if abs(rate) < args.settle_rate:
            if quiet_since is None:
                quiet_since = now
            samples.append((ang, p_pos or math.nan, p_neg or math.nan))
            if now - quiet_since >= args.settle_hold:
                break
        else:
            quiet_since = None
            samples.clear()

        if now - t_settle0 > args.settle_timeout:
            # 안 멈춘다 = 아직 움직이는 중이거나 미세 진동이다. 중단은 아니고
            # "정착 실패"로 표시해 남긴다. 판단은 데이터를 보고 사람이 한다.
            phase = "timeout"
            break

    n = max(1, len(samples))
    ang_m = sum(s[0] for s in samples) / n if samples else ang
    var = sum((s[0] - ang_m) ** 2 for s in samples) / n if samples else 0.0
    pp = [s[1] for s in samples if not math.isnan(s[1])]
    pn = [s[2] for s in samples if not math.isnan(s[2])]
    return {
        "angle_deg": ang_m,
        "angle_std": math.sqrt(var),
        "p_pos_meas": sum(pp) / len(pp) if pp else math.nan,
        "p_neg_meas": sum(pn) / len(pn) if pn else math.nan,
        "settle_s": time.monotonic() - t0,
        "settled": phase != "timeout",
        "n_sample": len(samples),
    }


# ════════════════════════════════════════════════════════════════════════════
def reachable(p_pos: float, p_neg: float, args) -> str | None:
    """도달 불가능한 조합이면 이유를, 가능하면 None 을 준다.

    챔버는 **한쪽으로만** 갈 수 있다:
      · 양압 챔버 — 레일(양압)에서 채우고 대기로 뺀다 → 대기압 **아래로 못 간다**
      · 음압 챔버 — 진공 레일로 빼고 대기를 넣는다   → 대기압 **위로 못 간다**
    그리고 양 끝에는 차압이 남아 있어야 밸브가 유량을 낼 수 있다 (레일에 붙으면
    차압 0 이라 아무리 열어도 안 움직인다).
    """
    if p_pos < args.pos_min:
        return f"P+ {p_pos:.1f} < {args.pos_min:.1f} (양압 챔버는 대기압 아래로 못 간다)"
    if p_pos > args.pos_max:
        return f"P+ {p_pos:.1f} > {args.pos_max:.1f} (양압 레일 차압이 모자란다)"
    if p_neg > args.neg_max:
        return f"P− {p_neg:.1f} > {args.neg_max:.1f} (음압 챔버는 대기압 위로 못 간다)"
    if p_neg < args.neg_min:
        return f"P− {p_neg:.1f} < {args.neg_min:.1f} (진공 레일 차압이 모자란다)"
    return None


def build_points(args) -> tuple[list, list]:
    """(center, diff, P+, 방향) 목록과, 도달 불가로 뺀 목록을 같이 준다.

    공통압마다 차압을 올렸다 내린다 — 되돌아오는 쪽의 차이가 히스테리시스다.
    """
    if getattr(args, "diff_list", None):
        diffs = list(args.diff_list)
    else:
        diffs = []
        d = args.diff_start
        while d <= args.diff_stop + 1e-9:
            diffs.append(round(d, 3))
            d += args.diff_step
    raw = []
    for c in args.centers:
        keep = [d for d in diffs if reachable(c + d / 2.0, c - d / 2.0, args) is None]
        for d in diffs:
            raw.append((c, d, "up"))
        if not args.one_way and keep:
            for d in reversed([x for x in diffs if x != max(keep)]):
                raw.append((c, d, "down"))

    ok, bad = [], []
    for c, d, w in raw:
        p_pos, p_neg = c + d / 2.0, c - d / 2.0
        why = reachable(p_pos, p_neg, args)
        if why is None:
            ok.append((c, d, p_pos, w))
        elif w == "up":                 # 같은 점을 왕복으로 두 번 알리지 않는다
            bad.append((c, d, why))
    return ok, bad


# ════════════════════════════════════════════════════════════════════════════
#  분석 — 왕복 두 곡선에서 마찰을 분리한다
# ════════════════════════════════════════════════════════════════════════════
# 한 방향 스윕 한 점은 **토크가 아니라 구간**이다. 정지 마찰이 고정값이 아니라
# 필요한 만큼 공급되는 밴드라서, 팔이 멈춘 자리면 어디든 균형이 맞기 때문이다:
#
#     tau_act ∈ [ m·g·L·sin(θ) − tau_fric_max , m·g·L·sin(θ) + tau_fric_max ]
#
# 올릴 때는 마찰이 아래로, 내릴 때는 위로 작용하므로 **같은 각도**에서
#     tau_up   = m·g·L·sin(θ) + tau_fric
#     tau_down = m·g·L·sin(θ) − tau_fric
# 이고, 두 곡선을 각도 기준으로 겹치면
#     진짜 토크 = (up + down)/2      마찰 = (up − down)/2
# 가 나온다. 압력 기준으로 짝지으면 안 된다 — 같은 압력에서 각도가 다르다.
def analyze(path: str, args) -> int:
    with open(path, newline="", encoding="utf-8") as fh:
        rows = [r for r in csv.DictReader(fh)]
    if not rows:
        print("[중단] 빈 파일이다.", file=sys.stderr)
        return 2

    kmax = args.mass * G * args.link          # 2.943 N·m
    out_rows = []
    centers = sorted({r["center_kpa"] for r in rows}, key=float)
    print(f"\n분석: {path}")
    print(f"중력 상수 m·g·L = {kmax:.4f} N·m,  각도 0점 {args.angle_offset:g}°")

    for c in centers:
        sel = [r for r in rows if r["center_kpa"] == c and int(r["settled"])]
        up = sorted([r for r in sel if r["sweep"] == "up"],
                    key=lambda r: float(r["angle_deg"]))
        dn = sorted([r for r in sel if r["sweep"] == "down"],
                    key=lambda r: float(r["angle_deg"]))
        print(f"\n── 공통압 {c} kPa  (올림 {len(up)} 점 / 내림 {len(dn)} 점)")
        if len(up) < 2 or len(dn) < 2:
            print("   왕복 두 곡선이 다 있어야 마찰을 뗄 수 있다 "
                  "(--one-way 로 돌렸거나 정착 실패가 많다). 건너뛴다.")
            continue

        # 두 곡선이 **겹치는** 각도 구간에서만 비교한다. 밖은 외삽이라 의미 없다.
        lo = max(float(up[0]["angle_deg"]), float(dn[0]["angle_deg"]))
        hi = min(float(up[-1]["angle_deg"]), float(dn[-1]["angle_deg"]))
        if hi - lo < 1.0:
            print(f"   겹치는 각도 구간이 {hi-lo:.1f}° 뿐이라 비교할 수 없다.")
            continue

        def interp(curve, key, th):
            xs = [float(r["angle_deg"]) for r in curve]
            ys = [float(r[key]) for r in curve]
            for i in range(1, len(xs)):
                if xs[i] >= th:
                    f = (th - xs[i-1]) / max(1e-9, xs[i] - xs[i-1])
                    return ys[i-1] + f * (ys[i] - ys[i-1])
            return ys[-1]

        # 같은 각도면 중력 토크도 같다. 그래서 마찰은 "같은 각도를 만드는 데
        # 필요한 **차압**이 올릴 때와 내릴 때 얼마나 다른가" 로 나타난다.
        print(f"   {'각도':>7} {'τ_중력':>8} │ {'diff_up':>8} {'diff_dn':>8} "
              f"{'간격':>7}  (간격이 곧 마찰이다)")
        n = 7
        for k in range(n):
            th = lo + (hi - lo) * k / (n - 1)
            tau = kmax * math.sin(math.radians(th - args.angle_offset))
            du = interp(up, "diff_kpa", th)
            dd = interp(dn, "diff_kpa", th)
            out_rows.append((c, th, tau, du, dd))
            print(f"   {th:7.2f} {tau:8.3f} │ {du:8.2f} {dd:8.2f} {du-dd:7.2f}")

        # 마찰을 토크 단위로: 겹치는 구간에서 dτ/d(diff) 를 회귀로 잡는다.
        pts = [(float(r["diff_kpa"]),
                kmax * math.sin(math.radians(float(r["angle_deg"]) - args.angle_offset)))
               for r in up + dn]
        mx = sum(p[0] for p in pts) / len(pts)
        my = sum(p[1] for p in pts) / len(pts)
        den = sum((p[0]-mx)**2 for p in pts)
        slope = sum((p[0]-mx)*(p[1]-my) for p in pts) / den if den > 1e-9 else float("nan")
        gaps = [interp(up, "diff_kpa", lo + (hi-lo)*k/6) - interp(dn, "diff_kpa", lo + (hi-lo)*k/6)
                for k in range(7)]
        gap_m = sum(gaps) / len(gaps)
        print(f"\n   dτ/d(차압) = {slope:+.4f} N·m/kPa   (= 유효 토크이득)")
        print(f"   왕복 차압 간격 평균 {gap_m:+.2f} kPa "
              f"→ 마찰 ±{abs(slope*gap_m)/2:.3f} N·m")
        if abs(slope) > 1e-9:
            span = abs(slope) * (float(up[-1]["diff_kpa"]) - float(up[0]["diff_kpa"]))
            print(f"   측정 토크 폭 {span:.3f} N·m 대비 마찰 "
                  f"{100*abs(slope*gap_m)/2/max(1e-9,span):.1f} %")
            print("   → 이 비율이 피드포워드만 썼을 때의 **오차 하한**이다.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description="액추에이터 정특성 (P+,P−)→각도 매핑",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--axis", type=int, default=1, help="측정할 축 (1~6). 한 번에 하나만")
    ap.add_argument("--enc-idx", type=int, default=None,
                    help="board/analog 인덱스 (기본 = axis-1)")
    ap.add_argument("--centers", default="101.3,108,115",
                    help="공통압 목록 [kPa abs], 쉼표. P+=center+diff/2, P−=center−diff/2. "
                         "챔버가 한쪽으로만 갈 수 있어 도달 범위가 좁다 — 기본값은 "
                         "대기압(=차압 0 에서 무부하)에서 시작해 올라간다")
    ap.add_argument("--diff-start", type=float, default=0.0, help="차압 시작 [kPa]")
    ap.add_argument("--diff-stop", type=float, default=40.0, help="차압 끝 [kPa]")
    ap.add_argument("--diff-step", type=float, default=5.0, help="차압 간격 [kPa]")
    ap.add_argument("--diffs", default=None,
                    help="차압을 **직접 나열**한다 [kPa], 쉼표. --diff-start/stop/step 대신. "
                         "중력 토크가 m·g·L·sin(θ) 라 위로 갈수록 각도가 차압에 "
                         "민감해진다 — 등간격으로는 위쪽이 통째로 묻힌다")
    ap.add_argument("--preset", choices=("lift90",), default=None,
                    help="lift90 = 0~85° 를 고르게 훑는 차압 목록을 자동으로 만든다. "
                         "--mass/--link 와 1축 실측(차압 40 → 46.9°)에서 계산한다")
    ap.add_argument("--one-way", action="store_true",
                    help="되돌아오는 구간을 생략한다 (기본은 왕복 — 히스테리시스를 잰다)")

    g = ap.add_argument_group("속도 — 느릴수록 안전하다")
    ap.add_argument("--slew", type=float, default=2.0,
                    help="목표 압력 변화율 [kPa/s]. 계단은 절대 안 준다")
    ap.add_argument("--release-slew", type=float, default=4.0,
                    help="끝내거나 중단할 때 대기압으로 내리는 속도 [kPa/s]")
    ap.add_argument("--settle-rate", type=float, default=0.3,
                    help="이 각속도 [°/s] 아래면 멈춘 것으로 본다")
    ap.add_argument("--settle-hold", type=float, default=3.0,
                    help="그 상태가 이만큼 [s] 유지돼야 한 점으로 인정한다")
    ap.add_argument("--settle-timeout", type=float, default=60.0,
                    help="이 시간 [s] 안에 안 멈추면 '정착 실패'로 기록하고 넘어간다")
    ap.add_argument("--rate-tau", type=float, default=0.3, help="각속도 LPF 시정수 [s]")

    s = ap.add_argument_group("안전 한계 — 하나라도 걸리면 즉시 대기압으로")
    ap.add_argument("--max-rate", type=float, default=25.0, help="각속도 상한 [°/s]")
    ap.add_argument("--ang-min", type=float, default=-5.0, help="각도 하한 [°]")
    ap.add_argument("--ang-max", type=float, default=125.0, help="각도 상한 [°]")
    ap.add_argument("--track-tol", type=float, default=4.0,
                    help="압력 추종 오차 허용 [kPa]")
    ap.add_argument("--track-grace", type=float, default=5.0,
                    help="그 오차가 이만큼 [s] 지속되면 중단")
    ap.add_argument("--enc-timeout", type=float, default=1.0, help="엔코더 무갱신 한계 [s]")

    lim = ap.add_argument_group("도달 범위 — 챔버는 한쪽으로만 갈 수 있다")
    ap.add_argument("--pos-min", type=float, default=103.0,
                    help="P+ 하한. 양압 챔버는 대기압(101.3) 아래로 못 간다")
    ap.add_argument("--pos-max", type=float, default=None,
                    help="P+ 상한 (기본 = 양압 레일 − 10, config 에서 읽는다)")
    ap.add_argument("--neg-max", type=float, default=100.0,
                    help="P− 상한. 음압 챔버는 대기압 위로 못 간다")
    ap.add_argument("--neg-min", type=float, default=None,
                    help="P− 하한 (기본 = 음압 레일 + 10, config 에서 읽는다)")
    ap.add_argument("--osc-window", type=float, default=4.0, help="진동 판정 창 [s]")
    ap.add_argument("--osc-flips", type=int, default=6,
                    help="그 창 안에서 각속도 부호가 이만큼 뒤집히면 진동으로 본다")

    m = ap.add_argument_group("기구 — 각도에서 토크·힘을 역산하는 데 쓴다")
    ap.add_argument("--mass", type=float, default=2.0, help="링크 끝 질량 [kg]")
    ap.add_argument("--link", type=float, default=0.15, help="링크 길이 [m]")
    ap.add_argument("--reel-dia", type=float, default=0.05, help="릴 지름 [m]")
    ap.add_argument("--angle-offset", type=float, default=0.0,
                    help="중력 토크 = m·g·L·sin(θ − offset) [°]. "
                         "θ=offset 에서 중력 토크가 0 이다")

    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2293)
    ap.add_argument("--connect-timeout", type=float, default=30.0)
    ap.add_argument("--send-hz", type=float, default=20.0)
    ap.add_argument("--out", default=None, help="CSV 경로 (기본 ~/result/actuator_map_<ts>.csv)")
    ap.add_argument("--analyze", default=None, metavar="CSV",
                    help="측정 CSV 를 읽어 왕복 두 곡선에서 **마찰을 분리**한다. "
                         "실기에 아무것도 보내지 않는다")
    ap.add_argument("--dry-run", action="store_true", help="계획만 찍고 끝낸다")
    ap.add_argument("--yes", action="store_true", help="확인 프롬프트를 건너뛴다")
    args = ap.parse_args()

    if args.analyze:
        return analyze(args.analyze, args)
    if not 1 <= args.axis <= NUM_AXES:
        ap.error("--axis 는 1~6")
    if args.enc_idx is None:
        args.enc_idx = args.axis - 1
    args.centers = [float(v) for v in args.centers.replace(" ", "").split(",") if v]
    if args.diff_step <= 0:
        ap.error("--diff-step 은 0 보다 커야 한다")

    # ── 차압 목록 ────────────────────────────────────────────────────────
    args.diff_list = None
    if args.preset == "lift90":
        # 목표 각도를 고르게 잡고 **거기에 필요한 차압**을 역산한다.
        #   τ(θ) = m·g·L·sin(θ),  τ = k·차압   (k 는 아래 실측에서)
        # 실측 기준점: 1축에서 차압 40 kPa → 46.9° (20260912_210653)
        #   k = 2.943·sin(46.9°)/40 = 0.0537 N·m/kPa
        # 등간격 차압으로 뜨면 위쪽이 묻힌다 — 60°→90° 가 차압 7 kPa 안에 들어간다.
        k = args.mass * G * args.link * math.sin(math.radians(46.9)) / 40.0
        tgt = [0, 10, 20, 30, 40, 50, 55, 60, 65, 70, 74, 78, 81, 83, 85]
        args.diff_list = [round(args.mass * G * args.link *
                                math.sin(math.radians(a_)) / k, 1) for a_ in tgt]
        args.preset_angles = tgt
    elif args.diffs:
        try:
            args.diff_list = [float(v) for v in args.diffs.replace(" ", "").split(",") if v]
        except ValueError:
            ap.error("--diffs 는 쉼표로 구분한 숫자다")
    if args.diff_list:
        args.diff_start = min(args.diff_list)
        args.diff_stop = max(args.diff_list)

    cfg_path = os.path.join(_HERE, "..", "config", "powerpack_config.yaml")
    if args.pos_max is None or args.neg_min is None:
        import yaml
        with open(cfg_path, encoding="utf-8") as fh:
            _lp = yaml.safe_load(fh)["/pack2/pp_controller"]["ros__parameters"]["LinePID"]
        # 레일에 바짝 붙으면 차압이 0 이라 밸브를 활짝 열어도 안 움직인다. 10 kPa 를 남긴다.
        if args.pos_max is None:
            args.pos_max = float(_lp["pos"]["ref"]) - 10.0
        if args.neg_min is None:
            args.neg_min = float(_lp["neg"]["ref"]) + 10.0

    pts, dropped = build_points(args)
    if not pts:
        print("[중단] 도달 가능한 점이 하나도 없다. --centers / --diff-* 를 확인할 것.",
              file=sys.stderr)
        for c, d, why in dropped[:10]:
            print(f"   center {c:g} diff {d:g}: {why}", file=sys.stderr)
        return 2
    lo_neg = min(c - d / 2.0 for c, d, _, _ in pts)
    hi_pos = max(p for _, _, p, _ in pts)

    print("\n=== 액추에이터 정특성 측정 계획 ===")
    print(f"축 {args.axis} (board/analog[{args.enc_idx}])")
    print(f"공통압 {', '.join('%g' % c for c in args.centers)} kPa abs")
    if args.diff_list:
        lab = ", ".join(f"{v:g}" for v in args.diff_list)
        print(f"차압 [{lab}] kPa"
              f"{'' if args.one_way else ' (왕복 — 히스테리시스 측정)'}")
        if getattr(args, "preset_angles", None):
            print("  목표 각도    " + " ".join(f"{a_:>5d}" for a_ in args.preset_angles))
            print("  필요 차압    " + " ".join(f"{v:5.1f}" for v in args.diff_list))
            print("  ⚠ 90° 위는 중력 복원 토크가 **줄어들어** 정적 평형이 불안정하다 "
                  "(각도↑ → 복원토크↓ → 스톱까지 간다).")
            print("    그래서 85° 에서 멈춘다. 그 위를 쓰려면 위치 피드백이 필요하다.")
    else:
        print(f"차압 {args.diff_start:g} → {args.diff_stop:g} kPa, {args.diff_step:g} 간격"
          f"{'' if args.one_way else ' (왕복 — 히스테리시스 측정)'}")
    print(f"압력 범위: P+ 최대 {hi_pos:.1f} / P− 최소 {lo_neg:.1f} kPa abs")
    if dropped:
        print(f"\n도달 불가로 **{len(dropped)} 점을 뺐다** "
              f"(P+ {args.pos_min:g}~{args.pos_max:g} / P− {args.neg_min:g}~{args.neg_max:g}):")
        for c, d, why in dropped[:8]:
            print(f"   center {c:g} diff {d:g} — {why}")
        if len(dropped) > 8:
            print(f"   ... 외 {len(dropped)-8} 점")
        print()
    print(f"점 {len(pts)} 개 · 목표 변화율 {args.slew:g} kPa/s · "
          f"정착 {args.settle_rate:g} °/s 가 {args.settle_hold:g} s")
    est = sum(args.settle_hold + 6.0 for _ in pts) + len(pts) * args.diff_step / max(0.1, args.slew)
    print(f"대략 {est/60.0:.0f} 분 (정착이 빠르면 더 짧다)")
    print(f"중력 토크 = {args.mass:g}·9.81·{args.link:g}·sin(θ−{args.angle_offset:g}°) "
          f"= {args.mass*G*args.link:.3f}·sin(...) N·m,  릴 반지름 {args.reel_dia/2:.4f} m")
    print("\n안전 한계: 각속도 %g °/s · 각도 [%g, %g]° · 추종 %g kPa/%g s · 진동 %d회/%g s"
          % (args.max_rate, args.ang_min, args.ang_max, args.track_tol,
             args.track_grace, args.osc_flips, args.osc_window))
    print("중단되면 **양 챔버를 %g kPa/s 로 대기압까지 내린다** (끊지 않는다)."
          % args.release_slew)
    if args.dry_run:
        for i, (c, d, p, w) in enumerate(pts, 1):
            print(f"  {i:3d} {w:4s} center {c:6.1f}  diff {d:5.1f}  "
                  f"→ P+ {p:6.1f} / P− {c-d/2:6.1f}")
        return 0
    if not args.yes:
        if input("\n비상정지를 확인했으면 RUN 을 입력: ").strip() != "RUN":
            print("취소했다.")
            return 2

    cfg = read_config(cfg_path)
    out = args.out or os.path.expanduser(
        f"~/result/actuator_map_{datetime.now():%Y%m%d_%H%M%S}.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    rclpy.init()
    rig = RigView(cfg, args.enc_idx)
    group = (args.axis - 1,)
    guard = Guard(args.max_rate, args.ang_min, args.ang_max, args.track_tol,
                  args.track_grace, args.enc_timeout, args.osc_window, args.osc_flips)
    conn = None
    drv = None
    rc = 0
    rows = 0
    f = open(out, "w", newline="", buffering=1)
    wr = csv.writer(f)
    wr.writerow(["idx", "sweep", "center_kpa", "diff_kpa",
                 "p_pos_ref", "p_neg_ref", "p_pos_meas", "p_neg_meas",
                 "angle_deg", "angle_std_deg", "settled", "settle_s", "n_sample",
                 "tau_grav_Nm", "force_N"])
    try:
        conn = _connect(args)
        drv = Driver(conn, group, args.send_hz, args.slew)
        # 시작 전에 대기압을 잠깐 유지해 컨트롤러 적분을 안정시킨다.
        t_end = time.monotonic() + 2.0
        while time.monotonic() < t_end:
            drv.hold(); _spin(rig, drv.dt)

        for i, (c, d, p_pos, way) in enumerate(pts, 1):
            p_neg = c - d / 2.0
            print(f"\n[{i}/{len(pts)}] {way:4s} center {c:g} diff {d:g} "
                  f"→ P+ {p_pos:.1f} / P− {p_neg:.1f} kPa", flush=True)
            r = goto_and_settle(rig, drv, guard, p_pos, p_neg, args)
            th = math.radians(r["angle_deg"] - args.angle_offset)
            tau = args.mass * G * args.link * math.sin(th)
            force = tau / (args.reel_dia / 2.0) if args.reel_dia > 0 else math.nan
            wr.writerow([i, way, f"{c:g}", f"{d:g}",
                         f"{p_pos:.2f}", f"{p_neg:.2f}",
                         f"{r['p_pos_meas']:.2f}", f"{r['p_neg_meas']:.2f}",
                         f"{r['angle_deg']:.3f}", f"{r['angle_std']:.3f}",
                         int(r["settled"]), f"{r['settle_s']:.1f}", r["n_sample"],
                         f"{tau:.4f}", f"{force:.2f}"])
            rows += 1
            flag = "" if r["settled"] else "  ← 정착 실패"
            print(f"      각도 {r['angle_deg']:7.2f}° (σ {r['angle_std']:.2f}) "
                  f"실측 {r['p_pos_meas']:.1f}/{r['p_neg_meas']:.1f} "
                  f"τ {tau:+.3f} N·m  F {force:+.1f} N  "
                  f"{r['settle_s']:.0f} s{flag}", flush=True)
        print(f"\n[완료] {rows} 점을 {out} 에 적었다.")
    except Abort as exc:
        print(f"\n[중단] 안전 감시: {exc}", file=sys.stderr)
        rc = 3
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C", file=sys.stderr)
        rc = 130
    except (ConnectionError, OSError, TimeoutError) as exc:
        print(f"\n[위험] 통신 실패: {exc}", file=sys.stderr)
        rc = 1
    finally:
        # ── 어떤 경로로 끝나든 반드시 대기압까지 **기울여** 내린다 ──────────
        # 끊으면 컨트롤러에 마지막 목표가 남아 압력이 유지된다.
        if drv is not None:
            print(f"[복귀] 양 챔버를 {args.release_slew:g} kPa/s 로 대기압까지 내린다...")
            t_end = time.monotonic() + 120.0
            try:
                while time.monotonic() < t_end:
                    if drv.step_toward(ATM_KPA, ATM_KPA, slew=args.release_slew):
                        break
                    _spin(rig, drv.dt)
                for _ in range(int(1.0 * args.send_hz)):
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
        if rows:
            print(f"[기록] {out}  ({rows} 점)")
    return rc


if __name__ == "__main__":
    sys.exit(main())
