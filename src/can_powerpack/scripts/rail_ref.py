#!/usr/bin/env python3
"""레일 목표를 바꿔 가며 레일 제어를 확인한다.

무엇을 보는가
-------------
`controller/rail_ref_kpa` 로 (P+, P−) 목표를 주고, 레일이 **얼마나 빨리·정확히**
따라오는지 잰다. 채널은 건드리지 않는다 — 레일만 본다.

RailFF(피드포워드) 도입 전후를 가르는 지표는 **정착 시간** 하나다:
  · 피드포워드 없을 때: 개도 100 %%(활짝 열림)가 원점이라 적분이 동작점을 통째로
    만들어야 했다. 160 목표에서 **120 초**, 40~50 초 주기 왕복 (20260912 실측).
  · 피드포워드 있을 때: 목표가 바뀌면 개도가 **즉시** 그 목표의 동작점으로 뛴다.
    적분은 표 오차(열 표류 ~10 kPa 상당)만 메우면 된다.

목표를 계단으로 주지 않는다
---------------------------
`--slew` [kPa/s] 로 기울인다. 계단을 주면 피드포워드 개도도 같이 계단이 되어
밸브를 때린다. 실제 운전에서도 레일 목표는 천천히 움직인다.

사용
----
    ros2 launch can_powerpack control.launch.py      # 창 1
    python3 rail_ref.py --dry-run                    # 계획
    python3 rail_ref.py                              # 기본 계단열
    python3 rail_ref.py --pos 150,170,190 --neg 30   # 직접 지정
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, UInt16MultiArray

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
from valve_deadzone import read_config          # noqa: E402

NAMESPACE = "/pack2"
RAIL_POS_BOARD, RAIL_NEG_BOARD = 1, 2


class RailRef(Node):
    """목표를 발행하고 실측을 읽는다. **pwm 은 건드리지 않는다** — 제어는 컨트롤러가 한다."""

    def __init__(self, cfg: dict) -> None:
        super().__init__("rail_ref")
        self.offs, self.gains, self.atm = cfg["offs"], cfg["gains"], cfg["atm"]
        self._raw: list[int] | None = None
        self.ref_pos = math.nan
        self.ref_neg = math.nan
        self.create_subscription(UInt16MultiArray, f"{NAMESPACE}/board/sensors",
                                 self._on_sensors, 20)
        self._pub = self.create_publisher(Float64MultiArray,
                                          f"{NAMESPACE}/controller/rail_ref_kpa", 5)

    def _on_sensors(self, msg: UInt16MultiArray) -> None:
        self._raw = list(msg.data)

    def kpa(self, board: int) -> float | None:
        if self._raw is None or len(self._raw) < board:
            return None
        raw = self._raw[board - 1]
        if raw == 0:
            return None
        return (float(raw) - self.offs[board]) * self.gains[board] + self.atm

    def n_sub(self) -> int:
        """이 토픽을 **받는 쪽이 있는가.** 없으면 발행은 허공으로 간다 —
        컨트롤러가 안 떠 있거나, control_mode 2 라 구독조차 안 만든 경우다."""
        return self._pub.get_subscription_count()

    def publish(self) -> None:
        if math.isnan(self.ref_pos) or math.isnan(self.ref_neg):
            return
        m = Float64MultiArray()
        m.data = [float(self.ref_pos), float(self.ref_neg)]
        self._pub.publish(m)


def spin(rig: RailRef, dt: float) -> None:
    rclpy.spin_once(rig, timeout_sec=0.0)
    rig.publish()
    time.sleep(dt)


def run_step(rig: RailRef, tp: float, tn: float, args) -> dict:
    """목표를 기울여 옮긴 뒤 유지하며 정착·오버슛·정상오차를 잰다."""
    dt = 1.0 / args.send_hz
    step = args.slew * dt
    t0 = time.monotonic()
    t_arrive = None                 # 목표 램프가 끝난 시각
    t_settle = None                 # 처음 ±tol 안에 들어와 유지된 시각
    in_band_since = None
    peak = -1e9
    hist: list[tuple[float, float, float]] = []

    while True:
        done = True
        for name, tgt in (("ref_pos", tp), ("ref_neg", tn)):
            cur = getattr(rig, name)
            if abs(tgt - cur) <= step:
                setattr(rig, name, tgt)
            else:
                setattr(rig, name, cur + math.copysign(step, tgt - cur))
                done = False
        spin(rig, dt)

        pp, pn = rig.kpa(RAIL_POS_BOARD), rig.kpa(RAIL_NEG_BOARD)
        if pp is None or pn is None:
            if time.monotonic() - t0 > 5.0:
                raise RuntimeError("레일 압력을 못 읽는다 — 브리지/CAN 확인")
            continue
        if rig.n_sub() == 0:
            raise RuntimeError("목표 구독자가 사라졌다 — 컨트롤러가 죽었는지 확인할 것")
        now = time.monotonic()
        hist.append((now - t0, pp, pn))
        if done and t_arrive is None:
            t_arrive = now
        if t_arrive is not None:
            peak = max(peak, pp)
            # 정착: ±tol 안에 --settle-hold 초 연속으로 머물러야 한다.
            # 리플이 p-p 10 kPa 라 순간값이 아니라 **머문 시간**으로 본다.
            if abs(pp - tp) <= args.tol:
                if in_band_since is None:
                    in_band_since = now
                elif t_settle is None and now - in_band_since >= args.settle_hold:
                    t_settle = in_band_since
            else:
                in_band_since = None
            if now - t_arrive >= args.hold:
                break
        if now - t0 > args.hold + 240.0:
            break

    tail = [h for h in hist if h[0] >= hist[-1][0] - args.avg_window]
    mp = sum(h[1] for h in tail) / max(1, len(tail))
    mn = sum(h[2] for h in tail) / max(1, len(tail))
    rip = (max(h[1] for h in tail) - min(h[1] for h in tail)) if tail else math.nan
    return dict(
        p_pos=mp, p_neg=mn, err_pos=mp - tp, err_neg=mn - tn, rip_pos=rip,
        overshoot=(peak - tp) if peak > -1e8 else math.nan,
        settle_s=(t_settle - t0) if t_settle else math.nan,
        ramp_s=(t_arrive - t0) if t_arrive else math.nan,
        total_s=time.monotonic() - t0)


# ════════════════════════════════════════════════════════════════════════════
#  시나리오 — 한 번에 여러 범위를 훑는다
# ════════════════════════════════════════════════════════════════════════════
# 무엇을 가르려고 이렇게 짰는가 (실측 맵 rail_map_20260912_163805 기준):
#
#  · 플랜트 이득이 동작점마다 **1.67~9.88 kPa/%p 로 5.9 배** 달라진다. 게인
#    스케줄이 그걸 상쇄하도록 돼 있으니, 낮은 동작점(이득 작음)과 높은 동작점
#    (이득 큼)을 **둘 다** 봐야 스케줄이 맞는지 안다.
#  · P− 는 유입이 거의 단독 결정하지만 유입은 P+ 에도 같은 크기로 작용한다
#    (+1.247 대 −1.236). 그래서 **P+ 고정 · P− 만 변경** 구간이 있어야
#    커플링 보상이 되는지 갈린다.
#  · 큰 계단과 작은 계단을 나눈다. 큰 계단은 램프·포화를, 작은 계단은
#    정상오차·리플을 본다. 둘이 섞이면 원인을 못 가른다.
#  · 같은 목표를 **올라가서 한 번, 내려와서 한 번** 준다. 차이가 나면 적분이
#    아직 동작점을 만들고 있다는 뜻이다 (피드포워드가 제 몫을 하면 방향 무관).
SCENARIOS = {
    "full": [
        # (P+, P−, 설명)
        (160, 35, "기준 동작점 — 여기서 출발"),
        (170, 35, "작은 계단 +10 (소신호, 정상오차·리플)"),
        (160, 35, "작은 계단 −10 (같은 목표를 내려와서 — 방향 차이)"),
        (150, 35, "작은 계단 −10"),
        (160, 35, "작은 계단 +10 (올라와서 — 위 150→160 과 비교)"),

        (200, 35, "큰 계단 +40 (램프·오버슛)"),
        (145, 35, "큰 계단 −55 (내려가는 쪽 권한 — 예전 버그가 여기서 났다)"),

        (125, 35, "낮은 동작점 — 플랜트 이득이 작다(~1.7 kPa/%p). 게인배율이 커져야 한다"),
        (135, 35, "낮은 동작점 소신호"),
        (125, 35, "낮은 동작점 소신호 (반대 방향)"),

        (215, 35, "높은 동작점 — 이득이 크다. 게인배율이 작아져야 한다"),
        (225, 35, "높은 동작점 소신호 (릴리프 270 아래)"),
        (215, 35, "높은 동작점 소신호 (반대 방향)"),

        (160, 35, "기준으로 복귀 — 맨 앞과 같은 목표. 열 표류가 여기서 보인다"),
        (160, 45, "**P+ 고정, P− 만 +10** — 커플링 보상 확인"),
        (160, 30, "**P+ 고정, P− 만 −15** — 같은 확인, 반대 방향"),
        (160, 35, "P− 복귀"),

        (190, 45, "P+·P− **동시 변경** — 실제 운전에 가장 가깝다"),
        (140, 30, "동시 변경, 반대 방향"),
        (160, 35, "종료 동작점"),
    ],
    "quick": [
        (160, 35, "기준"),
        (200, 35, "큰 계단 +40"),
        (145, 35, "큰 계단 −55"),
        (125, 35, "낮은 동작점 (이득 작음)"),
        (215, 35, "높은 동작점 (이득 큼)"),
        (160, 45, "P+ 고정, P− 만 변경 (커플링)"),
        (160, 35, "복귀 — 맨 앞과 비교하면 표류가 보인다"),
    ],
}


def hold_forever(rig: RailRef, tp: float, tn: float, args) -> int:
    """목표까지 기울인 뒤 **계속 유지한다.** Ctrl-C 까지.

    채널 시험(pressure_sweep_server.py)을 돌리는 동안 레일을 한 값에 붙들어 두는
    용도다. 컨트롤러는 마지막으로 받은 목표를 유지하므로 발행을 멈춰도 값은
    남지만, 그러면 **레일이 실제로 그 값을 지키고 있는지 볼 방법이 없다.**
    계속 띄워 두고 한 줄 상태를 보는 편이 낫다.
    """
    dt = 1.0 / args.send_hz
    step = args.slew * dt
    t0 = time.monotonic()
    hist: list[tuple[float, float]] = []
    last = 0.0
    print(f"\n=== 레일 목표 유지: P+ {tp:g} / P− {tn:g} ===")
    print("Ctrl-C 로 종료. 이 창을 띄워 둔 채 다른 창에서 채널 시험을 돌리면 된다.\n")
    try:
        while True:
            for name, tgt in (("ref_pos", tp), ("ref_neg", tn)):
                cur = getattr(rig, name)
                setattr(rig, name, tgt if abs(tgt - cur) <= step
                        else cur + math.copysign(step, tgt - cur))
            spin(rig, dt)
            pp, pn = rig.kpa(RAIL_POS_BOARD), rig.kpa(RAIL_NEG_BOARD)
            now = time.monotonic()
            if pp is not None:
                hist.append((now, pp))
                hist[:] = [h for h in hist if now - h[0] <= args.avg_window]
            if now - last >= 2.0 and pp is not None and pn is not None:
                last = now
                v = [h[1] for h in hist]
                rip = (max(v) - min(v)) if len(v) > 10 else float("nan")
                mean = sum(v) / len(v) if v else pp
                print(f"  [{now-t0:6.0f} s] P+ {pp:7.2f} (평균 {mean:7.2f}, "
                      f"오차 {mean-tp:+6.2f}, {args.avg_window:g}s 리플 {rip:5.2f})"
                      f"   P− {pn:6.2f} ({pn-tn:+5.2f})", flush=True)
            if rig.n_sub() == 0:
                print("\n[중단] 목표 구독자가 사라졌다 — 컨트롤러가 죽었는지 확인할 것.",
                      file=sys.stderr)
                return 3
    except KeyboardInterrupt:
        print("\n[종료] Ctrl-C")
        return 130


def main() -> int:
    ap = argparse.ArgumentParser(description="레일 목표 추종 확인",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--pos", default="145,160,180,200,170,145",
                    help="양압 레일 목표열 [kPa abs], 쉼표")
    ap.add_argument("--neg", default="30",
                    help="음압 레일 목표열. 하나면 전 구간 고정, 여러 개면 --pos 와 짝을 맞춘다")
    ap.add_argument("--slew", type=float, default=5.0,
                    help="목표 변화율 [kPa/s]. 계단을 주면 피드포워드 개도도 계단이 된다")
    ap.add_argument("--hold", type=float, default=45.0, help="목표 도달 후 유지 [s]")
    ap.add_argument("--tol", type=float, default=3.0, help="정착 판정 밴드 [kPa]")
    ap.add_argument("--settle-hold", type=float, default=3.0,
                    help="밴드 안에 이만큼 [s] 연속으로 머물러야 정착으로 본다")
    ap.add_argument("--avg-window", type=float, default=15.0, help="정상값 평균 구간 [s]")
    # 컨트롤러의 RailRef.{pos,neg}_{min,max}_kpa 와 맞춰 둔다. 벗어난 목표는
    # 컨트롤러가 잘라내고 경고를 쏟으므로, 보낼 때 미리 맞추는 게 낫다.
    ap.add_argument("--pos-min", type=float, default=101.325)
    ap.add_argument("--pos-max", type=float, default=250.0)
    ap.add_argument("--neg-min", type=float, default=10.0)
    ap.add_argument("--neg-max", type=float, default=101.325)
    ap.add_argument("--hold-ref", nargs=2, type=float, metavar=("POS", "NEG"),
                    default=None,
                    help="이 목표까지 기울인 뒤 **계속 유지한다** (Ctrl-C 까지). "
                         "채널 시험을 돌리는 동안 레일을 붙들어 두는 용도")
    ap.add_argument("--scenario", choices=tuple(SCENARIOS), default=None,
                    help="정해진 시나리오로 여러 범위를 한 번에 훑는다. "
                         "full=20단계, quick=7단계. --pos/--neg 를 무시한다")
    ap.add_argument("--send-hz", type=float, default=20.0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--yes", action="store_true")
    args = ap.parse_args()

    if args.scenario:
        seq = SCENARIOS[args.scenario]
        pos = [float(a_) for a_, _, _ in seq]
        neg = [float(b_) for _, b_, _ in seq]
        labels = [c for _, _, c in seq]
    else:
        pos = [float(v) for v in args.pos.replace(" ", "").split(",") if v]
        neg = [float(v) for v in args.neg.replace(" ", "").split(",") if v]
        if len(neg) == 1:
            neg = neg * len(pos)
        if len(neg) != len(pos):
            ap.error("--neg 는 1 개이거나 --pos 와 같은 개수여야 한다")
        labels = [""] * len(pos)

    if args.hold_ref:
        print(f"\n=== 레일 목표 유지 모드 ===\nP+ {args.hold_ref[0]:g} / "
              f"P− {args.hold_ref[1]:g} 로 {args.slew:g} kPa/s 로 옮긴 뒤 계속 유지한다.")
        if args.dry_run:
            return 0
        if not args.yes:
            if input("RUN 을 입력: ").strip() != "RUN":
                print("취소했다."); return 2
    else:
        print("\n=== 레일 목표 추종 확인 ===")
    print(f"단계 {len(pos)} 개 · 목표 변화율 {args.slew:g} kPa/s · 유지 {args.hold:g} s")
    print(f"정착 = |P+ − 목표| ≤ {args.tol:g} kPa 가 {args.settle_hold:g} s 연속")
    for i, (a, b, lab) in enumerate(zip(pos, neg, labels), 1):
        print(f"  {i:2d}  P+ {a:6.1f}   P− {b:5.1f}   {lab}")
    est = sum(abs(pos[i] - (pos[i-1] if i else 101.3)) for i in range(len(pos))) / args.slew \
        + len(pos) * args.hold
    print(f"대략 {est/60.0:.0f} 분")
    print("\n채널은 건드리지 않는다 — pwm 을 발행하지 않고 레일 목표만 준다.")
    print("피드포워드가 제 몫을 하면 **정착이 20~30 초 안**이어야 한다 (예전 120 초).")
    if args.dry_run:
        return 0
    if not args.yes:
        if input("\n비상정지를 확인했으면 RUN 을 입력: ").strip() != "RUN":
            print("취소했다."); return 2

    cfg = read_config(os.path.join(_HERE, "..", "config", "powerpack_config.yaml"))
    out = args.out or os.path.expanduser(
        f"~/result/rail_ref_{datetime.now():%Y%m%d_%H%M%S}.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    rclpy.init()
    rig = RailRef(cfg)
    rc, rows = 0, 0
    f = open(out, "w", newline="", buffering=1)
    wr = csv.writer(f)
    wr.writerow(["idx", "label", "ref_pos", "ref_neg", "p_pos", "p_neg",
                 "err_pos", "err_neg", "overshoot", "ramp_s", "settle_s",
                 "rip_pos", "total_s"])
    try:
        # 현재 레일 압력에서 출발한다 — 첫 목표를 현재값으로 놓고 램프를 시작해야
        # 기동 순간에 목표가 튀지 않는다.
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            rclpy.spin_once(rig, timeout_sec=0.0)
            time.sleep(0.02)
        # ── 받는 쪽이 있는지 먼저 본다 ────────────────────────────────────
        # 이게 없으면 "목표를 줬는데 아무 반응이 없다" 로만 보이고 원인을 알 수 없다.
        if rig.n_sub() == 0:
            print("\n[중단] controller/rail_ref_kpa 를 **구독하는 노드가 없다.**\n"
                  "  1) pp_controller 가 떠 있는가?  ros2 node list 로 확인\n"
                  "  2) control_mode 가 2 면 구독은 하지만 무시한다 — "
                  "control_mode:=0 으로 띄울 것\n"
                  "  3) 컨트롤러가 이 기능이 없는 옛 빌드일 수 있다 — colcon build 후 재기동",
                  file=sys.stderr)
            return 2
        print(f"[확인] 목표 구독자 {rig.n_sub()} 개")

        p0, n0 = rig.kpa(RAIL_POS_BOARD), rig.kpa(RAIL_NEG_BOARD)
        if p0 is None or n0 is None:
            print("[중단] 레일 압력을 못 읽는다 — 브리지가 떠 있는지 확인할 것.\n"
                  "  board/sensors 가 오지 않는다. can_bridge_node 와 CAN 을 볼 것.",
                  file=sys.stderr)
            return 2
        # 컨트롤러가 받는 범위 밖이면 잘려서 경고만 쏟아진다. 여기서 미리 맞춘다.
        # (펌프를 안 켰으면 레일이 대기압 아래라 이 경우가 그대로 걸린다.)
        lo = min(min(pos), args.pos_min)
        p_start = max(args.pos_min, min(args.pos_max, p0))
        n_start = max(args.neg_min, min(args.neg_max, n0))
        if p_start != p0 or n_start != n0:
            print(f"[시작] 현재 레일 P+ {p0:.1f} / P− {n0:.1f} 는 컨트롤러 허용 범위 "
                  f"[{args.pos_min:g},{args.pos_max:g}] / [{args.neg_min:g},{args.neg_max:g}] "
                  f"밖이라 {p_start:.1f} / {n_start:.1f} 에서 출발한다")
        else:
            print(f"[시작] 현재 레일 P+ {p0:.1f} / P− {n0:.1f} 에서 출발한다")
        rig.ref_pos, rig.ref_neg = p_start, n_start

        if args.hold_ref:
            rc = hold_forever(rig, args.hold_ref[0], args.hold_ref[1], args)
            return rc
        del lo

        for i, (tp, tn, lab) in enumerate(zip(pos, neg, labels), 1):
            print(f"\n[{i}/{len(pos)}] 목표 P+ {tp:.1f}  P− {tn:.1f}"
                  + (f"   — {lab}" if lab else ""), flush=True)
            r = run_step(rig, tp, tn, args)
            wr.writerow([i, lab, f"{tp:g}", f"{tn:g}", f"{r['p_pos']:.2f}", f"{r['p_neg']:.2f}",
                         f"{r['err_pos']:+.2f}", f"{r['err_neg']:+.2f}",
                         f"{r['overshoot']:+.2f}", f"{r['ramp_s']:.1f}",
                         f"{r['settle_s']:.1f}", f"{r['rip_pos']:.2f}",
                         f"{r['total_s']:.1f}"])
            rows += 1
            st = f"{r['settle_s']:.0f} s" if r["settle_s"] == r["settle_s"] else "**미정착**"
            print(f"      P+ {r['p_pos']:7.2f} (오차 {r['err_pos']:+5.2f}, "
                  f"오버슛 {r['overshoot']:+5.1f}, 리플 {r['rip_pos']:4.1f})   "
                  f"P− {r['p_neg']:6.2f} ({r['err_neg']:+5.2f})   정착 {st}", flush=True)
        print(f"\n[완료] {rows} 단계 → {out}")
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C", file=sys.stderr); rc = 130
    except RuntimeError as exc:
        print(f"\n[중단] {exc}", file=sys.stderr); rc = 3
    finally:
        # 목표 발행을 멈춘다. 컨트롤러는 마지막 값을 유지하므로, yaml 기본으로
        # 되돌리려면 노드를 다시 띄워야 한다 — 그 사실을 알려 준다.
        f.close()
        rig.destroy_node()
        rclpy.shutdown()
        if rows:
            print(f"[기록] {out}  ({rows} 단계)")
        print("[주의] 마지막 레일 목표가 컨트롤러에 **남아 있다**. "
              "yaml 값으로 되돌리려면 컨트롤러를 다시 띄울 것.")
    return rc


if __name__ == "__main__":
    sys.exit(main())
