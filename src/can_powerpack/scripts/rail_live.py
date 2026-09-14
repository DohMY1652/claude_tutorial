#!/usr/bin/env python3
"""레일 목표를 손으로 바꿔 가며 추종을 **실시간으로 본다**.

rail_ref.py 는 정해진 계단열을 자동으로 도는 도구다. 이건 반대로,
목표를 타이핑해서 아무 때나 바꾸고 그 반응을 눈으로 보는 도구다.

화면
----
  · 목표 / 실측 / 오차 / 15 s 리플 / 개도(ff 대비)
  · 최근 90 초 궤적을 ASCII 로 그린다 (목표는 `·`, 실측은 `█`)

입력 (엔터로 실행)
------------------
  160        양압 레일 목표 [kPa abs]
  n 40       음압 레일 목표
  160 40     둘 다
  +10 / -10  현재 목표에서 상대 변경
  r          리플·오차 통계 초기화
  q          종료

안전
----
목표는 **계단으로 주지 않는다** — `--slew` [kPa/s] 로 기울인다. 피드포워드가
목표를 그대로 따라 개도를 내므로, 목표가 튀면 밸브도 튄다.
종료할 때 목표 발행을 멈출 뿐 **컨트롤러에는 마지막 값이 남는다.**
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, UInt16MultiArray

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
from valve_deadzone import read_config          # noqa: E402

NAMESPACE = "/pack2"
RAIL_POS_BOARD, RAIL_NEG_BOARD = 1, 2
PLOT_W, PLOT_H = 78, 14


class Live(Node):
    def __init__(self, cfg: dict) -> None:
        super().__init__("rail_live")
        self.offs, self.gains, self.atm = cfg["offs"], cfg["gains"], cfg["atm"]
        self._raw: list[int] | None = None
        self.rail = []                       # controller/rail_dbg
        self.ref_pos = math.nan
        self.ref_neg = math.nan
        self.tgt_pos = math.nan              # 사용자가 입력한 최종 목표
        self.tgt_neg = math.nan
        self.hist: deque = deque(maxlen=4000)   # (t, P+, ref+)
        self.create_subscription(UInt16MultiArray, f"{NAMESPACE}/board/sensors",
                                 self._on_sensors, 20)
        self.create_subscription(Float64MultiArray, f"{NAMESPACE}/controller/rail_dbg",
                                 self._on_rail, 10)
        self._pub = self.create_publisher(Float64MultiArray,
                                          f"{NAMESPACE}/controller/rail_ref_kpa", 5)

    def _on_sensors(self, m): self._raw = list(m.data)
    def _on_rail(self, m):    self.rail = list(m.data)

    def kpa(self, b):
        if self._raw is None or len(self._raw) < b: return None
        raw = self._raw[b - 1]
        if raw == 0: return None
        return (float(raw) - self.offs[b]) * self.gains[b] + self.atm

    def n_sub(self): return self._pub.get_subscription_count()

    def publish(self):
        if math.isnan(self.ref_pos) or math.isnan(self.ref_neg): return
        m = Float64MultiArray(); m.data = [float(self.ref_pos), float(self.ref_neg)]
        self._pub.publish(m)


def plot(hist, w=PLOT_W, h=PLOT_H) -> list[str]:
    """최근 궤적을 ASCII 로. 목표 `·`, 실측 `█`."""
    if len(hist) < 2:
        return ["  (데이터 대기)"]
    t0 = hist[-1][0]
    pts = [p for p in hist if t0 - p[0] <= 90.0]
    if len(pts) < 2:
        return ["  (데이터 대기)"]
    vals = [p[1] for p in pts] + [p[2] for p in pts if not math.isnan(p[2])]
    lo, hi = min(vals), max(vals)
    if hi - lo < 5.0:                         # 최소 눈금 폭 — 잡음이 화면을 채우지 않게
        mid = (hi + lo) / 2; lo, hi = mid - 2.5, mid + 2.5
    pad = (hi - lo) * 0.08; lo -= pad; hi += pad
    grid = [[" "] * w for _ in range(h)]
    span = max(1e-9, pts[-1][0] - pts[0][0])
    for tm, p, r in pts:
        x = min(w - 1, int((tm - pts[0][0]) / span * (w - 1)))
        for v, ch in ((r, "·"), (p, "█")):
            if math.isnan(v): continue
            y = int((hi - v) / (hi - lo) * (h - 1))
            if 0 <= y < h: grid[y][x] = ch
    out = []
    for i, row in enumerate(grid):
        lab = f"{hi - (hi-lo)*i/(h-1):7.1f} |"
        out.append(lab + "".join(row))
    out.append(" " * 8 + "+" + "-" * w)
    out.append(" " * 9 + f"{-min(90.0, span):.0f} s" + " " * (w - 14) + "now")
    return out


def reader(node: Live, stop: threading.Event, msg: list) -> None:
    while not stop.is_set():
        try:
            line = sys.stdin.readline()
        except Exception:
            return
        if not line:
            return
        s = line.strip()
        if not s:
            continue
        if s in ("q", "quit", "exit"):
            stop.set(); return
        if s == "r":
            node.hist.clear(); msg[0] = "통계 초기화"; continue
        try:
            if s.startswith("n "):
                node.tgt_neg = float(s[2:]); msg[0] = f"음압 목표 {node.tgt_neg:g}"
            elif s[0] in "+-" and len(s) > 1:
                node.tgt_pos += float(s); msg[0] = f"양압 목표 {node.tgt_pos:g}"
            else:
                parts = s.split()
                node.tgt_pos = float(parts[0])
                if len(parts) > 1: node.tgt_neg = float(parts[1])
                msg[0] = f"목표 P+ {node.tgt_pos:g}" + (
                    f" / P− {node.tgt_neg:g}" if len(parts) > 1 else "")
        except ValueError:
            msg[0] = f"못 읽었다: {s!r}  (예: 160 / n 40 / 160 40 / +10 / q)"


def main() -> int:
    ap = argparse.ArgumentParser(description="레일 목표 대화형 + 실시간 표시")
    ap.add_argument("--slew", type=float, default=5.0, help="목표 변화율 [kPa/s]")
    ap.add_argument("--hz", type=float, default=5.0, help="화면 갱신율")
    ap.add_argument("--send-hz", type=float, default=20.0)
    args = ap.parse_args()

    cfg = read_config(os.path.join(_HERE, "..", "config", "powerpack_config.yaml"))
    rclpy.init()
    node = Live(cfg)

    t_end = time.monotonic() + 3.0
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.n_sub() == 0:
        print("[중단] controller/rail_ref_kpa 를 구독하는 노드가 없다.\n"
              "  pp_controller 가 떠 있는가? control_mode 2 면 무시한다 "
              "— control_mode:=0 으로 띄울 것.", file=sys.stderr)
        rclpy.shutdown(); return 2
    p0, n0 = node.kpa(RAIL_POS_BOARD), node.kpa(RAIL_NEG_BOARD)
    if p0 is None or n0 is None:
        print("[중단] 레일 압력을 못 읽는다 — 브리지/CAN 확인.", file=sys.stderr)
        rclpy.shutdown(); return 2
    # 현재 압력에서 출발한다. 목표가 튀면 피드포워드 개도도 같이 튄다.
    node.ref_pos = node.tgt_pos = max(102.0, min(250.0, p0))
    node.ref_neg = node.tgt_neg = max(10.0, min(101.0, n0))

    stop = threading.Event(); msg = [""]
    threading.Thread(target=reader, args=(node, stop, msg), daemon=True).start()
    os.system("clear")
    t0 = time.monotonic()
    dt_send = 1.0 / args.send_hz
    last_draw = 0.0
    try:
        while not stop.is_set():
            # 목표를 기울여 옮긴다
            step = args.slew * dt_send
            for a, b in (("ref_pos", "tgt_pos"), ("ref_neg", "tgt_neg")):
                cur, tgt = getattr(node, a), getattr(node, b)
                if math.isnan(tgt): continue
                setattr(node, a, tgt if abs(tgt - cur) <= step
                        else cur + math.copysign(step, tgt - cur))
            rclpy.spin_once(node, timeout_sec=0.0)
            node.publish()
            pp = node.kpa(RAIL_POS_BOARD)
            if pp is not None:
                node.hist.append((time.monotonic() - t0, pp, node.ref_pos))
            now = time.monotonic()
            if now - last_draw >= 1.0 / args.hz:
                last_draw = now
                draw(node, args, msg)
            time.sleep(dt_send)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()
        print("\n[주의] 마지막 레일 목표가 컨트롤러에 **남아 있다**. "
              "yaml 값으로 되돌리려면 컨트롤러를 다시 띄울 것.")
    return 0


def draw(node: Live, args, msg) -> None:
    pp, pn = node.kpa(RAIL_POS_BOARD), node.kpa(RAIL_NEG_BOARD)
    r = node.rail
    o = "\033[H\033[J"
    o += "========== 레일 목표 대화형 ==========\n"
    o += "  160 | n 40 | 160 40 | +10 | -10 | r(초기화) | q(종료)\n"
    if msg[0]:
        o += f"  > {msg[0]}\n"
    o += "-" * 88 + "\n"
    ep = (pp - node.ref_pos) if pp is not None else float("nan")
    en = (pn - node.ref_neg) if pn is not None else float("nan")
    o += (f"  P+  목표 {node.ref_pos:7.2f}"
          + (f" → {node.tgt_pos:.1f}" if abs(node.tgt_pos - node.ref_pos) > 0.05 else "      ")
          + f"   실측 {pp if pp is None else round(pp,2):>8}   오차 {ep:+7.2f}\n")
    o += (f"  P−  목표 {node.ref_neg:7.2f}"
          + (f" → {node.tgt_neg:.1f}" if abs(node.tgt_neg - node.ref_neg) > 0.05 else "      ")
          + f"   실측 {pn if pn is None else round(pn,2):>8}   오차 {en:+7.2f}\n")
    # 최근 15 s 리플
    t_now = node.hist[-1][0] if node.hist else 0.0
    win = [h[1] for h in node.hist if t_now - h[0] <= 15.0]
    if len(win) > 10:
        o += (f"  15 s 리플 p-p {max(win)-min(win):6.2f} kPa   평균 {sum(win)/len(win):7.2f}\n")
    if len(r) >= 6:
        o += (f"  개도  방출 {r[4]:6.2f} % (ff {r[2]:6.2f})   "
              f"유입 {r[5]:6.2f} % (ff {r[3]:6.2f})")
        if len(r) >= 11:
            o += f"   적분 {r[9]:+6.2f}  게인배율 {r[10]:.2f}"
        o += "\n"
    o += "-" * 88 + "\n"
    o += "\n".join(plot(list(node.hist))) + "\n"
    sys.stdout.write(o); sys.stdout.flush()


if __name__ == "__main__":
    sys.exit(main())
