#!/usr/bin/env python3
"""레일 정특성 측정 — (벤트 개도, 유입 개도) → (P+, P−) 룩업테이블.

왜 2차원인가
------------
레일 두 개는 **펌프 하나로 이어진 닫힌 회로**다 (흡입 = 음압 라인,
토출 = 양압 라인). 그 회로의 유일한 출구·입구가 이 두 밸브다:

    board1 v1 (PWM 0) : 양압 라인 → 대기 **방출**.  닫으면 P+ 가 오른다.
    board2 v1 (PWM 3) : 대기 → 음압 라인 **유입**.  닫으면 P− 가 내려간다.

그런데 둘은 독립이 아니다. 방출을 열면 회로에서 기체가 빠져 P+ 만이 아니라
**P− 도 같이 내려간다.** 유입을 열면 회로에 기체가 들어와 P− 만이 아니라
**P+ 도 같이 오른다.** 지금 LinePID 가 SISO 두 개로 도는데, 서로의 작용을
외란으로만 보기 때문에 느리고 왕복한다 (20260912 실측: 160 목표에서
정착까지 120 초, 1주기 레일 136±13).

그래서 (u_vent, u_admit) → (P+, P−) 를 **2×2 정적 맵**으로 뜬다. 이걸 뒤집으면
목표 압력쌍에서 두 밸브 개도를 바로 낼 수 있고, PID 는 그 위의 잔차만 맡는다.
피드포워드가 동작점을 잡아 주므로 **높은 레일압에서도** 적분이 한계까지 차지
않는다 — 지금 160 에서 겪는 문제의 근본 해결이다.

안전
----
채널 밸브는 **전부 닫는다**(0 %). 챔버 압력이 얼어붙으므로 액추에이터가 움직이지
않는다. 위험한 것은 두 레일 밸브를 **동시에 닫는 구석**이다 — 펌프가 P+ 를 계속
올리고 P− 를 계속 내린다. 그래서:

  · P+ > --p-max 또는 P− < --p-min 이면 **즉시 두 밸브를 활짝 연다** (자연 릴리프)
  · 지령은 계단이 아니라 --slew [%/s] 로 기울인다
  · 압력 변화율이 --max-rate [kPa/s] 를 넘으면 중단
  · 어떤 경로로 끝나든 두 밸브를 100 % 로 열고 끝낸다

**pp_controller 가 떠 있으면 거부한다** — board/pwm_cmd 를 둘이 쓰면 지령이
번갈아 나가 아무 의미가 없다. 브리지만 띄우고 돌릴 것.

사용
----
    ros2 run can_powerpack can_bridge_node --ros-args \
        --params-file .../powerpack_config.yaml -r __ns:=/pack2     # 창 1
    python3 rail_map.py --dry-run                                    # 계획 확인
    python3 rail_map.py                                              # 측정
    python3 rail_map.py --analyze ~/result/rail_map_<ts>.csv         # 표·역맵
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
from valve_deadzone import read_config           # noqa: E402

NAMESPACE = "/pack2"
PWM_TOTAL = 25 * 3
RAIL_POS_BOARD, RAIL_NEG_BOARD = 1, 2
IDX_VENT = 0        # board1 v1 — 양압 라인 → 대기
IDX_ADMIT = 3       # board2 v1 — 대기 → 음압 라인


def channel_park(cfg_path: str, enable: bool) -> list[float]:
    """컨트롤러와 **같은 식**으로 채널 파킹 지령을 만든다.

        park = max(0, 데드존표 최솟값 − park_below_pct)        (Controller.cpp:535)

    왜 필요한가 — 실제 운전은 파킹 ON 이다. 그런데 맵을 채널 **완전 폐쇄**로 뜨면
    운전 조건과 달라진다. 20260912 실측: 같은 개도 66 에서 파킹 OFF 는 P+ 200,
    파킹 ON 은 130 이었다. **70 kPa 어치 누설**이 12채널 파킹 밸브에서 나온다.
    맵이 그걸 모르면 피드포워드가 12 %p 어긋난 개도를 내고, 그 차이를 적분이
    메우느라 느려진다. 그래서 **쓸 조건 그대로 떠야 한다.**

    반환: PWM 평면(75개) 전체. 라인 밸브 자리는 0 이다 (호출자가 덮어쓴다).
    """
    import yaml
    with open(cfg_path, encoding="utf-8") as fh:
        prm = yaml.safe_load(fh)["/pack2/pp_controller"]["ros__parameters"]
    dz = prm.get("valve_deadzone", {})
    ch = prm.get("channel_config", {})
    below = float(dz.get("park_below_pct", 5.0))
    offset = int(prm.get("channel_board_offset", 5))
    out = [0.0] * PWM_TOTAL
    if not enable:
        return out
    # 보드 슬롯 순서는 {v1=micro, v2=atm, v3=macro} 다 (Controller::to_pwm).
    for gid in range(12):
        z = (ch.get(f"ch{gid}") or {}).get("deadzone", {})
        base = (gid + offset - 1) * 3
        for slot, key in ((0, "micro_u_pct"), (1, "atm_u_pct")):
            tab = z.get(key)
            if not tab:
                continue
            out[base + slot] = max(0.0, min(tab) - below)
    return out


class Rail(Node):
    """board/sensors 를 읽고 board/pwm_cmd 를 쓴다. 채널 밸브는 항상 0 이다."""

    def __init__(self, cfg: dict, park: list[float] | None = None) -> None:
        super().__init__("rail_map")
        # 채널 밸브 바탕값. 0 이면 완전 폐쇄, 파킹값이면 운전 조건과 같다.
        self.park = park if park is not None else [0.0] * PWM_TOTAL
        self.offs, self.gains, self.atm = cfg["offs"], cfg["gains"], cfg["atm"]
        self._raw: list[int] | None = None
        self.u_vent = 100.0        # 시작은 **둘 다 활짝 열림** = 무부하
        self.u_admit = 100.0
        self.create_subscription(UInt16MultiArray, f"{NAMESPACE}/board/sensors",
                                 self._on_sensors, 20)
        self._pub = self.create_publisher(UInt16MultiArray,
                                          f"{NAMESPACE}/board/pwm_cmd", 5)

    def _on_sensors(self, msg: UInt16MultiArray) -> None:
        self._raw = list(msg.data)

    def kpa(self, board: int) -> float | None:
        if self._raw is None or len(self._raw) < board:
            return None
        raw = self._raw[board - 1]
        if raw == 0:
            return None
        return (float(raw) - self.offs[board]) * self.gains[board] + self.atm

    def rival_publishers(self) -> list[str]:
        mine = self.get_name()
        return [f"{i.node_namespace.rstrip('/')}/{i.node_name}"
                for i in self.get_publishers_info_by_topic(f"{NAMESPACE}/board/pwm_cmd")
                if i.node_name != mine]

    def publish(self) -> None:
        # 채널 밸브는 파킹값(또는 0). 파킹값은 어떤 차압의 데드존보다도 낮아
        # 유량은 0 이지만, 12채널이 레일에 물린 채 **미세 누설**을 만든다 —
        # 그게 실제 운전 조건이므로 맵도 같은 조건에서 떠야 한다.
        data = [int(round(v * 40.95)) for v in self.park]
        data[IDX_VENT] = int(round(max(0.0, min(100.0, self.u_vent)) * 40.95))
        data[IDX_ADMIT] = int(round(max(0.0, min(100.0, self.u_admit)) * 40.95))
        m = UInt16MultiArray()
        m.data = data
        self._pub.publish(m)

    def relieve(self) -> None:
        """안전 상태 — 두 밸브를 활짝 연다. 양압은 대기로 빠지고 음압은 대기가 찬다."""
        self.u_vent = 100.0
        self.u_admit = 100.0
        self.publish()


class Abort(Exception):
    pass


def spin(rig: Rail, dt: float) -> None:
    rclpy.spin_once(rig, timeout_sec=0.0)
    rig.publish()
    time.sleep(dt)


def goto_and_hold(rig: Rail, tv: float, ta: float, args) -> dict:
    """개도를 기울여 옮긴 뒤 **정해진 시간만큼** 잡고 통계를 낸다.

    왜 "변화율이 작아질 때까지" 가 아닌가 — 펌프가 왕복식이라 레일에
    **4.55 Hz, p-p 13 kPa** 짜리 리플이 상시로 얹힌다 (20260912 궤적 실측).
    압력이 완전히 평형이어도 순간 변화율은 그 리플 때문에 어떤 문턱이든 계속
    넘는다. 그래서 예전 방식은 9점 중 7점이 "미정착" 으로 끝났고, 거기 찍힌
    ±10 kPa/s 는 추세가 아니라 **리플이 LPF 를 통과하며 남긴 잔재**였다.

    게다가 진짜 시정수는 매우 길다 — 같은 점을 300 초 잡으니 179.6 → 190.6 으로
    계속 올랐고 끝에도 +0.1~0.2 kPa/s 였다 (발열로 보인다). 완전 열평형을
    기다리면 한 점에 5분 이상이라 격자를 못 돈다.

    그래서 **모든 점을 같은 시간 잡는다.** 절대값은 덜 익어도 점들 사이의
    **상대 관계**는 유지되므로 피드포워드로 쓰기에 충분하다 — 나머지는 PID 가 먹는다.
    대신 `drift` 를 같이 남겨 그 점이 얼마나 덜 익었는지 항상 볼 수 있게 한다.
    """
    dt = 1.0 / args.send_hz
    step = args.slew * dt
    t0 = time.monotonic()
    buf: list[tuple[float, float, float]] = []      # (t, P+, P−)
    arrived_at = None

    while True:
        done = True
        for name, tgt in (("u_vent", tv), ("u_admit", ta)):
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
                raise Abort("레일 압력을 못 읽는다 — 브리지/CAN 확인")
            continue
        now = time.monotonic()

        # ── 안전 (명시적으로 켰을 때만) ──────────────────────────────────
        # 기본은 꺼져 있다 — 릴리프가 하드웨어로 막는다. 아래 주석 참조.
        if args.p_max is not None and pp > args.p_max:
            raise Abort(f"P+ {pp:.1f} > {args.p_max:g} kPa")
        if args.p_min is not None and pn < args.p_min:
            raise Abort(f"P− {pn:.1f} < {args.p_min:g} kPa")

        if not done:
            continue
        if arrived_at is None:
            arrived_at = now
            buf.clear()
        buf.append((now - arrived_at, pp, pn))
        if now - arrived_at >= args.hold:
            break

    return _summarize(buf, args, time.monotonic() - t0)


def _mean(xs):
    return sum(xs) / max(1, len(xs))


def _summarize(buf, args, secs) -> dict:
    """마지막 --avg-window 구간의 평균·리플과, 이동평균의 드리프트."""
    if not buf:
        return dict(p_pos=float("nan"), p_neg=float("nan"), rip_pos=float("nan"),
                    rip_neg=float("nan"), drift_pos=float("nan"),
                    drift_neg=float("nan"), secs=secs, n=0)
    t_end = buf[-1][0]
    win = [b for b in buf if b[0] >= t_end - args.avg_window] or buf[-1:]
    mp, mn = _mean([b[1] for b in win]), _mean([b[2] for b in win])
    rip_p = max(b[1] for b in win) - min(b[1] for b in win)
    rip_n = max(b[2] for b in win) - min(b[2] for b in win)

    # 드리프트 = 유지 구간 **후반 절반**을 직선으로 맞춘 기울기.
    # 리플은 평균이 0 이므로 최소제곱 기울기에 거의 안 들어간다.
    half = [b for b in buf if b[0] >= t_end * 0.5]
    if len(half) > 10:
        xs = [b[0] for b in half]
        mx = _mean(xs)
        den = sum((x - mx) ** 2 for x in xs)
        def slope(idx):
            my = _mean([b[idx] for b in half])
            return sum((b[0] - mx) * (b[idx] - my) for b in half) / den if den > 1e-9 else 0.0
        dp, dn = slope(1), slope(2)
    else:
        dp = dn = float("nan")
    return dict(p_pos=mp, p_neg=mn, rip_pos=rip_p, rip_neg=rip_n,
                drift_pos=dp, drift_neg=dn, secs=secs, n=len(win),
                relief=int(mp >= args.relief_kpa))


def probe(rig: Rail, args) -> int:
    """개도 한 쌍을 오래 잡고 **궤적을 통째로** 기록한다.

    요약값(평균·끝 변화율)만으로는 두 가지를 못 가른다:
      · 시정수가 길어서 아직 오르는 중인가  → 단조 증가 후 평평
      · 릴리프가 열려 내려오는 중인가        → 올라가다 **꺾여서** 내려온다
    격자를 돌리기 전에 이걸 먼저 갈라야 위쪽 줄이 통째로 쓰레기가 되는 걸 막는다.
    """
    out = args.out or os.path.expanduser(
        f"~/result/rail_probe_{datetime.now():%Y%m%d_%H%M%S}.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    dt = 1.0 / args.send_hz
    step = args.slew * dt
    tv, ta = args.probe
    print(f"\n=== 궤적 측정: 방출 {tv:g} %  유입 {ta:g} %  ×  {args.probe_secs:g} s ===")
    print("압력 감시는 꺼져 있다 — 릴리프(270~280)가 하드웨어로 막는다"
          if args.p_max is None else f"P+ > {args.p_max:g} 이면 중단한다")
    f = open(out, "w", newline="", buffering=1)
    wr = csv.writer(f)
    wr.writerow(["t", "u_vent", "u_admit", "p_pos", "p_neg"])
    t0 = time.monotonic()
    peak = -1e9
    rc = 0
    try:
        while True:
            now = time.monotonic()
            el = now - t0
            if el > args.probe_secs:
                break
            for name, tgt in (("u_vent", tv), ("u_admit", ta)):
                cur = getattr(rig, name)
                setattr(rig, name, tgt if abs(tgt - cur) <= step
                        else cur + math.copysign(step, tgt - cur))
            spin(rig, dt)
            pp, pn = rig.kpa(RAIL_POS_BOARD), rig.kpa(RAIL_NEG_BOARD)
            if pp is None or pn is None:
                continue
            wr.writerow([f"{el:.2f}", f"{rig.u_vent:.1f}", f"{rig.u_admit:.1f}",
                         f"{pp:.2f}", f"{pn:.2f}"])
            if pp > peak:
                peak = pp
            if args.p_max is not None and pp > args.p_max:
                raise Abort(f"P+ {pp:.1f} > {args.p_max:g}")
            if args.p_min is not None and pn < args.p_min:
                raise Abort(f"P− {pn:.1f} < {args.p_min:g}")
            if int(el) != int(el - dt) and int(el) % 5 == 0:
                print(f"  {el:6.0f} s   P+ {pp:7.2f}   P− {pn:6.2f}"
                      f"   (최고 {peak:.1f})", flush=True)
    except Abort as exc:
        print(f"\n[중단] 안전: {exc}", file=sys.stderr); rc = 3
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C", file=sys.stderr); rc = 130
    finally:
        f.close()
        print(f"[기록] {out}   최고 P+ {peak:.1f} kPa")
        print("  단조 증가 후 평평  → 시정수 문제. --settle-timeout 을 늘리면 된다")
        print("  올라가다 **꺾여서** 내려옴 → 릴리프다. 그 코너는 격자에서 빼야 한다")
    return rc


def build_grid(args) -> list[tuple[float, float]]:
    def seq(a, b, st):
        out, v = [], a
        while v <= b + 1e-9:
            out.append(round(v, 2)); v += st
        return out
    vs = seq(args.vent_start, args.vent_stop, args.vent_step)
    as_ = seq(args.admit_start, args.admit_stop, args.admit_step)
    pts = []
    # **닫는 쪽으로 갈수록 위험하다.** 열린 구석에서 시작해 한 줄씩 조여 간다.
    for i, a in enumerate(sorted(as_, reverse=True)):
        row = sorted(vs, reverse=True)
        if i % 2:                      # 지그재그 — 줄 바뀔 때 지령 점프를 줄인다
            row = list(reversed(row))
        for v in row:
            pts.append((v, a))
    return pts


def analyze(path: str) -> int:
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        print("[중단] 빈 파일이다.", file=sys.stderr)
        return 2
    # 미정착 점도 **버리지 않는다.** 평형에 덜 갔을 뿐 어디로 가던 중인지는
    # rate_* 가 말해 준다. 다만 표에 별표를 붙여 눈으로 구별되게 한다.
    has_drift = "drift_pos" in rows[0]
    if has_drift:
        big = [r for r in rows if abs(float(r["drift_pos"])) > 0.1]
        if big:
            print(f"\n⚠ {len(big)}/{len(rows)} 점이 드리프트 |{'>'}0.1| kPa/s 다 "
                  "— 아직 익는 중이라는 뜻이고, 표의 * 가 그것이다.")
    vs = sorted({float(r["u_vent"]) for r in rows})
    as_ = sorted({float(r["u_admit"]) for r in rows})
    cell = {(float(r["u_vent"]), float(r["u_admit"])): r for r in rows}

    n_rel = sum(1 for r in rows if int(r.get("relief", 0)))
    if n_rel:
        print(f"\n⚠ {n_rel} 점이 **릴리프 구간**이다 (표의 R). 그 점은 펌프·밸브 특성이"
              " 아니라 릴리프의 조절압이라 역맵에 넣으면 안 된다.")
    for key, lab in (("p_pos", "P+ [kPa]"), ("p_neg", "P− [kPa]")):
        print(f"\n── {lab}   (행 = 유입 개도 %, 열 = 방출 개도 %)")
        print("  유입\\방출 " + "".join(f"{v:9.0f}" for v in vs))
        for a in sorted(as_, reverse=True):
            line = f"    {a:6.0f}  "
            for v in vs:
                r = cell.get((v, a))
                if r is None:
                    line += "        ·"
                else:
                    if int(r.get("relief", 0)):
                        mark = "R"          # 릴리프 특성 — 맵에 넣으면 안 된다
                    elif has_drift:
                        mark = "*" if abs(float(r["drift_pos"])) > 0.1 else " "
                    else:
                        mark = " " if int(r.get("settled", 1)) else "*"
                    line += f"{float(r[key]):8.1f}{mark}"
            print(line)

    if has_drift:
        print("\n── 리플 p-p [kPa]  (펌프 왕복 4.55 Hz — 제어 외란으로 그대로 들어간다)")
        print("  유입\\방출 " + "".join(f"{v:>18.0f}" for v in vs))
        for a in sorted(as_, reverse=True):
            line = f"    {a:6.0f}  "
            for v in vs:
                r = cell.get((v, a))
                line += (" " * 18) if r is None else \
                    f"{float(r['rip_pos']):9.2f}{float(r['rip_neg']):9.2f}"
            print(line)
        print("           (각 칸 = P+ 리플, P− 리플)")

        print("\n── 드리프트 [kPa/s]  (0 에 가까울수록 익었다)")
        print("  유입\\방출 " + "".join(f"{v:>18.0f}" for v in vs))
        for a in sorted(as_, reverse=True):
            line = f"    {a:6.0f}  "
            for v in vs:
                r = cell.get((v, a))
                line += (" " * 18) if r is None else \
                    f"{float(r['drift_pos']):+9.2f}{float(r['drift_neg']):+9.2f}"
            print(line)
        print("           (각 칸 = P+ 드리프트, P− 드리프트)")
    elif any("rate_pos" in r for r in rows):
        print("\n── 끝 변화율 [kPa/s]  (0 에 가까울수록 평형에 가깝다)")
        print("  유입\\방출 " + "".join(f"{v:>18.0f}" for v in vs))
        for a in sorted(as_, reverse=True):
            line = f"    {a:6.0f}  "
            for v in vs:
                r = cell.get((v, a))
                if r is None or "rate_pos" not in r:
                    line += " " * 18
                else:
                    line += f"{float(r['rate_pos']):+9.1f}{float(r['rate_neg']):+9.1f}"
            print(line)
        print("           (각 칸 = P+ 변화율, P− 변화율)")

    # 역맵에 쓸 감도 — 각 입력이 각 출력을 얼마나 움직이는가
    print("\n── 감도 (선형 회귀)")
    import itertools
    fit_rows = [r for r in rows if not int(r.get("relief", 0))]
    if len(fit_rows) < len(rows):
        print(f"   (릴리프 {len(rows)-len(fit_rows)} 점을 회귀에서 뺐다)")
    X = [[1.0, float(r["u_vent"]), float(r["u_admit"])] for r in fit_rows]
    for key, lab in (("p_pos", "P+"), ("p_neg", "P−")):
        y = [float(r[key]) for r in fit_rows]
        n = len(X)
        XtX = [[sum(X[k][i] * X[k][j] for k in range(n)) for j in range(3)] for i in range(3)]
        Xty = [sum(X[k][i] * y[k] for k in range(n)) for i in range(3)]
        # 3x3 가우스 소거
        M = [XtX[i] + [Xty[i]] for i in range(3)]
        for i in range(3):
            p = max(range(i, 3), key=lambda r_: abs(M[r_][i]))
            M[i], M[p] = M[p], M[i]
            if abs(M[i][i]) < 1e-12:
                print(f"   {lab}: 특이 행렬 — 격자가 한 줄뿐인 것 같다"); break
            for r_ in range(3):
                if r_ == i: continue
                f = M[r_][i] / M[i][i]
                for c in range(i, 4):
                    M[r_][c] -= f * M[i][c]
        else:
            b = [M[i][3] / M[i][i] for i in range(3)]
            print(f"   {lab} ≈ {b[0]:+8.2f} {b[1]:+7.4f}·방출 {b[2]:+7.4f}·유입 [kPa]")
    print("\n두 입력이 두 출력을 **같이** 움직이면 (계수가 둘 다 유의하면) SISO 두 개로는")
    print("못 잡는다는 뜻이다 — 역맵 피드포워드가 필요하다는 근거가 이 표다.")
    return 0


def invert(path: str, args) -> int:
    """측정 표를 뒤집어 (P+_ref, P−_ref) → (방출, 유입) 피드포워드를 만든다.

    **구조가 거의 삼각형이라** 2×2 를 통째로 뒤집을 필요가 없다 (20260912 실측):
      · P− 는 유입이 거의 단독으로 정한다 — 유입 +1.27 kPa/%%p 대 방출 −0.32
      · P+ 는 그 유입 아래에서 방출이 정한다
    그래서 순차적으로 푼다:
        유입 = g(P−_ref)                 1차원 표
        방출 = h(P+_ref, 유입)           유입 수준별 곡선을 유입으로 보간

    국소 이득 |dP+/d방출| 도 같이 낸다. 동작점마다 1.67~9.88 kPa/%%p 로 **6배**
    달라져서 (최대 이득 위치가 유입에 따라 움직인다) 고정 게인 PID 로는 못 덮는다.
    채널 쪽에서 gain_dp_ref_kpa 로 한 것과 같은 스케줄링을 레일에도 건다.
    """
    with open(path, newline="", encoding="utf-8") as fh:
        rows = [r for r in csv.DictReader(fh) if not int(r.get("relief", 0))]
    if not rows:
        print("[중단] 쓸 수 있는 점이 없다 (전부 릴리프).", file=sys.stderr)
        return 2
    A = sorted({float(r["u_admit"]) for r in rows})
    cell = {(float(r["u_vent"]), float(r["u_admit"])): r for r in rows}

    # ── 유입 → P− (방출 평균) ────────────────────────────────────────────
    pneg_of_admit = []
    for a_ in A:
        vs = sorted(v for (v, aa) in cell if aa == a_)
        if not vs:
            continue
        pneg_of_admit.append((a_, _mean([float(cell[(v, a_)]["p_neg"]) for v in vs])))
    pneg_of_admit.sort(key=lambda x: x[1])

    lines = []
    lines.append("    # ── 레일 피드포워드 (rail_map.py --invert 생성) ──────────────────")
    lines.append(f"    #   원본: {os.path.basename(path)}")
    lines.append("    #   P− 는 유입이 거의 단독 결정(+1.27 kPa/%p 대 방출 −0.32)이라")
    lines.append("    #   유입을 먼저 P− 로 정하고, 그 유입에서 방출로 P+ 를 맞춘다.")
    lines.append("    RailFF:")
    lines.append("      enable: true")
    lines.append("      # 유입 개도 [%] ← P− 목표 [kPa abs]")
    lines.append("      admit:")
    lines.append("        p_neg_kpa: [" + ", ".join(f"{p:.2f}" for _, p in pneg_of_admit) + "]")
    lines.append("        u_pct:     [" + ", ".join(f"{a_:.1f}" for a_, _ in pneg_of_admit) + "]")
    lines.append("      # 방출 개도 [%] ← P+ 목표. 유입 수준별 곡선을 유입으로 보간한다.")
    lines.append("      vent:")
    lines.append("        admit_pct: [" + ", ".join(f"{a_:.1f}" for a_ in A) + "]")
    lines.append("        curves:")
    slopes = []
    for a_ in A:
        vs = sorted(v for (v, aa) in cell if aa == a_)
        pp = [(float(cell[(v, a_)]["p_pos"]), v) for v in vs]
        pp.sort()                       # P+ 오름차순 (보간 전제)
        lines.append(f'          "{a_:.0f}":')
        lines.append("            p_pos_kpa: [" + ", ".join(f"{p:.2f}" for p, _ in pp) + "]")
        lines.append("            u_pct:     [" + ", ".join(f"{v:.1f}" for _, v in pp) + "]")
        for i in range(len(vs) - 1):
            d = (float(cell[(vs[i + 1], a_)]["p_pos"]) - float(cell[(vs[i], a_)]["p_pos"])) \
                / (vs[i + 1] - vs[i])
            slopes.append(abs(d))
    slopes.sort()
    med = slopes[len(slopes) // 2]
    lines.append("      # PID 게인 스케줄. 국소 이득 |dP+/d방출| 가 "
                 f"{slopes[0]:.2f}~{slopes[-1]:.2f} kPa/%p 로 "
                 f"{slopes[-1]/max(1e-6,slopes[0]):.1f}배 달라진다 —")
    lines.append("      # 이득이 큰 곳에서 게인을 줄이고 작은 곳에서 키운다.")
    lines.append("      #   scale = clamp(gain_ref / |dP+/d방출|, min, max)")
    lines.append(f"      gain_ref_kpa_per_pct: {med:.2f}   # 중앙값")
    lines.append("      gain_scale_min: 0.3")
    lines.append("      gain_scale_max: 3.0")

    print("\n".join(lines))
    print("\n# ── 검산: 이 표로 되찾은 개도 ──────────────────────────────────")
    print(f"# {'P+ 목표':>8}{'P− 목표':>8}{'유입':>8}{'방출':>8}   실측 대조")
    for a_ in A:
        vs = sorted(v for (v, aa) in cell if aa == a_)
        for v in (vs[0], vs[len(vs) // 2], vs[-1]):
            r = cell[(v, a_)]
            tp, tn = float(r["p_pos"]), float(r["p_neg"])
            ua = _interp([p for _, p in pneg_of_admit], [x for x, _ in pneg_of_admit], tn)
            uv = _vent_ff(cell, A, tp, ua)
            print(f"# {tp:8.1f}{tn:8.1f}{ua:8.1f}{uv:8.1f}   (실제 유입 {a_:.0f} 방출 {v:.0f})")
    return 0


def _interp(xs, ys, x):
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for i in range(1, len(xs)):
        if xs[i] >= x:
            f = (x - xs[i - 1]) / max(1e-9, xs[i] - xs[i - 1])
            return ys[i - 1] + f * (ys[i] - ys[i - 1])
    return ys[-1]


def _vent_ff(cell, A, p_pos, admit):
    """유입 수준별 P+→방출 곡선을 만들고 유입으로 보간한다."""
    per = []
    for a_ in A:
        vs = sorted(v for (v, aa) in cell if aa == a_)
        pp = sorted((float(cell[(v, a_)]["p_pos"]), v) for v in vs)
        per.append((a_, _interp([p for p, _ in pp], [v for _, v in pp], p_pos)))
    return _interp([a_ for a_, _ in per], [u for _, u in per], admit)


def main() -> int:
    ap = argparse.ArgumentParser(description="레일 정특성 (개도쌍 → 압력쌍) 측정",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--vent-start", type=float, default=20.0, help="방출 개도 시작 [%%]")
    ap.add_argument("--vent-stop", type=float, default=60.0, help="방출 개도 끝 [%%]")
    ap.add_argument("--vent-step", type=float, default=20.0)
    ap.add_argument("--admit-start", type=float, default=60.0,
                    help="유입 개도 시작 [%%]. 20 %% 에서는 펌프가 굶어 방출과 무관하게 "
                         "P+ 가 대기압이다 — 쓸모없는 영역이라 기본에서 뺐다")
    ap.add_argument("--admit-stop", type=float, default=100.0, help="유입 개도 끝 [%%]")
    ap.add_argument("--admit-step", type=float, default=20.0)

    ap.add_argument("--slew", type=float, default=10.0, help="개도 변화율 [%%/s]")
    ap.add_argument("--hold", type=float, default=90.0,
                    help="한 점을 잡는 시간 [s]. 펌프 리플(4.55 Hz, p-p 13 kPa) 때문에 "
                         "'변화율이 작아질 때까지' 로는 영원히 못 멈춘다 — 모든 점을 "
                         "같은 시간 잡아 상대 관계를 맞춘다")
    ap.add_argument("--avg-window", type=float, default=20.0,
                    help="평균·리플을 내는 마지막 구간 [s]. 리플 4.55 Hz 기준 90 주기가 "
                         "들어가 완전히 걷힌다")

    # ── 압력 감시는 **기본 꺼져 있다** ────────────────────────────────────
    # 양압 라인에 270~280 kPa 릴리프가 물려 있고 음압은 최대 진공까지 써도 된다.
    # 하드웨어가 이미 막고 있는데 그 **아래**에 소프트 한계를 걸면, 보호가 아니라
    # 멀쩡한 측정을 중단시키는 것뿐이다 (20260912 에 260 으로 걸었다가 2번 점에서
    # 262 로 멈췄다 — 릴리프는 열리지도 않았다).
    # 센서 고장이나 릴리프 고착을 잡고 싶을 때만 명시적으로 켠다.
    ap.add_argument("--p-max", type=float, default=None,
                    help="P+ 상한 [kPa abs]. 기본 끔 — 릴리프(270~280)가 보호한다")
    ap.add_argument("--p-min", type=float, default=None,
                    help="P− 하한 [kPa abs]. 기본 끔 — 최대 진공까지 써도 된다")
    ap.add_argument("--max-rate", type=float, default=None,
                    help="레일 변화율 상한 [kPa/s]. 기본 끔")
    ap.add_argument("--relief-kpa", type=float, default=265.0,
                    help="이 압력 위에 머문 점은 펌프·밸브 특성이 아니라 **릴리프 특성**이다. "
                         "중단하지 않고 CSV 와 분석 표에 표시만 한다")

    ap.add_argument("--send-hz", type=float, default=50.0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--analyze", default=None, metavar="CSV",
                    help="측정 CSV 를 읽어 표와 감도를 낸다 (실기 불필요)")
    ap.add_argument("--park", dest="park", action="store_true", default=True,
                    help="채널 밸브를 **파킹값**에 둔다 (실제 운전과 같은 조건). "
                         "기본 켜짐 — 맵은 쓸 조건에서 떠야 한다")
    ap.add_argument("--no-park", dest="park", action="store_false",
                    help="채널 밸브를 0 %%(완전 폐쇄)로 둔다. 누설을 배제한 순수 "
                         "펌프·레일 특성을 볼 때만")
    ap.add_argument("--probe", nargs=2, type=float, metavar=("VENT", "ADMIT"),
                    default=None,
                    help="격자 대신 이 개도쌍 하나를 오래 잡고 **궤적**을 기록한다. "
                         "시정수인지 릴리프인지 가르는 용도다")
    ap.add_argument("--probe-secs", type=float, default=300.0, help="궤적 길이 [s]")
    ap.add_argument("--invert", default=None, metavar="CSV",
                    help="측정 CSV 를 뒤집어 피드포워드 yaml 블록을 낸다 (실기 불필요)")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--yes", action="store_true")
    args = ap.parse_args()

    if args.analyze:
        return analyze(args.analyze)
    if args.invert:
        return invert(args.invert, args)

    # 채널 파킹은 **계획 출력·probe·격자** 모두에서 쓴다. 여기서 한 번만 만든다.
    cfgp = os.path.join(_HERE, "..", "config", "powerpack_config.yaml")
    park = channel_park(cfgp, args.park)

    if args.probe:
        if args.dry_run:
            # 실기에 아무것도 보내지 않는다 — 계획만 찍는다.
            print(f"\n=== 궤적 측정 계획 ===\n방출 {args.probe[0]:g} %  유입 {args.probe[1]:g} % "
                  f"× {args.probe_secs:g} s")
            nz = [v for v in park if v > 0]
            print("채널 밸브: " + (f"파킹값 {min(nz):.1f}~{max(nz):.1f} %" if nz else "0 %(완전 폐쇄)"))
            return 0
        cfg = read_config(cfgp)
        rclpy.init()
        rig = Rail(cfg, park)
        try:
            t_end = time.monotonic() + 2.0
            while time.monotonic() < t_end:
                rig.relieve(); spin(rig, 0.02)
            rivals = rig.rival_publishers()
            if rivals:
                print(f"[중단] board/pwm_cmd 를 {', '.join(rivals)} 도 발행 중이다.",
                      file=sys.stderr)
                return 2
            return probe(rig, args)
        finally:
            for _ in range(int(2.0 * args.send_hz)):
                rig.relieve(); spin(rig, 1.0 / args.send_hz)
            rig.destroy_node(); rclpy.shutdown()

    pts = build_grid(args)
    print("\n=== 레일 정특성 측정 계획 ===")
    print(f"방출(board1 v1) {args.vent_start:g}~{args.vent_stop:g} %, {args.vent_step:g} 간격")
    print(f"유입(board2 v1) {args.admit_start:g}~{args.admit_stop:g} %, {args.admit_step:g} 간격")
    print(f"격자 {len(pts)} 점 · 개도 {args.slew:g} %/s · 점마다 {args.hold:g} s 유지 "
          f"(마지막 {args.avg_window:g} s 로 평균)")
    print(f"대략 {len(pts)*(args.hold+8)/60.0:.0f} 분")
    guards = [n for n, v in (("P+ ≤ %s" % args.p_max, args.p_max),
                             ("P− ≥ %s" % args.p_min, args.p_min),
                             ("변화율 ≤ %s" % args.max_rate, args.max_rate)) if v is not None]
    print("안전 감시: " + (" · ".join(guards) if guards else
          "**꺼짐** — 양압은 릴리프(270~280)가, 음압은 최대 진공까지 허용이라 하드웨어가 막는다"))
    print(f"P+ 가 {args.relief_kpa:g} 이상인 점은 릴리프 특성이라 표에 R 로 표시한다 (중단하지 않는다)")
    nz = [v for v in park if v > 0]
    if nz:
        print(f"채널 밸브: **파킹값** {min(nz):.1f}~{max(nz):.1f} % (실제 운전과 같은 조건). "
              f"유량은 0 이지만 미세 누설이 레일에 실린다 — 그게 쓸 조건이다.")
    else:
        print("채널 밸브: **0 %(완전 폐쇄)** — 누설 없음. 순수 펌프·레일 특성.")
    print("**열린 구석(둘 다 100 %)에서 시작해 조여 간다.** 둘 다 닫힌 구석이 가장 위험하다.")
    if args.dry_run:
        for i, (v, a) in enumerate(pts, 1):
            print(f"  {i:3d}  방출 {v:5.0f} %  유입 {a:5.0f} %")
        return 0
    if not args.yes:
        if input("\n비상정지를 확인했으면 RUN 을 입력: ").strip() != "RUN":
            print("취소했다."); return 2

    cfg = read_config(cfgp)
    out = args.out or os.path.expanduser(
        f"~/result/rail_map_{datetime.now():%Y%m%d_%H%M%S}.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    rclpy.init()
    rig = Rail(cfg, park)
    rc, rows = 0, 0
    f = open(out, "w", newline="", buffering=1)
    wr = csv.writer(f)
    # rate_* = 그 점을 떠날 때의 변화율. settled=0 인 점이 평형에서 얼마나
    # 멀었는지, 어느 쪽으로 가던 중이었는지를 이것으로 안다.
    # rip_* = 마지막 창의 p-p (펌프 리플). drift_* = 유지 구간 후반의 기울기 —
    # 그 점이 얼마나 덜 익었는지를 말한다 (0 에 가까울수록 평형).
    wr.writerow(["idx", "u_vent", "u_admit", "p_pos", "p_neg",
                 "rip_pos", "rip_neg", "drift_pos", "drift_neg",
                 "secs", "n_sample", "relief"])
    try:
        # 다른 발행자가 있으면 지령이 번갈아 나가 측정이 무의미하다.
        t_end = time.monotonic() + 2.0
        while time.monotonic() < t_end:
            rig.relieve(); spin(rig, 0.02)
        rivals = rig.rival_publishers()
        if rivals:
            print(f"\n[중단] board/pwm_cmd 를 **{', '.join(rivals)}** 도 발행하고 있다.\n"
                  "       pp_controller 를 내리고 브리지만 띄운 뒤 다시 실행할 것.",
                  file=sys.stderr)
            return 2

        for i, (v, a) in enumerate(pts, 1):
            print(f"\n[{i}/{len(pts)}] 방출 {v:.0f} %  유입 {a:.0f} %", flush=True)
            r = goto_and_hold(rig, v, a, args)
            wr.writerow([i, f"{v:g}", f"{a:g}", f"{r['p_pos']:.2f}", f"{r['p_neg']:.2f}",
                         f"{r['rip_pos']:.2f}", f"{r['rip_neg']:.2f}",
                         f"{r['drift_pos']:+.3f}", f"{r['drift_neg']:+.3f}",
                         f"{r['secs']:.1f}", r["n"], r["relief"]])
            rows += 1
            if r["relief"]:
                print("      ** 릴리프 구간 — 이 점은 펌프·밸브 특성이 아니다 **")
            print(f"      P+ {r['p_pos']:7.2f} (리플 {r['rip_pos']:5.2f}, "
                  f"드리프트 {r['drift_pos']:+.3f} kPa/s)   "
                  f"P− {r['p_neg']:6.2f} (리플 {r['rip_neg']:5.2f})   "
                  f"{r['secs']:.0f} s", flush=True)
        print(f"\n[완료] {rows} 점 → {out}")
    except Abort as exc:
        print(f"\n[중단] 안전: {exc}", file=sys.stderr); rc = 3
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C", file=sys.stderr); rc = 130
    finally:
        # 어떤 경로로 끝나든 두 밸브를 활짝 연다 = 자연 릴리프
        print("[복귀] 두 레일 밸브를 100 % 로 연다...")
        try:
            for _ in range(int(2.0 * args.send_hz)):
                rig.relieve(); spin(rig, 1.0 / args.send_hz)
        except Exception:
            pass
        f.close()
        rig.destroy_node()
        rclpy.shutdown()
        if rows:
            print(f"[기록] {out}  ({rows} 점)")
            print(f"       분석:  python3 rail_map.py --analyze {out}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
