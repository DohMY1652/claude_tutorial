#!/usr/bin/env python3
"""valve_warmup.py — 밸브 코일을 예열한다. 압력은 건드리지 않는다.

왜 필요한가
-----------
데드존(밸브가 열리기 시작하는 지령)이 온도에 따라 움직이는 것으로 의심된다.
표는 실기를 막 켠 상태에서 쟀고, 실제 제어는 몇 분 돌린 뒤의 상태에서 한다.
같은 열 상태에서 재고 쓰려면 먼저 데워야 한다.

두 가지 방식 (--mode)
---------------------
발열은 **I²R** 이라 지령(=전류)에 제곱으로 붙는다. 100 % = 250 mA 기준으로
40 % 는 100 mA 라 발열이 **1/6** 이다. 빨리 데우려면 크게 열어야 한다.

  full  (기본) — 두 밸브를 **모두 크게** 연다. 지령 [--lo-pct, --hi-pct] 난수 보행.
        압력은 움직이지만 **갇혀 있다**: 양압 챔버는 micro(레일)와 atm(대기)이
        동시에 열려 있어 레일압과 대기압 **사이**에서만 논다. 음압도 진공레일과
        대기 사이다. 구조적으로 과압이 될 수 없다.
        그 위에 하드 가드를 둔다 — 양압이 --p-hi 를 넘으면 그 채널 micro 를 닫고
        atm 을 활짝, 음압이 --p-lo 아래로 가면 micro 를 닫고 atm 을 활짝.
        대가: 공압을 계속 흘려보내므로 펌프가 풀부하로 돈다. 레일은 목표에서
        내려간다 — 예열 중에는 정상이다.

  below — 압력을 전혀 건드리지 않는다. 지령을 **데드존 아래**에서만 흔들어
        코일에만 전류를 흘린다 (표 최솟값 − below − band ~ − below).
        표 **최솟값**을 쓰므로 어떤 차압에서도 닫혀 있다. 대신 전류가 낮아 느리다.
        챔버가 대기압에서 --p-band 이상 벗어나면 그 채널을 닫고, --abort-band 를
        넘으면 전체를 중단한다 (밸브가 새거나 표가 크게 틀렸다는 뜻이다).

레일은 어느 방식이든 valve_deadzone.py 와 같은 루프로 잡는다.

전제
----
  · can_bridge 가 돌고 있어야 한다
  · pp_controller 는 **꺼져 있어야 한다** (board/pwm_cmd 발행자가 둘이면 안 된다)
  · 펌프는 켜 둔다 — 레일까지 같이 데우려면 실제 운전과 같은 조건이어야 한다

사용
----
    python3 valve_warmup.py                    # 5분, 전 채널, full (빠름)
    python3 valve_warmup.py --minutes 8
    python3 valve_warmup.py --mode below       # 압력을 안 건드리는 방식 (느림)
    python3 valve_warmup.py --lo-pct 70 --hi-pct 100   # 더 세게
    python3 valve_warmup.py --dry-run          # 계획만 확인
"""
from __future__ import annotations

import argparse
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from valve_deadzone import (  # noqa: E402  (같은 디렉터리의 검증된 구현을 재사용)
    DEFAULT_YAML, N_AXES, NEG_BOARD0, POS_BOARD0, V_ATM, V_MICRO,
    Rig, parse_axes, read_config, spin,
)


def channel_tables(yaml_path: str) -> dict:
    """채널×밸브별 표 최솟값 [%] — 가열 지령의 기준선."""
    import yaml
    with open(yaml_path, encoding="utf-8") as fp:
        prm = yaml.safe_load(fp)["/pack2/pp_controller"]["ros__parameters"]
    cc = prm.get("channel_config") or {}
    vd = prm.get("valve_deadzone") or {}
    out = {}
    for gid in range(2 * N_AXES):
        side = "pos" if gid < N_AXES else "neg"
        for slot, role in ((V_MICRO, "micro"), (V_ATM, "atm")):
            u = None
            ch = cc.get(f"ch{gid}", {}).get("deadzone", {})
            if isinstance(ch.get(f"{role}_u_pct"), list) and ch[f"{role}_u_pct"]:
                u = min(ch[f"{role}_u_pct"])
            elif isinstance(vd.get(side, {}).get(f"{role}_u_pct"), list):
                u = min(vd[side][f"{role}_u_pct"])
            if u is not None:
                out[(gid, slot)] = float(u)
    return out


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--minutes", type=float, default=5.0, help="예열 시간 [분]")
    ap.add_argument("--axes", default="1-6", help="대상 축 (1-based). 기본 1-6")
    ap.add_argument("--side", default="both", choices=("pos", "neg", "both"))
    ap.add_argument("--mode", default="full", choices=("full", "below"),
                    help="full = 두 밸브를 크게 열어 빨리 데운다 (압력이 움직인다). "
                         "below = 데드존 아래에서만 흔들어 압력을 안 건드린다 (느리다)")
    ap.add_argument("--lo-pct", type=float, default=55.0,
                    help="[full] 지령 하한 [%%]")
    ap.add_argument("--hi-pct", type=float, default=100.0,
                    help="[full] 지령 상한 [%%]. 100 %% = 250 mA")
    ap.add_argument("--p-hi", type=float, default=180.0,
                    help="[full] 양압 챔버 상한 [kPa abs]. 넘으면 micro 를 닫고 atm 활짝")
    ap.add_argument("--p-lo", type=float, default=18.0,
                    help="[full] 음압 챔버 하한 [kPa abs]. 밑돌면 micro 를 닫고 atm 활짝")
    ap.add_argument("--below-pct", type=float, default=3.0,
                    help="표 최솟값에서 **최소한** 이만큼 아래로만 지령한다 [%%p]. "
                         "표 자체의 오차를 흡수하는 안전 여유다")
    ap.add_argument("--band-pct", type=float, default=6.0,
                    help="지령이 흔들릴 폭 [%%p]. 범위는 "
                         "[표최솟값 − below − band, 표최솟값 − below]")
    ap.add_argument("--step-sec", type=float, default=0.5,
                    help="지령을 새로 뽑는 주기 [s]")
    ap.add_argument("--p-band", type=float, default=5.0,
                    help="챔버가 대기압에서 이만큼 벗어나면 그 채널을 닫는다 [kPa]")
    ap.add_argument("--abort-band", type=float, default=15.0,
                    help="이만큼 벗어나면 전체 중단 [kPa]")
    ap.add_argument("--vent-sec", type=float, default=15.0,
                    help="시작 전 전 채널을 대기압으로 내리는 시간 [s]")
    ap.add_argument("--rail-pos", type=float, default=None, help="양압 레일 목표 (기본: yaml)")
    ap.add_argument("--rail-neg", type=float, default=None, help="음압 레일 목표 (기본: yaml)")
    ap.add_argument("--rail-ramp", type=float, default=6.0)
    ap.add_argument("--host", default="127.0.0.1")     # 인자 호환용 (미사용)
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--seed", type=int, default=None, help="난수 씨앗 (재현용)")
    ap.add_argument("--dry-run", action="store_true", help="지령 범위만 찍고 끝낸다")
    ap.add_argument("--yes", action="store_true")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    axes = parse_axes(args.axes)
    sides = {"pos": [True], "neg": [False], "both": [True, False]}[args.side]
    gids = sorted((a if is_pos else N_AXES + a) for is_pos in sides for a in axes)

    cfg = read_config(args.yaml)
    tmin = channel_tables(args.yaml)
    rail_pos = args.rail_pos if args.rail_pos is not None else cfg["line_pos"]["ref"]
    rail_neg = args.rail_neg if args.rail_neg is not None else cfg["line_neg"]["ref"]

    print(f"밸브 예열 — {args.minutes:g} 분, 채널 {gids}")
    print(f"레일 목표 {rail_pos:.0f} / {rail_neg:.0f} kPa (yaml LinePID)")
    band = {}
    if args.mode == "full":
        print(f"\n방식 full — 두 밸브를 모두 {args.lo_pct:g}~{args.hi_pct:g} % 로 흔든다.")
        print(f"  전류 {args.lo_pct*2.5:.0f}~{args.hi_pct*2.5:.0f} mA "
              f"(below 방식 대비 발열 약 "
              f"{((args.lo_pct+args.hi_pct)/2/40.0)**2:.1f} 배)")
        print("  압력은 움직이지만 두 밸브가 함께 열려 있어 **레일압과 대기압 사이**에")
        print("  갇힌다. 구조적으로 과압이 되지 않는다.")
        print(f"  하드 가드: 양압 챔버 > {args.p_hi:g} → micro 닫고 atm 활짝 | "
              f"음압 챔버 < {args.p_lo:g} → micro 닫고 atm 활짝")
        for gid in gids:
            for slot in (V_MICRO, V_ATM):
                band[(gid, slot)] = (args.lo_pct, args.hi_pct)
        print(f"\n  대상 채널 {gids}")
        print(f"  예상: 양압 챔버는 레일(≈{rail_pos:.0f})과 대기(101.3) 사이, "
              f"음압은 진공레일(≈{rail_neg:.0f})과 대기 사이에서 논다.")
        print("  공압을 계속 흘려보내므로 펌프가 풀부하로 돌고 레일은 목표에서 내려간다"
              " — 예열 중에는 정상이다.")
    else:
        print(f"\n방식 below — 지령을 **데드존 아래**에서만 흔든다 "
              f"(표 최솟값 − {args.below_pct:g} ~ − {args.below_pct + args.band_pct:g} %p)")
        print("  유량 0, 코일만 데운다. 전류가 낮아 느리다.")
        print(f"{'채널':>6} {'micro 표최소':>11} {'micro 지령':>13} "
              f"{'atm 표최소':>10} {'atm 지령':>13}")
        for gid in gids:
            row = f"  ch{gid:<3} "
            for slot in (V_MICRO, V_ATM):
                t0 = tmin.get((gid, slot))
                if t0 is None:
                    band[(gid, slot)] = None
                    row += f"{'표 없음':>11} {'-':>13} "
                    continue
                hi = max(0.0, t0 - args.below_pct)
                lo = max(0.0, hi - args.band_pct)
                band[(gid, slot)] = (lo, hi)
                row += f"{t0:11.2f} {f'{lo:.1f}~{hi:.1f}':>13} "
            print(row)
    if args.dry_run:
        return 0

    if args.mode == "below":
        print(f"\n안전: 챔버가 대기압 ±{args.p_band:g} kPa 를 벗어나면 그 채널을 닫고, "
              f"±{args.abort_band:g} 를 넘으면 전체 중단한다.")
    print("확인: 펌프 **켜짐**, pp_controller **꺼짐**.")
    if not args.yes and input("진행할까? [y/N] ").strip().lower() not in ("y", "yes"):
        print("취소")
        return 1

    if args.seed is not None:
        random.seed(args.seed)

    import rclpy
    rclpy.init()
    rig = Rig(cfg, rail_pos, rail_neg, p_max=190.0, rail_ramp=args.rail_ramp,
              lead=1.0e6, idle_open_pct=100.0)
    rig.priority = None          # 양쪽 레일 모두 조절한다
    try:
        spin(rig, 1.5)
        if rig.n_msg == 0:
            print("board/sensors 를 못 받았다 — can_bridge 가 돌고 있나?")
            return 1
        rivals = rig.rival_publishers()
        if rivals:
            print(f"board/pwm_cmd 에 다른 발행자가 있다: {rivals}\n"
                  f"pp_controller 를 끄고 다시 실행해라.")
            return 1

        base = {}
        if args.mode == "below":
            # 전 채널을 대기압으로. 이래야 "움직이면 이상" 판정이 성립한다.
            print(f"\n전 채널을 대기압으로 내린다 ({args.vent_sec:g} s)...")
            rig.close_all_channels()
            for board in range(POS_BOARD0, NEG_BOARD0 + N_AXES):
                rig.set_valve(board, V_ATM, 100.0)
            spin(rig, args.vent_sec)
            rig.close_all_channels()
            spin(rig, 2.0)
            base = {gid: rig.kpa((POS_BOARD0 if gid < N_AXES else NEG_BOARD0 - N_AXES) + gid)
                    for gid in gids}
            print("  기준 챔버압: " + " ".join(
                f"ch{g}={base[g]:.1f}" for g in gids if base[g] is not None))

        # ② 예열
        print(f"\n예열 시작 — {args.minutes:g} 분\n")
        t_end = time.monotonic() + args.minutes * 60.0
        t_next = 0.0
        t_log = 0.0
        disabled = set()
        state = {k: (v[0] + v[1]) * 0.5 for k, v in band.items() if v}
        while time.monotonic() < t_end:
            now = time.monotonic()
            if now >= t_next:
                t_next = now + args.step_sec
                for gid in gids:
                    board = (POS_BOARD0 if gid < N_AXES else NEG_BOARD0 - N_AXES) + gid
                    p = rig.kpa(board)
                    is_pos = gid < N_AXES
                    if args.mode == "full":
                        # 하드 가드 — 두 밸브가 함께 열려 있어 원래 갇히지만,
                        # 한쪽이 막히거나 표가 틀린 경우를 대비한 마지막 방어선이다.
                        over = (p is not None and
                                ((is_pos and p > args.p_hi) or
                                 (not is_pos and p < args.p_lo)))
                        if over:
                            if gid not in disabled:
                                disabled.add(gid)
                                print(f"  [가드] ch{gid} 챔버 {p:.1f} — micro 닫고 atm 활짝")
                            rig.set_valve(board, V_MICRO, 0.0)
                            rig.set_valve(board, V_ATM, 100.0)
                            continue
                        if gid in disabled and p is not None:
                            margin = 8.0
                            back = ((is_pos and p < args.p_hi - margin) or
                                    (not is_pos and p > args.p_lo + margin))
                            if back:
                                disabled.discard(gid)
                                print(f"  [복귀] ch{gid} 챔버 {p:.1f}")
                            else:
                                rig.set_valve(board, V_MICRO, 0.0)
                                rig.set_valve(board, V_ATM, 100.0)
                                continue
                        for slot in (V_MICRO, V_ATM):
                            lo, hi = band[(gid, slot)]
                            v = state[(gid, slot)] + random.uniform(-1, 1) * (hi - lo) * 0.5
                            state[(gid, slot)] = v = min(hi, max(lo, v))
                            rig.set_valve(board, slot, v)
                        continue
                    # ── below 방식 ────────────────────────────────────────
                    if p is not None and base.get(gid) is not None:
                        drift = abs(p - base[gid])
                        if drift > args.abort_band:
                            raise RuntimeError(
                                f"ch{gid} 챔버가 기준에서 {drift:.1f} kPa 벗어났다 "
                                f"({p:.1f}) — 밸브가 새거나 표가 크게 틀렸다")
                        if drift > args.p_band and gid not in disabled:
                            disabled.add(gid)
                            print(f"  [차단] ch{gid} 챔버 {p:.1f} (기준 {base[gid]:.1f}) "
                                  f"— 이 채널은 닫아 둔다")
                    for slot in (V_MICRO, V_ATM):
                        b = band.get((gid, slot))
                        if b is None or gid in disabled:
                            rig.set_valve(board, slot, 0.0)
                            continue
                        # 난수 보행 — 계단으로 튀지 않게 범위 안에서 조금씩 움직인다
                        lo, hi = b
                        v = state[(gid, slot)] + random.uniform(-1, 1) * (hi - lo) * 0.5
                        state[(gid, slot)] = v = min(hi, max(lo, v))
                        rig.set_valve(board, slot, v)
            rclpy.spin_once(rig, timeout_sec=0.005)
            rig.publish()
            if now >= t_log:
                t_log = now + 15.0
                left = (t_end - now) / 60.0
                ps = []
                for gid in gids:
                    board = (POS_BOARD0 if gid < N_AXES else NEG_BOARD0 - N_AXES) + gid
                    v = rig.kpa(board)
                    if v is not None:
                        ps.append((gid, v))
                if args.mode == "full":
                    pos = [v for g, v in ps if g < N_AXES]
                    neg = [v for g, v in ps if g >= N_AXES]
                    cur = sum(state.values()) / max(1, len(state)) * 2.5
                    extra = (f"양압챔버 {min(pos):.0f}~{max(pos):.0f}" if pos else "") + \
                            (f" 음압챔버 {min(neg):.0f}~{max(neg):.0f}" if neg else "") + \
                            f" | 평균전류 {cur:.0f} mA"
                else:
                    dr = [abs(v - base[g]) for g, v in ps if base.get(g) is not None]
                    extra = f"챔버 드리프트 최대 {max(dr) if dr else float('nan'):.2f} kPa"
                print(f"  남은 {left:4.1f} 분 | {rig.rail_status()} | " + extra
                      + (f" | 가드 {sorted(disabled)}" if disabled else ""), flush=True)

        print("\n[완료] 예열을 마쳤다. 바로 이어서 측정/제어를 실행해라 — "
              "식으면 열 상태가 달라진다.")
        return 0
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C")
        return 130
    except RuntimeError as exc:
        print(f"\n[중단] {exc}", file=sys.stderr)
        return 1
    finally:
        rig.close_all_channels()
        spin(rig, 0.5)
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
