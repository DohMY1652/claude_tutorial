#!/usr/bin/env python3
"""pressure_id_seq CSV 에서 **정적 액추에이터 모델**을 뽑는다.

무엇을 맞추는가
---------------
정지 상태에서는 액추에이터 토크와 중력 토크가 균형이므로, 각도 하나가 곧 토크다:

    τ_act(P+, P−) = m·g·L·sin(θ)          [N·m]

그래서 (P+, P−) → θ 를 모아 두면 τ_act 의 압력 의존성을 그대로 볼 수 있다.

이 액추에이터는 실린더가 아니라 **flexible duct hose** 다. 옆면이 고정돼 있지
않아서 양압이면 부풀고 음압이면 오그라든 채로 있다 — 즉 유효 면적이 압력에
따라 변한다. 그래서 `F = A·(P+ − P−)` 에 A 를 상수로 두면 안 맞는다. 대신
두 챔버를 따로 두고 게이지압의 2차까지 연다:

    τ_act = a_p·u + b_p·u² + a_n·v + b_n·v² + k·u·v − s·(f0 + f1·u)

      u = P+ − 대기압   (양압 챔버 게이지압, ≥0)
      v = 대기압 − P−   (음압 챔버 게이지압, ≥0)
      s = +1 올라오며 멈춤 / −1 내려오며 멈춤   (마찰 이력)

  · a_p, a_n  — 각 챔버의 기본 유효 면적 (×r_reel)
  · b_p, b_n  — 그 면적이 자기 압력에 따라 변하는 몫 (호스가 부풀고 오그라든다)
  · k         — **두 챔버의 간섭.** 한쪽이 눌려 있으면 다른 쪽이 덜 낸다.
                분리형(k=0)으로 한 챔버 데이터만 맞춰 두 챔버 데이터를 예측하면
                토크를 크게 과대예측한다 (실측 잔차 +2° → −17°).
  · f0, f1    — 마찰 반폭. 양압이 높을수록 넓어진다 (부푼 벽이 더 문다).

★ 이 데이터로 **못** 가르는 것
------------------------------
정적 측정에서는 팔이 늘 그 차압의 평형에 앉는다. 즉 **각도와 압력이 거의 완전히
얽혀 있다.** "면적이 압력에 따라 준다" 와 "면적이 각도(챔버 부피)에 따라 준다" 는
이 데이터로 구별되지 않는다 — 위 계수는 둘의 합을 담고 있다.

가르려면 **팔을 한 각도에 고정해 놓고 압력만 훑어야 한다** (locked-rotor).
`angle_ctrl.py` 로 각도를 잡아 두고 배분만 바꾸는 것도 같은 효과다.

m·g·L 과 r_reel 은 `actuator_map.py` 기본값이다. 이 값이 틀리면 면적의 **절대
크기**가 같은 비율로 틀린다 — 챔버 간 비나 압력 의존성 같은 **모양**은 안 변한다.

사용 예
-------
    python3 actuator_model_fit.py ~/result/pressure_id_*.csv
    python3 actuator_model_fit.py ~/result/pressure_id_20260914_180329.csv --tail 0.3
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import random
import sys

ATM = 101.325


# ════════════════════════════════════════════════════════════════════════════
#  CSV → 정적 점
# ════════════════════════════════════════════════════════════════════════════
def load_points(csv_path: str, args) -> list[dict]:
    """지령이 일정한 구간마다 뒤쪽 일부를 평균해 한 점으로 만든다.

    계획(`_meta.json`)이 옆에 있으면 시나리오 이름을 붙인다. 지령이 같은 단이
    연속되면 CSV 에서는 한 구간으로 합쳐지므로 계획 쪽도 똑같이 합쳐서 짝짓는다
    — 안 그러면 인덱스가 밀려 라벨이 전부 어긋난다.
    """
    path = os.path.expanduser(csv_path)
    rows = list(csv.DictReader(open(path, encoding="utf-8")))
    if not rows:
        return []
    # 열 이름은 두 세대가 있다. 예전 것은 `t_s` 와 축 접미사(`_ax1`)를 쓴다.
    cols = rows[0]
    def pick(*names: str) -> str | None:
        for n in names:
            if n in cols:
                return n
            hit = [k for k in cols if k.startswith(n + "_ax")]
            if hit:
                return sorted(hit)[0]
        return None
    C = {k: pick(*v) for k, v in {
        "t": ("time_s", "t_s"), "pr": ("p_pos_ref",), "nr": ("p_neg_ref",),
        "pm": ("p_pos_meas",), "nm": ("p_neg_meas",), "ang": ("angle_deg",),
    }.items()}
    missing = [k for k, v in C.items() if v is None]
    if missing:
        print(f"   건너뜀 — 필요한 열이 없다: {', '.join(missing)}")
        return []

    groups, cur, key = [], [], None
    for r in rows:
        k = (r[C["pr"]], r[C["nr"]])
        if k != key:
            if cur:
                groups.append((key, cur))
            key, cur = k, []
        cur.append(r)
    if cur:
        groups.append((key, cur))

    meta = path.replace(".csv", "_meta.json")
    plan = None
    if os.path.exists(meta):
        raw = json.load(open(meta, encoding="utf-8"))["plan"]
        plan, run = [], None
        for p in raw:                      # 지령이 같은 연속 단을 합친다
            k = (p["p_pos"], p["p_neg"])
            if run and run[0] == k:
                run[1] += 1
            else:
                if run:
                    plan.append(run)
                run = [k, 1, p]
        if run:
            plan.append(run)

    pts, prev, k_plan = [], None, 0
    for (pr, nr), grp in groups:
        t = [float(r[C["t"]]) for r in grp]
        if t[-1] - t[0] < args.min_hold:    # 램프 중간 조각 — 계획에는 없다
            continue
        i, k_plan = k_plan, k_plan + 1      # 유지 구간만 계획 단과 1:1 이다
        sel = grp[int(len(grp) * (1 - args.tail)):]
        ang = [float(r[C["ang"]]) for r in sel if r[C["ang"]]]
        pp = [float(r[C["pm"]]) for r in sel if r[C["pm"]]]
        pn = [float(r[C["nm"]]) for r in sel if r[C["nm"]]]
        if not ang or not pp or not pn:
            continue
        m = sum(ang) / len(ang)
        sd = math.sqrt(sum((x - m) ** 2 for x in ang) / len(ang))
        p_pos, p_neg = sum(pp) / len(pp), sum(pn) / len(pn)
        d = None if prev is None else m - prev
        pts.append(dict(
            src=os.path.basename(path), t0=t[0],
            scen=(plan[i][2]["scen"] if plan and i < len(plan) else "?"),
            p_pos=p_pos, p_neg=p_neg, ang=m, ang_sd=sd,
            u=max(0.0, p_pos - ATM), v=max(0.0, ATM - p_neg),
            s=0 if d is None else (1 if d > args.dir_band else
                                   (-1 if d < -args.dir_band else 0)),
            tau=args.mass * 9.81 * args.link * math.sin(math.radians(m - args.angle_offset)),
        ))
        prev = m
    return pts


# ════════════════════════════════════════════════════════════════════════════
def lstsq(A: list[list[float]], b: list[float]) -> list[float]:
    n = len(A[0])
    AT = list(zip(*A))
    M = [[sum(AT[i][k] * AT[j][k] for k in range(len(A))) for j in range(n)] for i in range(n)]
    v = [sum(AT[i][k] * b[k] for k in range(len(A))) for i in range(n)]
    for i in range(n):
        p = max(range(i, n), key=lambda r: abs(M[r][i]))
        M[i], M[p] = M[p], M[i]
        v[i], v[p] = v[p], v[i]
        if abs(M[i][i]) < 1e-12:
            continue
        for r in range(n):
            if r != i:
                f = M[r][i] / M[i][i]
                for c in range(i, n):
                    M[r][c] -= f * M[i][c]
                v[r] -= f * v[i]
    return [v[i] / M[i][i] if abs(M[i][i]) > 1e-12 else 0.0 for i in range(n)]


# ── 논문 값 (Park et al., Mechatronics 97 (2024) 103099) ────────────────────
# 표 3 — 실측 다항식. F [N], x [mm], P [kPa 게이지]
PAP_E = (-0.8884, 0.01138, 2.077)                       # 식 17: e00, e10, e01
PAP_C = (6.372, -0.2148, -1.517, -0.001426, 0.00656)    # 식 16: c00,c10,c01,c11,c20
# 표 5 — CoAM 기구
PAP_RE, PAP_XI, PAP_XSUM = 38.6, 20.0, 120.0            # mm
PAP_FRIC = 6.8e-2 + 6.6e-2                              # 식 33: α12 + α34
PAP_DI = 54.5                                           # 표 2, mm


def paper_x2(ang_deg: float) -> float:
    """식 31: 팽창 챔버의 변위 x2 = r_e·θ + x_i. 식 30 으로 x1 = x_sum − x2."""
    return PAP_RE * math.radians(ang_deg) + PAP_XI


BASES = {
    "ΔP 만":        lambda x: [x["u"] + x["v"], -x["s"]],
    "ΔP 2차":       lambda x: [x["u"] + x["v"], (x["u"] + x["v"]) ** 2, -x["s"]],
    "챔버 분리":     lambda x: [x["u"], x["u"]**2, x["v"], x["v"]**2, -x["s"]],
    "분리+마찰(u)":  lambda x: [x["u"], x["u"]**2, x["v"], x["v"]**2, -x["s"], -x["s"]*x["u"]],
    "분리+교차+마찰": lambda x: [x["u"], x["u"]**2, x["v"], x["v"]**2, x["u"]*x["v"],
                                -x["s"], -x["s"]*x["u"]],
}
FULL = "분리+교차+마찰"
LABELS = ["a_p  (u)", "b_p  (u²)", "a_n  (v)", "b_n  (v²)",
          "k    (u·v)", "f0   (마찰)", "f1   (마찰·u)"]


def err_deg(e: float, x: dict, kmax: float) -> float:
    """토크 잔차를 그 각도에서의 각도 오차로 환산한다 (dτ/dθ = kmax·cos θ)."""
    return abs(e) / max(1e-6, kmax * abs(math.cos(math.radians(x["ang"])))) * 180 / math.pi


def cross_val(D: list[dict], basis, k: int, kmax: float, seed: int) -> tuple[float, float]:
    idx = list(range(len(D)))
    random.Random(seed).shuffle(idx)
    errs, degs = [], []
    for f in range(k):
        te = [D[i] for j, i in enumerate(idx) if j % k == f]
        tr = [D[i] for j, i in enumerate(idx) if j % k != f]
        if len(tr) <= len(basis(D[0])):
            continue
        c = lstsq([basis(x) for x in tr], [x["tau"] for x in tr])
        for x in te:
            e = x["tau"] - sum(a * b for a, b in zip(c, basis(x)))
            errs.append(e)
            degs.append(err_deg(e, x, kmax))
    return (math.sqrt(sum(e * e for e in errs) / len(errs)),
            sum(degs) / len(degs)) if errs else (float("nan"), float("nan"))


def compare_paper(D: list[dict], kmax: float, args) -> None:
    """논문(BiPAM/CoAM) 모델과 대조한다. 자세한 해설은
    `docs/논문모델_대조.md` 에 있다."""
    e00, e10, e01 = PAP_E
    c00, c10, c01, c11, c20 = PAP_C
    for x in D:
        x["x2"] = paper_x2(x["ang"])
        x["x1"] = PAP_XSUM - x["x2"]

    print("\n" + "═" * 72)
    print("논문 대조 — Park et al., Mechatronics 97 (2024) 103099")
    print("═" * 72)
    print("\n[논문이 예측하는 유효 면적]")
    print(f"   팽창(양압)  식 17·표 3: ∂F/∂P = e01 = {e01:g} N/kPa = {e01*10:.1f} cm²"
          f"   — 압력·변위 무관")
    print(f"      식 15 의 이론값 πD_i²/4 = {math.pi*PAP_DI**2/400:.1f} cm² 와 "
          f"{100*abs(e01*10-math.pi*PAP_DI**2/400)/(math.pi*PAP_DI**2/400):.0f} % 차이")
    print(f"   수축(음압)  식 16·표 3: |∂F/∂P| = {abs(c01):.3f} + {abs(c11):.6f}·x1 N/kPa")
    for xx in (20, 60, 99):
        print(f"      x1={xx:3d} mm → {(abs(c01)+abs(c11)*xx)*10:.1f} cm²")
    A_e, A_c = e01*10, (abs(c01)+abs(c11)*60)*10
    print(f"   → 면적비 A_p/A_n = {A_e/A_c:.2f}")
    print(f"   식 33 마찰 = 액추에이터 힘의 {100*PAP_FRIC:.1f} %  (베어링 캡스턴)")
    print("   식 29 F_act = F_con + F_exp — 두 챔버가 더해진다. **교차항 없음**")

    PAP = {
        "P1 논문 그대로 (면적 상수, 마찰∝힘)":
            lambda x: [x["u"], x["v"], -x["s"]*(x["u"]+x["v"])],
        "P2 P1 + 수축 면적이 변위에 (식16 c11)":
            lambda x: [x["u"], x["v"], x["v"]*x["x1"], -x["s"]*(x["u"]+x["v"])],
        "P3 P2 + 팽창 면적도 변위에":
            lambda x: [x["u"], x["u"]*x["x2"], x["v"], x["v"]*x["x1"],
                       -x["s"]*(x["u"]+x["v"])],
    }
    print(f"\n[논문 구조를 우리 데이터에 맞춰 본다]  {args.folds}-겹 교차검증")
    print(f"   {'모델':<40} {'항':>3} {'검증 각도':>9}")
    ours = {f"E{i+1} 우리 것: {k}": v
            for i, (k, v) in enumerate(list(BASES.items())[3:])}
    for n, B in {**PAP, **ours}.items():
        c = lstsq([B(x) for x in D], [x["tau"] for x in D])
        print(f"   {n:<40} {len(c):>3} {cross_val(D, B, args.folds, kmax, args.seed)[1]:>8.2f}°")

    c = lstsq([PAP["P1 논문 그대로 (면적 상수, 마찰∝힘)"](x) for x in D],
              [x["tau"] for x in D])
    r_e = PAP_RE/1000
    Ap, An = c[0]/r_e/1000*1e4, c[1]/r_e/1000*1e4
    print("\n[P1 의 계수로 본 항목별 대조]")
    print(f"   {'항목':<28} {'우리':>12} {'논문':>12}")
    print(f"   {'면적비 A_p/A_n (가정 무관)':<28} {Ap/An:>12.2f} {A_e/A_c:>12.2f}")
    print(f"   {'마찰 / 액추에이터 힘':<28} "
          f"{100*c[2]/((c[0]+c[1])/2):>11.1f}% {100*PAP_FRIC:>11.1f}%")
    print(f"   {'A_p [cm²] (r_e 38.6 mm)':<28} {Ap:>12.1f} {A_e:>12.1f}")
    print(f"   {'A_n [cm²]':<28} {An:>12.1f} {A_c:>12.1f}")
    sc = (Ap*A_e + An*A_c)/(Ap*Ap + An*An)
    print(f"\n   두 면적을 동시에 맞추는 축척 {sc:.3f}")
    print(f"   ⇒ 실제 중력상수는 {kmax:.3f} 가 아니라 **{kmax*sc:.2f} N·m** 여야 한다")
    print(f"      (R_arm 0.3 m, r_cm·W 0.36 N·m 이면 payload {(kmax*sc-0.36)/(0.3*9.81):.2f} kg)")
    print(f"      그 축척에서 우리 A_p {Ap*sc:.1f} / A_n {An*sc:.1f} cm²")

    c2 = lstsq([PAP["P2 P1 + 수축 면적이 변위에 (식16 c11)"](x) for x in D],
               [x["tau"] for x in D])
    print("\n[어긋나는 곳 — 변위 의존의 크기]")
    print("   수축 면적이 x1 0→100 mm 에서")
    print(f"      논문 (c11/c01)  {100*abs(c11)*100/abs(c01):+.1f} %")
    print(f"      우리 (P2 적합)  {100*c2[2]*100/c2[1]:+.1f} %   "
          f"← 부호는 같고 크기가 {c2[2]/c2[1]/(abs(c11)/abs(c01)):.0f} 배")
    print("   정적 측정에서는 각도와 압력이 얽혀 있어 '변위 탓'과 '압력 탓'을")
    print("   구별할 수 없다 (위 표의 P3 와 E1 을 비교할 것).")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="pressure_id CSV → 정적 액추에이터 모델",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("csv", nargs="+", help="pressure_id_*.csv (여러 개 가능)")
    ap.add_argument("--tail", type=float, default=0.4,
                    help="각 유지구간의 뒤쪽 이 비율만 평균한다 (과도 제외)")
    ap.add_argument("--min-hold", type=float, default=1.0,
                    help="이보다 짧은 구간은 램프 조각으로 보고 버린다 [s]")
    ap.add_argument("--sd-max", type=float, default=0.3,
                    help="유지 중 각도 σ 가 이보다 크면 안 멈춘 것으로 보고 버린다 [°]")
    ap.add_argument("--dp-min", type=float, default=3.0,
                    help="게이지압 합이 이보다 작은 점은 버린다 [kPa]")
    ap.add_argument("--dir-band", type=float, default=0.3,
                    help="앞 구간 대비 이만큼 움직여야 올림/내림으로 센다 [°]")
    ap.add_argument("--scen", default="1,3,4,5",
                    help="쓸 시나리오. 기본은 정적인 것만 (라벨이 없으면 전부 쓴다)")
    ap.add_argument("--folds", type=int, default=5, help="교차검증 겹 수")
    ap.add_argument("--paper", action="store_true",
                    help="논문(Park et al. 2024, BiPAM/CoAM) 모델과 항목별로 대조한다. "
                         "해설은 docs/논문모델_대조.md")
    ap.add_argument("--seed", type=int, default=0)
    g = ap.add_argument_group("기구 — 절대 크기가 여기 걸린다")
    g.add_argument("--mass", type=float, default=2.0, help="링크 끝 질량 [kg]")
    g.add_argument("--link", type=float, default=0.15, help="링크 길이 [m]")
    g.add_argument("--reel-dia", type=float, default=0.05, help="릴 지름 [m]")
    g.add_argument("--angle-offset", type=float, default=0.0,
                   help="중력 토크 = m·g·L·sin(θ − offset) [°]")
    a = ap.parse_args()

    kmax = a.mass * 9.81 * a.link
    r_reel = a.reel_dia / 2.0
    keep = {v for v in a.scen.replace(" ", "").split(",") if v}

    files: list[str] = []
    for pat in a.csv:
        files += sorted(glob.glob(os.path.expanduser(pat))) or [pat]
    pts: list[dict] = []
    for f in files:
        if f.endswith("_meta.json"):
            continue
        got = load_points(f, a)
        pts += got
        print(f"[읽음] {os.path.basename(f)}  정적 구간 {len(got)}")

    D = [x for x in pts
         if x["ang_sd"] < a.sd_max and x["s"] and x["u"] + x["v"] > a.dp_min
         and (x["scen"] == "?" or x["scen"] in keep)]
    if len(D) < 12:
        print(f"\n[중단] 쓸 만한 정적 점이 {len(D)} 개뿐이다. "
              f"--scen / --sd-max 를 확인할 것.", file=sys.stderr)
        return 2

    print(f"\n정적 점 {len(D)} 개 "
          f"(σ<{a.sd_max:g}°, 게이지 합>{a.dp_min:g} kPa, 방향이 잡힌 것만)")
    print(f"   u  0 ~ {max(x['u'] for x in D):.0f} kPa · "
          f"v  0 ~ {max(x['v'] for x in D):.0f} kPa · "
          f"각도 {min(x['ang'] for x in D):.0f} ~ {max(x['ang'] for x in D):.0f}°")
    from collections import Counter
    print(f"   시나리오 {dict(Counter(x['scen'] for x in D))}")

    print(f"\n{'모델':<18} {'항':>3} {'학습 RMS':>9} "
          f"{'%d-겹 검증' % a.folds:>10} {'검증 각도':>10}")
    for name, B in BASES.items():
        c = lstsq([B(x) for x in D], [x["tau"] for x in D])
        e = [x["tau"] - sum(p * q for p, q in zip(c, B(x))) for x in D]
        tr = math.sqrt(sum(v * v for v in e) / len(e))
        cv, cvd = cross_val(D, B, a.folds, kmax, a.seed)
        print(f"{name:<18} {len(c):>3} {tr:>9.4f} {cv:>10.4f} {cvd:>9.2f}°")

    B = BASES[FULL]
    c = lstsq([B(x) for x in D], [x["tau"] for x in D])
    print(f"\n★ {FULL}")
    for l, v in zip(LABELS, c):
        print(f"     {l:<13} {v:+.6f}")
    print(f"""
  τ_act [N·m] = {c[0]:+.5f}·u {c[1]:+.6f}·u² {c[2]:+.5f}·v {c[3]:+.6f}·v² {c[4]:+.6f}·u·v
                − s·({c[5]:.4f} {c[6]:+.5f}·u)
     u = P+ − {ATM:g} , v = {ATM:g} − P−  [kPa 게이지] , s = +1 올라와 / −1 내려와 멈춤
     정적 평형:  τ_act = {kmax:.3f}·sin(θ − {a.angle_offset:g}°)""")

    print(f"\n유효 면적 [cm²]  A = (∂τ/∂P)/r_reel,  r_reel {r_reel*1000:.0f} mm "
          f"(상대 챔버가 대기압일 때)")
    print(f"   {'게이지압':>8} {'양압 챔버':>10} {'음압 챔버':>10} {'양/음':>7}")
    umax, vmax = max(x["u"] for x in D), max(x["v"] for x in D)
    for P in (5, 10, 20, 30, 40, 50):
        if P > max(umax, vmax):
            break
        Ap = (c[0] + 2 * c[1] * P) / r_reel / 1000 * 1e4
        An = (c[2] + 2 * c[3] * P) / r_reel / 1000 * 1e4
        print(f"   {P:8d} {Ap:10.1f} {An:10.1f} {Ap/An:7.2f}"
              f"{'' if P <= min(umax, vmax) else '   ← 한쪽은 외삽'}")

    print(f"\n교차항 k = {c[4]:+.6f} N·m/kPa² → 반대쪽이 1 kPa 눌릴 때마다 "
          f"양압쪽 유효면적 {c[4]/r_reel/1000*1e4:+.2f} cm²")
    dpmax = max(x["u"] + x["v"] for x in D)
    for v_ in (0, 10, 20, 35):
        if 20 + v_ > dpmax:
            break
        Ap = (c[0] + 2 * c[1] * 20 + c[4] * v_) / r_reel / 1000 * 1e4
        print(f"   u=20, v={v_:2d} → A_p {Ap:5.1f} cm²")

    print(f"\n마찰 반폭  f = {c[5]:.4f} {c[6]:+.5f}·u  N·m")
    for U in (0, 20, 40, 55):
        if U > umax:
            break
        f = c[5] + c[6] * U
        print(f"   u={U:2d} → ±{f:.3f} N·m = 45° 에서 ±"
              f"{f/(kmax*math.cos(math.radians(45)))*180/math.pi:.1f}°")

    print(f"\n적용 범위: u+v ≤ {dpmax:.0f} kPa 안에서만 맞춘 것이다. 밖은 외삽이고,")
    print("           특히 각도와 압력이 얽혀 있어 '면적이 압력 탓' 인지")
    print("           '각도(챔버 부피) 탓' 인지는 이 데이터로 못 가른다.")
    print("           가르려면 팔을 한 각도에 고정하고 압력만 훑어야 한다.")

    if a.paper:
        compare_paper(D, kmax, a)
    return 0


if __name__ == "__main__":
    sys.exit(main())
