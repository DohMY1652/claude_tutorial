#!/usr/bin/env python3
"""
pump_fit_model.py — 피스톤 펌프 모델 + 피팅 라이브러리 (ROS 무관)

`include/PistonPump.hpp` 의 `pneu::pump_piston_avg` / `PumpTable` 을 Python 으로 포팅했다.
식·단위(절대 Pa 입력, kg/s 출력)·번인 회전수·`Npis` 곱·능력경계 판정(5 kPa 스윕,
토출 > 0.02 g/s)까지 동일하다. 검증: 해설서 그림 B 의 두 점(음압 −90 → 양압 335 kPa,
−80 → 745 kPa)을 재현해야 한다 (`selftest_port()` 참조).

**컨트롤러가 펌프를 쓰는 곳은 능력경계 하나뿐이다.** `PressureRefGen::decide_rail_setpoint`
가 `cap_ppos(P⁻) → P⁺ 상한` 으로만 쓰고, `flow_out`/`flow_in` 은 생성기에서 호출되지 않는다
(`flow_in` 은 repo 전체에서 죽은 API). 2D 유량 맵은 시뮬 충실도용이다.
→ 목적함수에서 능력경계에 큰 가중치를 준다.

**성능**: 크랭크 궤적 θ(t) 는 동작점과 무관하게 같으므로, 여러 (P⁺,P⁻) 점의 피스톤 질량을
**배열로 동시에** 적분한다. 점마다 2400 스텝을 따로 도는 대신 2400 번의 벡터 연산이 되어
격자 계산이 수십 배 빨라진다 (피팅에는 이게 결정적이다).
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import valve_fit_model as vm   # noqa: E402  (nelder_mead / random_search / dpdt 재사용)

# ── 물리 상수 (PneumaticFlow.hpp 와 동일) ──────────────────────────────────
P_ATM = 101325.0          # [Pa]
R_AIR = 287.0
T_PIS = 323.15            # 피스톤(펌프) 온도 [K] — 챔버(293.15)보다 뜨겁다
KAPPA = 1.4

FRONTIER_THRESH_GPS = 0.02   # 능력경계 판정: 토출 > 0.02 g/s (PistonPump.hpp:117 과 동일)
FRONTIER_STEP_PA = 5.0e3     # 스윕 간격 (동일)


# ══════════════════════════════════════════════════════════════════════════
# 기하 / 재매개화
# ══════════════════════════════════════════════════════════════════════════
class PumpGeom:
    """PistonPump.hpp 의 PumpGeom 과 같은 필드·기본값."""

    def __init__(self, delta=0.041, r=0.02, l=0.07, rpm=3000.0,
                 Spis=38.485e-4, Npis=2, Cb_out=1.46e-6, Cb_in=33.47e-6,
                 dt=1e-4, nrev=12):
        self.delta = delta
        self.r = r
        self.l = l
        self.omega = rpm * 2.0 * math.pi / 60.0
        self.Spis = Spis
        self.Npis = int(Npis)
        self.Cb_out = Cb_out
        self.Cb_in = Cb_in
        self.dt = dt
        self.nrev = int(nrev)

    @property
    def rpm(self):
        return self.omega * 60.0 / (2.0 * math.pi)

    # ── 파생량 (해석·리포트용) ──
    @property
    def v_swept(self):
        """피스톤 1개 1회전 소기량 [m³] = Spis·2r"""
        return self.Spis * 2.0 * self.r

    @property
    def v_dead(self):
        """사구간(클리어런스) 부피 [m³] = Spis·(delta − 2r)"""
        return self.Spis * (self.delta - 2.0 * self.r)

    @property
    def compression_ratio(self):
        """(V_dead + V_swept)/V_dead — 데드헤드(=능력경계)를 지배한다."""
        vd = self.v_dead
        return (vd + self.v_swept) / vd if vd > 1e-15 else float('inf')

    def copy(self):
        g = PumpGeom.__new__(PumpGeom)
        g.__dict__.update(self.__dict__)
        return g

    def as_dict(self):
        return dict(delta_m=self.delta, crank_m=self.r, rod_m=self.l,
                    piston_area_m2=self.Spis, cb_out_m2=self.Cb_out,
                    cb_in_m2=self.Cb_in, rpm=self.rpm, n_piston=self.Npis)

    def __repr__(self):
        return (f'PumpGeom(delta={self.delta:.5g} r={self.r:.5g} l={self.l:.5g} '
                f'rpm={self.rpm:.0f} Spis={self.Spis:.5g} Npis={self.Npis} '
                f'Cb_out={self.Cb_out:.4g} Cb_in={self.Cb_in:.4g})')


# 피팅 좌표. 기하 8개를 그대로 쓰지 않는 이유:
#   유량은 소기량(Spis·2r)에, 데드헤드는 압축비(V_dead)에 각각 걸리므로 이 조합이
#   물리적으로 분리돼 있고 조건수가 훨씬 낫다. l/r 은 부피 파형 형상만 살짝 바꿔
#   거의 식별되지 않으므로 실측값으로 고정하는 것이 정석이다.
FIT_PARAM_NAMES = ['V_swept', 'V_dead', 'A_out', 'A_in', 'l_over_r']

FIT_BOUNDS = {
    'V_swept': (1.0e-6, 1.0e-3),      # 1 mL ~ 1 L / 피스톤 / 회전
    'V_dead': (1.0e-8, 1.0e-4),       # 0.01 ~ 100 mL
    'A_out': (1.0e-8, 1.0e-4),        # 0.01 ~ 100 mm²
    'A_in': (1.0e-8, 1.0e-4),
    'l_over_r': (1.5, 20.0),
}


def geom_to_fit(g):
    return np.array([g.v_swept, g.v_dead, g.Cb_out, g.Cb_in, g.l / g.r])


def fit_to_geom(x, r_fixed, rpm, npis, dt=1e-4, nrev=12):
    """피팅 좌표 → PumpGeom. 크랭크 반경 r 은 실측값으로 고정한다
    (소기량과 r 이 곱으로만 나타나 따로 갈리지 않는다)."""
    v_swept = max(FIT_BOUNDS['V_swept'][0], abs(x[0]))
    v_dead = max(FIT_BOUNDS['V_dead'][0], abs(x[1]))
    spis = v_swept / (2.0 * r_fixed)
    delta = v_dead / spis + 2.0 * r_fixed
    return PumpGeom(delta=delta, r=r_fixed, l=abs(x[4]) * r_fixed, rpm=rpm,
                    Spis=spis, Npis=npis, Cb_out=abs(x[2]), Cb_in=abs(x[3]),
                    dt=dt, nrev=nrev)


def fit_project(x, weight=1.0e6):
    """경계 **안으로 투영**하고 이동 거리를 벌점으로 낸다. (x_clipped, penalty) 반환.

    투영을 쓰는 이유: 예전에는 `pen>0` 이면 데이터를 보지 않고 벌점만 돌려줬는데,
    벌점이 `1e6·((lo−v)/span)²` 이고 `span` 이 1e-4 라 **경계를 아주 살짝 벗어나면
    벌점이 3e-32 까지 작아진다.** 그래서 Nelder-Mead 가 "경계 밖 미세 이탈이 어떤
    실제 피팅보다 싸다" 를 학습해 cost=3.3e-32 을 보고하면서 압축비 350·A_in 하한
    붙은 물리적으로 불가능한 기하를 내놨다. 투영하면 항상 실현 가능한 점의 실제
    데이터 잔차가 평가되므로 그 평평한 탈출구가 사라진다.
    """
    xc = np.array(x, dtype=float)
    pen = 0.0
    for i, name in enumerate(FIT_PARAM_NAMES):
        lo, hi = FIT_BOUNDS[name]
        v = abs(xc[i])
        span = hi - lo
        if v < lo:
            pen += ((lo - v) / span) ** 2
            v = lo
        elif v > hi:
            pen += ((v - hi) / span) ** 2
            v = hi
        xc[i] = v
    return xc, weight * pen


# ══════════════════════════════════════════════════════════════════════════
# 압축성 유동 Φ (벡터, 절대 Pa)
# ══════════════════════════════════════════════════════════════════════════
_PCR = (2.0 / (KAPPA + 1.0)) ** (KAPPA / (KAPPA - 1.0))
_PHI_CHOKED = math.sqrt(KAPPA * (2.0 / (KAPPA + 1.0)) ** ((KAPPA + 1.0) / (KAPPA - 1.0)))
_PHI_C = math.sqrt(2.0 * KAPPA / (KAPPA - 1.0))


def phi_pa(p_in, p_out):
    """orifice_phi 와 동일. 배열 입력. 역류·비유동은 0."""
    p_in = np.asarray(p_in, dtype=float)
    p_out = np.asarray(p_out, dtype=float)
    out = np.zeros(np.broadcast(p_in, p_out).shape, dtype=float)
    valid = (p_in > 1e-9) & (p_out < p_in)
    if not np.any(valid):
        return out
    pr = np.ones_like(out)
    np.divide(p_out, p_in, out=pr, where=valid)
    np.clip(pr, 0.0, 1.0, out=pr)
    choked = valid & (pr <= _PCR)
    sub = valid & (pr > _PCR)
    out[choked] = _PHI_CHOKED
    if np.any(sub):
        t = np.maximum(0.0, pr[sub] ** (2.0 / KAPPA) - pr[sub] ** ((KAPPA + 1.0) / KAPPA))
        out[sub] = _PHI_C * np.sqrt(t)
    return out


# ══════════════════════════════════════════════════════════════════════════
# 1주기 평균 유량 — 여러 동작점을 동시에 적분
# ══════════════════════════════════════════════════════════════════════════
def pump_avg(g, ppos_abs, pneg_abs):
    """(P⁺,P⁻) [절대 Pa] → (토출, 흡입) 평균 질량유량 [kg/s]. 브로드캐스트 가능.

    PistonPump.hpp:40-81 과 같은 forward-Euler. θ 궤적이 동작점과 무관하므로 피스톤 질량만
    배열로 두고 한 번의 루프로 전 격자를 처리한다 (수치적으로는 점별 루프와 동일).
    """
    ppos = np.asarray(ppos_abs, dtype=float)
    pneg = np.asarray(pneg_abs, dtype=float)
    shape = np.broadcast(ppos, pneg).shape
    ppos_f = np.broadcast_to(ppos, shape).ravel().astype(float)
    pneg_f = np.broadcast_to(pneg, shape).ravel().astype(float)
    n = ppos_f.size
    if n == 0:
        z = np.zeros(shape)
        return z, z

    trev = 2.0 * math.pi / g.omega
    nstep = int(round(g.nrev * trev / g.dt))
    t_lastrev = (g.nrev - 1) * trev
    rt = R_AIR * T_PIS
    sqrt_rt = math.sqrt(rt)

    def vp(th):
        s = math.sin(th)
        return g.Spis * (g.delta - g.r + g.l - g.r * math.cos(th)
                         - math.sqrt(max(0.0, g.l * g.l - g.r * g.r * s * s)))

    th = 0.0
    m = np.full(n, P_ATM * vp(0.0) / rt)
    acc_out = np.zeros(n)
    acc_in = np.zeros(n)
    t_acc = 0.0

    for s in range(1, nstep + 1):
        v = max(1e-12, vp(th))          # 스칼라 — 모든 동작점이 같은 크랭크 각을 공유
        ppis = m * rt / v
        mdot_out = g.Cb_out * ppis / sqrt_rt * phi_pa(ppis, ppos_f)
        mdot_in = g.Cb_in * pneg_f / sqrt_rt * phi_pa(pneg_f, ppis)
        m = np.maximum(0.0, m + (mdot_in - mdot_out) * g.dt)
        th += g.omega * g.dt
        if s * g.dt > t_lastrev:        # 마지막 1주기만 평균 (앞은 번인)
            acc_out += mdot_out * g.dt
            acc_in += mdot_in * g.dt
            t_acc += g.dt

    if t_acc <= 0.0:
        t_acc = trev
    out = (g.Npis * acc_out / t_acc).reshape(shape)
    inn = (g.Npis * acc_in / t_acc).reshape(shape)
    return out, inn


def frontier(g, pneg_gauge, ppos_max_gauge=1.2e6,
             step_pa=FRONTIER_STEP_PA, thresh_gps=FRONTIER_THRESH_GPS):
    """능력경계 [게이지 Pa]. PistonPump.hpp:107-121 과 동일한 판정
    (5 kPa 스윕에서 토출 > 0.02 g/s 인 마지막 지점).

    (음압 × 양압) 전 격자를 한 번에 적분하므로 C++ 의 이중 루프보다 훨씬 빠르다.
    """
    pn = np.atleast_1d(np.asarray(pneg_gauge, dtype=float))
    pp = np.arange(0.0, ppos_max_gauge + step_pa, step_pa)
    grid_pp, grid_pn = np.meshgrid(pp, pn, indexing='ij')      # [n_pp, n_pn]
    mo, _ = pump_avg(g, grid_pp + P_ATM, grid_pn + P_ATM)
    alive = (mo * 1e3) > thresh_gps
    out = np.zeros(pn.size)
    for j in range(pn.size):
        idx = np.nonzero(alive[:, j])[0]
        out[j] = pp[idx[-1]] if idx.size else 0.0             # 마지막 통과점
    return out if np.ndim(pneg_gauge) else float(out[0])


def cap_ppos(g, pneg_gauge, **kw):
    """단일 음압 셋포인트에서의 양압 상한 [게이지 Pa]."""
    return frontier(g, [pneg_gauge], **kw)[0]


class PumpMap:
    """2D 유량 테이블 + 이중선형 보간 (PistonPump.hpp 의 PumpTable 과 같은 역할).

    레일 ODE 를 매 스텝 적분할 때 `pump_avg` 를 직접 부르면 스텝마다 크랭크 2400 회를
    돌아 쓸 수 없다. 격자를 한 번 만들어 보간한다 — C++ 도 같은 이유로 테이블을 쓴다.
    """

    def __init__(self, g, ppos_max_gauge=8.0e5, pneg_deep_gauge=-1.0e5, n=17):
        self.gx = P_ATM + np.linspace(0.0, ppos_max_gauge, n)          # 절대 Pa
        self.gy = P_ATM + np.linspace(pneg_deep_gauge, 0.0, n)
        GX, GY = np.meshgrid(self.gx, self.gy, indexing='ij')
        self.out, _ = pump_avg(g, GX, GY)

    def flow(self, ppos_abs, pneg_abs):
        """토출 [kg/s]. 격자 밖은 경계값으로 고정 (C++ interp2 와 동일하게 외삽 안 함)."""
        x = min(max(float(ppos_abs), self.gx[0]), self.gx[-1])
        y = min(max(float(pneg_abs), self.gy[0]), self.gy[-1])
        ix = min(int(np.searchsorted(self.gx, x) - 1), len(self.gx) - 2)
        iy = min(int(np.searchsorted(self.gy, y) - 1), len(self.gy) - 2)
        ix = max(ix, 0); iy = max(iy, 0)
        fx = (x - self.gx[ix]) / (self.gx[ix + 1] - self.gx[ix])
        fy = (y - self.gy[iy]) / (self.gy[iy + 1] - self.gy[iy])
        v = self.out
        return float(max(0.0, (1 - fx) * ((1 - fy) * v[ix, iy] + fy * v[ix, iy + 1])
                         + fx * ((1 - fy) * v[ix + 1, iy] + fy * v[ix + 1, iy + 1])))


# ══════════════════════════════════════════════════════════════════════════
# 레일 질량수지 — 실험 데이터에서 펌프 유량을 뽑는 식
# ══════════════════════════════════════════════════════════════════════════
def mdot_from_rail(dpdt_kpa_s, volume_m3, n_poly=1.0, temp_k=vm.TEMP_K):
    """dP/dt [kPa/s] → 질량유량 [kg/s].  ṁ = V·dP/dt·1000/(n·R·T)

    양 밸브를 닫으면 밸브 유량이 0 이므로
        ṁ_pump = +V⁺/(R·T)·dP⁺/dt + leak⁺   (양압 레일)
               = −V⁻/(R·T)·dP⁻/dt + leak⁻   (음압 레일)
    두 식이 질량보존으로 같아야 하므로 매 점에서 교차검증이 된다.
    레일은 초 단위로 느려 등온(n=1)이 맞다 (해설서 4.1절, PressureRefGen n_rail=1.0).
    """
    return np.asarray(dpdt_kpa_s, dtype=float) * volume_m3 * 1000.0 / (n_poly * vm.RGAS * temp_k)


def leak_kgps(delta_p_kpa, k_lpm_per_kpa):
    """기생 누설 [kg/s]. 시뮬과 같은 선형 모델 (LPM/kPa × 경험 LPM 환산)."""
    return np.maximum(0.0, np.asarray(delta_p_kpa, dtype=float)) * k_lpm_per_kpa * vm.LPM_TO_KGPS


def exp_decay_fit(t, p_kpa, p_inf_guess=None):
    """압력 감쇠에서 시상수 τ 와 점근값을 뽑는다 (펌프 OFF 누설 시험).

    누설이 ΔP 에 선형이면 P(t) = P_inf + (P0−P_inf)·exp(−t/τ),  τ = V·1000/(R·T·k)
    → **ΔV 회차의 τ 비가 (V+ΔV)/V** 이므로 V 와 k 가 함께 풀린다 (펌프 무관).
    반환: (tau_s, p0, p_inf, r2)
    """
    t = np.asarray(t, dtype=float)
    p = np.asarray(p_kpa, dtype=float)
    if t.size < 10:
        return None
    p_inf = float(p_inf_guess) if p_inf_guess is not None else vm.P_ATM_KPA
    y = p - p_inf
    # 부호를 통일해 로그 회귀 (양압 감쇠는 +, 음압 회복은 −)
    sgn = 1.0 if y[0] >= 0 else -1.0
    y = y * sgn
    ok = y > max(1e-3, 0.02 * abs(y[0]))     # 잡음 바닥 근처는 버린다
    if np.count_nonzero(ok) < 10:
        return None
    a = np.polyfit(t[ok], np.log(y[ok]), 1)
    tau = -1.0 / a[0] if a[0] < 0 else float('inf')
    p0 = sgn * math.exp(a[1]) + p_inf
    pred = sgn * np.exp(np.polyval(a, t[ok])) + p_inf
    ss_res = float(np.sum((p[ok] - pred) ** 2))
    ss_tot = float(np.sum((p[ok] - np.mean(p[ok])) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float('nan')
    return dict(tau=tau, p0=p0, p_inf=p_inf, r2=r2, n=int(np.count_nonzero(ok)))


def volume_from_tau(tau_bare, tau_extra, extra_ml):
    """이중 부피법: τ 비 = (V+ΔV)/V  →  V = ΔV/(r−1) [mL]"""
    if not (tau_bare and tau_extra) or tau_bare <= 0:
        return None, None
    r = tau_extra / tau_bare
    if r <= 1.0 + 1e-6:
        return None, r
    return extra_ml / (r - 1.0), r


def leak_from_tau(tau_s, volume_ml, temp_k=vm.TEMP_K):
    """τ = V·1000/(R·T·k·LPM_TO_KGPS)  →  k [LPM/kPa]"""
    if not tau_s or tau_s <= 0:
        return None
    return volume_ml * 1e-6 * 1000.0 / (vm.RGAS * temp_k * tau_s * vm.LPM_TO_KGPS)


# ══════════════════════════════════════════════════════════════════════════
# 피팅
# ══════════════════════════════════════════════════════════════════════════
LOG_PARAMS = ('V_swept', 'V_dead', 'A_out', 'A_in')   # 스케일이 자릿수로 다른 것들


def _to_z(x):
    """피팅 좌표 → 최적화 좌표. 크기 파라미터는 **로그 공간**으로 옮긴다.

    이유 두 가지:
      1. 1e-8 ~ 1e-3 를 넘나드는 값과 O(1) 값을 한 심플렉스에 섞으면 Nelder-Mead 의
         초기 스텝(5%)이 어떤 축에서는 무의미하고 어떤 축에서는 과도해진다.
      2. 기존값이 예전 펌프라 **자릿수가 틀릴 수 있다**. 곱셈 노이즈로 그 주변만 뒤지면
         진짜 값이 탐색 공간에 아예 없다 (실측: 소기량이 기존값의 0.09배라 ×[0.3,2.5]
         범위로는 도달 불가였다). 로그 공간 균등 샘플링이 자릿수를 가로지른다.
    """
    z = np.array(x, dtype=float)
    for i, name in enumerate(FIT_PARAM_NAMES):
        if name in LOG_PARAMS:
            z[i] = math.log(max(abs(z[i]), FIT_BOUNDS[name][0] * 1e-3))
    return z


def _from_z(z):
    x = np.array(z, dtype=float)
    for i, name in enumerate(FIT_PARAM_NAMES):
        if name in LOG_PARAMS:
            x[i] = math.exp(min(max(x[i], -60.0), 10.0))
    return x


def _flow_cost(g, pts, scale):
    """유량 잔차. pts = [(ppos_g, pneg_g, mdot_target, weight, one_sided), ...]

    스톨점(능력경계)도 여기서 함께 다룬다 — 측정 스톨은 `ṁ_pump = leak⁺(P⁺)` 인 지점이므로
    "유량이 누설과 같은 맵 점"이다. 압력을 비교하면 모델의 경계 판정 기준(> 0.02 g/s)과
    실측 정의(dP/dt ≈ 0, 즉 누설과 균형)가 어긋나 계통 오차가 생긴다.
    one_sided 는 안전 상한에 먼저 닿아 스톨을 못 본 점 — 모델이 그 이상 내면 벌점 0.
    """
    if not pts:
        return 0.0
    a = np.array(pts, dtype=float)
    mo, _ = pump_avg(g, a[:, 0] + P_ATM, a[:, 1] + P_ATM)
    mo = np.atleast_1d(mo)
    d = (mo - a[:, 2]) / max(scale, 1e-12)
    d = np.where((a[:, 4] > 0.5) & (d >= 0), 0.0, d)
    w = a[:, 3]
    return float(np.sum(w * d * d) / max(np.sum(w), 1e-12))


LOG_PARAMS = ('V_swept', 'V_dead', 'A_out', 'A_in')   # 스케일이 자릿수로 다른 것들


def _to_z(x):
    """피팅 좌표 → 최적화 좌표. 크기 파라미터는 **로그 공간**으로 옮긴다.

    이유 두 가지:
      1. 1e-8 ~ 1e-3 를 넘나드는 값과 O(1) 값을 한 심플렉스에 섞으면 Nelder-Mead 의
         초기 스텝(5%)이 어떤 축에서는 무의미하고 어떤 축에서는 과도해진다.
      2. 기존값이 예전 펌프라 **자릿수가 틀릴 수 있다**. 곱셈 노이즈로 그 주변만 뒤지면
         진짜 값이 탐색 공간에 아예 없다 (실측: 소기량이 기존값의 0.09배라 ×[0.3,2.5]
         범위로는 도달 불가였다). 로그 공간 균등 샘플링이 자릿수를 가로지른다.
    """
    z = np.array(x, dtype=float)
    for i, name in enumerate(FIT_PARAM_NAMES):
        if name in LOG_PARAMS:
            z[i] = math.log(max(abs(z[i]), FIT_BOUNDS[name][0] * 1e-3))
    return z


def _from_z(z):
    x = np.array(z, dtype=float)
    for i, name in enumerate(FIT_PARAM_NAMES):
        if name in LOG_PARAMS:
            x[i] = math.exp(min(max(x[i], -60.0), 10.0))
    return x


def _stall_cost(g, stalls, scale_pa):
    """스톨 **압력** 잔차. stalls = [(pneg_g, ppos_stall_g, leak_gps, one_sided), ...]

    유량만 맞추면 데드헤드 위치가 정해지지 않는다 (실측: 맵 RMS 19% 로 맞으면서 압축비가
    469 까지 올라가 능력경계가 1200 kPa 로 튀었다). 그래서 압력 항을 함께 넣는데,
    **모델 경계 판정 임계를 측정 누설로 맞춘다** — 측정 스톨은 `ṁ_pump = leak⁺` 인 지점이라
    기본 임계(0.02 g/s)로 비교하면 계통 오차가 생긴다.
    """
    if not stalls:
        return 0.0
    acc = 0.0
    for pn, ps, leak_gps, one_sided in stalls:
        pred = frontier(g, [pn], ppos_max_gauge=max(ps * 3.0, 1.2e6),
                        thresh_gps=max(leak_gps, 1e-4))[0]
        d = (pred - ps) / scale_pa
        if one_sided > 0.5 and d >= 0:
            d = 0.0
        acc += d * d
    return acc / len(stalls)


def fit(flow_points, r_fixed, rpm, npis=2, base=None, stall_points=None, w_stall=3.0,
        n_samples=80, n_starts=3, seed=0, dt=4e-4, nrev=6, verbose=True):
    """펌프 기하 피팅.

    flow_points : [(ppos_gauge_pa, pneg_gauge_pa, mdot_kgps, weight, one_sided), ...]
                  Phase M 의 맵 점과 Phase F 의 스톨점을 **같은 형태**로 넘긴다
                  (스톨점은 목표 유량 = 누설, 가중치를 크게).
    r_fixed     : 실측 크랭크 반경 [m] (소기량과 곱으로만 나타나 따로 갈리지 않는다)
    dt/nrev     : 피팅 중에는 성긴 적분으로 비용을 줄인다. 최종은 기본값으로 재구성.
    """
    base_geom = base or PumpGeom()
    stalls = list(stall_points or [])
    scale = float(np.mean([abs(p[2]) for p in flow_points])) if flow_points else 1.0
    scale_pa = float(np.mean([abs(p[1]) for p in stalls])) if stalls else 1.0

    def cost(z):
        # 경계 밖이면 **투영해서 실제 잔차를 계산**하고 이동 거리를 더한다.
        # 예전의 `return pen` 조기 반환은 경계 바로 밖에 cost≈0 인 탈출구를 만들었다.
        x, pen = fit_project(_from_z(z))
        g = fit_to_geom(x, r_fixed, rpm, npis, dt=dt, nrev=nrev)
        return (_flow_cost(g, flow_points, scale)
                + w_stall * _stall_cost(g, stalls, scale_pa) + pen)

    if verbose:
        print(f'  1단계 전역 탐색 {n_samples} 샘플 (로그 균등, dt={dt:.0e}, nrev={nrev})...')
    rng = np.random.RandomState(seed)
    z0 = _to_z(geom_to_fit(base_geom))
    base_cost = cost(z0)
    ranked = [(base_cost, z0)]
    for _ in range(max(0, n_samples - 1)):
        z = np.empty(len(FIT_PARAM_NAMES))
        for i, name in enumerate(FIT_PARAM_NAMES):
            lo, hi = FIT_BOUNDS[name]
            if name in LOG_PARAMS:
                z[i] = math.log(lo) + rng.rand() * (math.log(hi) - math.log(lo))
            else:
                z[i] = lo + rng.rand() * (hi - lo)
        ranked.append((cost(z), z))
    ranked.sort(key=lambda a: a[0])
    if verbose:
        print(f'    탐색 최선 {ranked[0][0]:.5g}  (기존값 {base_cost:.5g})')

    best_z, best_e = z0, base_cost
    for i, (_, guess) in enumerate(ranked[:n_starts]):
        z1, e1, nfev = vm.nelder_mead(cost, guess, max_iter=3000, max_feval=6000)
        # 다중 시작 재시작 — 로그 공간에서도 한 번에 안 내려가는 경우가 있다.
        # 재시작이 더 나빠질 수도 있으므로 **점과 그 점의 비용을 함께** 골라야 한다.
        # (예전엔 e=min(e,e2) 로 비용만 골라 z 는 재시작 결과를 남겨서, 보고한 비용이
        #  반환한 파라미터의 비용이 아닌 경우가 있었다.)
        z2, e2, _ = vm.nelder_mead(cost, z1, max_iter=3000, max_feval=6000)
        z, e = (z2, e2) if e2 <= e1 else (z1, e1)
        if verbose:
            print(f'  2단계 정밀 탐색 {i+1}/{n_starts}: cost={e:.5g} (nfev={nfev})')
        if e < best_e:
            best_z, best_e = z, e

    geom = fit_to_geom(fit_project(_from_z(best_z))[0], r_fixed, rpm, npis)
    return geom, best_e, dict(search_best=float(ranked[0][0]), base_cost=float(base_cost),
                              n_points=len(flow_points))


def sensitivity(geom, flow_points, r_fixed, rpm, npis=2, rel=0.10, dt=4e-4, nrev=6,
                stall_points=None, w_stall=3.0):
    """피팅 좌표별 ±rel 섭동 시 목적함수 상대 변화. 0 에 가까우면 데이터로 안 갈린다."""
    x = geom_to_fit(geom)
    scale = float(np.mean([abs(p[2]) for p in flow_points])) if flow_points else 1.0

    def c(xx):
        g = fit_to_geom(xx, r_fixed, rpm, npis, dt=dt, nrev=nrev)
        e = _flow_cost(g, flow_points, scale)
        if stall_points:
            sp = float(np.mean([abs(q[1]) for q in stall_points]))
            e += w_stall * _stall_cost(g, stall_points, sp)
        return e

    base_e = c(x)
    out = {}
    for i, name in enumerate(FIT_PARAM_NAMES):
        d = []
        for sgn in (+1.0, -1.0):
            xx = x.copy()
            xx[i] = xx[i] * (1.0 + sgn * rel)
            d.append(abs(c(xx) - base_e))
        out[name] = max(d) / base_e if base_e > 0 else float('nan')
    return out


# ══════════════════════════════════════════════════════════════════════════
def selftest_port():
    """포팅 검증 — 해설서 그림 B 의 두 점을 재현해야 한다.
    (pressure_ref_test.cpp:46-83 이 C++ 쪽에서 하는 것과 같은 대조)"""
    g = PumpGeom()
    print('PumpGeom 기본값 (= PistonPump.hpp 하드코딩 = 예전 펌프)')
    print(f'  {g}')
    print(f'  소기량 {g.v_swept*1e6:.1f} mL/피스톤/회전  사구간 {g.v_dead*1e6:.2f} mL'
          f'  압축비 {g.compression_ratio:.1f}')
    print(f'  소기 유량 = {g.v_swept*1e6*g.Npis*g.rpm/60:.0f} mL/s '
          f'= {g.v_swept*g.Npis*g.rpm/60*1000*60:.0f} LPM (체적효율 무시)')

    print('\n능력경계 (해설서 그림 B 대조)')
    ok = True
    for pn_kpa, expect in ((-90.0, 335.0), (-80.0, 745.0)):
        got = cap_ppos(g, pn_kpa * 1e3) / 1e3
        flag = 'OK' if abs(got - expect) <= 5.0 else 'MISMATCH'
        ok = ok and flag == 'OK'
        print(f'  음압 {pn_kpa:+.0f} kPa → 양압 상한 {got:6.1f} kPa  (해설서 {expect:.0f})  {flag}')

    print('\n펌프 공급량 (음압 −60 kPa)')
    for pp_kpa in (0.0, 100.0, 300.0):
        mo, mi = pump_avg(g, pp_kpa * 1e3 + P_ATM, -60e3 + P_ATM)
        print(f'  P⁺={pp_kpa:5.0f} kPa → 토출 {float(mo)*1e3:.4f} g/s, 흡입 {float(mi)*1e3:.4f} g/s')

    print('\n실제 정격 범위')
    pns = np.array([-74.325, -60.0, -40.0, -30.0]) * 1e3
    for pn, fr in zip(pns, frontier(g, pns)):
        print(f'  음압 {pn/1e3:+7.2f} kPa → 양압 상한 {fr/1e3:7.1f} kPa')

    print('\n포팅 검증: ' + ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(selftest_port())
