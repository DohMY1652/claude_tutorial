#!/usr/bin/env python3
"""
valve_fit_model.py — 13-variable 비례밸브 모델 + 피팅 솔버 (ROS 무관 라이브러리)

레퍼런스: current_hysteresis_fitting_v02_26_04_01.m
  파라미터 순서·초기값·목적함수(SSE + 발산 페널티)·sub-step 수(20)·abs() 래핑·wn 상한(150)
  까지 그 MATLAB 스크립트와 동일하게 맞췄다. 그래야 같은 CSV 로 두 구현을 교차검증할 수 있다.

소비 측(C++)과의 대응:
  Controller.cpp  AcadosMpc::update_linearization / compute_input_reference
  VirtualPowerpack.cpp  step_valve
  두 곳 모두 같은 식을 쓰고, ChannelConfig 기본값이 아래 BASE_INITIAL 과 일치한다
  (= 지금 코드에 박힌 값이 이 MATLAB 스크립트의 과거 피팅 산물이라는 뜻).

**레퍼런스와 다른 점**: MATLAB 은 유량 Q 를 측정 입력으로 받았지만 이 리그에는 유량계가 없다.
Q 는 챔버압 미분으로 합성한다(derive_q_from_dpdt). 그 결과 A_max 와 챔버 부피 V 가 분리되지
않으므로(지배식에 A_max/V 곱만 나타난다) V 를 이중 부피법으로 따로 고정해야 한다.
"""

import math

import numpy as np

# ── 파라미터 정의 ──────────────────────────────────────────────────────────
# MATLAB base_initial 및 C++ ChannelConfig 기본값과 동일한 순서/값 (단일 출처)
PARAM_NAMES = [
    'A_max', 'k_shape', 'C_k', 'C_p', 'C_z',
    'A_bw', 'beta_bw', 'gamma_bw', 'alpha_shape',
    'wn_up', 'zeta_up', 'wn_down', 'zeta_down',
]
BASE_INITIAL = np.array([
    0.2845, 33.09, 0.0288, 0.00012, 0.0,
    260649.5, 179.0, 0.06, 3884.2,
    40.0, 1.2, 45.0, 1.0,
])

# ── 물리 상수 ──────────────────────────────────────────────────────────────
I_MAX = 0.30           # 지령 100% 에 대응하는 코일 전류 [A]
RGAS = 287.0           # [J/(kg·K)]
TEMP_K = 293.15        # [K]
KAPPA = 1.4
P_ATM_KPA = 101.325

# 이 밸브 모델이 쓰는 **경험적** LPM 환산 계수. 표준 LPM 이 아니다 (약 10.7배 차이).
# 13-variable 모델을 fit 할 때 함께 나온 단위라 모델 내부에서는 이 값으로 일관해야 한다.
LPM_TO_KGPS = 0.0002155
# 진짜 표준 LPM → kg/s (= rho0/60/1000). 카탈로그값·오리피스 물리 환산에만 쓴다.
STD_LPM_TO_KGPS = 1.204 / 60.0 / 1000.0

# 물리적으로 타당한 파라미터 범위. 레퍼런스 MATLAB 은 abs() 만 걸고 경계가 없었는데,
# 이 모델은 (A_max, k_shape, C_k, alpha_shape) 가 거의 자유롭게 상쇄되는 **평평한 다양체**를
# 갖는다. 경계가 없으면 최적화가 그 다양체의 극단(k_shape 350, alpha 6만 등)으로 달아나고,
# 데이터 적합도(R²)는 유지되는데 크래킹 임계 같은 물리량이 크게 틀어진다.
PARAM_BOUNDS = {
    'A_max': (0.01, 2.0),
    'k_shape': (5.0, 120.0),
    'C_k': (0.0, 0.30),
    'C_p': (0.0, 2.0e-3),      # 상류압이 스풀을 밀어 올려 돕는다 → 부호는 +
    'C_z': (-0.05, 0.05),
    'A_bw': (0.0, 2.0e6),
    'beta_bw': (0.0, 2000.0),
    'gamma_bw': (0.0, 2.0),
    'alpha_shape': (50.0, 20000.0),
    'wn_up': (5.0, 150.0), 'zeta_up': (0.2, 5.0),
    'wn_down': (5.0, 150.0), 'zeta_down': (0.2, 5.0),
}
BOUND_PENALTY = 1.0e6

N_SUB_STEPS = 20       # MATLAB 과 동일한 Euler 마이크로스텝 수
MOVMEAN_WINDOW = 20    # MATLAB movmean(·, 20)
WN_CAP = 150.0         # MATLAB min(abs(wn), 150)
DIVERGE_PENALTY = 1e7


# ══════════════════════════════════════════════════════════════════════════
# 압축성 유동 Φ
# ══════════════════════════════════════════════════════════════════════════
def phi(p_in_abs, p_out_abs, kappa=KAPPA):
    """무차원 유량함수. 절대압 [kPa] 입력. 역류(Pr>1)와 Pin<=0 은 0."""
    p_in = np.asarray(p_in_abs, dtype=float)
    p_out = np.asarray(p_out_abs, dtype=float)
    out = np.zeros(np.broadcast(p_in, p_out).shape, dtype=float)

    valid = (p_in > 1e-9) & (p_out < p_in)
    if not np.any(valid):
        return out

    pr = np.ones_like(out)
    np.divide(p_out, p_in, out=pr, where=valid)
    pr = np.clip(pr, 0.0, 1.0)

    p_cr = (2.0 / (kappa + 1.0)) ** (kappa / (kappa - 1.0))
    choked = valid & (pr <= p_cr)
    sub = valid & (pr > p_cr)

    out[choked] = math.sqrt(kappa * (2.0 / (kappa + 1.0)) ** ((kappa + 1.0) / (kappa - 1.0)))
    if np.any(sub):
        t = np.maximum(0.0, pr[sub] ** (2.0 / kappa) - pr[sub] ** ((kappa + 1.0) / kappa))
        out[sub] = math.sqrt(2.0 * kappa / (kappa - 1.0)) * np.sqrt(t)
    return out


EXP_LIMIT = 700.0   # float64 exp 한계(≈709). 난수 탐색이 k_shape·F 를 극단으로 밀면 넘친다.


def sigmoid_pow(k_shape, force_net, alpha):
    """sigmoid(k·F)^alpha 를 오버플로 없이. MATLAB exp 는 Inf 로 흘러가지만 Python 은 예외를 던진다."""
    kf = k_shape * force_net
    if kf < -EXP_LIMIT:
        return 0.0
    if kf > EXP_LIMIT:
        return 1.0
    return (1.0 / (1.0 + math.exp(-kf))) ** alpha


def unpack(params):
    """MATLAB 과 동일한 abs() 래핑 / wn 상한을 적용해 물리적으로 유효한 값으로 만든다."""
    p = np.asarray(params, dtype=float)
    return dict(
        A_max=abs(p[0]), k_shape=abs(p[1]), C_k=abs(p[2]),
        C_p=p[3], C_z=p[4],                                  # 부호 있는 항
        A_bw=abs(p[5]), beta_bw=abs(p[6]), gamma_bw=abs(p[7]),
        alpha_shape=abs(p[8]),
        wn_up=min(abs(p[9]), WN_CAP), zeta_up=abs(p[10]),
        wn_down=min(abs(p[11]), WN_CAP), zeta_down=abs(p[12]),
    )


def effective_area(current_a, p_in_abs, params, z=0.0):
    """유효면적 A_eff (모델 상대단위). 히스테리시스 상태 z 는 외부에서 준다."""
    q = unpack(params)
    f = np.clip(current_a + q['C_z'] * z + q['C_p'] * p_in_abs - q['C_k'], -500.0, 500.0)
    kf = np.clip(q['k_shape'] * f, -EXP_LIMIT, EXP_LIMIT)
    sigma = 1.0 / (1.0 + np.exp(-kf))
    return q['A_max'] * sigma ** q['alpha_shape']


def cracking_current_pct(p_in_abs, params, area_frac=1e-6):
    """크래킹 임계 [%] — A_eff 가 A_max 의 area_frac 배가 되는 지령.
    effective_area 를 면적→전류 방향으로 역산한 것. C_p·Pin 때문에 상류압 의존이다."""
    q = unpack(params)
    frac = min(max(area_frac, 1e-12), 1.0 - 1e-12)
    sigma = frac ** (1.0 / q['alpha_shape'])
    f = math.log(sigma / (1.0 - sigma)) / q['k_shape']
    i_req = f - q['C_z'] * 0.0 - q['C_p'] * p_in_abs + q['C_k']
    return float(np.clip(i_req / I_MAX * 100.0, 0.0, 100.0))


# ══════════════════════════════════════════════════════════════════════════
# 순방향 시뮬레이션 (MATLAB simulate_physics_model 과 동일)
# ══════════════════════════════════════════════════════════════════════════
def fold_euler(wn, zeta, dt, n_sub=N_SUB_STEPS):
    """q_static 이 한 샘플 동안 상수이므로 20-step Euler 는 **선형사상**이다.
    그걸 미리 접어 2×2 행렬 하나로 만든다 — 결과는 루프를 그대로 도는 것과 완전히 동일하고
    (부동소수 반복 순서까지 같은 선형 재귀), 목적함수 1회 비용이 20배 줄어든다.

    원본 갱신 순서:
        dx2 = wn²(qs − x1) − 2ζwn·x2
        x1 += h·x2          (옛 x2 사용)
        x2 += h·dx2         (옛 x1, x2 사용)
    → [x1;x2] ← A[x1;x2] + b·qs,  A = [[1, h], [−h·wn², 1−2ζwn·h]],  b = [0, h·wn²]
    """
    h = dt / n_sub
    a = np.array([[1.0, h], [-h * wn * wn, 1.0 - 2.0 * zeta * wn * h]])
    b = np.array([0.0, h * wn * wn])
    m = np.eye(2)
    acc = np.zeros((2, 2))
    for _ in range(n_sub):
        acc += m            # Σ A^i , i = 0..n_sub-1
        m = a @ m
    return m, acc @ b


def current_direction(current_a, di=None):
    """전류 증감 방향 래치 (1=열림, 0=닫힘). 전류만의 함수이므로 파라미터와 무관 —
    피팅 반복마다 다시 계산하지 않도록 미리 뽑아 둔다."""
    cur = np.asarray(current_a, dtype=float)
    if di is None:
        di = np.diff(cur, prepend=cur[0] if cur.size else 0.0)
    out = np.ones(cur.size, dtype=np.int8)
    d = 1
    for k in range(cur.size):
        if di[k] > 1e-4:
            d = 1
        elif di[k] < -1e-4:
            d = 0
        out[k] = d
    return di, out


def simulate_q(current_a, p_in_abs, p_out_abs, dt, params, q0=0.0, phi_pre=None,
               di_pre=None, dir_pre=None, pinphi_pre=None):
    """전류·상하류압 시계열 → 유량 시계열 [LPM].

    current_a : [N] 실측 코일 전류 [A]  (지령이 아니라 실측을 쓰는 것이 레퍼런스보다 정확하다 —
                드라이버 전류루프 동특성이 피팅에서 빠진다)
    dt        : 샘플 간격 [s]
    *_pre     : prepare_segment() 가 미리 계산해 둔 값 (파라미터와 무관한 부분)
    """
    q = unpack(params)
    cur = np.asarray(current_a, dtype=float)
    pin = np.asarray(p_in_abs, dtype=float)
    n = cur.size
    if n == 0:
        return np.zeros(0)

    if pinphi_pre is None:
        ph = phi(pin, p_out_abs) if phi_pre is None else np.asarray(phi_pre, dtype=float)
        pinphi = pin * ph
    else:
        pinphi = pinphi_pre

    if di_pre is None or dir_pre is None:
        di, direction = current_direction(cur)
    else:
        di, direction = di_pre, dir_pre

    # 루프 안에서 dict 조회를 없앤다
    a_max, k_shape, c_k = q['A_max'], q['k_shape'], q['C_k']
    c_p, c_z, alpha = q['C_p'], q['C_z'], q['alpha_shape']
    a_bw, beta_bw, gamma_bw = q['A_bw'], q['beta_bw'], q['gamma_bw']
    m_up, c_up = fold_euler(q['wn_up'], q['zeta_up'], dt)
    m_dn, c_dn = fold_euler(q['wn_down'], q['zeta_down'], dt)

    q_pred = np.empty(n)
    z = 0.0
    x1 = float(q0)
    x2 = 0.0

    for k in range(n):
        d = di[k]
        if d != 0.0:
            z += a_bw * d - beta_bw * abs(d) * z - gamma_bw * d * abs(z)
            if z > 1e6:
                z = 1e6
            elif z < -1e6:
                z = -1e6

        f = cur[k] + c_z * z + c_p * pin[k] - c_k
        if f > 500.0:
            f = 500.0
        elif f < -500.0:
            f = -500.0
        qs = a_max * sigmoid_pow(k_shape, f, alpha) * pinphi[k]

        if direction[k]:
            x1, x2 = (m_up[0, 0] * x1 + m_up[0, 1] * x2 + c_up[0] * qs,
                      m_up[1, 0] * x1 + m_up[1, 1] * x2 + c_up[1] * qs)
        else:
            x1, x2 = (m_dn[0, 0] * x1 + m_dn[0, 1] * x2 + c_dn[0] * qs,
                      m_dn[1, 0] * x1 + m_dn[1, 1] * x2 + c_dn[1] * qs)
        q_pred[k] = x1

    return q_pred


def sse(params, seg):
    """단일 세그먼트 SSE + 발산 페널티. seg 는 prepare_segment() 결과.

    seg['mask'] 가 있으면 그 샘플만 잔차에 넣는다. 모델 상태(z, 2차 동특성)는 전 구간을
    연속 적분해야 하지만(히스테리시스가 레벨을 넘어 이어지므로), 챔버압 미분으로 얻은 Q 는
    **대상 밸브만 열려 있는 구간에서만 유효**하기 때문이다. 반대 밸브로 챔버를 초기화하는
    구간에서는 전류가 0 이라 모델도 자연히 0 으로 감쇠하므로 적분을 끊을 필요가 없다.
    """
    q_pred = _run(params, seg)
    m = seg.get('mask')
    resid = (seg['Q'] - q_pred) if m is None else (seg['Q'][m] - q_pred[m])
    err = float(np.sum(resid ** 2))
    ref = seg.get('q_ref', 0.0)
    # MATLAB 과 동일 취지: 초기 발산이면 큰 페널티
    if q_pred.size > 1 and ref > 0 and q_pred[1] > ref * 2.0:
        err += DIVERGE_PENALTY
    return err


def bound_penalty(params):
    """Nelder-Mead 는 무제약이라 경계를 벌점으로 넣는다 (범위 대비 상대 초과의 제곱)."""
    q = unpack(params)
    pen = 0.0
    for name, (lo, hi) in PARAM_BOUNDS.items():
        v = q[name]
        span = hi - lo
        if v < lo:
            pen += ((lo - v) / span) ** 2
        elif v > hi:
            pen += ((v - hi) / span) ** 2
    return BOUND_PENALTY * pen


def global_sse(params, segments):
    return sum(sse(params, s) for s in segments) + bound_penalty(params)


def _run(params, seg):
    return simulate_q(seg['I'], seg['P_in'], seg['P_out'], seg['dt'], params,
                      q0=seg.get('q0', 0.0), phi_pre=seg.get('Phi'),
                      di_pre=seg.get('dI'), dir_pre=seg.get('dir'),
                      pinphi_pre=seg.get('PinPhi'))


def predict(params, seg):
    return _run(params, seg)


def r_squared(params, segments):
    ss_res = 0.0
    ss_tot = 0.0
    for s in segments:
        pred = predict(params, s)
        m = s.get('mask')
        obs = s['Q'] if m is None else s['Q'][m]
        prd = pred if m is None else pred[m]
        ss_res += float(np.sum((obs - prd) ** 2))
        ss_tot += float(np.sum((obs - np.mean(obs)) ** 2))
    return 1.0 - ss_res / ss_tot if ss_tot > 0 else float('nan')


# ══════════════════════════════════════════════════════════════════════════
# 유량 합성 — 유량계가 없으므로 챔버압 미분에서 얻는다
# ══════════════════════════════════════════════════════════════════════════
def movmean(x, window=MOVMEAN_WINDOW):
    """MATLAB movmean 과 같은 중심 이동평균 (경계는 있는 샘플만 평균)."""
    x = np.asarray(x, dtype=float)
    if window <= 1 or x.size == 0:
        return x.copy()
    csum = np.concatenate(([0.0], np.cumsum(x)))
    n = x.size
    half_lo = (window - 1) // 2
    half_hi = window // 2
    idx = np.arange(n)
    lo = np.maximum(0, idx - half_lo)
    hi = np.minimum(n, idx + half_hi + 1)
    return (csum[hi] - csum[lo]) / (hi - lo)


def savgol_deriv_coeffs(window, dt, poly=2):
    """Savitzky-Golay 1차 미분 필터 계수. 창 안에서 다항식을 최소제곱 맞춘 뒤 중심의 기울기.

    이동평균을 반복하는 것보다 **첨두를 훨씬 덜 깎는다** — 여기서 잡아야 하는 밸브 과도가
    시상수 수십 ms 라 이게 결정적이다.
    """
    w = int(window) | 1                     # 홀수로
    half = w // 2
    x = np.arange(-half, half + 1) * dt
    a = np.vander(x, poly + 1, increasing=True)
    pinv = np.linalg.pinv(a)
    return pinv[1], half                    # 1차 계수 = 중심의 미분


def dpdt(t, p_kpa, window=9, poly=2):
    """Savitzky-Golay 미분으로 dP/dt [kPa/s].

    이전 구현은 movmean(20) 을 압력과 미분에 각각 걸었다(=100 ms 를 두 번). 밸브 2차
    동특성의 지배 시상수가 40~50 ms 라 그 필터가 신호보다 넓어, **진짜 파라미터로도**
    R² 0.67 밖에 안 나오는 상한을 만들고 있었다.
    """
    t = np.asarray(t, dtype=float)
    p = np.asarray(p_kpa, dtype=float)
    n = p.size
    if n < 5:
        return np.zeros(n)
    dt = float(np.median(np.diff(t)))
    coef, half = savgol_deriv_coeffs(window, dt, poly)
    if n <= 2 * half:
        return np.gradient(p, t)
    # 경계는 반사 확장 (첨두 왜곡 없이 길이 유지)
    ext = np.concatenate((p[half:0:-1], p, p[-2:-half - 2:-1]))
    return np.convolve(ext, coef[::-1], mode='valid')[:n]


def q_from_dpdt(dp_dt_kpa_s, volume_m3, n_poly=1.0):
    """dP/dt [kPa/s] → 유량 [LPM] (모델의 경험 LPM 단위).

    dP/dt = Q·(n·R·T·LPM_TO_KGPS)/(V·1000)  를 뒤집은 것.
    n_poly 기본 1.0(등온) — Controller 의 피드포워드와 VirtualPowerpack 챔버가 등온이므로
    이 모델의 소비 측과 일치한다. (PressureRefGen 은 n_ch=1.4 를 쓰니 리포트에 불일치를 남긴다.)
    """
    return np.asarray(dp_dt_kpa_s, dtype=float) * volume_m3 * 1000.0 \
        / (n_poly * RGAS * TEMP_K) / LPM_TO_KGPS


def prepare_segment(t, current_a, p_in_abs, p_out_abs, q_lpm):
    """솔버가 먹는 형태로 정리. Φ 를 미리 계산해 반복 비용을 줄인다."""
    t = np.asarray(t, dtype=float)
    dt = float(np.mean(np.diff(t))) if t.size > 1 else 0.005
    seg = dict(
        t=t,
        I=movmean(np.asarray(current_a, dtype=float)),
        P_in=np.asarray(p_in_abs, dtype=float),
        P_out=np.asarray(p_out_abs, dtype=float),
        Q=np.asarray(q_lpm, dtype=float),   # dpdt 가 이미 SG 필터를 거쳤다 — 재평활하면 첨두가 또 깎인다
        dt=dt,
    )
    seg['Phi'] = phi(seg['P_in'], seg['P_out'])
    seg['PinPhi'] = seg['P_in'] * seg['Phi']
    seg['dI'], seg['dir'] = current_direction(seg['I'])
    return seg


# ══════════════════════════════════════════════════════════════════════════
# 솔버 — scipy 가 없으므로 fminsearch(Nelder-Mead) 동등 구현
# ══════════════════════════════════════════════════════════════════════════
def nelder_mead(func, x0, max_iter=15000, max_feval=50000,
                tol_f=1e-6, tol_x=1e-6):
    """MATLAB fminsearch 와 같은 알고리즘·같은 초기 심플렉스 구성 규칙."""
    x0 = np.asarray(x0, dtype=float)
    n = x0.size
    rho, chi, psi, sigma = 1.0, 2.0, 0.5, 0.5

    # fminsearch 의 초기 심플렉스: 각 좌표를 5% 늘리고, 0 인 좌표는 0.00025 로
    sim = np.tile(x0, (n + 1, 1))
    for i in range(n):
        if sim[i + 1, i] != 0.0:
            sim[i + 1, i] *= 1.05
        else:
            sim[i + 1, i] = 0.00025

    fsim = np.array([func(s) for s in sim], dtype=float)
    nfev = n + 1
    order = np.argsort(fsim)
    sim, fsim = sim[order], fsim[order]

    for _ in range(max_iter):
        if nfev >= max_feval:
            break
        if (np.max(np.abs(sim[1:] - sim[0])) <= tol_x
                and np.max(np.abs(fsim[1:] - fsim[0])) <= tol_f):
            break

        centroid = np.mean(sim[:-1], axis=0)
        xr = centroid + rho * (centroid - sim[-1])
        fr = func(xr); nfev += 1

        if fr < fsim[0]:
            xe = centroid + rho * chi * (centroid - sim[-1])
            fe = func(xe); nfev += 1
            sim[-1], fsim[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < fsim[-2]:
            sim[-1], fsim[-1] = xr, fr
        else:
            if fr < fsim[-1]:
                xc = centroid + psi * rho * (centroid - sim[-1])
                fc = func(xc); nfev += 1
                if fc <= fr:
                    sim[-1], fsim[-1] = xc, fc
                else:
                    sim, fsim, nfev = _shrink(func, sim, fsim, sigma, nfev)
            else:
                xcc = centroid - psi * (centroid - sim[-1])
                fcc = func(xcc); nfev += 1
                if fcc < fsim[-1]:
                    sim[-1], fsim[-1] = xcc, fcc
                else:
                    sim, fsim, nfev = _shrink(func, sim, fsim, sigma, nfev)

        order = np.argsort(fsim)
        sim, fsim = sim[order], fsim[order]

    return sim[0], float(fsim[0]), nfev


def _shrink(func, sim, fsim, sigma, nfev):
    for i in range(1, sim.shape[0]):
        sim[i] = sim[0] + sigma * (sim[i] - sim[0])
        fsim[i] = func(sim[i])
        nfev += 1
    return sim, fsim, nfev


def random_search(func, base=None, n_samples=200, seed=0):
    """MATLAB 1단계와 동일: base 에 [0.5,1.5] 배 노이즈, C_z/wn 은 별도 범위로 랜덤화.
    seed 를 고정해 재현 가능하게 한다 (MATLAB 원본은 재현 불가였다).

    `base` 는 단일 벡터 또는 벡터 리스트다. 리스트면 표본을 각 base 에 고르게 나눠 뿌린다 —
    제너릭 BASE_INITIAL 하나만으로는 채널마다 진짜 최적점이 [0.5,1.5]배 범위 밖에 있을 때
    Nelder-Mead 가 애초에 나쁜 분지에서 시작해 못 빠져나온다. 이미 잘 피팅된 다른 채널/밸브의
    파라미터를 추가 base 로 주면(웜스타트) 그 분지 근처에서도 같이 탐색한다.
    """
    if base is None:
        bases = [BASE_INITIAL]
    elif isinstance(base, (list, tuple)):
        bases = [np.asarray(b, dtype=float) for b in base]
    else:
        bases = [np.asarray(base, dtype=float)]

    rng = np.random.RandomState(seed)
    results = []
    per_base = max(1, n_samples // len(bases))
    for b in bases:
        for s in range(per_base):
            if s == 0:
                guess = b.copy()
            else:
                guess = b * (0.5 + rng.rand(b.size))
                guess[4] = (rng.rand() - 0.5) * 0.1        # C_z 부호 랜덤화
                guess[9] = 10.0 + rng.rand() * 50.0        # wn_up
                guess[11] = 10.0 + rng.rand() * 50.0       # wn_down
                # 경계 안으로 투영 — 밖에서 시작하면 벌점 지형에서 헤맨다
                for j, name in enumerate(PARAM_NAMES):
                    lo, hi = PARAM_BOUNDS[name]
                    guess[j] = min(max(guess[j], lo), hi)
            results.append((func(guess), guess))
    results.sort(key=lambda r: r[0])
    return results


def fit(segments, base=None, n_samples=200, n_starts=3, seed=0, verbose=True):
    """2단계 피팅. 반환: (params, sse, r2, 진단 dict)"""
    cost = lambda p: global_sse(p, segments)

    if verbose:
        print(f"  1단계 전역 난수 탐색 {n_samples} 샘플...")
    ranked = random_search(cost, base, n_samples, seed)

    best_p, best_e = None, float('inf')
    for i, (_, guess) in enumerate(ranked[:n_starts]):
        p, e, nfev = nelder_mead(cost, guess)
        if verbose:
            print(f"  2단계 정밀 탐색 {i+1}/{n_starts}: SSE={e:.4g} (nfev={nfev})")
        if e < best_e:
            best_p, best_e = p, e

    # 저장은 물리적으로 유효한 형태(abs 래핑 적용 후)로
    q = unpack(best_p)
    params = np.array([min(max(q[k], PARAM_BOUNDS[k][0]), PARAM_BOUNDS[k][1])
                       for k in PARAM_NAMES])
    return params, best_e, r_squared(params, segments), dict(
        n_segments=len(segments),
        n_samples=int(sum(s['Q'].size for s in segments)),
        search_best=float(ranked[0][0]),
    )


def sensitivity(params, segments, rel=0.10):
    """파라미터별 ±rel 섭동 시 SSE 변화율. 값이 0 에 가까우면 데이터로 식별되지 않는다는 뜻."""
    base_e = global_sse(params, segments)
    out = {}
    for i, name in enumerate(PARAM_NAMES):
        deltas = []
        for sign in (+1.0, -1.0):
            p = np.array(params, dtype=float)
            p[i] = p[i] * (1.0 + sign * rel) if p[i] != 0.0 else sign * rel
            deltas.append(abs(global_sse(p, segments) - base_e))
        out[name] = max(deltas) / base_e if base_e > 0 else float('nan')
    return out
