#include "Mppi.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace mppi {

// ============================================================================
// 압축성 유동 Φ — VirtualPowerpack::get_phi 와 동일
// ============================================================================
namespace {
const float PHI_PCR = std::pow(2.0f / (KAPPA + 1.0f), KAPPA / (KAPPA - 1.0f));
const float PHI_CHOKED =
    std::sqrt(KAPPA * std::pow(2.0f / (KAPPA + 1.0f), (KAPPA + 1.0f) / (KAPPA - 1.0f)));
const float PHI_C = std::sqrt(2.0f * KAPPA / (KAPPA - 1.0f));
constexpr float EXP_A = 2.0f / KAPPA;
constexpr float EXP_B = (KAPPA + 1.0f) / KAPPA;
}  // namespace

// ── 추적 비용: e² 대신 log(1+e²) ────────────────────────────────────────────
// 이 플랜트의 이득이 극단적이다 — 챔버 50 mL 에 밸브를 완전히 열면 4 ms 에 99 kPa 가
// 움직인다. 그래서 지평 10 스텝 롤아웃 중 일부는 1000 kPa 를 스윙하고, e² 로 재면
// 비용이 정상 샘플의 1000 배를 넘는다 (계측: (Jmax−Jmin)/(mean−Jmin) = 2285).
// 그 꼬리 하나가 어떤 산포 척도(max·mean)든 부풀려 가중치를 균일하게 만들고, 선택이
// 사라지면 명목 시퀀스가 무작위 보행으로 박스 끝까지 표류한다 (첫스텝 포화 99.8%).
//
// log(1+e²) 는 **단조**라 순서를 보존하면서 크기만 압축한다 (e=100 → 9.2, e=1 → 0.69).
// 발산 롤아웃은 여전히 최악으로 정렬되지만 다른 샘플들의 상대 비교를 지배하지 않는다.
static inline float track_cost(float e)
{
  return std::log1p(e * e);
}

float phi(float Pin, float Pout)
{
  if (Pin < 1e-9f || Pout >= Pin) return 0.0f;
  const float Pr = std::clamp(Pout / Pin, 0.0f, 1.0f);
  if (Pr <= PHI_PCR) return PHI_CHOKED;
  // pow 를 두 번 부르는 대신 log 한 번 + exp 두 번. pow ≈ log+exp 이므로 log 하나를 아낀다.
  const float lp = std::log(Pr);
  const float d  = std::exp(EXP_A * lp) - std::exp(EXP_B * lp);
  return PHI_C * std::sqrt(std::max(0.0f, d));
}

// ============================================================================
// PlantParams 사전계산
// ============================================================================
void PlantParams::finalize()
{
  // F_crack: A_eff = frac·A_max ⇔ sigmoid(k·F)^alpha = frac
  //   ⇒ sigma = frac^(1/alpha),  F = logit(sigma)/k     — Pin·z 무관 상수
  const double frac  = std::clamp((double)crack_area_frac, 1e-30, 1.0 - 1e-12);
  const double alpha = std::max(1e-9, (double)alpha_shape);
  const double sigma = std::clamp(std::pow(frac, 1.0 / alpha), 1e-12, 1.0 - 1e-12);
  F_crack = (float)(std::log(sigma / (1.0 - sigma)) / std::max(1e-9, (double)k_shape));

  // F_open: alpha·log1p(exp(−k·F)) > 90 이면 A_eff 가 float 0 으로 언더플로한다.
  //   ⇒ exp(−k·F) > expm1(90/alpha)  ⇒  k·F < −log(expm1(90/alpha))
  const double x_min = -std::log(std::expm1(90.0 / alpha));
  F_open = (float)(x_min / std::max(1e-9, (double)k_shape));
}

float PlantParams::u_crack(float Pin, float z) const
{
  const float I_req = F_crack - C_z * z - C_p * Pin + C_k;
  return std::clamp(I_req / std::max(1e-9f, I_MAX) * 100.0f, 0.0f, 100.0f);
}

// ============================================================================
// 유효면적 — A_eff = A_max · sigmoid(k·F_net)^alpha
//   pow(sigma, alpha) 를 직접 부르면 alpha≈3884 에서 언더플로 전에 큰 비용을 낸다.
//   log 항등식으로 바꾸면 exp 한 번 + log1p 한 번이고, 조기 탈출까지 붙는다.
// ============================================================================
float area_eff(const PlantParams& p, float u_pct, float Pin, float z)
{
  u_pct = std::clamp(u_pct, 0.0f, 100.0f);
  const float I     = u_pct / 100.0f * p.I_MAX;
  const float F_net = std::clamp(I + p.C_z * z + p.C_p * Pin - p.C_k, -500.0f, 500.0f);
  if (F_net <= p.F_open) return 0.0f;              // 스풀이 들리지 않는다
  const float x = p.k_shape * F_net;
  //  −log(sigmoid(x)) = log1p(exp(−x)).  x > 30 이면 exp(−x) < 1e-13 이라 0 에 수렴한다.
  const float L = (x > 30.0f) ? std::exp(-x) : std::log1p(std::exp(-x));
  const float e = p.alpha_shape * L;
  if (e > 90.0f) return 0.0f;
  return p.A_max * std::exp(-e);
}

float q_static(const PlantParams& p, float u_pct, float Pin, float Pout, float z)
{
  const float ph = phi(Pin, Pout);
  if (ph <= 0.0f) return 0.0f;
  const float a = area_eff(p, u_pct, Pin, z);
  if (a <= 0.0f) return 0.0f;
  return a * Pin * ph;
}

// ============================================================================
// Bouc-Wen — VirtualPowerpack::step_valve 의 앞부분과 동일
// ============================================================================
float step_bw(const PlantParams& p, ValveState& vs, float u_pct)
{
  const float I      = std::clamp(u_pct, 0.0f, 100.0f) / 100.0f * p.I_MAX;
  const float dI     = I - vs.prevI;
  const float abs_dI = std::abs(dI);
  vs.z = std::clamp(vs.z + p.A_bw * dI - p.beta_bw * abs_dI * vs.z
                        - p.gamma_bw * dI * std::abs(vs.z),
                    -1e6f, 1e6f);
  if      (dI >  1e-4f) vs.dir = 1;
  else if (dI < -1e-4f) vs.dir = 0;
  vs.prevI = I;
  return vs.z;
}

// ============================================================================
// 2차 동특성 — 오일러 순서까지 VirtualPowerpack 과 동일하게 맞췄다
//   (x1 을 **이전** x2 로 먼저 갱신한 뒤 x2 를 갱신한다)
// ============================================================================
float valve_dyn(const PlantParams& p, ValveState& vs, float Q_static, float dt)
{
  const float wn   = (vs.dir == 1) ? p.wn_up   : p.wn_down;
  const float zeta = (vs.dir == 1) ? p.zeta_up : p.zeta_down;
  const float dx2  = wn * wn * (Q_static - vs.q) - 2.0f * zeta * wn * vs.qd;
  vs.q  += dt * vs.qd;
  vs.qd += dt * dx2;
  return std::max(0.0f, vs.q);
}

// ============================================================================
// 한 스텝 전진
// ============================================================================
void step(const PlantParams& p, ChannelState& s, const std::array<float, 3>& u,
          const Exogenous& ex, float V, float dt)
{
  // 밸브별 상·하류압. update_linearization 의 배정과 동일하다.
  float pin[3], pout[3];
  if (p.is_positive) {
    pin[V_MICRO] = ex.P_micro; pout[V_MICRO] = s.P;          // 레일 → 챔버
    pin[V_MACRO] = ex.P_macro; pout[V_MACRO] = s.P;          // 탱크 → 챔버
    pin[V_ATM]   = s.P;        pout[V_ATM]   = ex.P_atm;     // 챔버 → 대기
  } else {
    pin[V_MICRO] = s.P;        pout[V_MICRO] = ex.P_micro;   // 챔버 → 음압레일
    pin[V_MACRO] = s.P;        pout[V_MACRO] = p.ejector_p_limit;  // 챔버 → 이젝터
    pin[V_ATM]   = ex.P_atm;   pout[V_ATM]   = s.P;          // 대기 → 챔버
  }

  float q[3];
  for (int j = 0; j < 3; ++j) {
    const float z = step_bw(p, s.v[j], u[j]);
    float Qs;
    if (!p.is_positive && j == V_MACRO && s.P <= p.ejector_p_limit) {
      Qs = 0.0f;                       // 이젝터 도달 하한 아래에서는 흡입이 없다
    } else {
      Qs = q_static(p, u[j], pin[j], pout[j], z);
    }
    q[j] = valve_dyn(p, s.v[j], Qs, dt);
  }

  // 누설은 상태를 갖지 않는다 — atm 밸브의 z 를 쓴다 (calc_rounds 와 동일한 취급).
  float q_leak = 0.0f;
  if (p.leakage_u > 0.0f) {
    q_leak = p.is_positive ? q_static(p, p.leakage_u, s.P, ex.P_atm, s.v[V_ATM].z)
                           : q_static(p, p.leakage_u, ex.P_atm, s.P, s.v[V_ATM].z);
  }

  const float q_net = p.is_positive
      ? (q[V_MICRO] + q[V_MACRO] - q[V_ATM] - q_leak)
      : (q[V_ATM] + q_leak - q[V_MICRO] - q[V_MACRO]);

  // 등온 이상기체: dP/dt = (R·T·ṁ/1000 − P·V̇)/V   [kPa/s]
  const float Vc   = std::max(1e-12f, V);
  const float mdot = q_net * LPM_TO_KGPS;
  s.P += dt * (RGAS_AIR * TEMP_K * mdot / 1000.0f - s.P * ex.Vdot) / Vc;
  // 절대 진공 아래는 비물리다. 발산한 롤아웃이 NaN 으로 번지지 않게 여기서 막는다.
  s.P = std::clamp(s.P, 1.0f, 5000.0f);
}

// ============================================================================
// 밸브 상태 추정 전진 — step() 과 같은 상·하류압 배정을 쓰되 z 는 건드리지 않는다
// ============================================================================
void advance_valve_estimate(const PlantParams& p, ChannelState& s,
                            const std::array<float, 3>& u_applied,
                            const Exogenous& ex, float dt,
                            bool integrate_chamber, float V)
{
  float pin[3], pout[3];
  if (p.is_positive) {
    pin[V_MICRO] = ex.P_micro; pout[V_MICRO] = s.P;
    pin[V_MACRO] = ex.P_macro; pout[V_MACRO] = s.P;
    pin[V_ATM]   = s.P;        pout[V_ATM]   = ex.P_atm;
  } else {
    pin[V_MICRO] = s.P;        pout[V_MICRO] = ex.P_micro;
    pin[V_MACRO] = s.P;        pout[V_MACRO] = p.ejector_p_limit;
    pin[V_ATM]   = ex.P_atm;   pout[V_ATM]   = s.P;
  }
  float q[3];
  for (int j = 0; j < 3; ++j) {
    float Qs;
    if (!p.is_positive && j == V_MACRO && s.P <= p.ejector_p_limit)
      Qs = 0.0f;
    else
      Qs = q_static(p, u_applied[(size_t)j], pin[j], pout[j], s.v[(size_t)j].z);
    q[j] = valve_dyn(p, s.v[(size_t)j], Qs, dt);
  }
  if (!integrate_chamber) return;

  // 챔버압도 함께 전진 — step() 과 **같은 식**이다 (누설 포함, 등온 이상기체).
  float q_leak = 0.0f;
  if (p.leakage_u > 0.0f) {
    q_leak = p.is_positive ? q_static(p, p.leakage_u, s.P, ex.P_atm, s.v[V_ATM].z)
                           : q_static(p, p.leakage_u, ex.P_atm, s.P, s.v[V_ATM].z);
  }
  const float q_net = p.is_positive
      ? (q[V_MICRO] + q[V_MACRO] - q[V_ATM] - q_leak)
      : (q[V_ATM] + q_leak - q[V_MICRO] - q[V_MACRO]);
  const float Vc = std::max(1e-12f, (V > 0.0f) ? V : ex.V0);
  s.P += dt * (RGAS_AIR * TEMP_K * (q_net * LPM_TO_KGPS) / 1000.0f - s.P * ex.Vdot) / Vc;
  s.P = std::clamp(s.P, 1.0f, 5000.0f);
}

// ============================================================================
// RNG — xoshiro128++ + Box-Muller
// ============================================================================
Rng::Rng(uint32_t seed)
{
  // splitmix32 로 상태를 벌린다. seed 0 이어도 전 상태가 0 이 되지 않게.
  uint32_t x = seed + 0x9E3779B9u;
  for (int i = 0; i < 4; ++i) {
    uint32_t z = (x += 0x9E3779B9u);
    z = (z ^ (z >> 16)) * 0x21F0AAADu;
    z = (z ^ (z >> 15)) * 0x735A2D97u;
    s[i] = z ^ (z >> 15);
  }
  if ((s[0] | s[1] | s[2] | s[3]) == 0u) s[0] = 1u;
}

void Rng::reseed(uint64_t seed)
{
  // splitmix64 로 64비트 시드를 32비트 상태 4개로 벌린다. 표본 인덱스마다 독립적인
  // 스트림을 얻으므로 어느 스레드가 어느 표본을 잡아도 같은 노이즈가 나온다.
  uint64_t z = seed + 0x9E3779B97F4A7C15ull;
  for (int i = 0; i < 4; ++i) {
    z += 0x9E3779B97F4A7C15ull;
    uint64_t t = z;
    t = (t ^ (t >> 30)) * 0xBF58476D1CE4E5B9ull;
    t = (t ^ (t >> 27)) * 0x94D049BB133111EBull;
    s[i] = (uint32_t)((t ^ (t >> 31)) & 0xFFFFFFFFull);
  }
  if ((s[0] | s[1] | s[2] | s[3]) == 0u) s[0] = 1u;
  has_cache = false;
}

uint32_t Rng::next_u32()
{
  const uint32_t r = ((s[0] + s[3]) << 7 | (s[0] + s[3]) >> 25) + s[0];
  const uint32_t t = s[1] << 9;
  s[2] ^= s[0]; s[3] ^= s[1]; s[1] ^= s[2]; s[0] ^= s[3];
  s[2] ^= t;
  s[3] = (s[3] << 11) | (s[3] >> 21);
  return r;
}

float Rng::uniform01()
{
  // (0,1) 개구간 — log(0) 을 피한다
  return (float)((next_u32() >> 8) + 1) * (1.0f / 16777218.0f);
}

float Rng::normal()
{
  if (has_cache) { has_cache = false; return cache; }
  const float u1 = uniform01(), u2 = uniform01();
  const float r  = std::sqrt(-2.0f * std::log(u1));
  const float th = 6.28318530718f * u2;
  cache = r * std::sin(th);
  has_cache = true;
  return r * std::cos(th);
}

// ============================================================================
// Solver
// ============================================================================
Solver::Solver(const PlantParams& pp, const Params& pr, uint32_t seed)
: pp_(pp), pr_(pr), rng_(seed)
{
  pp_.finalize();
  pr_.NP = std::max(1, pr_.NP);
  pr_.K  = std::max(2, pr_.K);
  pr_.substeps = std::clamp(pr_.substeps, 1, 8);
  nseq_ = pr_.NP * 3;

  nom_.assign((size_t)nseq_, 0.0f);
  noise_.assign((size_t)pr_.K * (size_t)nseq_, 0.0f);
  dnom_.assign((size_t)nseq_, 0.0f);
  sortbuf_.assign((size_t)pr_.K, 0.0f);
  cost_.assign((size_t)pr_.K, 0.0f);
  w_.assign((size_t)pr_.K, 0.0f);
  eps_.assign((size_t)nseq_, 0.0f);
}

void Solver::reset()
{
  std::fill(nom_.begin(), nom_.end(), 0.0f);
}

Stats Solver::take_stats()
{
  Stats out = st_;
  st_ = Stats{};
  return out;
}

// ── 샘플 하나의 롤아웃 + 비용 ──────────────────────────────────────────────
float Solver::rollout_cost(const ChannelState& x0, const Exogenous& ex,
                           const std::array<float, 3>& uref, int sample)
{
  const int   NP  = pr_.NP;
  const float dt  = pr_.Ts / (float)pr_.substeps;
  const float scl = std::max(1e-3f, pr_.track_scale_kpa);

  // 노이즈 — 샘플 0 은 명목 그대로 평가한다. 그래야 갱신이 명목보다 나빠질 수 없다.
  if (sample == 0) {
    std::fill(eps_.begin(), eps_.end(), 0.0f);
  } else {
    const float b  = std::clamp(pr_.noise_beta, 0.0f, 0.99f);
    const float bs = std::sqrt(std::max(0.0f, 1.0f - b * b));
    // 뒤쪽 explore_frac 만큼은 큰 sigma — 크래킹 임계를 건너뛰기 위한 표본이다.
    const int n_expl = (int)(pr_.explore_frac * (float)pr_.K);
    const float sg = (sample >= pr_.K - n_expl) ? pr_.sigma_explore_pct : pr_.sigma_pct;
    float prev[3] = {0.f, 0.f, 0.f};
    for (int k = 0; k < NP; ++k) {
      for (int j = 0; j < 3; ++j) {
        prev[j] = b * prev[j] + bs * rng_.normal();
        eps_[(size_t)(k * 3 + j)] = sg * prev[j];
      }
    }
  }

  ChannelState s = x0;
  Exogenous exk = ex;          // 스텝마다 레일압을 갱신해 넘긴다
  // 스테이지 레퍼런스 계수 — 지평 안에서 지수적으로 접근한다.
  const bool  ramp = (ex.tau_ref > 1e-4f);
  const float decay = ramp ? std::exp(-pr_.Ts / ex.tau_ref) : 0.0f;
  float gap = ex.P_ref - ex.P0;         // 남은 오차 (지수 감쇠)
  float* noi = &noise_[(size_t)sample * (size_t)nseq_];
  float u_prev[3] = {uref[0], uref[1], uref[2]};
  double J = 0.0;

  for (int k = 0; k < NP; ++k) {
    std::array<float, 3> u_app{};
    for (int j = 0; j < 3; ++j) {
      // uref 대비 보정 한계와 절대 한계를 QP 의 LL/UL 과 동일하게 겹친다:
      //   du ∈ [max(−uref, du_min), min(100−uref, du_max)]
      const float lo = std::max(-uref[j], pr_.du_min);
      const float hi = std::min(100.0f - uref[j], pr_.du_max);
      const float e_kj = eps_[(size_t)(k * 3 + j)];
      const float du = std::clamp(nom_[(size_t)(k * 3 + j)] + e_kj, lo, hi);
      // **원 노이즈**를 저장한다 (인가값이 아니라). 이유는 solve() 의 갱신 주석 참조.
      noi[k * 3 + j] = e_kj;

      const float u_raw = std::clamp(uref[j] + du, 0.0f, 100.0f);

      // 명령 테이퍼 — 플랜트가 실제로 받는 값이다. 목표 근처에서 크래킹 임계 위쪽
      // 여유분만 줄이므로, 이 불연속을 롤아웃에 넣어야 MPPI 가 그 경계를 안다.
      float u_out = u_raw;
      if (pr_.taper_in_rollout) {
        float pin_j;
        const float p_rail = ex.P_micro + ex.dP_micro_dt * (float)k * pr_.Ts;
        if (pp_.is_positive)
          pin_j = (j == V_MICRO) ? p_rail : (j == V_MACRO) ? ex.P_macro : s.P;
        else
          pin_j = (j == V_ATM) ? ex.P_atm : s.P;
        const float uc = pp_.u_crack(pin_j, s.v[j].z);
        // 테이퍼는 실제 컨트롤러와 같이 **최종 목표** 기준으로 판정한다
        // (스테이지 레퍼런스가 아니다 — 실기에서는 cfg_.ref_value 를 쓴다).
        const float tp = std::clamp(std::abs(ex.P_ref - s.P)
                                        / std::max(1e-3f, pp_.cmd_taper_kpa),
                                    0.0f, 1.0f);
        u_out = (u_raw <= uc) ? 0.0f : uc + (u_raw - uc) * tp;
      }
      u_app[(size_t)j] = u_out;

      const float d  = (u_raw - uref[j]) * 0.01f;
      const float dd = (u_raw - u_prev[j]) * 0.01f;
      J += (double)(pr_.w_effort * d * d + pr_.w_du * dd * dd);
      u_prev[j] = u_raw;
    }

    const float V = std::max(1e-12f, ex.V0 + ex.Vdot * (float)k * pr_.Ts);
    // 레일 궤적 — 명목 기반 1차 예측. 상수 가정이 만드는 ≈5~34 kPa 오차를 없앤다.
    exk.P_micro = ex.P_micro + ex.dP_micro_dt * (float)k * pr_.Ts;
    for (int sub = 0; sub < pr_.substeps; ++sub) step(pp_, s, u_app, exk, V, dt);

    if (ramp) gap *= decay;
    const float ref_k = ramp ? (ex.P_ref - gap) : ex.P_ref;
    const float e = (s.P - ref_k) / scl;
    J += (double)(pr_.w_track * track_cost(e));
  }

  J /= (double)NP;
  const float ref_T = ramp ? (ex.P_ref - gap) : ex.P_ref;
  const float ef = (s.P - ref_T) / scl;
  J += (double)(pr_.w_track * pr_.terminal_mult * track_cost(ef));

  // 발산한 롤아웃은 채택되지 않게 큰 비용을 준다 (P 는 step() 이 이미 클램프한다).
  if (!std::isfinite(J)) J = 1e18;
  return (float)J;
}

std::array<float, 3> Solver::solve(const ChannelState& x0, const Exogenous& ex,
                                   const std::array<float, 3>& uref)
{
  const auto t0 = std::chrono::steady_clock::now();
  const int K = pr_.K;

  float jmin = 0.0f, jmax = 0.0f;
  double jsum = 0.0;
  for (int i = 0; i < K; ++i) {
    const float c = rollout_cost(x0, ex, uref, i);
    cost_[(size_t)i] = c;
    jsum += (double)c;
    if (i == 0) { jmin = jmax = c; }
    else { jmin = std::min(jmin, c); jmax = std::max(jmax, c); }
  }

  // λ 를 **비용 산포에 대한 비율**로 쓴다. 절대 λ 는 오차 크기·부피·레일압이 바뀌면
  // 비용 스케일이 함께 바뀌어 매번 재튜닝해야 하고, 잘못되면 가중치가 한 샘플로
  // 붕괴하거나(탐색 소멸) 균일해진다(평균화).
  //
  // 산포 척도로 (Jmax − Jmin) 을 쓰면 **이상치 하나에 무너진다.** 발산 직전까지 간
  // 롤아웃 하나가 Jmax 를 키우면 분모가 커져 정상 샘플 전부가 w≈1 이 되고, 그러면
  // 갱신이 "클램프된 샘플의 산술평균" 이 된다. uref=0 이면 Δu 박스가 [0, du_max] 라
  // 대칭 노이즈가 한쪽으로만 잘려 **평균이 양수**가 되고, 피드포워드가 "닫아라" 라고
  // 한 밸브를 매 틱 3%p 쯤 열어 버린다 (계측: 유효샘플 76%, 오버슈트 7°, 정착 실패).
  //
  // 그래서 **평균 초과비용**(mean − min)을 쓴다. 항상 양수이고, 이상치는 평균에
  // 1/K 만큼만 기여하므로 산포 추정이 무너지지 않는다. w = exp(−초과/(λ·평균초과))
  // 이므로 λ 는 "평균만큼 나쁜 샘플을 e^(−1/λ) 로 깎는다" 로 읽힌다.
  // 산포는 **중앙값 기준**으로 잡는다. 평균은 꼬리 하나에 함께 끌려 올라가므로
  // (계측: mean 기준으로도 유효샘플이 113/128 로 균일해졌다) 중앙값이 필요하다.
  // 중앙값은 K/2 개가 바뀌어야 움직이므로 발산 샘플 몇 개에 영향받지 않는다.
  sortbuf_.assign(cost_.begin(), cost_.begin() + K);
  std::nth_element(sortbuf_.begin(), sortbuf_.begin() + K / 2, sortbuf_.begin() + K);
  const float jmed = sortbuf_[(size_t)(K / 2)];
  const float raw_spread = jmed - jmin;
  const float spread = std::max(1e-6f, raw_spread);
  const float denom = std::max(1e-9f, pr_.lambda * spread);
  (void)jsum;

  // 비용이 평평하면(전 샘플이 같은 궤적 — 크래킹 임계 아래라 유량이 전부 0 인 경우가
  // 대표적이다) 갱신할 정보가 없다. 그때 Σŵε 는 순수 잡음이고, 500 Hz 로 누적되면
  // 명목이 박스 끝까지 표류해 거기서 잠긴다 (계측: 첫스텝 포화 99.9%).
  // 정보가 없는 틱에는 명목을 0(순수 피드포워드)으로 감쇠시킨다.
  if (raw_spread <= pr_.flat_spread) {
    for (int m = 0; m < nseq_; ++m) nom_[(size_t)m] *= pr_.flat_decay;
    const std::array<float, 3> duf{
        std::clamp(nom_[0], std::max(-uref[0], pr_.du_min), std::min(100.0f - uref[0], pr_.du_max)),
        std::clamp(nom_[1], std::max(-uref[1], pr_.du_min), std::min(100.0f - uref[1], pr_.du_max)),
        std::clamp(nom_[2], std::max(-uref[2], pr_.du_min), std::min(100.0f - uref[2], pr_.du_max))};
    ++st_.calls; ++st_.flat;
    const auto tf = std::chrono::steady_clock::now();
    const float fus = (float)std::chrono::duration<double, std::micro>(tf - t0).count();
    st_.sum_us += (double)fus;
    st_.max_us = std::max(st_.max_us, fus);   // 평평 틱도 롤아웃 K 개를 전부 돌린다
    st_.sum_cost += (double)jmin;
    st_.sum_spread += (double)raw_spread;
    return duf;
  }
  double eta = 0.0;
  for (int i = 0; i < K; ++i) {
    const float w = std::exp(-(cost_[(size_t)i] - jmin) / denom);
    w_[(size_t)i] = w;
    eta += (double)w;
  }
  const float inv_eta = (eta > 0.0) ? (float)(1.0 / eta) : 0.0f;

  // 갱신 = 명목 + **원 노이즈**의 가중 평균 (MPPI 표준형).
  //
  // 인가값(클램프된 Δu)의 가중 평균을 쓰면 **한쪽만 막힌 박스에서 편향**이 생긴다.
  // uref=0 이면 Δu 박스가 [0, du_max] 라 대칭 노이즈의 음수 절반이 전부 0 으로 잘리고,
  // 인가값 평균은 구조적으로 > 0 이 된다 — 즉 "닫아라" 라는 피드포워드를 MPPI 가 매 틱
  // 밀어 올린다. 계측: 그 형태로 오버슈트 5.4°, 최종오차 6.5°, 정착 실패였다.
  // 원 노이즈로 평균하면 "0 이 최선" 인 상황에서 명목이 음수로 내려가고 인가 시점에
  // 0 으로 잘려 Δu=0 이 정확히 표현된다. 클램프 정보는 비용 쪽에 이미 반영돼 있다
  // (롤아웃은 클램프된 값으로 굴렸다).
  double sw2 = 0.0;
  std::fill(dnom_.begin(), dnom_.end(), 0.0f);
  for (int i = 0; i < K; ++i) {
    const float wn = w_[(size_t)i] * inv_eta;
    if (wn <= 0.0f) continue;
    sw2 += (double)wn * (double)wn;
    const float* noi = &noise_[(size_t)i * (size_t)nseq_];
    for (int m = 0; m < nseq_; ++m) dnom_[(size_t)m] += wn * noi[m];
  }
  // 명목은 절대 박스로만 제한한다 (uref 는 틱마다 바뀌므로 여기서 섞지 않는다).
  // 제한이 없으면 워밍 스타트가 여러 틱에 걸쳐 발산할 수 있다.
  for (int m = 0; m < nseq_; ++m)
    nom_[(size_t)m] = std::clamp(nom_[(size_t)m] + dnom_[(size_t)m], pr_.du_min, pr_.du_max);

  const std::array<float, 3> du0{
      std::clamp(nom_[0], std::max(-uref[0], pr_.du_min), std::min(100.0f - uref[0], pr_.du_max)),
      std::clamp(nom_[1], std::max(-uref[1], pr_.du_min), std::min(100.0f - uref[1], pr_.du_max)),
      std::clamp(nom_[2], std::max(-uref[2], pr_.du_min), std::min(100.0f - uref[2], pr_.du_max))};

  // 워밍 스타트: 한 스텝 밀고 마지막을 복제한다. 다음 틱의 명목이 이번 해의 꼬리가
  // 되어 K 를 줄여도 성능이 유지된다 (MPPI 가 실시간에서 성립하는 주된 이유).
  for (int k = 0; k + 1 < pr_.NP; ++k)
    for (int j = 0; j < 3; ++j)
      nom_[(size_t)(k * 3 + j)] = nom_[(size_t)((k + 1) * 3 + j)];
  if (pr_.NP >= 2)
    for (int j = 0; j < 3; ++j)
      nom_[(size_t)((pr_.NP - 1) * 3 + j)] = nom_[(size_t)((pr_.NP - 2) * 3 + j)];

  const auto t1 = std::chrono::steady_clock::now();
  const float us =
      (float)std::chrono::duration<double, std::micro>(t1 - t0).count();
  ++st_.calls;
  st_.sum_us += (double)us;
  st_.max_us = std::max(st_.max_us, us);
  st_.sum_eff += (sw2 > 0.0) ? (1.0 / sw2) : 0.0;
  st_.sum_cost += (double)jmin;
  // 이상치 비율 — 크면 발산하는 롤아웃이 있다는 뜻이다 (substeps 를 올릴 신호).
  st_.sum_spread += (double)spread;
  st_.sum_outlier += (spread > 0.0f) ? (double)((jmax - jmin) / spread) : 0.0;
  for (int j = 0; j < 3; ++j) {
    const float lo = std::max(-uref[(size_t)j], pr_.du_min);
    const float hi = std::min(100.0f - uref[(size_t)j], pr_.du_max);
    if (du0[(size_t)j] <= lo + 1e-4f || du0[(size_t)j] >= hi - 1e-4f) { ++st_.sat_first; break; }
  }
  return du0;
}

}  // namespace mppi
