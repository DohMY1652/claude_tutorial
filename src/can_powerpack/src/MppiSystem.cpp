#include "MppiSystem.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace mppi {

// ============================================================================
// 파라미터 / 상태
// ============================================================================
void SysParams::finalize()
{
  for (auto& c : ch) for (auto& t : c) t.finalize();
  line.finalize();
}

void SysState::resize(int n_ch)
{
  P_ch.assign((size_t)n_ch, 101.325f);
  v.assign((size_t)(n_ch * 3 + 2), ValveState{});
}

// ============================================================================
// 축 동역학 — placeholder. 실측 전에는 의도적으로 아무 것도 하지 않는다.
// 여기에 식을 넣는 순간 롤아웃이 검증되지 않은 파라미터에 의존하게 되므로,
// RUNBOOK.md 에 축 동역학 실측 절차가 추가된 뒤에 활성화할 것.
// ============================================================================
void axis_step_placeholder(const AxisDynamics& ad, float& theta_deg, float& omega_dps,
                           float tau_net_nm, float dt)
{
  if (!ad.enabled) return;    // 기본 경로: 부피는 외생 입력(SysExo::V/Vdot)으로 들어온다
  (void)theta_deg; (void)omega_dps; (void)tau_net_nm; (void)dt;
  // TODO(실측 후): J·ω̇ = τ_net − damping·ω − sign(ω)·coulomb − m·g·L·sin(θ)
  //               θ̇ = ω,  θ → 챔버 부피.
  //               VirtualPowerpack 의 회전 동역학과 **같은 식**이어야 한다.
}

// ============================================================================
// 전체 시스템 한 스텝
//
// 순서는 VirtualPowerpack::integrate 와 동일한 staggered Euler 다:
//   ① 스텝 시작 시점의 레일압으로 채널 밸브 유량을 계산하고 draw/fill 을 누적
//   ② 그 draw/fill 로 레일압을 전진
//   ③ 채널 유량으로 챔버압을 전진
// 같은 유량 값을 레일과 챔버 양쪽에 쓰므로 스텝 안에서 질량이 보존된다.
// ============================================================================
void sys_step(const SysParams& p, SysState& s, const float* u,
              const SysExo& ex, float dt)
{
  const int n = p.n_ch;
  float draw_pos = 0.0f;   // 양압 레일에서 빠져나가는 유량 [LPM]
  float fill_neg = 0.0f;   // 음압 레일로 들어오는 유량 [LPM] (진공을 망친다)

  // ── ① 채널 ───────────────────────────────────────────────────────────────
  for (int g = 0; g < n; ++g) {
    const ChannelPlant& cpv = p.ch[(size_t)g];
    const PlantParams&  cp  = cpv[V_MICRO];     // 채널 공통 필드용
    const bool  pos = (g < p.n_pos);
    const float P   = s.P_ch[(size_t)g];

    const float uu[3] = { u[sys_i_micro(g)],                 // micro
                          ex.u_macro[(size_t)g],             // macro (최적화 대상 아님)
                          u[sys_i_atm(n, g)] };              // atm

    float pin[3], pout[3];
    if (pos) {
      pin[V_MICRO] = s.P_pos;   pout[V_MICRO] = P;            // 레일 → 챔버
      pin[V_MACRO] = p.P_macro; pout[V_MACRO] = P;            // 탱크 → 챔버
      pin[V_ATM]   = P;         pout[V_ATM]   = p.P_atm;      // 챔버 → 대기
    } else {
      pin[V_MICRO] = P;         pout[V_MICRO] = s.P_neg;      // 챔버 → 음압 레일
      pin[V_MACRO] = P;         pout[V_MACRO] = cp.ejector_p_limit;
      pin[V_ATM]   = p.P_atm;   pout[V_ATM]   = P;            // 대기 → 챔버
    }

    float q[3];
    for (int j = 0; j < 3; ++j) {
      const PlantParams& pj = cpv[(size_t)j];   // **밸브별 파라미터**
      ValveState& vs = s.v[(size_t)s.iv_ch(g, j)];
      const float z = step_bw(pj, vs, uu[j]);
      float Qs;
      if (!pos && j == V_MACRO && P <= pj.ejector_p_limit) Qs = 0.0f;
      else Qs = q_static(pj, uu[j], pin[j], pout[j], z);
      q[j] = valve_dyn(pj, vs, Qs, dt);
    }

    const PlantParams& pl = cpv[V_ATM];
    float q_leak = 0.0f;
    if (pl.leakage_u > 0.0f) {
      q_leak = pos ? q_static(pl, pl.leakage_u, P, p.P_atm, s.v[(size_t)s.iv_ch(g, V_ATM)].z)
                   : q_static(pl, pl.leakage_u, p.P_atm, P, s.v[(size_t)s.iv_ch(g, V_ATM)].z);
    }

    const float q_net = pos ? (q[V_MICRO] + q[V_MACRO] - q[V_ATM] - q_leak)
                            : (q[V_ATM] + q_leak - q[V_MICRO] - q[V_MACRO]);

    // **여기가 채널별 독립 MPPI 에 없던 것이다** — 채널이 레일에서 빼가는 양을 누적한다.
    if (pos) draw_pos += q[V_MICRO];      // macro 는 탱크에서 오므로 레일과 무관
    else     fill_neg += q[V_MICRO];

    // 챔버압 전진 (등온 이상기체)
    const float Vc = std::max(1e-12f, ex.V[(size_t)g]);
    s.P_ch[(size_t)g] += dt * (RGAS_AIR * TEMP_K * (q_net * LPM_TO_KGPS) / 1000.0f
                               - P * ex.Vdot[(size_t)g]) / Vc;
    s.P_ch[(size_t)g] = std::clamp(s.P_ch[(size_t)g], pos ? 50.0f : cp.ejector_p_limit,
                                   pos ? 800.0f : 110.0f);
  }

  // ── ② 레일 ───────────────────────────────────────────────────────────────
  // 펌프 한 대로 이어진 닫힌 회로: 흡입구 = 음압 레일, 토출구 = 양압 레일.
  // 흡입량 = 토출량이므로 **같은 항**이 양압에 +, 음압에 − 로 들어간다 (질량보존).
  // 이 결합이 "양압을 올리면 음압이 억눌린다"는 능력경계의 근원이다.
  float Q_pump = 0.0f;
  if (p.pump) {
    Q_pump = (float)(p.pump->flow_out((double)s.P_pos * 1000.0,
                                      (double)s.P_neg * 1000.0) / (double)LPM_TO_KGPS);
  }

  {   // board1 v1 — 양압 레일 → 대기. **회로의 유일한 출구이고 유일한 조절 수단이다.**
      // 레일압을 올리는 수단은 펌프뿐이므로 이 밸브는 "덜 버리는 것" 만 할 수 있다.
    ValveState& vs = s.v[(size_t)s.iv_vent()];
    const float uz = step_bw(p.line, vs, u[sys_i_vent(n)]);
    const float f  = valve_dyn(p.line, vs,
                               q_static(p.line, u[sys_i_vent(n)], s.P_pos, p.P_atm, uz), dt);
    const float lk = p.leak_pos * std::max(0.0f, s.P_pos - p.P_atm);
    s.P_pos += dt * (RGAS_AIR * TEMP_K
                     * ((Q_pump - f - lk - draw_pos) * LPM_TO_KGPS) / 1000.0f) / p.V_pos_m3;
    s.P_pos = std::clamp(s.P_pos, p.pos_min, p.pos_max);
  }
  {   // board2 v1 — 대기 → 음압 레일. 회로의 유일한 입구.
    ValveState& vs = s.v[(size_t)s.iv_admit()];
    const float uz = step_bw(p.line, vs, u[sys_i_admit(n)]);
    const float f  = valve_dyn(p.line, vs,
                               q_static(p.line, u[sys_i_admit(n)], p.P_atm, s.P_neg, uz), dt);
    const float lk = p.leak_neg * std::max(0.0f, p.P_atm - s.P_neg);
    s.P_neg += dt * (RGAS_AIR * TEMP_K
                     * ((f + lk + fill_neg - Q_pump) * LPM_TO_KGPS) / 1000.0f) / p.V_neg_m3;
    s.P_neg = std::clamp(s.P_neg, p.neg_min, p.neg_max);
  }
}

// ============================================================================
// 솔버
// ============================================================================
namespace {
inline float track_cost_sys(float e) { return std::log1p(e * e); }
}  // namespace

SystemSolver::SystemSolver(const SysParams& sp, const SysMppiParams& mp, uint32_t seed)
: sp_(sp), mp_(mp), seed_(seed)
{
  sp_.finalize();
  mp_.NP = std::max(1, mp_.NP);
  mp_.K  = std::max(2, mp_.K);
  mp_.substeps = std::clamp(mp_.substeps, 1, 8);

  // 라인 밸브를 제어하지 않으면 제어 벡터에서 뺀다 (차원 26 → 24).
  nu_    = mp_.control_lines ? sys_nu(sp_.n_ch) : (2 * sp_.n_ch);
  nseq_  = nu_ * mp_.NP;
  n_grp_ = sp_.n_ch + (mp_.control_lines ? 1 : 0);

  nom_.assign((size_t)nseq_, 0.0f);
  dnom_.assign((size_t)nseq_, 0.0f);
  du0_.assign((size_t)nu_, 0.0f);
  cost_.assign((size_t)mp_.K, 0.0f);
  w_.assign((size_t)mp_.K, 0.0f);
  sortbuf_.assign((size_t)mp_.K, 0.0f);
  noise_.assign((size_t)mp_.K * (size_t)nseq_, 0.0f);

  // 표본별 RNG·작업공간. 핫패스에서 할당하지 않는다.
  rng_.reserve((size_t)mp_.K);
  eps_.resize((size_t)mp_.K);
  sc_prev_.resize((size_t)mp_.K); sc_gap_.resize((size_t)mp_.K);
  sc_uapp_.resize((size_t)mp_.K); sc_uprv_.resize((size_t)mp_.K);
  sc_jg_.resize((size_t)mp_.K);
  work_.resize((size_t)mp_.K);
  for (int i = 0; i < mp_.K; ++i) {
    rng_.emplace_back(seed_ + (uint32_t)i);
    eps_[(size_t)i].assign((size_t)nseq_, 0.0f);
    sc_prev_[(size_t)i].assign((size_t)nu_, 0.0f);
    sc_gap_[(size_t)i].assign((size_t)sp_.n_ch, 0.0f);
    sc_uapp_[(size_t)i].assign((size_t)nu_, 0.0f);
    sc_uprv_[(size_t)i].assign((size_t)nu_, 0.0f);
    sc_jg_[(size_t)i].assign((size_t)n_grp_, 0.0f);
    work_[(size_t)i].resize(sp_.n_ch);
  }
  gcost_.assign((size_t)mp_.K, 0.0f);
  gsort_.assign((size_t)mp_.K, 0.0f);
  gw_.assign((size_t)mp_.K, 0.0f);
  gacc_.assign((size_t)(2 * mp_.NP), 0.0f);
  u_full_.assign((size_t)sys_nu(sp_.n_ch), 0.0f);
  u_line_[0] = u_line_[1] = 0.0f;
}

void SystemSolver::reset() { std::fill(nom_.begin(), nom_.end(), 0.0f); }

SysStats SystemSolver::take_stats() { SysStats o = st_; st_ = SysStats{}; return o; }

// ── 표본 하나의 롤아웃 + 비용 ──────────────────────────────────────────────
float SystemSolver::rollout(const SysState& x0, const SysExo& ex,
                            const std::vector<float>& uref, int sample)
{
  const int   n   = sp_.n_ch;
  const int   NP  = mp_.NP;
  const float dt  = mp_.Ts / (float)mp_.substeps;
  const float scl = std::max(1e-3f, mp_.track_scale_kpa);
  const float rscl= std::max(1e-3f, mp_.rail_scale_kpa);

  std::vector<float>& eps = eps_[(size_t)sample];
  float* noi = &noise_[(size_t)sample * (size_t)nseq_];

  // 노이즈 — 표본 0 은 명목 그대로 평가한다 (갱신이 명목보다 나빠질 수 없게).
  // **표본 인덱스로 시드**하므로 어느 스레드가 잡아도 같은 노이즈가 나온다.
  if (sample == 0) {
    std::fill(eps.begin(), eps.end(), 0.0f);
  } else {
    Rng& r = rng_[(size_t)sample];
    r.reseed(((uint64_t)seed_ << 32) ^ (call_ * 0x9E3779B97F4A7C15ull) ^ (uint64_t)sample);
    const int   n_expl = (int)(mp_.explore_frac * (float)mp_.K);
    const float sg = (sample >= mp_.K - n_expl) ? mp_.sigma_explore_pct : mp_.sigma_pct;
    const float b  = std::clamp(mp_.noise_beta, 0.0f, 0.99f);
    const float bs = std::sqrt(std::max(0.0f, 1.0f - b * b));
    std::vector<float>& prev = sc_prev_[(size_t)sample];
    std::fill(prev.begin(), prev.end(), 0.0f);
    for (int k = 0; k < NP; ++k)
      for (int j = 0; j < nu_; ++j) {
        prev[(size_t)j] = b * prev[(size_t)j] + bs * r.normal();
        eps[(size_t)(k * nu_ + j)] = sg * prev[(size_t)j];
      }
  }

  SysState& s = work_[(size_t)sample];
  s = x0;

  // 스테이지 레퍼런스 (Mppi.hpp 3.6(d) 와 같은 이유로 1차 접근 궤적)
  const bool  ramp  = (ex.ref_tau_s > 1e-4f);
  const float decay = ramp ? std::exp(-mp_.Ts / ex.ref_tau_s) : 0.0f;
  std::vector<float>& gap = sc_gap_[(size_t)sample];
  for (int g = 0; g < n; ++g) gap[(size_t)g] = ex.P_ref[(size_t)g] - x0.P_ch[(size_t)g];

  std::vector<float>& u_app = sc_uapp_[(size_t)sample];
  std::vector<float>& u_prev = sc_uprv_[(size_t)sample];
  std::copy(uref.begin(), uref.end(), u_prev.begin());
  std::vector<float>& jg = sc_jg_[(size_t)sample];
  std::fill(jg.begin(), jg.end(), 0.0f);
  const int GR = n - 0;            // 라인 그룹 인덱스 = n (= sp_.n_ch)
  double J = 0.0;

  for (int k = 0; k < NP; ++k) {
    for (int j = 0; j < nu_; ++j) {
      const float lo = std::max(-uref[(size_t)j], -mp_.du_limit_pct);
      const float hi = std::min(100.0f - uref[(size_t)j], mp_.du_limit_pct);
      const float e  = eps[(size_t)(k * nu_ + j)];
      const float du = std::clamp(nom_[(size_t)(k * nu_ + j)] + e, lo, hi);
      noi[k * nu_ + j] = e;                        // 원 노이즈 저장 (MPPI 표준형 갱신)
      const float u_raw = std::clamp(uref[(size_t)j] + du, 0.0f, 100.0f);
      u_app[(size_t)j] = u_raw;
      const float d  = (u_raw - uref[(size_t)j]) * 0.01f;
      const float dd = (u_raw - u_prev[(size_t)j]) * 0.01f;
      const float ce = mp_.w_effort * d * d + mp_.w_du * dd * dd;
      J += (double)(ce / (float)nu_);
      // 노력·변화율 비용은 그 밸브가 속한 그룹에 귀속시킨다.
      jg[(size_t)((j < n) ? j : ((j < 2 * n) ? (j - n) : GR))] += ce;
      u_prev[(size_t)j] = u_raw;
    }

    // 라인 밸브를 제어하지 않으면 LinePID 의 현재 출력을 지평 동안 상수로 둔다.
    // (레일은 여전히 예측된다 — 그것이 중앙집중의 핵심이다.)
    if (!mp_.control_lines) {
      u_line_[0] = ex.u_vent;
      u_line_[1] = ex.u_admit;
    }

    // sys_step 은 길이 sys_nu 의 배열을 기대한다. 라인 미제어 모드에서는 채널 24개
    // 뒤에 외생 라인 개도 2개를 붙여 넘긴다.
    const float* uptr;
    if (mp_.control_lines) {
      uptr = u_app.data();
    } else {
      std::copy(u_app.begin(), u_app.begin() + 2 * n, u_full_.begin());
      u_full_[(size_t)sys_i_vent(n)]  = u_line_[0];
      u_full_[(size_t)sys_i_admit(n)] = u_line_[1];
      uptr = u_full_.data();
    }
    for (int sub = 0; sub < mp_.substeps; ++sub) sys_step(sp_, s, uptr, ex, dt);
    if (sample == 0 && k == 0) { pred1_pos_ = s.P_pos; pred1_neg_ = s.P_neg; }

    // 비용 — 채널 평균과 레일 평균을 각각 정규화해 w_track / w_rail 이 상대 우선순위가
    // 되게 한다. 챔버 12개 합을 그냥 쓰면 레일 항이 12배 묻힌다.
    double ec = 0.0;
    for (int g = 0; g < n; ++g) {
      if (ramp) gap[(size_t)g] *= decay;
      const float ref_k = ramp ? (ex.P_ref[(size_t)g] - gap[(size_t)g]) : ex.P_ref[(size_t)g];
      const float t = track_cost_sys((s.P_ch[(size_t)g] - ref_k) / scl);
      ec += (double)t;
      jg[(size_t)g] += mp_.w_track * t;        // 채널 g 의 추종은 채널 g 에 귀속
    }
    J += (double)mp_.w_track * ec / (double)n;
    // 레일 비용은 **제어할 때만** 의미가 있다. 제어하지 않는 것에 벌점을 걸면
    // 채널이 레일을 지키려 챔버 추종을 희생하는 본말전도가 된다.
    float er = 0.0f;
    if (mp_.control_lines) {
      er = 0.5f * (track_cost_sys((s.P_pos - ex.rail_pos_sp) / rscl)
                 + track_cost_sys((s.P_neg - ex.rail_neg_sp) / rscl));
      J += (double)(mp_.w_rail * er);
      jg[(size_t)GR] += mp_.w_rail * er;
    // 레일 비용의 일부를 각 채널에 **공유 가격**으로 배분한다 — 채널이 레일을 끌어내리는
    // 것에 대한 대가다. 0 이면 채널은 레일을 전혀 신경쓰지 않는다.
      if (mp_.rail_share > 0.0f)
        for (int g = 0; g < n; ++g) jg[(size_t)g] += mp_.rail_share * mp_.w_rail * er;
    }
  }

  J /= (double)NP;
  if (mp_.terminal_mult > 0.0f) {
    double et = 0.0;
    for (int g = 0; g < n; ++g) {
      const float ref_T = ramp ? (ex.P_ref[(size_t)g] - gap[(size_t)g]) : ex.P_ref[(size_t)g];
      et += (double)track_cost_sys((s.P_ch[(size_t)g] - ref_T) / scl);
    }
    J += (double)(mp_.w_track * mp_.terminal_mult) * et / (double)n;
  }
  if (!std::isfinite(J)) J = 1e18;
  return (float)J;
}

const std::vector<float>& SystemSolver::solve(const SysState& x0, const SysExo& ex,
                                              const std::vector<float>& uref,
                                              const ParallelFor& pfor)
{
  const auto t0 = std::chrono::steady_clock::now();
  const int K = mp_.K;
  ++call_;

  pfor(K, [&](int i) { cost_[(size_t)i] = rollout(x0, ex, uref, i); });

  // ── 그룹별(factored) 가중 평균 ─────────────────────────────────────────
  // 롤아웃은 전체 시스템으로 굴렸으므로 레일 결합이 이미 예측에 들어 있다.
  // 그런데 **가중 평균을 26개 제어에 한꺼번에** 하면 공로 배분이 뭉개진다
  // (계측: 그렇게 했을 때 IAE 6 → 30). 그래서 그룹마다 자기 비용으로 자기 제어만
  // 갱신한다. 채널 g 는 {micro g, atm g}, 라인 그룹은 {vent, admit} 을 갖는다.
  const int n = sp_.n_ch;
  ++st_.calls;
  double eff_acc = 0.0; int eff_n = 0; bool any_flat = false;
  float jmin_all = cost_[0];
  for (int i = 1; i < K; ++i) jmin_all = std::min(jmin_all, cost_[(size_t)i]);
  st_.sum_cost += (double)jmin_all;

  for (int grp = 0; grp < n_grp_; ++grp) {
    for (int i = 0; i < K; ++i) gcost_[(size_t)i] = sc_jg_[(size_t)i][(size_t)grp];
    float gmin = gcost_[0];
    for (int i = 1; i < K; ++i) gmin = std::min(gmin, gcost_[(size_t)i]);
    gsort_.assign(gcost_.begin(), gcost_.begin() + K);
    std::nth_element(gsort_.begin(), gsort_.begin() + K / 2, gsort_.begin() + K);
    const float gmed = gsort_[(size_t)(K / 2)];
    const float spread = gmed - gmin;

    // 이 그룹이 갖는 제어 인덱스
    int idx[2];
    if (grp < n) { idx[0] = sys_i_micro(grp); idx[1] = sys_i_atm(n, grp); }
    else         { idx[0] = sys_i_vent(n);    idx[1] = sys_i_admit(n);    }

    if (spread <= mp_.flat_spread) {
      // 정보가 없는 그룹은 명목을 0(순수 피드포워드)으로 감쇠 — Mppi.hpp 3.6(b)
      for (int k = 0; k < mp_.NP; ++k)
        for (int t = 0; t < 2; ++t)
          nom_[(size_t)(k * nu_ + idx[t])] *= mp_.flat_decay;
      any_flat = true;
      continue;
    }
    const float denom = std::max(1e-9f, mp_.lambda * std::max(1e-6f, spread));
    double eta = 0.0;
    for (int i = 0; i < K; ++i) {
      const float w = std::exp(-(gcost_[(size_t)i] - gmin) / denom);
      gw_[(size_t)i] = w; eta += (double)w;
    }
    const float inv = (eta > 0.0) ? (float)(1.0 / eta) : 0.0f;
    double sw2 = 0.0;
    // **표본을 바깥 루프로 돌린다.** 제어 인덱스를 바깥으로 두면 stride 260 float 로
    // 260 KB 배열을 인덱스마다 다시 훑어 틱당 ~69 MB 트래픽이 된다 (계측: 최대 20 ms).
    // 이렇게 하면 표본당 그 그룹의 20개만 읽으므로 총 K×20×그룹수 회로 끝난다.
    std::fill(gacc_.begin(), gacc_.begin() + 2 * mp_.NP, 0.0f);
    for (int i = 0; i < K; ++i) {
      const float wn = gw_[(size_t)i] * inv;
      if (wn <= 0.0f) continue;
      sw2 += (double)wn * (double)wn;
      const float* base = &noise_[(size_t)i * (size_t)nseq_];
      for (int k = 0; k < mp_.NP; ++k)
        for (int t = 0; t < 2; ++t)
          gacc_[(size_t)(k * 2 + t)] += wn * base[k * nu_ + idx[t]];
    }
    for (int k = 0; k < mp_.NP; ++k)
      for (int t = 0; t < 2; ++t) {
        const int m = k * nu_ + idx[t];
        nom_[(size_t)m] = std::clamp(nom_[(size_t)m] + gacc_[(size_t)(k * 2 + t)],
                                     -mp_.du_limit_pct, mp_.du_limit_pct);
      }
    if (sw2 > 0.0) { eff_acc += 1.0 / sw2; ++eff_n; }
  }
  if (any_flat) ++st_.flat;
  if (eff_n) st_.sum_eff += eff_acc / (double)eff_n;
  st_.sum_spread += 0.0;

  for (int j = 0; j < nu_; ++j) {
    const float lo = std::max(-uref[(size_t)j], -mp_.du_limit_pct);
    const float hi = std::min(100.0f - uref[(size_t)j], mp_.du_limit_pct);
    du0_[(size_t)j] = std::clamp(nom_[(size_t)j], lo, hi);
    if (du0_[(size_t)j] <= lo + 1e-4f || du0_[(size_t)j] >= hi - 1e-4f) ++st_.sat_first;
  }

  pred1_valid_ = true;   // 1스텝 예측은 rollout(sample 0, k 0) 에서 기록됐다

  // 워밍 스타트: 한 스텝 밀고 마지막 복제
  for (int k = 0; k + 1 < mp_.NP; ++k)
    for (int j = 0; j < nu_; ++j)
      nom_[(size_t)(k * nu_ + j)] = nom_[(size_t)((k + 1) * nu_ + j)];
  if (mp_.NP >= 2)
    for (int j = 0; j < nu_; ++j)
      nom_[(size_t)((mp_.NP - 1) * nu_ + j)] = nom_[(size_t)((mp_.NP - 2) * nu_ + j)];

  const float us = (float)std::chrono::duration<double, std::micro>(
      std::chrono::steady_clock::now() - t0).count();
  st_.sum_us += (double)us;
  st_.max_us = std::max(st_.max_us, us);
  return du0_;
}

}  // namespace mppi
