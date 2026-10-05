#include "PressureCtrl.hpp"

#include <algorithm>
#include <cmath>

// ============================================================================
// 채널 압력 PID — 기본형 + anti-windup
//
//   e_eff = ±(P_ref − P_meas)      음압 채널은 부호를 뒤집어 "micro 를 열 방향" 으로 통일
//   u     = P(e_eff) + Ki·∫e_eff dt + Kd·de_eff/dt + Kv·dP_ref/dt   [%]
//   P(e)  = kp·e                                    (|e| ≤ kp_break_kpa, 정착 구간)
//         = sign(e)·(kp·brk + kp_far·(|e|−brk))     (그 밖, 유량 제한 구간)
//   적분 이득도 같은 경계로 갈린다: 안은 ki, 밖은 ki_far (접근 중 누적 억제)
//
// anti-windup 은 다섯으로 막는다: 실효 상한 기준 포화 판정 · 적분 데드밴드 ·
// 구간별 적분 이득(ki_far) · 누적 클램프 · 레퍼런스 스텝에서 리셋.
//
// 게인은 **방향별**이다 (g_up / g_down). 양압 채널은 채우는 밸브가 세우는 밸브의
// 5.4 배라 같은 이득을 쓰면 올리는 방향만 과도하게 세다 — Config 주석 참조.
//
//   u > 0 → micro = u,   atm = 0
//   u < 0 → atm = |u|,   micro = 0
//   macro = 0 (항상)
//
// 반대 밸브를 동시에 열지 않는 것이 요점이다. 같이 열면 레일과 대기가 통해 공압이
// 통째로 낭비되고, 유량이 상쇄돼 제어권도 사라진다.
// ============================================================================

PressureCtrl::PressureCtrl(const Config& cfg) : cfg_(cfg) {
  reset();
}

void PressureCtrl::reset() {
  last_   = Output{};
  integ_  = 0.0f;
  u_last_ = p_last_ = d_last_ = ff_last_ = 0.0f;
  gs_last_ = 1.0f; isat_last_ = 0;
  err_    = 0.0f;
  e_prev_ = 0.0f;
  pm_prev_ = 0.0f;
  de_filt_ = 0.0f;
  wrong_side_ = false;
  ref_prev_ = 0.0f;
  dref_filt_ = 0.0f;
  first_  = true;
}

PressureCtrl::Output PressureCtrl::compute(const Input& in) {

  // dt 는 스케줄러 스파이크를 그대로 믿지 않는다. 미분·적분이 한 틱에 폭주한다.
  const float dt = std::clamp(in.dt_sec, 1.0e-4f, 0.1f);

  // ── 오차 ──────────────────────────────────────────────────────────────────
  // 부호 규약: 양압 채널은 micro 를 열면 압력이 오르고 atm 을 열면 내려간다.
  // 음압 채널은 micro 가 진공 레일이라 열면 압력이 **내려가고**, atm 을 열면 대기가
  // 들어와 올라간다. 그래서 음압 채널의 오차를 뒤집어 두면 아래 로직과 게인 부호가
  // 양압과 완전히 같아진다.
  //   e > 0  →  micro 를 열어야 한다 (양압은 충전, 음압은 배기)
  //   e < 0  →  atm 을 열어야 한다   (양압은 방출, 음압은 대기 유입)
  const float sgn = cfg_.is_positive ? +1.0f : -1.0f;
  err_ = in.P_ref_kpa - in.P_meas_kpa;
  const float e = sgn * err_;

  // ── 방향별 게인 선택 ──────────────────────────────────────────────────────
  // err_ = P_ref − P_meas 의 부호가 곧 **물리적 방향**이다 (채널 종류와 무관):
  //   err_ > 0 → 압력을 올려야 한다 → g_up
  //   err_ < 0 → 압력을 내려야 한다 → g_down
  // 양압 채널은 올리는 쪽이 micro, 음압 채널은 올리는 쪽이 atm 이다.
  const Gains& g = (err_ >= 0.0f) ? cfg_.g_up : cfg_.g_down;

  // ── 챔버가 대기압의 **반대편**에 있는가 ───────────────────────────────────
  // atm 밸브는 언제나 챔버를 **대기압 쪽으로** 민다 — 위에 있으면 빼고, 아래면
  // 넣는다. 그래서 양압 챔버가 대기압 아래로 내려가면(밸브가 닫힌 채 팔이 챔버를
  // 팽창시키면 그렇게 된다) atm 밸브의 작용 방향이 **뒤집힌다.** 그 구간에서
  // 압력을 올리는 올바른 밸브는 micro(레일)가 아니라 atm 이다:
  //   · 대기를 들이므로 차압이 작아 부드럽고, 대기압에서 **스스로 멈춘다**
  //   · micro 는 200 kPa 레일을 96 kPa 챔버에 꽂는 것이라 거칠다
  // 음압 채널이 대기압 **위로** 올라간 경우도 대칭으로 같다.
  const float atm_m = std::max(0.0f, cfg_.atm_margin_kpa);
  const bool wrong_side = cfg_.is_positive
                            ? (in.P_meas_kpa < cfg_.atm_kpa - atm_m)
                            : (in.P_meas_kpa > cfg_.atm_kpa + atm_m);
  // 그쪽으로 넘어가는 순간 적분을 버린다. 그때까지 쌓인 적분은 **반대편 영역의
  // 유물**이고(배기하느라 음수로 가득 차 있다), 그게 풀릴 때까지 밸브를 잘못
  // 붙들고 있는다. 실기 20260915_144459: 대기압 목표인데 i 가 −16.8 에서
  // 시작해 풀리는 데 9 초가 걸렸고 그동안 챔버가 95.8 까지 빨려 내려갔다
  // (그런 구간이 16 번, 최장 10 초).
  // 리셋도 atm 이 끝낼 수 있는 경우에만 한다. 목표가 대기압 너머면 레일로 갈
  // 것이고, 그때의 적분은 유물이 아니라 그대로 쓸 값이다.

  // ── atm 으로 끝낼 수 있는가 / 활짝 열어도 되는가 ──────────────────────────
  // atm 은 챔버를 대기압까지만 데려가고 거기서 멈춘다. 그래서 **목표가 대기압
  // 너머에 있으면 atm 으로는 끝낼 수 없다** — 그때는 처음부터 레일(micro)이 맞다.
  //   양압, 챔버 98 → 목표 101.3 : atm 이 정확히 거기서 멈춘다        → atm
  //   양압, 챔버 98 → 목표 130   : 어차피 레일이 필요하다              → micro
  //   음압, 챔버 103 → 목표 101.3: 대기로 빼면 된다                    → atm
  //   음압, 챔버 103 → 목표 46   : 대기압까지 가 봐야 소용없다          → micro
  // 이 조건을 빼면 음압이 망가진다 (실측 20260915_175418: 내림 오버슛이
  // +0.40 → +1.12, 최대 +0.71 → +3.43, 올림 정착이 +0.08 → +0.36 으로 나빠졌다.
  // 목표가 대기압 한참 아래인데 차압 1.5 kPa 짜리 atm 으로 빼려 했기 때문이다).
  const float to_atm_r = cfg_.atm_kpa   - in.P_meas_kpa;
  const float to_ref_r = in.P_ref_kpa - in.P_meas_kpa;
  const bool atm_reaches = std::abs(to_ref_r) <= std::abs(to_atm_r) + atm_m;
  // 이번 틱에 일할 밸브는 atm 이다. **하나의 판정**이다 — 이득 보정·적분·라우팅이
  // 전부 이것을 본다. 예전에는 라우팅에서 따로 한 번 더 계산했다.
  const bool atm_assist  = wrong_side && atm_reaches;
  // 그쪽으로 넘어가는 순간 적분을 버린다. 그때까지 쌓인 적분은 **반대편 영역의
  // 유물**이고(배기하느라 음수로 가득 차 있다), 그게 풀릴 때까지 밸브를 잘못
  // 붙들고 있는다. 실기 20260915_144459: 대기압 목표인데 i 가 −16.8 에서
  // 시작해 풀리는 데 9 초가 걸렸고 그동안 챔버가 95.8 까지 빨려 내려갔다
  // (그런 구간이 16 번, 최장 10 초).
  // 리셋도 atm 이 끝낼 수 있는 경우에만 한다 — 그래서 atm_reaches 를 같이 본다.
  // 목표가 대기압 너머면 레일로 갈 것이고, 그때의 적분은 유물이 아니라 그대로 쓸 값이다.
  if (atm_assist && !wrong_side_) integ_ = 0.0f;
  wrong_side_ = atm_assist;
  // 목표가 대기압이거나 **그 너머**면 활짝 열어도 지나칠 수 없다 (Gains::atm_boost_pct).
  // 목표가 챔버와 대기압 사이면 지나칠 수 있으므로 평소 법칙대로만 간다.
  const bool atm_safe = atm_assist &&
                        (sgn * (in.P_ref_kpa - cfg_.atm_kpa) >= -atm_m);
  const bool atm_boost = atm_safe && (g.atm_boost_pct > 0.0f);

  // ── 동작점(차압) 이득 보정 ────────────────────────────────────────────
  // 유량 ∝ 차압이므로 차압이 작으면 같은 지령으로도 덜 움직인다. 그 순간 열릴
  // 밸브의 차압으로 나눠 플랜트 이득 변화를 상쇄한다 (Gains 주석의 실측 참조).
  float gs = 1.0f;
  if (g.gain_dp_ref_kpa > 0.0f) {
    // 재라우팅 중이면 여는 밸브는 atm 이다 — e 부호만 보고 micro 차압을 쓰면
    // 플랜트 이득을 완전히 잘못 읽는다. 양압 챔버가 93 kPa 로 끌려갔을 때
    // micro 차압은 레일까지 107 kPa 지만 실제 구동 차압은 대기압까지 8 kPa 라,
    // gs 가 60/107 = 0.56 → 하한 0.6 으로 깔려 3.3 배를 스스로 깎고 있었다
    // (맞는 값은 60/8 = 7.5 → 상한 2.0). 대기압 반대편에서는 dp_atm 이 음수라
    // 크기만 쓴다.
    const float dp_act = atm_assist ? std::abs(in.dp_atm_kpa)
                                    : ((e >= 0.0f) ? in.dp_micro_kpa : in.dp_atm_kpa);
    if (dp_act > 0.5f)
      gs = std::clamp(g.gain_dp_ref_kpa / dp_act, g.gain_scale_min, g.gain_scale_max);
  }

  if (first_) {
    e_prev_ = e; ref_prev_ = in.P_ref_kpa; pm_prev_ = in.P_meas_kpa; first_ = false;
  }

  // ── 측정 기인 de/dt ───────────────────────────────────────────────────────
  // e = sgn·(ref − meas) 이므로 목표를 뺀 몫은 −sgn·d(meas)/dt 다. 목표 계단을
  // 빼는 것이 요점이다 (Gains::kd 주석의 330 %p 실측 참조).
  {
    const float draw = -sgn * (in.P_meas_kpa - pm_prev_) / dt;
    const float tau  = std::max(0.0f, cfg_.d_tau_s);
    const float a_lp = (tau > 0.0f) ? (dt / (tau + dt)) : 1.0f;
    de_filt_ += a_lp * (draw - de_filt_);
    if (!std::isfinite(de_filt_)) de_filt_ = 0.0f;
  }
  pm_prev_ = in.P_meas_kpa;

  // ── P (구간별 이득), D ────────────────────────────────────────────────────
  // |e| ≤ kp_break 는 kp, 밖은 kp_far. 경계에서 연속이 되게 잇는다 —
  //   p = sign(e)·(kp·kp_break + kp_far·(|e| − kp_break))
  // 계단이 생기면 그 자체가 진동원이 된다.
  const float p_term = [&]{
    const float brk  = g.kp_break_kpa;
    const float kp   = gs * g.kp;
    const float kfar = gs * ((g.kp_far >= 0.0f) ? g.kp_far : g.kp);
    if (brk <= 0.0f || std::abs(e) <= brk) return kp * e;
    const float s_e = (e > 0.0f) ? +1.0f : -1.0f;
    return s_e * (kp * brk + kfar * (std::abs(e) - brk));
  }();
  const float d_term = g.kd * de_filt_;
  e_prev_ = e;

  // ── I (anti-windup) ───────────────────────────────────────────────────────
  // 세 가지를 같이 쓴다.
  //   ① 조건부 적분: 이미 그 방향으로 포화해 있으면 쌓지 않는다. 더 쌓아도 명령이
  //      커지지 않고, 오차 부호가 뒤집힐 때 빠져나오는 데만 시간이 걸린다.
  //      포화 기준은 100 이 아니라 **이 틱의 실효 상한**(Input::u_limit_*)이다.
  //      크래킹 바이어스가 제어기 밖에서 더해지므로 하드웨어는 (임계−margin)+u 를
  //      100 에서 자른다. 100 을 기준으로 판정하면 밸브가 이미 활짝 열린 뒤에도
  //      "여유가 있다" 고 보고 계속 쌓는다 — 실측 20260907_204249 의 도달 후
  //      오버슛(12~14 kPa) 원인이 이것이다.
  //   ② 적분 밴드: |오차| > band_kpa 면 정지. 유량이 부족해 못 따라가는 구간에서
  //      쌓은 적분은 도달하는 순간 그대로 오버슛이 된다 (Gains::band_kpa 주석 참조).
  //   ③ 누적 클램프: ±i_limit_pct 안으로 자른다.
  // 과압 세이프티 래치 중에는 내 명령이 무시되므로 그때의 오차도 쌓지 않는다
  // (쌓아 두면 래치가 풀리는 순간 밸브가 활짝 열린다).
  const float u_cap  = std::max(1.0f, cfg_.u_max_pct);
  const float u_up   = std::clamp(in.u_limit_up_pct,   1.0f, u_cap);
  const float u_down = std::clamp(in.u_limit_down_pct, 1.0f, u_cap);

  // ── 목표압 변화율 (피드포워드용) ──────────────────────────────────────────
  // ref_prev_ 를 갱신하기 **전에** 잡는다.
  // 10 Hz 계단으로 들어오므로 그대로 미분하면 100 ms 마다 55 kPa/s 스파이크다 —
  // 반드시 저역 통과시킨다 (Config::ref_rate_tau_s 주석 참조).
  {
    const float draw = (in.P_ref_kpa - ref_prev_) / dt;
    const float tau  = std::max(0.0f, cfg_.ref_rate_tau_s);
    const float a_lp = (tau > 0.0f) ? (dt / (tau + dt)) : 1.0f;
    dref_filt_ += a_lp * (draw - dref_filt_);
    if (!std::isfinite(dref_filt_)) dref_filt_ = 0.0f;
  }

  // ── 레퍼런스 스텝에서 적분 리셋 ───────────────────────────────────────────
  // 목표가 크게 바뀌면 그때까지의 적분은 **직전 동작점의 유물**이다. 특히 대기압에
  // 오래 있었으면 누설과 싸우느라 한쪽 끝까지 차 있어서, 반대 방향 스텝이 들어오면
  // 먼저 풀리느라 밸브가 늦게 열린다. 버리고 새로 쌓는다.
  //
  // 주의: 사인의 **방향 반전**은 스텝이 아니고, 여기서 리셋하지도 않는다.
  // 반전 시점의 적분은 유물이 아니라 데드존 표의 오차를 보상하고 있는 실값이다
  // (20260911_165122: 9분 동안 필요한 지령이 micro/atm 양쪽 다 3~5 %p 밀렸다).
  // 버리면 그 보상까지 잃는다 — 반전은 위의 속도 피드포워드로 푼다.
  if (g.i_reset_on_step_kpa > 0.0f &&
      std::abs(in.P_ref_kpa - ref_prev_) > g.i_reset_on_step_kpa) {
    integ_ = 0.0f;
    // 스텝은 램프가 아니다. 계단을 미분한 값을 피드포워드에 넣으면 밸브를 슬램한다.
    dref_filt_ = 0.0f;
  }
  ref_prev_ = in.P_ref_kpa;

  // ── 반대 부호 적분 누설 (Gains::i_opp_leak_tau_s) ─────────────────────────
  // 정착 구간 밖에서 적분이 필요한 방향과 반대면 직전 동작점의 유물이다.
  // 램프로 동작점이 바뀌면 i_reset_on_step 이 안 걸리므로 여기서 흘려보낸다.
  if (g.i_opp_leak_tau_s > 0.0f && g.kp_break_kpa > 0.0f &&
      std::abs(e) > g.kp_break_kpa && e * integ_ < 0.0f) {
    const float a = dt / (g.i_opp_leak_tau_s + dt);
    integ_ -= a * integ_;                      // 0 을 향해 지수적으로
  }

  // ── 레퍼런스 속도 피드포워드 ──────────────────────────────────────────────
  // 필요한 지령은 오차가 아니라 목표의 속도에 비례한다. PI 는 그것을 적분으로만
  // 만들 수 있어 반드시 오차가 먼저 생기고, 방향 반전에서 제일 크게 벌어진다
  // (Gains::kv 주석의 실측 참조). 목표가 도는 순간 지령도 같이 돌게 한다.
  //
  // 방향 선택은 오차가 아니라 **속도의 부호**로 한다. 반전 직후에는 둘이 다르다.
  float ff_term = 0.0f;
  {
    const bool   up = (dref_filt_ >= 0.0f);            // 압력을 올리는 방향인가
    const Gains& gf = up ? cfg_.g_up : cfg_.g_down;
    if (gf.kv != 0.0f) {
      float gsf = 1.0f;
      if (gf.gain_dp_ref_kpa > 0.0f) {
        // 그 방향에서 실제로 열릴 밸브의 차압이다.
        //   양압: 올릴 때 micro(레일→챔버), 내릴 때 atm(챔버→대기)
        //   음압: 올릴 때 atm(대기→챔버), 내릴 때 micro(챔버→진공레일)
        const bool  use_micro = cfg_.is_positive ? up : !up;
        const float dp_act    = use_micro ? in.dp_micro_kpa : in.dp_atm_kpa;
        if (dp_act > 0.5f)
          gsf = std::clamp(gf.gain_dp_ref_kpa / dp_act, gf.gain_scale_min, gf.gain_scale_max);
      }
      // sgn 은 e 와 같은 규약이다: 음압 채널은 목표가 올라갈 때 atm(u<0)이다.
      ff_term = sgn * gsf * gf.kv * dref_filt_;
      const float lim = std::max(0.0f, cfg_.ff_limit_pct);
      ff_term = std::clamp(ff_term, -lim, +lim);
      if (!std::isfinite(ff_term)) ff_term = 0.0f;
    }
  }

  {
    // ff 도 하드웨어로 나가는 값이므로 포화 판정에 포함한다. 빼면 ff 가 이미
    // 밸브를 상한까지 밀어 놓은 뒤에도 적분이 계속 쌓인다.
    const float u_test  = p_term + d_term + integ_ + ff_term;
    const bool sat_hi   = (u_test >= +u_up)   && (e > 0.0f);
    const bool sat_lo   = (u_test <= -u_down) && (e < 0.0f);
    const bool off_band = (g.band_kpa > 0.0f) && (std::abs(e) > g.band_kpa);
    // 적분 이득도 구간별이다: 정착 구간 안은 ki, 밖은 ki_far (훨씬 작다).
    // 접근 중에 쌓인 적분이 도달 순간 그대로 오버슛이 되기 때문이다
    // (Gains::ki_far 주석의 실측 참조).
    const float ki_eff = gs * ((g.ki_far >= 0.0f && g.kp_break_kpa > 0.0f &&
                                std::abs(e) > g.kp_break_kpa) ? g.ki_far : g.ki);
    // 적분 데드밴드: 밸브가 분해할 수 없는 크기의 오차는 적분에서 뺀다.
    // 빼지 않으면 0.4 kPa 오차를 지우려고 지령이 크래킹까지 걸어 올라가 슬램한다
    // (Gains::i_deadband_kpa 주석의 실측 참조).
    float e_i = e;
    if (g.i_deadband_kpa > 0.0f) {
      const float d = std::min(std::abs(e), g.i_deadband_kpa);
      e_i = e - ((e > 0.0f) ? d : -d);
    }
    // 접근 속도 게이팅: 이 시간 뒤에 남아 있을 오차로 적분한다 (Gains::i_lead_s).
    // 플랜트가 멈춰 있으면 de_filt_ = 0 이라 그대로 e_i 다 — 교착이 없다.
    if (g.i_lead_s > 0.0f && e_i != 0.0f) {
      const float e_pred = e_i + g.i_lead_s * de_filt_;
      // 부호가 뒤집히면 0. 같으면 |e_i| 를 넘지 않게 자른다 (적분을 키우지 않는다).
      e_i = (e_pred * e_i <= 0.0f)
              ? 0.0f
              : ((e_i > 0.0f) ? std::min(e_pred, e_i) : std::max(e_pred, e_i));
    }
    // 공급 차압 게이팅 (Gains::i_dp_min_kpa) — **micro 방향에만** 건다.
    // micro 는 레일에서 공급받으므로 레일이 무너지면 정말로 유량이 0 이 된다.
    // atm 은 대기가 상대라 **절대 마르지 않는다.** 그런데 대기로 빼는 쪽은
    // 차압이 곧 오차 자신(챔버 − 대기압)이라, 목표가 대기압이면 도착할수록
    // 차압이 0 으로 간다 — 여기에 문턱을 걸면 영영 못 내려간다.
    // 실측 20260915_143259: 대기압 목표에서 적분이 +3.07 에 얼어 챔버가
    // 102.9~103.3 에 갇혔다 (게이팅 없을 때는 적분 −20 으로 atm 을 80 % 열어
    // 101.3 에 도달했다).
    const bool use_micro_i = (e >= 0.0f);      // 부호 규약: e>0 → micro
    const bool no_supply = (g.i_dp_min_kpa > 0.0f) && use_micro_i &&
                           (in.dp_micro_kpa < g.i_dp_min_kpa);
    // 부스트 구간에서는 적분을 멈춘다 (Gains::atm_boost_pct).
    // 바닥 지령이 이미 그 일을 하고 있어 적분은 보탤 것이 없고, 대기압을 지나
    // micro 로 넘겨주는 순간 그대로 밸브를 여는 발길질이 된다. 진입 시 0 으로
    // 리셋해 두었으므로 여기서 멈추면 넘겨줄 때도 0 이다.
    if (!in.safety_latched && !sat_hi && !sat_lo && !off_band && !no_supply &&
        !atm_boost) {
      const float raw = integ_ + ki_eff * e_i * dt;
      integ_ = std::clamp(raw, -g.i_limit_pct, +g.i_limit_pct);
      isat_last_ = (raw != integ_) ? 1 : 0;      // 누적 클램프에 걸렸다
    } else {
      isat_last_ = atm_boost ? 4 : (no_supply ? 3 : 2);  // 4 = atm 부스트, 3 = 공급 차압 없음
    }
  }

  // ── 합산 ──────────────────────────────────────────────────────────────────
  // 클램프도 실효 상한으로 한다 — 하드웨어가 못 받는 지령을 애초에 만들지 않는다.
  float u = p_term + integ_ + d_term + ff_term;
  // 비유한값(NaN/Inf)이 PWM 으로 나가면 밸브가 예측 불가로 열린다.
  if (!std::isfinite(u)) u = 0.0f;
  u = std::clamp(u, -u_down, +u_up);

  // ── 방향에 따라 한쪽 밸브에만 넣는다 ──────────────────────────────────────
  // 평소에는 u > 0 → micro, u < 0 → atm 이다. 다만 챔버가 대기압 **반대편**에
  // 있으면 (위 wrong_side 참조) 올리는 일을 atm 이 해야 한다 — 그쪽에서는
  // 대기를 들이는 것이 곧 압력을 올리는 것이고, 대기압에서 스스로 멈춘다.
  // 어느 밸브가 일할 수 있는지는 위(atm_assist)에서 이미 정했다 — 이득 보정도
  // 같은 판정을 써야 해서 앞으로 옮겼다.
  const bool reroute = atm_assist && (u > 0.0f);
  // 회수 구간 바닥 (Gains::atm_boost_pct). 목표가 대기압이거나 그 너머일 때만
  // 건다 — 그때 atm 은 대기압에서 스스로 멈추므로 지나칠 수 없다.
  float u_atm = std::abs(u);
  if (atm_boost) u_atm = std::max(u_atm, g.atm_boost_pct);
  Output out;
  out.u_pct[V_MICRO] = (u > 0.0f && !reroute)
                         ? std::clamp(+u, cfg_.u_min_pct, u_up)   : 0.0f;
  // 세 번째 조건은 u 가 정확히 0 인 틱을 위한 것이다 — 목표에 거의 닿아 P·I 가
  // 0 이어도 부스트 바닥은 계속 열어 둔다. 지나칠 수 없는 구간이라 안전하다.
  out.u_pct[V_ATM]   = (u < 0.0f || reroute || atm_boost)
                         ? std::clamp(u_atm, cfg_.u_min_pct, u_down) : 0.0f;
  out.u_pct[V_MACRO] = 0.0f;

  // 진단 스냅샷 — 로그에서 항을 분해할 수 있게 그대로 남긴다.
  u_last_  = u;
  p_last_  = p_term;
  d_last_  = d_term;
  ff_last_ = ff_term;
  gs_last_ = gs;

  last_ = out;
  return out;
}

// 내부 순서 {micro, macro, atm} → 보드 슬롯 {v1=micro, v2=atm, v3=macro}.
// 100 % = 4095 (= 100 × 40.95).
std::array<uint16_t, PressureCtrl::N_VALVE>
PressureCtrl::to_pwm(const Output& out) {
  auto pwm = [](float u_pct) -> uint16_t {
    if (!std::isfinite(u_pct)) u_pct = 0.0f;
    const float v = std::round(std::clamp(u_pct, 0.0f, 100.0f) * 40.95f);
    return static_cast<uint16_t>(std::clamp(v, 0.0f, 4095.0f));
  };
  return { pwm(out.u_pct[V_MICRO]),    // v1
           pwm(out.u_pct[V_ATM]),      // v2
           pwm(out.u_pct[V_MACRO]) };  // v3
}
