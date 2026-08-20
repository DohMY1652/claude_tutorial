#include "PressureRefGen.hpp"
#include "PneumaticFlow.hpp"
#include "Controller.hpp"   // QP 래퍼 (qpOASES 박스 QP)

#include <algorithm>
#include <cmath>

using pneu::P_ATM;
using pneu::R_AIR;
using pneu::T_CH;
using pneu::T_PIS;
using pneu::KAPPA;
using pneu::RHO0;
using pneu::orifice_phi;
using pneu::valve_phys_kgps;
using pneu::valve_capacity;

// ============================================================================
// Params
// ============================================================================
void PressureRefGen::Params::set_orifices(double d_fill_mm, double d_vent_mm,
                                         double d_boost_mm, double d_suck_mm,
                                         double d_admit_mm, double d_eject_mm)
{
  // 기하 면적 × Cd(방출계수) × eta(한 스텝 평균 개도).
  // eta 를 빼면 슬루 박스가 완전 개방을 가정해 낙관적이 된다.
  auto A = [this](double d_mm) {
    const double d = d_mm * 1e-3;
    return Cd * valve_open_eta * M_PI * d * d / 4.0;
  };
  A_fill  = A(d_fill_mm);
  A_vent  = A(d_vent_mm);
  A_boost = A(d_boost_mm);
  A_suck  = A(d_suck_mm);
  A_admit = A(d_admit_mm);
  A_eject = A(d_eject_mm);
}

PressureRefGen::PressureRefGen(const Params& p) : p_(p)
{
  if ((int)p_.Apos.size() != p_.N) p_.Apos.assign(p_.N, 1.9635e-3);
  if ((int)p_.Aneg.size() != p_.N) p_.Aneg.assign(p_.N, 1.9635e-3);

  // 결정변수 2N 개, 박스 제약뿐이므로 nc = 0
  qp_ = std::make_shared<QP>(2 * p_.N, 0);
  x_prev_ = Eigen::VectorXd::Zero(2 * p_.N);
}

// ============================================================================
// 펌프 능력 테이블 (PistonPump.hpp 공용 모델)
// ============================================================================
void PressureRefGen::build_pump_table()
{
  pump_.build(p_.pump, p_.Ppos_sp_max, p_.Pneg_cap_deep, p_.Pneg_shallow, p_.pump_grid_n);
}

// ============================================================================
// 레일 셋포인트: 프리뷰 총 수요 크기로 능력경계 위에서 배분
// ============================================================================
void PressureRefGen::decide_rail_setpoint(const std::vector<std::vector<double>>& F_preview,
                                          double& ppos_sp, double& pneg_sp,
                                          double& demand_norm) const
{
  double demand = 0.0;
  for (int i = 0; i < p_.N; ++i) {
    double peak = 0.0;
    if (i < (int)F_preview.size())
      for (double f : F_preview[(size_t)i]) peak = std::max(peak, f);
    demand += peak;
  }
  demand_norm = std::min(demand / std::max(1e-9, p_.N * p_.Fmax_ref), 1.0);

  // 음압레일: 수요 클수록 깊게
  pneg_sp = p_.Pneg_shallow + demand_norm * (p_.Pneg_cap_deep - p_.Pneg_shallow);
  pneg_sp = std::max(p_.Pneg_cap_deep, std::min(p_.Pneg_shallow, pneg_sp));

  // 양압레일: 수요 클수록 높게, 단 능력경계까지만
  const double ppos_target = p_.Ppos_sp_min + demand_norm * (p_.Ppos_sp_max - p_.Ppos_sp_min);
  ppos_sp = std::min(cap_ppos(pneg_sp), ppos_target);
}

// ============================================================================
// 이젝터: 측정값 우선, 없으면 특성곡선과 오리피스식의 교점 (이분법)
// ============================================================================
double PressureRefGen::ejector_flow(double Pch_gauge, const SupplyState& sup,
                                    double* P_ej_out) const
{
  // 구동 중이면 측정값을 쓴다 (실제 성능·지연이 반영된 가장 정확한 값).
  // 꺼져 있으면 측정값은 대기압이라 능력을 0 으로 오판하므로 특성곡선의 잠재력을 쓴다.
  if (sup.use_ej_meas && sup.ej_running) {
    if (P_ej_out) *P_ej_out = sup.P_ej;
    return valve_phys_kgps(Pch_gauge + P_ATM, sup.P_ej + P_ATM, p_.A_eject);
  }
  double lo = 0.0, hi = p_.Q_ej_max;
  for (int it = 0; it < 40; ++it) {
    const double Q = 0.5 * (lo + hi);
    const double P_ej = p_.P_ej_max * (1.0 - Q / p_.Q_ej_max);
    const double m = valve_phys_kgps(Pch_gauge + P_ATM, P_ej + P_ATM, p_.A_eject);
    if (m / RHO0 > Q) lo = Q; else hi = Q;
  }
  const double Q = 0.5 * (lo + hi);
  const double P_ej = p_.P_ej_max * (1.0 - Q / p_.Q_ej_max);
  if (P_ej_out) *P_ej_out = P_ej;
  return valve_phys_kgps(Pch_gauge + P_ATM, P_ej + P_ATM, p_.A_eject);
}

// ============================================================================
// 슬루 박스: 지금 이 순간의 차압에서 나오는 실제 밸브 유량으로 계산
// ============================================================================
void PressureRefGen::build_slew_box(const std::vector<AxisState>& axes, const SupplyState& sup,
                                    Eigen::VectorXd& lb, Eigen::VectorXd& ub,
                                    Eigen::VectorXd& dP_rail_max, Eigen::VectorXd& dN_rail_max) const
{
  const int N = p_.N;
  const double rt = p_.n_ch * R_AIR * T_CH;    // 챔버는 단열
  const double Ppp = sup.P_rail_pos, Pnp = sup.P_rail_neg, Ptk = sup.P_tank;

  for (int i = 0; i < N; ++i) {
    const auto& a = axes[(size_t)i];
    const double Pp0 = a.P_pos, Pn0 = a.P_neg;
    const double Vp = std::max(1e-9, a.V_pos), Vn = std::max(1e-9, a.V_neg);

    // 양압 상승 = 레일 충진 + 탱크 부스트 (동시 사용 가능하므로 합산)
    const double mfill  = valve_phys_kgps(Ppp + P_ATM, Pp0 + P_ATM, p_.A_fill);
    const double mboost = valve_phys_kgps(Ptk + P_ATM, Pp0 + P_ATM, p_.A_boost);
    const double mvent  = valve_phys_kgps(Pp0 + P_ATM, P_ATM,       p_.A_vent);
    // 음압 심화 = 음압레일 흡입 + 이젝터
    const double msuck  = valve_phys_kgps(Pn0 + P_ATM, Pnp + P_ATM, p_.A_suck);
    const double meject = ejector_flow(Pn0, sup);
    const double madmit = valve_phys_kgps(P_ATM, Pn0 + P_ATM, p_.A_admit);

    // 피스톤 운동에 의한 체적변화 항
    const double volP = p_.n_ch * (Pp0 + P_ATM) / Vp * a.dVdt_pos * p_.dt;
    const double volN = p_.n_ch * (Pn0 + P_ATM) / Vn * a.dVdt_neg * p_.dt;

    double dP_up = (mfill + mboost) * p_.dt * rt / Vp - volP;
    double dP_dn =  mvent           * p_.dt * rt / Vp + volP;
    double dN_dn = (msuck + meject) * p_.dt * rt / Vn + volN;
    double dN_up =  madmit          * p_.dt * rt / Vn - volN;
    dP_up = std::max(dP_up, 0.0); dP_dn = std::max(dP_dn, 0.0);
    dN_dn = std::max(dN_dn, 0.0); dN_up = std::max(dN_up, 0.0);

    double loP = std::max(0.0,             Pp0 - dP_dn);
    double hiP = std::min(p_.Pch_pos_max,  Pp0 + dP_up);
    if (hiP < loP) hiP = loP;
    double loN = std::max(p_.Pch_neg_min,  Pn0 - dN_dn);
    double hiN = std::min(0.0,             Pn0 + dN_up);
    if (hiN < loN) hiN = loN;

    lb(i) = loP; ub(i) = hiP; lb(N + i) = loN; ub(N + i) = hiN;

    // 레일만으로 도달 가능한 변화폭 → Jtank / Jeject 의 기준선
    dP_rail_max(i) = mfill * p_.dt * rt / Vp;
    dN_rail_max(i) = msuck * p_.dt * rt / Vn;
  }
}

// ============================================================================
// 목적함수 (MATLAB obj_fun_ch 와 동일)
// ============================================================================
double PressureRefGen::objective(const Eigen::VectorXd& x, const Eigen::VectorXd& xp,
                                 const std::vector<double>& F_ref, const SupplyState& sup,
                                 const std::vector<AxisState>& /*axes*/,
                                 const Eigen::VectorXd& dP_rail_max,
                                 const Eigen::VectorXd& dN_rail_max) const
{
  const int N = p_.N;
  const double Ppp = sup.P_rail_pos, Pnp = sup.P_rail_neg;
  const double Tauscale = p_.Pch_pos_max * *std::max_element(p_.Apos.begin(), p_.Apos.end());
  double Qscale = valve_capacity(p_.Ppos_sp_max + P_ATM, P_ATM, p_.A_max);
  if (Qscale <= 0.0) Qscale = 1.0;

  double J0 = 0.0, J2 = 0.0, J4 = 0.0, Jtank = 0.0, Jeject = 0.0;

  for (int i = 0; i < N; ++i) {
    const double Pp = x(i), Pn = x(N + i);
    const double F = Pp * p_.Apos[(size_t)i] - Pn * p_.Aneg[(size_t)i];
    const double e = (F - F_ref[(size_t)i]) / Tauscale;
    J0 += e * e;

    // J2: 총 가용유량 최대화 (부호가 음수 — 크면 좋다)
    J2 -= (valve_capacity(Ppp + P_ATM, Pp + P_ATM, p_.A_max)
         + valve_capacity(Pn + P_ATM, Pnp + P_ATM, p_.A_max)) / Qscale;

    // Jtank / Jeject: 레일이 못 대는 초과분에만 벌점
    Jtank  += std::max(0.0, std::max(0.0, Pp - xp(i))     - dP_rail_max(i)) / p_.Pch_pos_max;
    Jeject += std::max(0.0, std::max(0.0, xp(N + i) - Pn) - dN_rail_max(i))
              / std::abs(p_.Pch_neg_min);
  }
  J4 = (x - xp).lpNorm<1>() / p_.Pch_pos_max;

  // Jfast(빠른 쪽 우선)는 원본 2차 검토에서 **추종을 해친다**고 판정되어 제거됐다.
  // 슬루 박스가 이미 두 챔버의 물리적 능력을 담고 있어, Jtrk 하나만으로도 그 순간
  // 유리한 챔버를 자동으로 쓴다 (자기균형 인수인계). 가중치를 0 으로 두는 대신
  // 항 자체를 없애 매 SQP 반복의 수치 기울기 평가에서 완전히 빠지게 했다.
  return p_.wtrack * J0 + p_.w_flow * J2
       + p_.w_smooth * J4 + p_.w_tank * Jtank + p_.w_eject * Jeject;
}

// ============================================================================
// step — 레일 셋포인트 → 채널 최적화 → 사용량 진단
// ============================================================================
PressureRefGen::Result PressureRefGen::step(const std::vector<double>& F_ref_in,
                                            const std::vector<AxisState>& axes,
                                            const SupplyState& sup,
                                            const std::vector<std::vector<double>>& F_preview_in)
{
  const int N = p_.N;
  const int nx = 2 * N;
  Result res;
  res.P_pos_ref.assign((size_t)N, 0.0);
  res.P_neg_ref.assign((size_t)N, 0.0);
  res.F_achieved.assign((size_t)N, 0.0);
  res.lb_pos.assign((size_t)N, 0.0); res.ub_pos.assign((size_t)N, 0.0);
  res.lb_neg.assign((size_t)N, 0.0); res.ub_neg.assign((size_t)N, 0.0);
  res.starve_pos.assign((size_t)N, 0.0); res.starve_neg.assign((size_t)N, 0.0);

  // 목표 힘은 항상 ≥ 0 (이 시스템은 한 방향 힘만 낸다)
  std::vector<double> F_ref((size_t)N, 0.0);
  for (int i = 0; i < N && i < (int)F_ref_in.size(); ++i)
    F_ref[(size_t)i] = std::max(0.0, F_ref_in[(size_t)i]);

  // ── 1. 레일 셋포인트 ────────────────────────────────────────────────
  std::vector<std::vector<double>> F_preview = F_preview_in;
  if (F_preview.empty()) {
    F_preview.resize((size_t)N);
    for (int i = 0; i < N; ++i) F_preview[(size_t)i] = { F_ref[(size_t)i] };
  }
  decide_rail_setpoint(F_preview, res.rail_pos_sp, res.rail_neg_sp, res.demand_norm);

  // ── 2. 슬루 박스 ───────────────────────────────────────────────────
  Eigen::VectorXd lb(nx), ub(nx), dP_rail(N), dN_rail(N);
  build_slew_box(axes, sup, lb, ub, dP_rail, dN_rail);

  Eigen::VectorXd xp(nx);
  for (int i = 0; i < N; ++i) { xp(i) = axes[(size_t)i].P_pos; xp(N + i) = axes[(size_t)i].P_neg; }

  // ── 3. 박스 제약 SQP ───────────────────────────────────────────────
  // 목적함수가 비선형(J2)·비평활(J4 의 L1, hinge)이라 순수 QP 로 표현되지 않는다.
  // 매 반복 Gauss-Newton 이차모형을 세워 박스 QP 로 방향을 얻고, 실제 J 로 line search.
  constexpr double XS = 1e5;   // 압력 스케일 O(1e5) → O(1)
  const Eigen::VectorXd lbz = lb / XS, ubz = ub / XS;
  Eigen::VectorXd z = xp.cwiseMax(lb).cwiseMin(ub) / XS;

  auto Jof = [&](const Eigen::VectorXd& zz) {
    return objective(zz * XS, xp, F_ref, sup, axes, dP_rail, dN_rail);
  };

  // Gauss-Newton Hessian (J0 항만; F 가 x 에 선형이라 상수)
  const double Tauscale = p_.Pch_pos_max * *std::max_element(p_.Apos.begin(), p_.Apos.end());
  Eigen::MatrixXf H = Eigen::MatrixXf::Zero(nx, nx);
  for (int i = 0; i < N; ++i) {
    const double a = p_.Apos[(size_t)i] * XS, b = p_.Aneg[(size_t)i] * XS;
    const double c = 2.0 * p_.wtrack / (Tauscale * Tauscale);
    H(i, i)             += (float)(c * a * a);
    H(N + i, N + i)     += (float)(c * b * b);
    H(i, N + i)         += (float)(-c * a * b);
    H(N + i, i)         += (float)(-c * a * b);
  }
  // 비이차 항(J2/J4/hinge)을 감당할 정규화
  const double reg = 1e-3 * std::max(1.0, (double)H.diagonal().maxCoeff());
  for (int i = 0; i < nx; ++i) H(i, i) += (float)reg;

  double J = Jof(z);
  Eigen::VectorXd g(nx), zt(nx);
  Eigen::VectorXf d_sol;

  for (int it = 0; it < p_.max_iter; ++it) {
    // 수치 기울기 (중앙차분). nx ≤ 12 이므로 비용이 무시할 만하다.
    const double h = 1e-6;
    for (int k = 0; k < nx; ++k) {
      zt = z; zt(k) += h; const double Jp = Jof(zt);
      zt = z; zt(k) -= h; const double Jm = Jof(zt);
      g(k) = (Jp - Jm) / (2.0 * h);
    }
    if (g.norm() < p_.grad_tol) break;

    const Eigen::VectorXf gf = g.cast<float>();
    const Eigen::VectorXf lbd = (lbz - z).cast<float>();
    const Eigen::VectorXf ubd = (ubz - z).cast<float>();
    if (!qp_->solve(H, gf, lbd, ubd, d_sol)) { res.solver_ok = false; break; }

    Eigen::VectorXd d = d_sol.cast<double>();
    if (d.norm() < 1e-12) break;

    // backtracking line search — 실제 목적함수로 평가
    bool improved = false;
    for (double alpha = 1.0; alpha > 1e-3; alpha *= 0.5) {
      zt = (z + alpha * d).cwiseMax(lbz).cwiseMin(ubz);
      const double Jt = Jof(zt);
      if (Jt < J - 1e-14) { z = zt; J = Jt; improved = true; break; }
    }
    res.sqp_iters = it + 1;
    if (!improved) break;
  }

  const Eigen::VectorXd x = (z * XS).cwiseMax(lb).cwiseMin(ub);

  // ── 4. 결과 + 공급원 사용량 진단 ───────────────────────────────────
  const double rt = p_.n_ch * R_AIR * T_CH;
  for (int i = 0; i < N; ++i) {
    const auto& a = axes[(size_t)i];
    res.P_pos_ref[(size_t)i] = x(i);
    res.P_neg_ref[(size_t)i] = x(N + i);
    res.F_achieved[(size_t)i] = x(i) * p_.Apos[(size_t)i] - x(N + i) * p_.Aneg[(size_t)i];
    res.lb_pos[(size_t)i] = lb(i);     res.ub_pos[(size_t)i] = ub(i);
    res.lb_neg[(size_t)i] = lb(N + i); res.ub_neg[(size_t)i] = ub(N + i);

    // 양압 상승분: 레일이 최대한, 초과분은 탱크 부스트
    const double dup = std::max(0.0, x(i) - a.P_pos) * std::max(1e-9, a.V_pos) / rt / p_.dt;
    const double mfill_cap = valve_phys_kgps(sup.P_rail_pos + P_ATM, a.P_pos + P_ATM, p_.A_fill);
    const double mfill  = std::min(dup, mfill_cap);
    const double mboost = std::max(0.0, dup - mfill);
    // 음압 심화분: 음압레일이 최대한, 초과분은 이젝터 (정격 이내로 제한)
    const double ddn = std::max(0.0, a.P_neg - x(N + i)) * std::max(1e-9, a.V_neg) / rt / p_.dt;
    const double msuck_cap = valve_phys_kgps(a.P_neg + P_ATM, sup.P_rail_neg + P_ATM, p_.A_suck);
    const double mej_cap   = ejector_flow(a.P_neg, sup);
    const double msuck  = std::min(ddn, msuck_cap);
    const double meject = std::min(std::max(0.0, ddn - msuck), mej_cap);

    // 축별 유량 부족률 — Controller 의 macro 게이트가 이 값을 본다.
    // 분자는 "레일이 못 대는 양"이고 분모는 "이번 스텝 요구량"이므로 무차원이다.
    // 음압은 meject(이젝터 능력으로 잘린 값)가 아니라 (ddn − msuck) 를 쓴다 —
    // 이젝터가 꺼져 있어도 부족률이 정직하게 잡혀야 게이트가 열릴 수 있다.
    res.starve_pos[(size_t)i] = (dup > 1e-12) ? mboost / dup : 0.0;
    res.starve_neg[(size_t)i] = (ddn > 1e-12) ? std::max(0.0, ddn - msuck) / ddn : 0.0;

    res.m_fill += mfill; res.m_boost += mboost;
    res.m_suck += msuck; res.m_eject += meject;
  }
  res.m_tank_draw = res.m_boost + res.m_eject * p_.eject_ratio;
  res.tank_low = (sup.P_tank < p_.P_tank_stop);

  x_prev_ = x;
  has_prev_ = true;
  return res;
}
