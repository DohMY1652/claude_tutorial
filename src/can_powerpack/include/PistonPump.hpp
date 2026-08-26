#pragma once

// ============================================================================
// PistonPump — 피스톤 펌프 1주기 평균 유량 + 2D 능력 테이블
// ============================================================================
// pressure_reference_optimizer.m 의 pump_piston_avg / build_pump_table 포팅.
// (기하 원본: catkin_ws/src/mppi_brl/.../Dynamics/Pump.{cpp,h} 와 동일한 슬라이더-크랭크)
//
// 흡입구 = 음압 레일, 토출구 = 양압 레일. 같은 피스톤이 왕복하므로 흡입량 = 토출량이고,
// 이 때문에 양압을 높이면 음압이 억눌리는 트레이드오프(능력경계)가 생긴다.
//
// 검증: 이 모델은 해설서 그림 B 의 두 점을 정확히 재현한다 —
//   음압 −90 kPa → 양압 상한 335 kPa,  음압 −80 kPa → 745 kPa
//
// PressureRefGen(레일 셋포인트 배분)과 VirtualPowerpack(가상 하드웨어)이 공유한다.
// 매 스텝 적분은 비싸므로(1회전 = 2400 스텝) 기동 시 2D 테이블을 만들어 보간해 쓴다.
// ============================================================================

#include "PneumaticFlow.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace pneu {

struct PumpGeom {
  double delta{0.041};                    // 피스톤 최대 체적변화 길이 [m]
  double r{0.02};                         // 크랭크 [m]
  double l{0.07};                         // 링크 [m]
  double omega{3000.0 * 2.0 * M_PI / 60.0};  // 3000 rpm [rad/s]
  double Spis{38.485e-4};                 // 피스톤 단면적 [m²]
  int    Npis{2};                         // 피스톤 수 (180° 위상)
  double Cb_out{1.46e-6};                 // 토출 체크밸브 계수 [m²]
  double Cb_in{33.47e-6};                 // 흡입 체크밸브 계수 [m²]
  double dt{1e-4};                        // 적분 스텝 [s]
  int    nrev{12};                        // 정상 주기 도달용 회전수
};

// 1주기 평균 질량유량 [kg/s]. 입력은 **절대압** [Pa].
inline void pump_piston_avg(const PumpGeom& g, double Ppos_abs, double Pneg_abs,
                            double& mdot_out_avg, double& mdot_in_avg)
{
  const double Trev = 2.0 * M_PI / g.omega;
  const int    nstep = (int)std::lround(g.nrev * Trev / g.dt);
  const double t_lastrev = (g.nrev - 1) * Trev;

  // 식(2) 피스톤 챔버 체적
  auto Vp = [&](double th) {
    return g.Spis * (g.delta - g.r + g.l - g.r * std::cos(th)
                     - std::sqrt(std::max(0.0, g.l * g.l
                                 - g.r * g.r * std::sin(th) * std::sin(th))));
  };

  double th = 0.0;
  double m = P_ATM * Vp(th) / (R_AIR * T_PIS);
  double acc_out = 0.0, acc_in = 0.0, t_acc = 0.0;

  for (int s = 1; s <= nstep; ++s) {
    const double V = std::max(1e-12, Vp(th));
    const double Ppis = m * R_AIR * T_PIS / V;      // 식(1)

    double mdot_out = 0.0, mdot_in = 0.0;
    if (Ppis > Ppos_abs)
      mdot_out = g.Cb_out * Ppis / std::sqrt(R_AIR * T_PIS) * orifice_phi(Ppis, Ppos_abs);
    if (Pneg_abs > Ppis)
      mdot_in = g.Cb_in * Pneg_abs / std::sqrt(R_AIR * T_PIS) * orifice_phi(Pneg_abs, Ppis);

    m = std::max(0.0, m + (mdot_in - mdot_out) * g.dt);
    th += g.omega * g.dt;

    if (s * g.dt > t_lastrev) {                     // 마지막 1주기만 평균
      acc_out += mdot_out * g.dt;
      acc_in  += mdot_in  * g.dt;
      t_acc   += g.dt;
    }
  }
  if (t_acc <= 0.0) t_acc = Trev;
  mdot_out_avg = g.Npis * acc_out / t_acc;
  mdot_in_avg  = g.Npis * acc_in  / t_acc;
}

// ── 2D 능력 테이블 + 능력경계 ─────────────────────────────────────────────
class PumpTable {
public:
  // 격자 범위는 게이지 압력으로 지정한다.
  void build(const PumpGeom& g, double ppos_max_gauge, double pneg_deep_gauge,
             double pneg_shallow_gauge, int grid_n = 13, int cap_n = 21)
  {
    const int n = std::max(2, grid_n);
    gx_.resize(n); gy_.resize(n);
    for (int i = 0; i < n; ++i) {
      const double f = (double)i / (n - 1);
      gx_[i] = P_ATM + f * ppos_max_gauge;                        // 절대압
      gy_[i] = P_ATM + pneg_deep_gauge + f * (-pneg_deep_gauge);
    }
    out_.assign((size_t)n * n, 0.0);
    in_.assign((size_t)n * n, 0.0);
    for (int a = 0; a < n; ++a)
      for (int b = 0; b < n; ++b) {
        double mo = 0.0, mi = 0.0;
        pump_piston_avg(g, gx_[a], gy_[b], mo, mi);
        out_[(size_t)a * n + b] = mo;
        in_ [(size_t)a * n + b] = mi;
      }

    // 능력경계: 각 음압 셋포인트에서 유량이 살아 있는 최대 양압
    const int nc = std::max(2, cap_n);
    cap_x_.resize(nc); cap_y_.resize(nc);
    for (int j = 0; j < nc; ++j) {
      const double f = (double)j / (nc - 1);
      const double pn = pneg_deep_gauge + f * (pneg_shallow_gauge - pneg_deep_gauge);
      cap_x_[j] = pn;
      double pmax = 0.0;
      for (double pp = 0.0; pp <= ppos_max_gauge + 1.0; pp += 5e3) {
        double mo = 0.0, mi = 0.0;
        pump_piston_avg(g, pp + P_ATM, pn + P_ATM, mo, mi);
        if (mo * 1e3 > 0.02) pmax = pp;        // 0.02 g/s 기준 (원본과 동일)
      }
      cap_y_[j] = pmax;
    }
    ready_ = true;
  }

  bool ready() const { return ready_; }

  double flow_out(double Ppos_abs, double Pneg_abs) const   // 토출 [kg/s]
  { return ready_ ? std::max(0.0, interp2(out_, Ppos_abs, Pneg_abs)) : 0.0; }
  double flow_in(double Ppos_abs, double Pneg_abs) const    // 흡입 [kg/s]
  { return ready_ ? std::max(0.0, interp2(in_, Ppos_abs, Pneg_abs)) : 0.0; }

  // 실측 능력경계(pump_fit_solve.py Phase F)로 cap 테이블을 덮어쓴다. 측정 구간
  // 안은 실측을, 밖은 기하 시뮬레이션 외삽을 그대로 쓴다. 5-파라미터 슬라이더-크랭크는
  // 소기량×Cb_in 축퇴가 남아 데드헤드 외삽이 부정확하지만(자기검증 ~15%), 측정 구간
  // 안에서는 직접 측정이 언제나 우선한다. 두 벡터가 비었거나 길이가 안 맞으면,
  // 또는 build() 전이면 아무것도 안 하고 기하 테이블을 그대로 둔다.
  void override_cap_measured(std::vector<double> pneg_gauge, std::vector<double> ppos_max_gauge)
  {
    if (pneg_gauge.empty() || pneg_gauge.size() != ppos_max_gauge.size() || cap_x_.size() < 2)
      return;
    std::vector<size_t> idx(pneg_gauge.size());
    for (size_t i = 0; i < idx.size(); ++i) idx[i] = i;
    std::sort(idx.begin(), idx.end(),
             [&](size_t a, size_t b) { return pneg_gauge[a] < pneg_gauge[b]; });
    std::vector<double> mx, my;
    for (size_t k = 0; k < idx.size(); ++k) {
      const double x = pneg_gauge[idx[k]], y = ppos_max_gauge[idx[k]];
      // 거의 같은 음압 셋포인트(±0.5 kPa, 반복 측정)는 평균으로 합친다.
      if (!mx.empty() && std::abs(x - mx.back()) < 0.5e3) my.back() = 0.5 * (my.back() + y);
      else { mx.push_back(x); my.push_back(y); }
    }
    const double lo = mx.front(), hi = mx.back();
    std::vector<double> nx, ny;
    for (size_t i = 0; i < cap_x_.size(); ++i)
      if (cap_x_[i] < lo) { nx.push_back(cap_x_[i]); ny.push_back(cap_y_[i]); }
    for (size_t k = 0; k < mx.size(); ++k) { nx.push_back(mx[k]); ny.push_back(my[k]); }
    for (size_t i = 0; i < cap_x_.size(); ++i)
      if (cap_x_[i] > hi) { nx.push_back(cap_x_[i]); ny.push_back(cap_y_[i]); }
    cap_x_ = std::move(nx); cap_y_ = std::move(ny);
  }

  // 능력경계 보간: 음압 셋포인트(게이지) → 유지 가능한 최대 양압(게이지)
  double cap_ppos(double pneg_gauge) const
  {
    if (cap_x_.size() < 2) return 0.0;
    const int n = (int)cap_x_.size();
    if (pneg_gauge <= cap_x_.front()) return std::max(0.0, cap_y_.front());
    if (pneg_gauge >= cap_x_.back())  return std::max(0.0, cap_y_.back());
    int i = 0;
    while (i < n - 2 && cap_x_[i + 1] < pneg_gauge) ++i;
    const double f = (pneg_gauge - cap_x_[i]) / std::max(1e-12, cap_x_[i + 1] - cap_x_[i]);
    return std::max(0.0, cap_y_[i] + f * (cap_y_[i + 1] - cap_y_[i]));
  }

private:
  double interp2(const std::vector<double>& v, double x, double y) const
  {
    const int n = (int)gx_.size();
    auto locate = [n](const std::vector<double>& g, double q, double& frac) {
      if (q <= g.front()) { frac = 0.0; return 0; }
      if (q >= g.back())  { frac = 1.0; return n - 2; }
      int i = 0;
      while (i < n - 2 && g[i + 1] < q) ++i;
      frac = (q - g[i]) / std::max(1e-12, g[i + 1] - g[i]);
      return i;
    };
    double fx = 0.0, fy = 0.0;
    const int ix = locate(gx_, x, fx);
    const int iy = locate(gy_, y, fy);
    const double v00 = v[(size_t)ix * n + iy],       v01 = v[(size_t)ix * n + iy + 1];
    const double v10 = v[(size_t)(ix + 1) * n + iy], v11 = v[(size_t)(ix + 1) * n + iy + 1];
    return (1 - fx) * ((1 - fy) * v00 + fy * v01) + fx * ((1 - fy) * v10 + fy * v11);
  }

  std::vector<double> gx_, gy_, out_, in_;
  std::vector<double> cap_x_, cap_y_;
  bool ready_{false};
};

// ── 이젝터 특성곡선 (ZL112A 실측표) ───────────────────────────────────────
// 구동압(게이지 kPa) 0/100/200/300/400 기준. Trash 의 Ejector.cpp 데이터와 동일.
//   흡입 [LPM]        : 0, 31, 64, 90, 107
//   탱크 소비 [LPM]   : 0, 12.8, 34.6, 46.23, 57.53
//   도달 진공 [kPa abs]: 101.325, 80.325, 69.325, 44.325, 11.325
// 400 kPa 구동에서 11.325 kPa abs = 기존 ejector_p_limit 상수와 정확히 일치한다.
struct EjectorCurve {
  static double interp(double drive_kpa_gauge, const double (&y)[5])
  {
    const double x[5] = {0.0, 100.0, 200.0, 300.0, 400.0};
    const double q = std::clamp(drive_kpa_gauge, x[0], x[4]);
    int i = 0;
    while (i < 3 && x[i + 1] < q) ++i;
    const double f = (q - x[i]) / (x[i + 1] - x[i]);
    return y[i] + f * (y[i + 1] - y[i]);
  }
  static double suction_lpm(double drive_kpa_gauge)
  { static const double y[5] = {0.0, 31.0, 64.0, 90.0, 107.0};        return interp(drive_kpa_gauge, y); }
  static double consume_lpm(double drive_kpa_gauge)
  { static const double y[5] = {0.0, 12.8, 34.6, 46.23, 57.53};       return interp(drive_kpa_gauge, y); }
  static double reachable_kpa_abs(double drive_kpa_gauge)
  { static const double y[5] = {101.325, 80.325, 69.325, 44.325, 11.325}; return interp(drive_kpa_gauge, y); }
};

}  // namespace pneu
