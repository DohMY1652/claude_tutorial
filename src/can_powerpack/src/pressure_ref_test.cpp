// ============================================================================
// pressure_ref_test — PressureRefGen 단독 검증
// ============================================================================
// ROS 없이 생성기만 돌려 다음을 확인한다:
//   1. 펌프 능력경계가 해설서 그림 B 의 두 점을 재현하는지
//      (음압 −90 kPa 에서 양압 상한 335 kPa, −80 에서 745 kPa)
//   2. 챔버압이 레일압에 닿으면 슬루 상한이 0 으로 수렴하는지 (레일 초과 불가)
//   3. 도달 불가능한 목표 힘에도 해를 반환하는지 (소프트 추종)
//   4. 목표 힘 0 이면 두 챔버가 대기압 방향으로 가는지
// ============================================================================

#include "PressureRefGen.hpp"
#include "PneumaticFlow.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using pneu::P_ATM;

static PressureRefGen::Params make_params(int N)
{
  PressureRefGen::Params p;
  p.N  = N;
  p.dt = 0.02;
  p.Apos.assign(N, 1.9635e-3);      // Ø50 mm 피스톤
  p.Aneg.assign(N, 1.9635e-3);
  p.Pch_pos_max =  83.675e3;        // 185 kPa abs (보수적 정격)
  p.Pch_neg_min = -74.325e3;        // 27 kPa abs
  p.Pneg_cap_deep = -74.325e3;
  p.set_orifices(2.3, 4.0, 1.6, 4.0, 4.0, 4.0);
  return p;
}

// 축 하나 상태를 만든다 (부피는 릴 25 mm 기하의 대표값)
static PressureRefGen::AxisState axis_at(double Ppos_g, double Pneg_g)
{
  PressureRefGen::AxisState a;
  a.P_pos = Ppos_g; a.P_neg = Pneg_g;
  a.V_pos = 128.5e-6;   // 0° 에서 128.5 mL
  a.V_neg = 226.7e-6;
  return a;
}

int main()
{
  std::printf("\n=== 1. 펌프 능력경계 (해설서 그림 B 대조) ===\n");
  {
    auto p = make_params(1);
    p.Ppos_sp_max = 1000e3;      // 검증용으로 넓게 스윕
    p.Pneg_cap_deep = -95e3;
    PressureRefGen gen(p);
    gen.build_pump_table();

    // 능력경계를 직접 스윕 (테이블 격자와 무관하게, 0.02 g/s 기준은 원본과 동일)
    struct { double pneg_kpa, expect_kpa; } pts[] = {{-90.0, 335.0}, {-80.0, 745.0}};
    for (auto& pt : pts) {
      double pmax = 0.0;
      for (double pp = 0.0; pp <= 1200e3; pp += 5e3) {
        double mo = 0.0, mi = 0.0;
        gen.pump_piston_avg(pp + P_ATM, pt.pneg_kpa * 1e3 + P_ATM, mo, mi);
        if (mo * 1e3 > 0.02) pmax = pp;
      }
      std::printf("  음압 %+.0f kPa → 양압 상한 %6.1f kPa  (해설서 %.0f kPa)\n",
                  pt.pneg_kpa, pmax / 1e3, pt.expect_kpa);
    }
    // 펌프 공급량 몇 점
    for (double pp : {0.0, 100e3, 300e3}) {
      double mo = 0.0, mi = 0.0;
      gen.pump_piston_avg(pp + P_ATM, -60e3 + P_ATM, mo, mi);
      std::printf("  P⁺=%5.0f kPa, P⁻=-60 kPa → 토출 %.4f g/s, 흡입 %.4f g/s\n",
                  pp / 1e3, mo * 1e3, mi * 1e3);
    }
    // 보수적 정격(−74.325 kPa)에서의 능력경계
    for (double pn : {-74.325, -60.0, -40.0, -30.0}) {
      double pmax = 0.0;
      for (double pp = 0.0; pp <= 1200e3; pp += 5e3) {
        double mo = 0.0, mi = 0.0;
        gen.pump_piston_avg(pp + P_ATM, pn * 1e3 + P_ATM, mo, mi);
        if (mo * 1e3 > 0.02) pmax = pp;
      }
      std::printf("  [실제 정격 범위] 음압 %+7.2f kPa → 양압 상한 %7.1f kPa\n", pn, pmax / 1e3);
    }
  }

  std::printf("\n=== 1b. 레일 셋포인트 배분 (실제 보수적 정격) ===\n");
  {
    auto p = make_params(1);            // Pneg_cap_deep = -74.325 kPa, Ppos_sp_max = 400 kPa
    PressureRefGen gen(p);
    gen.build_pump_table();
    PressureRefGen::SupplyState sup;
    sup.P_rail_pos = 50e3; sup.P_rail_neg = -40e3; sup.P_tank = 700e3;
    sup.P_ej = -60e3; sup.use_ej_meas = true;
    std::vector<PressureRefGen::AxisState> axes{ axis_at(0.0, 0.0) };
    for (double Fd : {0.0, 30.0, 75.0, 150.0, 300.0}) {
      auto r = gen.step({Fd}, axes, sup);
      std::printf("  수요 %5.0f N → d̄=%.2f, 레일 SP 양압 %6.1f kPa / 음압 %+7.2f kPa"
                  "  (능력경계 %6.1f kPa)\n",
                  Fd, r.demand_norm, r.rail_pos_sp / 1e3, r.rail_neg_sp / 1e3,
                  gen.cap_ppos(r.rail_neg_sp) / 1e3);
    }
  }

  std::printf("\n=== 2. 슬루 박스: 챔버가 레일에 닿으면 상한이 닫힌다 ===\n");
  {
    auto p = make_params(1);
    PressureRefGen gen(p);
    gen.build_pump_table();
    PressureRefGen::SupplyState sup;
    sup.P_rail_neg = -40e3; sup.P_ej = -60e3; sup.use_ej_meas = true;
    sup.P_tank = 0.0;                       // 탱크 부스트 차단 → 레일만
    sup.P_rail_pos = 60e3;

    for (double Pp : {0.0, 30e3, 55e3, 59.5e3, 60e3}) {
      std::vector<PressureRefGen::AxisState> axes{ axis_at(Pp, 0.0) };
      auto r = gen.step({120.0}, axes, sup);   // 큰 목표로 상한까지 밀어본다
      std::printf("  P⁺=%5.1f kPa (레일 60.0) → ub⁺=%6.2f kPa, 여유 %6.3f kPa\n",
                  Pp / 1e3, r.ub_pos[0] / 1e3, (r.ub_pos[0] - Pp) / 1e3);
    }
  }

  std::printf("\n=== 3. 도달 불가능한 목표에도 해를 반환하는가 (소프트 추종) ===\n");
  {
    auto p = make_params(1);
    PressureRefGen gen(p);
    gen.build_pump_table();
    PressureRefGen::SupplyState sup;
    sup.P_rail_pos = 80e3; sup.P_rail_neg = -70e3; sup.P_tank = 700e3;
    sup.P_ej = -70e3; sup.use_ej_meas = true;
    std::vector<PressureRefGen::AxisState> axes{ axis_at(0.0, 0.0) };

    const double Fmax = p.Pch_pos_max * p.Apos[0] + std::abs(p.Pch_neg_min) * p.Aneg[0];
    std::printf("  정격 최대 힘 = %.1f N\n", Fmax);
    for (double Fd : {0.0, 50.0, Fmax, 1000.0}) {
      auto r = gen.step({Fd}, axes, sup);
      std::printf("  목표 %7.1f N → 달성 %6.2f N, P⁺=%6.2f P⁻=%+7.2f kPa, "
                  "solver_ok=%d iters=%d\n",
                  Fd, r.F_achieved[0], r.P_pos_ref[0] / 1e3, r.P_neg_ref[0] / 1e3,
                  (int)r.solver_ok, r.sqp_iters);
    }
  }

  std::printf("\n=== 4. 목표 0 이면 대기압 방향으로 ===\n");
  {
    auto p = make_params(1);
    PressureRefGen gen(p);
    gen.build_pump_table();
    PressureRefGen::SupplyState sup;
    sup.P_rail_pos = 80e3; sup.P_rail_neg = -70e3; sup.P_tank = 700e3;
    sup.P_ej = -70e3; sup.use_ej_meas = true;
    // 이미 압력이 걸린 상태에서 목표 0
    std::vector<PressureRefGen::AxisState> axes{ axis_at(60e3, -50e3) };
    auto r = gen.step({0.0}, axes, sup);
    std::printf("  현재 P⁺=60.0 P⁻=-50.0 → 목표 P⁺=%6.2f P⁻=%+7.2f kPa (달성 %.2f N)\n",
                r.P_pos_ref[0] / 1e3, r.P_neg_ref[0] / 1e3, r.F_achieved[0]);
    std::printf("    P⁺ 는 한 스텝에 완전 배기 가능(0), P⁻ 는 완화 슬루가 %.1f kPa/스텝이라\n"
                "    한 번에 대기압까지 못 돌아간다 — 남은 힘은 슬루 한계 때문이고 정상이다.\n",
                (r.ub_neg[0] - (-50e3)) / 1e3);
    std::printf("  음수 목표 클램프: ");
    auto r2 = gen.step({-50.0}, axes, sup);
    std::printf("목표 −50 N → 달성 %.2f N (목표 0 과 같으면 클램프 정상)\n", r2.F_achieved[0]);
  }

  // ========================================================================
  // 6. **과도 응답 속도 — w_tank 가 탱크 부스트를 얼마나 막는가**
  // ========================================================================
  // 실기 20260829_224219 의 30°→60° 계단에서 지연의 정체를 재현한다.
  // 그때 실측: 목표·τ_ref·P⁺ref 는 0.0~0.13 s 에 반응했는데 **실측 압력이
  // 0.29~1.03 s** 걸렸다. 그 사이 macro(부스트) 지령은 내내 0% 였고 탱크는
  // 520 kPa 로 가득했으며 starve 도 0% 였다 — 능력이 없어서가 아니라
  // **목적함수가 탱크를 안 썼다.**
  //
  // 챔버가 레퍼런스를 향해 **물리적으로 가능한 최대 속도**(슬루 박스 경계)로
  // 따라간다고 두고, 요구 힘의 90% 에 도달하는 데 몇 스텝 걸리는지 센다.
  std::printf("\n=== 6. 과도 응답: w_tank 가 속도에 미치는 영향 ===\n");
  {
    // 실기 공급 상태 (20260829_224219 의 계단 순간)
    const double RAIL_P = 168e3 - P_ATM, RAIL_N = 55e3 - P_ATM, TANK = 520e3 - P_ATM;
    const double F_HOLD = 1.30 / 0.025;   // 30° 유지 토크 → 힘
    const double F_STEP = 3.40 / 0.025;   // 60° 유지 토크 → 힘

    std::printf("  공급: 레일⁺ %.0f / 레일⁻ %.0f / 탱크 %.0f kPa abs\n",
                RAIL_P/1e3+P_ATM/1e3, RAIL_N/1e3+P_ATM/1e3, TANK/1e3+P_ATM/1e3);
    std::printf("  힘 %.0f N → %.0f N 계단 (30°→60° 유지토크에 해당)\n\n", F_HOLD, F_STEP);
    std::printf("  %8s %10s %10s %10s %10s %10s\n",
                "w_tank", "90%도달", "P+ 이동", "P- 이동", "최종 P+", "최종 P-");

    for (double wt : {15.0, 8.0, 4.0, 2.0, 1.0}) {
      auto p = make_params(1);
      p.w_tank = wt;
      PressureRefGen gen(p);
      gen.build_pump_table();
      PressureRefGen::SupplyState sup;
      sup.P_rail_pos = RAIL_P; sup.P_rail_neg = RAIL_N; sup.P_tank = TANK;
      sup.P_ej = RAIL_N; sup.use_ej_meas = true;

      // 유지 상태로 먼저 수렴시킨다
      auto axes = std::vector<PressureRefGen::AxisState>{ axis_at(21e3, -3e3) };
      for (int i = 0; i < 150; ++i) {
        auto r = gen.step({F_HOLD}, axes, sup);
        axes[0].P_pos = std::min(std::max(r.P_pos_ref[0], r.lb_pos[0]), r.ub_pos[0]);
        axes[0].P_neg = std::min(std::max(r.P_neg_ref[0], r.lb_neg[0]), r.ub_neg[0]);
      }
      const double p0 = axes[0].P_pos, n0 = axes[0].P_neg;

      // 계단
      int n90 = -1;
      for (int i = 0; i < 200; ++i) {
        auto r = gen.step({F_STEP}, axes, sup);
        axes[0].P_pos = std::min(std::max(r.P_pos_ref[0], r.lb_pos[0]), r.ub_pos[0]);
        axes[0].P_neg = std::min(std::max(r.P_neg_ref[0], r.lb_neg[0]), r.ub_neg[0]);
        const double F = axes[0].P_pos * p.Apos[0] - axes[0].P_neg * p.Aneg[0];
        if (n90 < 0 && F >= F_HOLD + 0.9 * (F_STEP - F_HOLD)) n90 = i + 1;
      }
      std::printf("  %8.1f %9s %+9.1f %+9.1f %10.1f %10.1f\n", wt,
                  n90 < 0 ? "미도달" : (std::to_string((int)(n90 * p.dt * 1000)) + " ms").c_str(),
                  (axes[0].P_pos - p0)/1e3, (axes[0].P_neg - n0)/1e3,
                  axes[0].P_pos/1e3 + P_ATM/1e3, axes[0].P_neg/1e3 + P_ATM/1e3);
    }
    std::printf("\n  P+ 이동이 크고 P- 이동이 작을수록 빠른 쪽(탱크 580 kPa)을 쓴다는 뜻이다.\n");
  }

  std::printf("\n=== 5. 레퍼런스가 측정 잡음을 쫓지 않는가 (평활 기준점) ===");
  {
    // 목표 힘은 **고정**인데 실측 챔버압만 ±15 kPa, 6 Hz 로 흔들리는 상황.
    // 이때 레퍼런스가 같이 흔들리면 그 진동이 다시 챔버를 흔들어 되먹임 고리가 된다
    // (실기 계측: 목표각 45° 고정인데 레퍼런스가 5.71 Hz 로 진동했다).
    // MATLAB 원본은 평활항 기준점을 **직전 레퍼런스**로 둔다 (`ch_cur = ch;`).
    for (int mode = 0; mode < 2; ++mode) {
      auto p = make_params(1);
      p.smooth_anchor_ref = (mode == 1);
      PressureRefGen gen(p);
      gen.build_pump_table();
      PressureRefGen::SupplyState sup;
      sup.P_rail_pos = 250e3; sup.P_rail_neg = -74e3; sup.P_tank = 480e3;
      sup.P_ej = -70e3; sup.use_ej_meas = true;

      double mn = 1e30, mx = -1e30, sum = 0, prev = 0; int n = 0;
      for (int k = 0; k < 200; ++k) {
        const double osc = 15e3 * std::sin(2.0 * M_PI * 6.0 * (double)k * p.dt);
        std::vector<PressureRefGen::AxisState> axes{ axis_at(60e3 + osc, -5e3) };
        auto r = gen.step({150.0}, axes, sup);
        const double ref = r.P_pos_ref[0] / 1e3;
        if (k >= 100) {                       // 과도 지난 뒤만 본다
          mn = std::min(mn, ref); mx = std::max(mx, ref);
          if (n) sum += std::fabs(ref - prev);
          prev = ref; ++n;
        }
      }
      std::printf("  smooth_anchor_ref=%-5s → 레퍼런스 %7.2f~%7.2f kPa (폭 %6.2f), "
                  "틱간 변화 평균 %6.3f kPa\n",
                  p.smooth_anchor_ref ? "true" : "false", mn, mx, mx - mn,
                  n > 1 ? sum / (n - 1) : 0.0);
    }
    std::printf("    실측은 ±15 kPa 로 흔들리는데 목표 힘은 고정이다 — 폭이 작을수록 좋다.\n");
  }

  std::printf("\n");
  return 0;
}
