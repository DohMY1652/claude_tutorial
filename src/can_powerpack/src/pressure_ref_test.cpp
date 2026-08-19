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

#include <cstdio>
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

  std::printf("\n");
  return 0;
}
