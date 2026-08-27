#pragma once

// ============================================================================
// PressureRefGen — 최적화 기반 압력 레퍼런스 생성기
// ============================================================================
// pressure_reference_optimizer.m 의 C++ 포팅.
//
//   MATLAB                        →  이 클래스
//   ─────────────────────────────────────────────────────────────────────
//   pressure_reference_init()      →  init()  (build_params + build_pump_table)
//   pressure_reference_step()      →  step()
//   decide_rail_setpoint()         →  decide_rail_setpoint()
//   optimize_channels()            →  optimize_channels()
//   obj_fun_ch()                   →  objective() / gradient()
//   update_sources()               →  compute_usage()   ← 적분 없음 (아래 참고)
//   valve_phys_kgps / orifice_phi  →  PneumaticFlow.hpp
//   pump_piston_avg / build_pump_table → 동일 이름
//
// **원본과 다른 점 (의도된 것)**
//  1. `update_sources()` 는 레일·탱크 압력을 적분하지만, 실기는 boards 1~4 로 네
//     공급원 압력을 전부 측정한다. 따라서 적분을 하지 않고 측정값을 그대로 받고,
//     원본의 수요 분해(레일 담당 / 탱크 부스트 초과분)만 진단용으로 계산한다.
//  2. 이젝터는 **구동 중일 때만** 측정값(board 4)을 하류 압력으로 쓴다 — 원본이
//     `sys.P_ej_meas` 로 권장한 경로다. 꺼져 있으면 측정값이 대기압이라 능력을 0 으로
//     오판하므로 특성곡선의 잠재력을 쓴다 (슬루 박스는 '열면 얼마나 흐르나'를 묻는
//     능력 판정이기 때문).
//  3. 챔버 부피는 원본의 고정값(0.75/0.4 L) 대신 각도-부피식 결과를 받는다.
//     원본 해설서 자신이 0.75 L 은 등가 스트로크 750 mm 라 비현실적이라고 표기했다.
//  4. fmincon(SQP) 대신 **박스 제약 SQP** — 매 반복 Gauss-Newton 이차모형을 만들어
//     기존 qpOASES 박스 QP 래퍼로 풀고, 실제 목적함수로 line search 한다.
//     목적함수가 비선형(J2)·비평활(J4 의 L1, Jtank/Jeject 의 hinge)이라 순수 QP 로는
//     표현되지 않기 때문이다.
//
// **단위**: 내부 전부 SI + **게이지 Pa** (원본과 동일). 힘은 N.
//   호출부(Controller)는 kPa 절대압을 쓰므로 경계에서 변환한다.
//   원본의 `tau` 는 실제로는 모멘트 암이 없는 **피스톤 힘 [N]** 이다
//   (tau = Ppos·Apos − Pneg·Aneg). 조인트 토크는 Controller 가 릴 반경으로 나눠 넘긴다.
// ============================================================================

#include <Eigen/Dense>

#include "PistonPump.hpp"

#include <memory>
#include <vector>

class QP;   // include/Controller.hpp 의 qpOASES 박스 QP 래퍼

class PressureRefGen {
public:
  // ── 파라미터 (MATLAB build_params 대응) ────────────────────────────────
  struct Params {
    int    N{1};                    // 축 수
    double dt{0.02};                // 생성기 주기 [s] (50 Hz)

    // 채널/액추에이터
    std::vector<double> Apos, Aneg;  // 챔버 유효면적 [m²]
    double Pch_pos_max{ 83.675e3};   // 양압 채널 상한 [Pa gauge]
    double Pch_neg_min{-74.325e3};   // 음압 채널 하한 [Pa gauge]

    // 폴리트로픽 지수
    double n_ch{1.4};                // 챔버 = 단열
    double n_rail{1.0};              // 레일/탱크 = 등온

    // 레일 셋포인트 배분
    int    Hpreview{100};            // 프리뷰 창 [스텝]. 1 이면 현재 수요만
    double Pneg_cap_deep{-74.325e3}; // 음압 최대 깊이 [Pa gauge]
    double Pneg_shallow{-30.0e3};    // 저수요 시 음압 셋포인트 [Pa gauge]
    // 평활항(J4)의 기준점을 직전 **레퍼런스**로 둘지(true, MATLAB 원본) 측정
    // 챔버압으로 둘지(false, 예전 포팅 동작). true 면 레퍼런스가 실측 잡음을
    // 쫓지 않는다 — PressureRefGen.cpp 의 xp 주석 참조.
    bool smooth_anchor_ref{true};
    double Ppos_sp_min{ 30.0e3};     // 양압레일 최소 셋포인트 [Pa gauge]
    double Ppos_sp_max{400.0e3};     // 양압레일 최대 셋포인트 [Pa gauge]
    double Fmax_ref{150.0};          // 수요 정규화 기준 힘 [N]

    // 압축탱크 (부스터)
    double P_tank_stop{450.0e3};     // 운전 하한 [Pa gauge] → tank_low 플래그

    // 이젝터 특성곡선 (측정값이 없을 때만 사용)
    double P_ej_max{-84.0e3};        // 무유량 최대 진공도 [Pa gauge]
    double Q_ej_max{100.0/60/1000};  // 정격 흡입유량 [m³/s]
    double eject_ratio{57.0/100.0};  // 탱크 소비/흡입 비

    // 목적함수 가중치 (MATLAB 값 그대로)
    double wtrack{100.0};
    double w_flow{0.3};
    double w_smooth{0.5};
    double w_tank{15.0};
    double w_eject{25.0};
    double A_max{0.284504};          // valve_capacity 상대단위 계수

    // 물리 오리피스 유효면적 [m²] (Cd·eta 포함)
    double Cd{0.8};
    // 한 스텝 평균 개도 계수 eta ∈ (0,1]. 슬루 박스는 "밸브를 열면 이번 스텝에 얼마나
    // 갈 수 있나"를 묻는데, 기하 오리피스 면적을 그대로 쓰면 **완전 개방**을 가정하게 된다.
    // 실제 비례밸브는 지령 100% 에서도 자기 최대 개도의 일부만 열리므로 그만큼 깎는다.
    // 근거와 값 선택은 config 의 valve_open_eta 주석 참조.
    double valve_open_eta{1.0};
    double A_fill{0}, A_vent{0}, A_boost{0};
    double A_suck{0}, A_admit{0}, A_eject{0};

    // 피스톤 펌프 (논문 식 1~3) — PistonPump.hpp 공용 모델
    pneu::PumpGeom pump;
    int    pump_grid_n{13};

    // SQP
    int    max_iter{12};
    double grad_tol{1e-9};

    void set_orifices(double d_fill_mm, double d_vent_mm, double d_boost_mm,
                      double d_suck_mm, double d_admit_mm, double d_eject_mm);
  };

  // ── 입력 ──────────────────────────────────────────────────────────────
  struct AxisState {
    double P_pos{0.0}, P_neg{0.0};       // 챔버 압력 [Pa gauge]
    double V_pos{1e-4}, V_neg{1e-4};     // 챔버 부피 [m³]
    double dVdt_pos{0.0}, dVdt_neg{0.0}; // 체적변화율 [m³/s]
  };
  struct SupplyState {
    double P_rail_pos{0.0};   // board 1 [Pa gauge]
    double P_rail_neg{0.0};   // board 2
    double P_tank{700e3};     // board 3
    double P_ej{0.0};         // board 4 — 측정 음압
    bool   use_ej_meas{true};
    // 이젝터가 지금 구동 중인지 (MacroSwitch 개방 여부).
    //
    // 슬루 박스는 "이 밸브를 열면 얼마나 흐를 수 있나"를 계산하는 **능력** 판정이다.
    // 이젝터가 꺼져 있으면 board 4 측정값은 대기압이라 그대로 쓰면 능력을 0 으로 오판한다.
    // → 구동 중이면 측정값(실제 성능·지연 반영)을, 꺼져 있으면 특성곡선의 도달 진공
    //   (잠재력)을 쓴다. macro 게이트는 이 값에 의존하지 않는다 (Result::starve_* 참조).
    bool   ej_running{false};
  };

  // ── 출력 ──────────────────────────────────────────────────────────────
  struct Result {
    std::vector<double> P_pos_ref, P_neg_ref;   // [Pa gauge]
    std::vector<double> F_achieved;             // [N]
    std::vector<double> lb_pos, ub_pos, lb_neg, ub_neg;   // 슬루 박스 [Pa gauge]
    double rail_pos_sp{0.0}, rail_neg_sp{0.0};  // [Pa gauge]
    double demand_norm{0.0};
    // 축별 **유량 부족률** [0,1] = (이번 스텝 요구 유량 − 레일이 낼 수 있는 유량) / 요구 유량.
    // 0 이면 레일만으로 충분하고, 1 이면 전량을 macro(탱크 부스트 / 이젝터)에 의존한다.
    //
    // 절대 유량 [kg/s] 대신 비율을 쓰는 이유: 부족분의 절대 크기는 챔버 부피·dt·압력
    // 스텝 크기에 모두 비례하므로 "0.0015 kg/s 가 큰가"를 사람이 판단할 수 없다.
    // 비율은 무차원이라 "레일이 수요의 몇 %를 못 대면 macro 를 부른다"로 바로 읽힌다.
    //
    // 또한 분자를 (요구 − 레일능력) 으로 두어 **이젝터 능력에 의존하지 않게** 했다.
    // 이젝터 유량을 분자에 넣으면 "이젝터가 꺼져 있으면 능력 0 → 부족분 0 → 게이트가
    // 안 열림 → 계속 꺼짐" 순환과, 켜진 뒤 측정값이 반영되며 게이트가 닫히는 채터링이
    // 생긴다. 부족률은 레일만 보므로 두 문제가 원천적으로 사라진다.
    std::vector<double> starve_pos, starve_neg;
    // 진단 (원본 update_sources 의 usage) — 절대 유량 합계 [kg/s]
    double m_fill{0.0}, m_boost{0.0}, m_suck{0.0}, m_eject{0.0}, m_tank_draw{0.0};
    bool   tank_low{false};
    int    sqp_iters{0};
    bool   solver_ok{true};
  };

  explicit PressureRefGen(const Params& p);

  // 펌프 능력 테이블 사전계산 (기동 시 1회, 수 초 소요)
  void build_pump_table();

  // 매 생성기 틱. F_ref [N] 는 축별 목표 힘 (≥ 0 으로 클램프된다).
  // F_preview 가 비어 있지 않으면 [axis][step] 로 프리뷰를 쓴다.
  Result step(const std::vector<double>& F_ref,
              const std::vector<AxisState>& axes,
              const SupplyState& sup,
              const std::vector<std::vector<double>>& F_preview = {});

  const Params& params() const { return p_; }
  const pneu::PumpTable& pump_table() const { return pump_; }

  // 펌프 1주기 평균 [kg/s]. 절대압 입력. (캘리브레이션/검증용)
  void pump_piston_avg(double Ppos_abs, double Pneg_abs,
                       double& mdot_out_avg, double& mdot_in_avg) const
  { pneu::pump_piston_avg(p_.pump, Ppos_abs, Pneg_abs, mdot_out_avg, mdot_in_avg); }
  double cap_ppos(double pneg_sp_gauge) const { return pump_.cap_ppos(pneg_sp_gauge); }

  // 실측 능력경계(pump_frontier_measured, Pa gauge)로 cap_ppos 테이블을 덮어쓴다.
  // build_pump_table() 이후에 불러야 한다. 빈 벡터면 아무것도 안 한다.
  void apply_measured_frontier(const std::vector<double>& pneg_gauge_pa,
                               const std::vector<double>& ppos_max_gauge_pa)
  { pump_.override_cap_measured(pneg_gauge_pa, ppos_max_gauge_pa); }

private:

  void decide_rail_setpoint(const std::vector<std::vector<double>>& F_preview,
                            double& ppos_sp, double& pneg_sp, double& demand_norm) const;

  // 이젝터 유량 [kg/s] (+ 도달 진공). 측정값 우선, 없으면 특성곡선 이분법.
  double ejector_flow(double Pch_gauge, const SupplyState& sup, double* P_ej_out = nullptr) const;

  void build_slew_box(const std::vector<AxisState>& axes, const SupplyState& sup,
                      Eigen::VectorXd& lb, Eigen::VectorXd& ub,
                      Eigen::VectorXd& dP_rail_max, Eigen::VectorXd& dN_rail_max) const;

  double objective(const Eigen::VectorXd& x, const Eigen::VectorXd& x_prev,
                   const std::vector<double>& F_ref, const SupplyState& sup,
                   const std::vector<AxisState>& axes,
                   const Eigen::VectorXd& dP_rail_max, const Eigen::VectorXd& dN_rail_max) const;

  Params p_;
  pneu::PumpTable pump_;

  std::shared_ptr<QP> qp_;
  Eigen::VectorXd x_prev_;      // 직전 해 (평활항·warm start)
  bool has_prev_{false};
};
