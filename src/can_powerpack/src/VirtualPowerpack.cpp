#include "VirtualPowerpack.hpp"

#include <algorithm>
#include <cmath>
#include <string>

using namespace std::chrono_literals;

// nested yaml 파라미터를 flat 이름으로 읽는다 (Controller.cpp 의 get_param_or 와 동일 패턴)
template <typename T>
static T gp(rclcpp::Node* n, const std::string& key, const T& defv) {
  if (!n->has_parameter(key)) {
    try { return n->declare_parameter<T>(key, defv); }
    catch (...) { /* 타입 불일치 — 아래에서 읽는다 */ }
  }
  T out = defv;
  return n->get_parameter(key, out) ? out : defv;
}

// yaml 에 정수로 적힌 double 파라미터도 받아준다 (CanBridge 의 declare_double_flexible 과 동일).
// declare_parameter<double>() 은 타입 불일치 시 파라미터를 등록한 채로 예외를 던지므로
// int 로 재선언하면 ParameterAlreadyDeclared 가 된다 → get_parameter 로 실제 타입에 맞춰 읽는다.
static double gpd(rclcpp::Node* n, const std::string& key, double defv) {
  if (!n->has_parameter(key)) {
    try { n->declare_parameter<double>(key, defv); }
    catch (...) { /* yaml 값이 int — 아래에서 정수로 읽는다 */ }
  }
  rclcpp::Parameter p;
  if (n->get_parameter(key, p)) {
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)  return p.as_double();
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) return static_cast<double>(p.as_int());
  }
  return defv;
}

// ============================================================================
// 생성자
// ============================================================================
VirtualPowerpack::VirtualPowerpack(const rclcpp::NodeOptions& opts)
: rclcpp::Node("virtual_powerpack", opts)
{
  // ── 채널 / 보드 구성 (pp_controller 와 반드시 동일해야 함) ──────────────
  num_total_channels_    = gp<int>(this, "num_total_channels",    12);
  num_positive_channels_ = gp<int>(this, "num_positive_channels",  6);
  num_actuators_         = gp<int>(this, "num_actuators",          1);
  channel_board_offset_  = gp<int>(this, "channel_board_offset",   5);
  period_ms_             = gp<int>(this, "period_ms",              2);
  sim_substeps_          = std::max(1, gp<int>(this, "Virtual.substeps", 10));

  P_pos_board_id_        = gp<int>(this, "line_pressure_boards.pos",       1);
  P_neg_board_id_        = gp<int>(this, "line_pressure_boards.neg",       2);
  P_macro_board_id_      = gp<int>(this, "line_pressure_boards.macro",     3);
  P_macro_neg_board_id_  = gp<int>(this, "line_pressure_boards.macro_neg", 4);
  line_pos_pwm_index_    = gp<int>(this, "LinePID.pos.pwm_index",  0);
  line_neg_pwm_index_    = gp<int>(this, "LinePID.neg.pwm_index",  3);
  macro_switch_pwm_index_= gp<int>(this, "MacroSwitch.pwm_index",  9);

  atm_offset_   = gpd(this, "Sensor_calibration.atm_offset", 101.325);
  P_atm_kpa_    = gpd(this, "Virtual.P_atm_kpa", atm_offset_);

  // ── 센서 캘리브레이션: pp_controller / CanBridge 와 같은 값을 써야 왕복이 맞는다 ──
  for (int bid = 1; bid <= NUM_BOARDS; ++bid) {
    const std::string base = "Sensor_calibration.boards." + std::to_string(bid);
    auto& c = calib_[(size_t)(bid - 1)];
    // 기본값: 압력 보드는 offset 1000mV / gain 0.25kPa/mV, 엔코더 보드는 1740mV / 0.0757deg/mV
    const bool is_encoder = (bid >= ANALOG_BOARD_START);
    c.offset = gpd(this, base + ".offset", is_encoder ? 1740.0 : 1000.0);
    c.gain   = gpd(this, base + ".gain",   is_encoder ? 0.07570 : 0.250);
  }
  // 엔코더는 CanBridge 처럼 raw 2점 실측(raw_0deg/raw_90deg)이 있으면 그것으로 재계산
  for (int bid = ANALOG_BOARD_START; bid <= NUM_BOARDS; ++bid) {
    const std::string base = "EncoderCalibration.boards." + std::to_string(bid);
    const double nan_v = std::numeric_limits<double>::quiet_NaN();
    const double raw_0  = gpd(this, base + ".raw_0deg",  nan_v);
    const double raw_90 = gpd(this, base + ".raw_90deg", nan_v);
    if (!std::isnan(raw_0) && !std::isnan(raw_90)) {
      auto to_orig_mv = [](double raw) {
        const double adc_mv = std::clamp(raw * (3300.0 / 4095.0), 0.0, 3300.0);
        return (4125.0 - adc_mv) / 0.825;
      };
      const double mv0 = to_orig_mv(raw_0), mv90 = to_orig_mv(raw_90);
      auto& c = calib_[(size_t)(bid - 1)];
      c.offset = mv0;
      c.gain   = (std::abs(mv90 - mv0) > 1e-9) ? 90.0 / (mv90 - mv0) : c.gain;
    }
  }

  // ── 공급원 / 라인 ────────────────────────────────────────────────────────
  // 펌프 기하 (기본값은 실기 제원 — 해설서 그림 B 의 능력경계를 재현한다)
  pump_geom_.delta  = gpd(this, "Virtual.pump.delta_m",      0.041);
  pump_geom_.r      = gpd(this, "Virtual.pump.crank_m",      0.02);
  pump_geom_.l      = gpd(this, "Virtual.pump.rod_m",        0.07);
  pump_geom_.Spis   = gpd(this, "Virtual.pump.piston_area_m2", 38.485e-4);
  pump_geom_.Cb_out = gpd(this, "Virtual.pump.cb_out_m2",    1.46e-6);
  pump_geom_.Cb_in  = gpd(this, "Virtual.pump.cb_in_m2",     33.47e-6);
  pump_geom_.omega  = gpd(this, "Virtual.pump.rpm", 3000.0) * 2.0 * M_PI / 60.0;
  pump_geom_.Npis   = gp<int>(this, "Virtual.pump.n_piston", 2);

  ej_floor_kpa_ = pneu::EjectorCurve::reachable_kpa_abs(
      gpd(this, "Virtual.tank.regulated_kpa_gauge", 700.0));

  // 압축탱크
  tank_volume_m3_      = gpd(this, "Virtual.tank.volume_ml",   213.0) * 1e-6;
  tank_charge_kpa_abs_ = gpd(this, "Virtual.tank.charge_kpa",  30000.0);
  tank_reg_kpa_abs_    = gpd(this, "Virtual.tank.regulated_kpa_gauge", 700.0) + 101.325;
  line_leak_pos_          = gpd(this, "Virtual.line_leak_pos_lpm_per_kpa",       0.002);
  line_leak_neg_          = gpd(this, "Virtual.line_leak_neg_lpm_per_kpa",       0.002);
  line_leak_macro_        = gpd(this, "Virtual.line_leak_macro_lpm_per_kpa",     0.002);
  line_leak_macro_neg_    = gpd(this, "Virtual.line_leak_macro_neg_lpm_per_kpa", 0.002);
  line_pos_min_kpa_       = gpd(this, "Virtual.line_pos_min_kpa",  50.0);
  line_pos_max_kpa_       = gpd(this, "Virtual.line_pos_max_kpa", 800.0);
  line_neg_min_kpa_       = gpd(this, "Virtual.line_neg_min_kpa",   5.0);
  line_neg_max_kpa_       = gpd(this, "Virtual.line_neg_max_kpa", 110.0);
  ejector_limit_kpa_      = gpd(this, "MPC_parameters.ejector_p_limit",  11.325);
  V_line_pos_ml_          = gpd(this, "Virtual.line_volume_pos_ml",       500.0);
  V_line_neg_ml_          = gpd(this, "Virtual.line_volume_neg_ml",       500.0);
  V_line_macro_ml_        = gpd(this, "Virtual.line_volume_macro_ml",    1000.0);
  V_line_macro_neg_ml_    = gpd(this, "Virtual.line_volume_macro_neg_ml", 500.0);

  sensor_lpf_enable_  = gp<bool>(this, "Virtual.sensor_lpf_enable", true);
  sensor_lpf_alpha_   = gpd(this, "Virtual.sensor_lpf_alpha",   0.2);
  current_mv_per_amp_ = gpd(this, "Virtual.current_mv_per_amp", 11000.0);

  default_volume_ml_  = gpd(this, "default_volume_ml",  1.0);
  actuator_connected_ = gp<bool>(this, "actuator_connected", true);

  // ── 채널별 밸브 파라미터 (pp_controller 의 channel_config.chN 과 같은 키) ──
  channel_valve_params_.resize(num_total_channels_);
  channel_tank_ml_.resize(num_total_channels_);
  const double tank_pos_ml = gpd(this, "tank_volume_pos_ml", 50.0);
  const double tank_neg_ml = gpd(this, "tank_volume_neg_ml", 50.0);

  for (int gid = 0; gid < num_total_channels_; ++gid) {
    const std::string pre = "channel_config.ch" + std::to_string(gid) + ".";
    auto& vp = channel_valve_params_[(size_t)gid];
    vp.I_MAX       = gpd(this, pre + "I_MAX",       0.30);
    vp.A_max       = gpd(this, pre + "A_max",       0.2845);
    vp.k_shape     = gpd(this, pre + "k_shape",     33.09);
    vp.C_k         = gpd(this, pre + "C_k",         0.0288);
    vp.C_p         = gpd(this, pre + "C_p",         0.00012);
    vp.C_z         = gpd(this, pre + "C_z",         0.0);
    vp.A_bw        = gpd(this, pre + "A_bw",        260649.5);
    vp.beta_bw     = gpd(this, pre + "beta_bw",     179.0);
    vp.gamma_bw    = gpd(this, pre + "gamma_bw",    0.06);
    vp.alpha_shape = gpd(this, pre + "alpha_shape", 3884.2);
    vp.wn_up       = gpd(this, pre + "wn_up",       40.0);
    vp.zeta_up     = gpd(this, pre + "zeta_up",     1.2);
    vp.wn_down     = gpd(this, pre + "wn_down",     45.0);
    vp.zeta_down   = gpd(this, pre + "zeta_down",   1.0);

    channel_tank_ml_[(size_t)gid] = (gid < num_positive_channels_) ? tank_pos_ml : tank_neg_ml;
  }

  // 라인 밸브 (board 1/2/4) — 채널 밸브와 다른 사양이면 Virtual.line_valve.* 로 조정
  {
    const std::string pre = "Virtual.line_valve.";
    auto& vp = line_valve_params_;
    vp.I_MAX       = gpd(this, pre + "I_MAX",       0.30);
    vp.A_max       = gpd(this, pre + "A_max",       0.2845);
    vp.k_shape     = gpd(this, pre + "k_shape",     33.09);
    vp.C_k         = gpd(this, pre + "C_k",         0.0288);
    vp.C_p         = gpd(this, pre + "C_p",         0.00012);
    vp.C_z         = gpd(this, pre + "C_z",         0.0);
    vp.A_bw        = gpd(this, pre + "A_bw",        260649.5);
    vp.beta_bw     = gpd(this, pre + "beta_bw",     179.0);
    vp.gamma_bw    = gpd(this, pre + "gamma_bw",    0.06);
    vp.alpha_shape = gpd(this, pre + "alpha_shape", 3884.2);
    vp.wn_up       = gpd(this, pre + "wn_up",       40.0);
    vp.zeta_up     = gpd(this, pre + "zeta_up",     1.2);
    vp.wn_down     = gpd(this, pre + "wn_down",     45.0);
    vp.zeta_down   = gpd(this, pre + "zeta_down",   1.0);
  }

  // ── 액추에이터 ──────────────────────────────────────────────────────────
  actuators_.resize((size_t)num_actuators_);
  for (int a = 0; a < num_actuators_; ++a) {
    const std::string pre = "Virtual.actuator" + std::to_string(a) + ".";
    // 채널 매핑은 pp_controller 의 PositionController.axisN 과 같은 기본값
    const std::string axis = "PositionController.axis" + std::to_string(a) + ".";
    auto& ac = actuators_[(size_t)a];
    ac.pos_gid = gp<int>(this, axis + "pos_gid", a);
    ac.neg_gid = gp<int>(this, axis + "neg_gid", num_positive_channels_ + a);

    const double dia_mm = gpd(this, "Geometry.piston_dia_mm",
                              2.0 * gpd(this, pre + "piston_radius_mm", 25.0));
    ac.piston_area_mm2 = M_PI * dia_mm * dia_mm / 4.0;
    ac.reel_radius_mm  = gpd(this, "Geometry.reel_radius_mm", 25.0);
    ac.pos_offset_mm   = gpd(this, pre + "pos_offset_mm",  40.0);
    ac.neg_offset_mm   = gpd(this, pre + "neg_offset_mm",  90.0);
    ac.tank_pos_ml     = tank_pos_ml;
    ac.tank_neg_ml     = tank_neg_ml;

    ac.torque_arm_m    = gpd(this, pre + "torque_arm_m",   0.07);
    ac.inertia         = std::max(1e-6, gpd(this, pre + "inertia_kgm2", 0.05));
    ac.damping         = gpd(this, pre + "damping_nms",    0.30);
    ac.coulomb_nm      = gpd(this, pre + "coulomb_nm",     0.50);
    // 부하 질량 / 링크 길이는 pp_controller 의 중력 FF 와 같은 값을 기본값으로
    ac.mass_kg         = gpd(this, axis + "mass_kg",       5.0);
    ac.link_length_m   = gpd(this, axis + "link_length_m", 0.2);
    ac.theta_min_deg   = gpd(this, pre + "theta_min_deg",  0.0);
    ac.theta_max_deg   = gpd(this, pre + "theta_max_deg",  105.0);
    ac.theta_deg       = gpd(this, pre + "theta_init_deg", ac.theta_min_deg);
    ac.omega_dps       = 0.0;
  }

  // ── 초기 상태 ────────────────────────────────────────────────────────────
  P_line_pos_kpa_       = gpd(this, "Virtual.init_line_pos_kpa",       P_atm_kpa_);
  P_line_neg_kpa_       = gpd(this, "Virtual.init_line_neg_kpa",       P_atm_kpa_);
  P_line_macro_kpa_     = gpd(this, "Virtual.init_line_macro_kpa",     P_atm_kpa_);
  P_line_macro_neg_kpa_ = gpd(this, "Virtual.init_line_macro_neg_kpa", P_atm_kpa_);

  // 압축탱크 초기 질량: 상류 30 MPa × 213 mL (≈ 76 g ≈ 표준 63 L)
  tank_mass_kg_ = tank_charge_kpa_abs_ * 1000.0 * tank_volume_m3_ / (pneu::R_AIR * pneu::T_CH);
  P_line_macro_kpa_ = std::min(tank_reg_kpa_abs_,
      tank_mass_kg_ * pneu::R_AIR * pneu::T_CH / std::max(1e-12, tank_volume_m3_) / 1000.0);

  // 펌프 능력 테이블 (기동 시 1회). 격자 범위는 라인 압력 가드와 맞춘다.
  {
    const auto t0 = std::chrono::steady_clock::now();
    pump_table_.build(pump_geom_,
                      (line_pos_max_kpa_ - P_atm_kpa_) * 1000.0,   // 양압 상한 (게이지 Pa)
                      (line_neg_min_kpa_ - P_atm_kpa_) * 1000.0,   // 음압 최대 깊이
                      -30.0e3, 13, 21);
    RCLCPP_INFO(get_logger(), "펌프 능력 테이블 완료 (%.2f s)",
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
  }

  P_ch_kpa_.assign((size_t)num_total_channels_, P_atm_kpa_);
  V_ch_m3_.assign((size_t)num_total_channels_, default_volume_ml_ * 1e-6);
  Vdot_ch_m3ps_.assign((size_t)num_total_channels_, 0.0);
  // 각도 0 에서의 부피로 초기화
  integrate(std::array<double, PWM_TOTAL>{}, 0.0);

  pwm_cmd_.fill(0.0);

  // ── 토픽: CanBridge 와 동일한 이름 / 타입 / 크기 ─────────────────────────
  pub_sensors_  = create_publisher<std_msgs::msg::UInt16MultiArray>("board/sensors",  10);
  pub_currents_ = create_publisher<std_msgs::msg::Float64MultiArray>("board/currents", 10);
  pub_analog_   = create_publisher<std_msgs::msg::Float64MultiArray>("board/analog",   10);
  sub_pwm_cmd_  = create_subscription<std_msgs::msg::UInt16MultiArray>(
      "board/pwm_cmd", rclcpp::QoS(10),
      std::bind(&VirtualPowerpack::on_pwm_cmd, this, std::placeholders::_1));

  sim_timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(1, period_ms_)),
      std::bind(&VirtualPowerpack::sim_step, this));

  RCLCPP_INFO(get_logger(),
    "=== VirtualPowerpack (no CAN hardware) ===\n"
    "  channels : %d total (%d pos + %d neg), board offset %d → boards %d~%d\n"
    "  actuators: %d (encoder boards %d~%d), connected=%s\n"
    "  bleeds   : board%d v1 → 대기 방출,  board%d v1 ← 대기 유입  (둘 다 정극성)\n"
    "  pump     : 슬라이더-크랭크 %.0f rpm, 피스톤 %d개 (능력경계 테이블 사용)\n"
    "  tank     : %.0f mL @ %.0f kPa → 레귤레이터 %.1f kPa (저장 %.1f g ≈ 표준 %.1f L)\n"
    "  timing   : %d ms tick × %d substeps",
    num_total_channels_, num_positive_channels_,
    num_total_channels_ - num_positive_channels_, channel_board_offset_,
    channel_board_offset_, channel_board_offset_ + num_total_channels_ - 1,
    num_actuators_, ANALOG_BOARD_START, ANALOG_BOARD_START + num_actuators_ - 1,
    actuator_connected_ ? "true" : "false",
    P_pos_board_id_, P_neg_board_id_,
    pump_geom_.omega * 60.0 / (2.0 * M_PI), pump_geom_.Npis,
    tank_volume_m3_ * 1e6, tank_charge_kpa_abs_, tank_reg_kpa_abs_,
    tank_mass_kg_ * 1e3, tank_mass_kg_ / 1.204 * 1e3,
    period_ms_, sim_substeps_);
}

// ============================================================================
// 압축성 유동 함수 Phi(P_in, P_out) — Controller 의 get_phi_local 과 동일
// ============================================================================
double VirtualPowerpack::get_phi(double P_in, double P_out)
{
  if (P_in < 1e-9 || P_out >= P_in) return 0.0;
  const double Pr   = std::clamp(P_out / P_in, 0.0, 1.0);
  const double P_cr = std::pow(2.0 / (KAPPA + 1.0), KAPPA / (KAPPA - 1.0));
  if (Pr <= P_cr)   // 초킹(choked) 유동
    return std::sqrt(KAPPA * std::pow(2.0 / (KAPPA + 1.0), (KAPPA + 1.0) / (KAPPA - 1.0)));
  return std::sqrt(2.0 * KAPPA / (KAPPA - 1.0)) * std::sqrt(std::max(0.0,
      std::pow(Pr, 2.0 / KAPPA) - std::pow(Pr, (KAPPA + 1.0) / KAPPA)));
}

// ============================================================================
// 13-variable 비례밸브 1스텝 → 유량 [LPM]
//   Bouc-Wen 히스테리시스 → 시그모이드 유효면적 → 압축성 유동 → 2차 동역학
// ============================================================================
double VirtualPowerpack::step_valve(double u_pct, double P_in, double P_out,
                                    const ValveParams& vp, ValveState& vs,
                                    double dt_sub) const
{
  u_pct = std::clamp(u_pct, 0.0, 100.0);
  const double I = u_pct / 100.0 * vp.I_MAX;

  // Bouc-Wen 히스테리시스
  const double dI     = I - vs.prev_I;
  const double abs_dI = std::abs(dI);
  vs.z = std::clamp(vs.z + vp.A_bw * dI
                          - vp.beta_bw  * abs_dI * vs.z
                          - vp.gamma_bw * dI * std::abs(vs.z), -1e6, 1e6);
  if      (dI >  1e-4) vs.dir = 1;
  else if (dI < -1e-4) vs.dir = 0;
  vs.prev_I = I;

  // 유효면적: A_max · sigmoid(k_shape · F_net)^alpha
  const double F_net    = std::clamp(I + vp.C_z * vs.z + vp.C_p * P_in - vp.C_k, -500.0, 500.0);
  const double sigma    = 1.0 / (1.0 + std::exp(-vp.k_shape * F_net));
  const double Area_eff = vp.A_max * std::pow(sigma, vp.alpha_shape);

  // 정적 유량 [LPM]
  const double Q_static = Area_eff * P_in * get_phi(P_in, P_out);

  // 2차 밸브 동역학 (오일러 1 서브스텝)
  const double wn   = (vs.dir == 1) ? vp.wn_up   : vp.wn_down;
  const double zeta = (vs.dir == 1) ? vp.zeta_up : vp.zeta_down;
  const double dx2  = wn * wn * (Q_static - vs.x1) - 2.0 * zeta * wn * vs.x2;
  vs.x1 += dt_sub * vs.x2;
  vs.x2 += dt_sub * dx2;

  return std::max(0.0, vs.x1);
}

// ============================================================================
// 등온 이상기체 챔버: dP/dt = (R·T·ṁ - P·V̇) / V
//   ṁ = Q[LPM] · LPM_TO_KGPS,  P/dP 는 kPa 단위
// ============================================================================
double VirtualPowerpack::pressure_derivative(double P_kpa, double net_flow_lpm,
                                             double volume_m3, double vol_dot_m3ps)
{
  const double V = std::max(1e-12, volume_m3);
  const double m_dot = net_flow_lpm * LPM_TO_KGPS;                  // [kg/s]
  return (RGAS_AIR * TEMP_K * m_dot / 1000.0 - P_kpa * vol_dot_m3ps) / V;  // [kPa/s]
}

// ============================================================================
// 물리 적분 1 서브스텝
// ============================================================================
void VirtualPowerpack::integrate(const std::array<double, PWM_TOTAL>& pwm, double dt_sub)
{
  auto u_of = [&](int flat_idx) -> double {
    if (flat_idx < 0 || flat_idx >= PWM_TOTAL) return 0.0;
    return std::clamp(pwm[(size_t)flat_idx] * PWM_TO_PCT, 0.0, 100.0);
  };
  auto valve_slot = [&](int board_id, int v_idx) -> int {
    return (board_id - 1) * PWM_PER_BOARD + v_idx;
  };

  // ── 1. 액추에이터 각도 → 챔버 부피 / 부피 변화율 ─────────────────────────
  // Controller::on_timer 의 각도-부피 식과 동일
  //   x     = reel_radius · θ_rad                                  [mm]
  //   V_pos = tank_pos + A·max(0, pos_offset + x)/1000              [mL]
  //   V_neg = tank_neg + A·max(0, neg_offset - x)/1000              [mL]
  for (int gid = 0; gid < num_total_channels_; ++gid) {
    V_ch_m3_[(size_t)gid]      = std::max(channel_tank_ml_[(size_t)gid], 1e-3) * 1e-6;
    Vdot_ch_m3ps_[(size_t)gid] = 0.0;
  }
  for (auto& ac : actuators_) {
    const double A  = ac.piston_area_mm2;
    const double x_mm  = ac.reel_radius_mm * ac.theta_deg * M_PI / 180.0;
    const double x_pos = ac.pos_offset_mm + x_mm;
    const double x_neg = ac.neg_offset_mm - x_mm;

    if (ac.pos_gid >= 0 && ac.pos_gid < num_total_channels_) {
      V_ch_m3_[(size_t)ac.pos_gid] = (ac.tank_pos_ml + A * std::max(0.0, x_pos) / 1000.0) * 1e-6;
      // dV/dt = A·reel·ω_rad [mm³/s] → m³/s (스트로크 리밋에 걸리면 0)
      Vdot_ch_m3ps_[(size_t)ac.pos_gid] =
          (x_pos > 0.0) ? A * ac.reel_radius_mm * ac.omega_dps * M_PI / 180.0 * 1e-9 : 0.0;
    }
    if (ac.neg_gid >= 0 && ac.neg_gid < num_total_channels_) {
      V_ch_m3_[(size_t)ac.neg_gid] = (ac.tank_neg_ml + A * std::max(0.0, x_neg) / 1000.0) * 1e-6;
      Vdot_ch_m3ps_[(size_t)ac.neg_gid] =
          (x_neg > 0.0) ? -A * ac.reel_radius_mm * ac.omega_dps * M_PI / 180.0 * 1e-9 : 0.0;
    }
  }
  if (!actuator_connected_) {
    // 액추에이터 미연결: 고정 탱크 부피만 (pp_controller 의 actuator_connected=false 와 대응)
    for (int gid = 0; gid < num_total_channels_; ++gid) {
      V_ch_m3_[(size_t)gid]      = std::max(channel_tank_ml_[(size_t)gid], 1e-3) * 1e-6;
      Vdot_ch_m3ps_[(size_t)gid] = 0.0;
    }
  }
  if (dt_sub <= 0.0) return;   // 초기화 호출: 부피만 세팅하고 종료

  // ── 2. 채널 밸브 유량 ────────────────────────────────────────────────────
  // v1=micro(공급) v2=atm(배기) v3=macro(대유량)
  std::vector<double> ch_net_flow((size_t)num_total_channels_, 0.0);
  double line_pos_draw = 0.0, line_macro_draw = 0.0;    // 양압 라인에서 빠져나가는 유량
  double line_neg_fill = 0.0, line_macro_neg_fill = 0.0; // 음압 라인으로 들어오는 유량

  for (int gid = 0; gid < num_total_channels_; ++gid) {
    const int bid = gid + channel_board_offset_;
    if (bid < 1 || bid > PWM_BOARDS) continue;

    const auto& vp   = channel_valve_params_[(size_t)gid];
    const double P_now = P_ch_kpa_[(size_t)gid];
    const int s_mi = valve_slot(bid, 0), s_at = valve_slot(bid, 1), s_ma = valve_slot(bid, 2);
    const double u_mi = u_of(s_mi), u_at = u_of(s_at), u_ma = u_of(s_ma);

    double f_mi, f_at, f_ma;
    if (gid < num_positive_channels_) {
      // 양압: 라인 → 챔버 (micro/macro), 챔버 → 대기 (atm)
      f_mi = step_valve(u_mi, P_line_pos_kpa_,   P_now,      vp, valves_[(size_t)s_mi], dt_sub);
      f_ma = step_valve(u_ma, P_line_macro_kpa_, P_now,      vp, valves_[(size_t)s_ma], dt_sub);
      f_at = step_valve(u_at, P_now,             P_atm_kpa_, vp, valves_[(size_t)s_at], dt_sub);
      ch_net_flow[(size_t)gid] = f_mi + f_ma - f_at;
      line_pos_draw   += f_mi;
      line_macro_draw += f_ma;
    } else {
      // 음압: 대기 → 챔버 (atm), 챔버 → 진공 라인 (micro/macro=에젝터)
      f_at = step_valve(u_at, P_atm_kpa_, P_now,                 vp, valves_[(size_t)s_at], dt_sub);
      f_mi = step_valve(u_mi, P_now,      P_line_neg_kpa_,       vp, valves_[(size_t)s_mi], dt_sub);
      f_ma = step_valve(u_ma, P_now,      P_line_macro_neg_kpa_, vp, valves_[(size_t)s_ma], dt_sub);
      ch_net_flow[(size_t)gid] = f_at - f_mi - f_ma;
      line_neg_fill       += f_mi;
      line_macro_neg_fill += f_ma;
    }

    valve_flow_lpm_[(size_t)s_mi] = f_mi;
    valve_flow_lpm_[(size_t)s_at] = f_at;
    valve_flow_lpm_[(size_t)s_ma] = f_ma;
    valve_u_pct_[(size_t)s_mi] = u_mi;
    valve_u_pct_[(size_t)s_at] = u_at;
    valve_u_pct_[(size_t)s_ma] = u_ma;
  }

  // ── 3. micro 라인 — 펌프 한 대로 이어진 닫힌 회로 ────────────────────────
  // 흡입구 = 음압 라인, 토출구 = 양압 라인. 대기와 통하는 곳은 board1(출구)·board2(입구)뿐.
  // 두 밸브 모두 정극성이므로 PWM 을 그대로 개도로 쓴다 — LinePID 의 (100−u) 반전은
  // "누출 밸브라서 부족하면 닫는다"는 뜻이고, 여기서 되돌리지 않는다.
  auto slot_or_zero = [&](int s) {
    return (size_t)std::clamp(s, 0, PWM_BOARDS * PWM_PER_BOARD - 1);
  };

  // 피스톤 펌프: 기동 시 만든 2D 능력 테이블을 보간한다 (kg/s → 이 파일의 LPM 단위로).
  // 흡입량 = 토출량이므로 같은 항이 양압에 +, 음압에 − 로 들어간다 (질량보존).
  const double m_pump = pump_table_.flow_out(P_line_pos_kpa_ * 1000.0, P_line_neg_kpa_ * 1000.0);
  const double Q_pump = m_pump / LPM_TO_KGPS;

  {
    // board 1 v1 — 양압 라인 → 대기 방출 (회로의 유일한 출구)
    const int s = line_pos_pwm_index_;
    const double u = u_of(s);
    const double f_vent = step_valve(u, P_line_pos_kpa_, P_atm_kpa_,
                                     line_valve_params_, valves_[slot_or_zero(s)], dt_sub);
    const double f_leak = line_leak_pos_ * std::max(0.0, P_line_pos_kpa_ - P_atm_kpa_);
    P_line_pos_kpa_ += dt_sub * pressure_derivative(
        P_line_pos_kpa_, Q_pump - f_vent - f_leak - line_pos_draw, V_line_pos_ml_ * 1e-6, 0.0);
    P_line_pos_kpa_ = std::clamp(P_line_pos_kpa_, line_pos_min_kpa_, line_pos_max_kpa_);
    valve_flow_lpm_[slot_or_zero(s)] = f_vent;
    valve_u_pct_[slot_or_zero(s)]    = u;
  }
  {
    // board 2 v1 — 대기 → 음압 라인 유입 (회로의 유일한 입구)
    const int s = line_neg_pwm_index_;
    const double u = u_of(s);
    const double f_admit = step_valve(u, P_atm_kpa_, P_line_neg_kpa_,
                                      line_valve_params_, valves_[slot_or_zero(s)], dt_sub);
    const double f_leak = line_leak_neg_ * std::max(0.0, P_atm_kpa_ - P_line_neg_kpa_);
    P_line_neg_kpa_ += dt_sub * pressure_derivative(
        P_line_neg_kpa_, f_admit + f_leak + line_neg_fill - Q_pump, V_line_neg_ml_ * 1e-6, 0.0);
    P_line_neg_kpa_ = std::clamp(P_line_neg_kpa_, line_neg_min_kpa_, line_neg_max_kpa_);
    valve_flow_lpm_[slot_or_zero(s)] = f_admit;
    valve_u_pct_[slot_or_zero(s)]    = u;
  }
  // ── 3b. 압축탱크 (board 3) — 회복 없는 소진형 자원 ───────────────────────
  // 상류(30 MPa)가 남아 있는 동안 레귤레이터가 700 kPa gauge 를 유지하고, 다 쓰면
  // 상류 압력을 그대로 내보낸다. board 3 센서는 이 레귤레이터 출력을 읽는다.
  {
    const double P_up_kpa = tank_mass_kg_ * pneu::R_AIR * pneu::T_CH
                          / std::max(1e-12, tank_volume_m3_) / 1000.0;   // 상류 절대압
    P_line_macro_kpa_ = std::min(tank_reg_kpa_abs_, std::max(P_atm_kpa_, P_up_kpa));
    // 채널 부스트(macro 양압 밸브)가 뽑아간 만큼 탱크 질량이 줄어든다. 회복 없음.
    tank_mass_kg_ = std::max(0.0, tank_mass_kg_ - line_macro_draw * LPM_TO_KGPS * dt_sub);
  }

  // ── 3c. macro 음압 라인 (board 4) — 이젝터 특성곡선 ──────────────────────
  // MacroSwitch(board4 v1)가 열린 만큼 탱크 공기로 이젝터를 구동한다. 구동압에 따라
  // 흡입 유량·도달 진공·탱크 소비가 정해진다 (해설서 6.7절: 정압 싱크는 틀렸다).
  {
    const int s = macro_switch_pwm_index_;
    const double u_sw = u_of(s);   // Controller 가 4095=ON 으로 직접 보낸다 (반전 없음)
    const double drive_g = std::max(0.0, P_line_macro_kpa_ - P_atm_kpa_) * (u_sw / 100.0);
    ej_suction_lpm_   = pneu::EjectorCurve::suction_lpm(drive_g);
    ej_consume_lpm_   = pneu::EjectorCurve::consume_lpm(drive_g);
    ej_reach_kpa_abs_ = pneu::EjectorCurve::reachable_kpa_abs(drive_g);

    // 도달 진공까지의 오리피스 유량을 정격 흡입량으로 제한 (정압 싱크 가정의 과대평가 방지).
    // 카탈로그 흡입량은 표준 LPM 이므로 이 파일의 내부 유량 단위로 환산해서 비교한다.
    constexpr double STD2INT = pneu::STD_LPM_TO_KGPS / LPM_TO_KGPS;
    const double f_orif = step_valve(u_sw, P_line_macro_neg_kpa_, ej_reach_kpa_abs_,
                                     line_valve_params_, valves_[slot_or_zero(s)], dt_sub);
    const double f_suction = std::min(f_orif, ej_suction_lpm_ * STD2INT);
    const double f_leak = line_leak_macro_neg_ * std::max(0.0, P_atm_kpa_ - P_line_macro_neg_kpa_);
    P_line_macro_neg_kpa_ += dt_sub * pressure_derivative(
        P_line_macro_neg_kpa_, line_macro_neg_fill + f_leak - f_suction,
        V_line_macro_neg_ml_ * 1e-6, 0.0);
    // 하한은 **최대 구동 시** 도달 진공으로 고정한다. 순간 도달진공(ej_reach_kpa_abs_)을
    // 하한으로 쓰면 MacroSwitch 가 꺼지는 틱마다 그 값이 대기압이 되어 라인이 강제로
    // 대기압까지 되돌아간다 — 이젝터가 꺼져도 진공은 유지되어야 하고, 복귀는 누설과
    // 채널 유입으로만 일어나야 한다. (이 버그 때문에 board 4 가 계속 101.3 에 붙어 있었다)
    P_line_macro_neg_kpa_ = std::clamp(P_line_macro_neg_kpa_, ej_floor_kpa_, 110.0);

    // 이젝터 구동은 탱크 공기를 먹는다 — 음압을 쓰면 양압 자원도 줄어드는 결합.
    // 소비량도 카탈로그 표준 LPM 이므로 표준 환산 상수를 쓴다 (경험적 LPM_TO_KGPS 를
    // 쓰면 10.7배 빨리 말라 76 g 탱크가 6초에 바닥난다).
    tank_mass_kg_ = std::max(0.0,
        tank_mass_kg_ - ej_consume_lpm_ * pneu::STD_LPM_TO_KGPS * dt_sub);

    valve_flow_lpm_[slot_or_zero(s)] = f_suction;
    valve_u_pct_[slot_or_zero(s)]    = u_sw;
  }

  // ── 4. 챔버 압력 ────────────────────────────────────────────────────────
  for (int gid = 0; gid < num_total_channels_; ++gid) {
    double& P = P_ch_kpa_[(size_t)gid];
    P += dt_sub * pressure_derivative(P, ch_net_flow[(size_t)gid],
                                      V_ch_m3_[(size_t)gid], Vdot_ch_m3ps_[(size_t)gid]);
    const bool pos = (gid < num_positive_channels_);
    P = std::clamp(P, pos ? 50.0 : ejector_limit_kpa_, pos ? 800.0 : 110.0);
  }

  // ── 5. 액추에이터 회전 동역학 ───────────────────────────────────────────
  //   J·ω̇ = A_piston·r·(P_pos - P_neg) - m·g·L·sin θ - b·ω - τ_coulomb·sgn(ω)
  if (actuator_connected_) {
    for (auto& ac : actuators_) {
      const double P_pos = (ac.pos_gid >= 0 && ac.pos_gid < num_total_channels_)
                           ? P_ch_kpa_[(size_t)ac.pos_gid] : P_atm_kpa_;
      const double P_neg = (ac.neg_gid >= 0 && ac.neg_gid < num_total_channels_)
                           ? P_ch_kpa_[(size_t)ac.neg_gid] : P_atm_kpa_;

      const double A_m2 = ac.piston_area_mm2 * 1e-6;                  // [m²]
      const double tau_p = A_m2 * ac.torque_arm_m * (P_pos - P_neg) * 1000.0;  // kPa→Pa
      const double theta_rad = ac.theta_deg * M_PI / 180.0;
      const double tau_g = -ac.mass_kg * 9.81 * ac.link_length_m * std::sin(theta_rad);

      const double omega_rps = ac.omega_dps * M_PI / 180.0;            // [rad/s]
      const double tau_b = -ac.damping * omega_rps;
      double tau_c = 0.0;
      if (std::abs(omega_rps) > 1e-4) tau_c = -ac.coulomb_nm * (omega_rps > 0 ? 1.0 : -1.0);
      else {
        // 정지 마찰: 정지 상태에서 구동 토크가 쿨롱 마찰보다 작으면 움직이지 않음
        const double tau_drive = tau_p + tau_g;
        tau_c = -std::clamp(tau_drive, -ac.coulomb_nm, ac.coulomb_nm);
      }

      const double alpha_rps2 = (tau_p + tau_g + tau_b + tau_c) / ac.inertia;
      ac.omega_dps += dt_sub * alpha_rps2 * 180.0 / M_PI;
      ac.theta_deg += dt_sub * ac.omega_dps;

      // 기계적 스토퍼
      if (ac.theta_deg <= ac.theta_min_deg) {
        ac.theta_deg = ac.theta_min_deg;
        ac.omega_dps = std::max(0.0, ac.omega_dps);
      } else if (ac.theta_deg >= ac.theta_max_deg) {
        ac.theta_deg = ac.theta_max_deg;
        ac.omega_dps = std::min(0.0, ac.omega_dps);
      }
    }
  }
}

// ============================================================================
// 센서 인코딩
// ============================================================================
// kPa → board/sensors 의 값 [mV]. Controller 의 SensorCalib::kpa() 역함수.
//   kpa = (mv - offset)·gain + atm_offset   ⇒   mv = (kpa - atm_offset)/gain + offset
uint16_t VirtualPowerpack::kpa_to_raw_mv(int board_id, double kpa) const
{
  if (board_id < 1 || board_id > NUM_BOARDS) return 0;
  const auto& c = calib_[(size_t)(board_id - 1)];
  if (std::abs(c.gain) < 1e-12) return 0;
  const double mv = (kpa - atm_offset_) / c.gain + c.offset;
  return static_cast<uint16_t>(std::clamp(std::round(mv), 0.0, 5000.0));
}

// 각도[deg] → 엔코더 raw ADC. CanBridge::sensor_routine 의 역변환:
//   adc_mv = raw·3300/4095,  orig_mV = (4125 - adc_mv)/0.825,  deg = (orig_mV - offset)·gain
uint16_t VirtualPowerpack::deg_to_raw_adc(int board_id, double deg) const
{
  if (board_id < 1 || board_id > NUM_BOARDS) return 0;
  const auto& c = calib_[(size_t)(board_id - 1)];
  if (std::abs(c.gain) < 1e-12) return 0;
  const double orig_mv = deg / c.gain + c.offset;
  const double adc_mv  = std::clamp(4125.0 - 0.825 * orig_mv, 0.0, 3300.0);
  return static_cast<uint16_t>(std::clamp(std::round(adc_mv * 4095.0 / 3300.0), 0.0, 4095.0));
}

// CanBridge 와 동일한 raw → 각도 변환 (양자화 오차까지 재현하기 위해 왕복시킨다)
double VirtualPowerpack::raw_adc_to_deg(int board_id, uint16_t raw) const
{
  if (board_id < 1 || board_id > NUM_BOARDS) return 0.0;
  const auto& c = calib_[(size_t)(board_id - 1)];
  const double adc_mv  = std::clamp((double)raw * (3300.0 / 4095.0), 0.0, 3300.0);
  const double orig_mv = (4125.0 - adc_mv) / 0.825;
  return (orig_mv - c.offset) * c.gain;
}

// ============================================================================
// 콜백 / 주기 실행
// ============================================================================
void VirtualPowerpack::on_pwm_cmd(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(pwm_mtx_);
  const int n = std::min((int)msg->data.size(), PWM_TOTAL);
  for (int i = 0; i < n; ++i) pwm_cmd_[(size_t)i] = (double)msg->data[(size_t)i];
}

void VirtualPowerpack::sim_step()
{
  std::array<double, PWM_TOTAL> pwm;
  {
    std::lock_guard<std::mutex> lk(pwm_mtx_);
    pwm = pwm_cmd_;
  }

  const double dt_sub = (std::max(1, period_ms_) / 1000.0) / sim_substeps_;
  for (int s = 0; s < sim_substeps_; ++s) integrate(pwm, dt_sub);

  publish_state();
  ++tick_;
}

void VirtualPowerpack::publish_state()
{
  // ── board/sensors : boards 1~16, 단위 mV (CanBridge 와 동일 포맷/LPF) ────
  std::array<double, PWM_BOARDS> mv_raw{};
  for (int i = 0; i < PWM_BOARDS; ++i) mv_raw[(size_t)i] = kpa_to_raw_mv(i + 1, P_atm_kpa_);

  auto set_board_kpa = [&](int bid, double kpa) {
    if (bid >= 1 && bid <= PWM_BOARDS) mv_raw[(size_t)(bid - 1)] = kpa_to_raw_mv(bid, kpa);
  };
  set_board_kpa(P_pos_board_id_,       P_line_pos_kpa_);
  set_board_kpa(P_neg_board_id_,       P_line_neg_kpa_);
  set_board_kpa(P_macro_board_id_,     P_line_macro_kpa_);
  set_board_kpa(P_macro_neg_board_id_, P_line_macro_neg_kpa_);
  for (int gid = 0; gid < num_total_channels_; ++gid)
    set_board_kpa(gid + channel_board_offset_, P_ch_kpa_[(size_t)gid]);

  // 밸브 전류 [mV] — I = u/100·I_MAX
  std::array<double, PWM_BOARDS * PWM_PER_BOARD> cur_raw{};
  for (int i = 0; i < PWM_BOARDS * PWM_PER_BOARD; ++i) {
    const int gid = (i / PWM_PER_BOARD + 1) - channel_board_offset_;
    const double I_max = (gid >= 0 && gid < num_total_channels_)
                         ? channel_valve_params_[(size_t)gid].I_MAX
                         : line_valve_params_.I_MAX;
    cur_raw[(size_t)i] = std::clamp(
        valve_u_pct_[(size_t)i] / 100.0 * I_max * current_mv_per_amp_, 0.0, 3300.0);
  }

  // CanBridge::rx_loop 의 1차 LPF 재현
  if (!sensor_filt_init_) {
    sensor_mv_filt_  = mv_raw;
    current_mv_filt_ = cur_raw;
    sensor_filt_init_ = true;
  } else if (sensor_lpf_enable_) {
    const double a = std::clamp(sensor_lpf_alpha_, 0.0, 1.0);
    for (size_t i = 0; i < mv_raw.size(); ++i)
      sensor_mv_filt_[i] = sensor_mv_filt_[i] * (1.0 - a) + mv_raw[i] * a;
    for (size_t i = 0; i < cur_raw.size(); ++i)
      current_mv_filt_[i] = current_mv_filt_[i] * (1.0 - a) + cur_raw[i] * a;
  } else {
    sensor_mv_filt_  = mv_raw;
    current_mv_filt_ = cur_raw;
  }

  std_msgs::msg::UInt16MultiArray msg_p;
  msg_p.data.resize(PWM_BOARDS);
  for (int i = 0; i < PWM_BOARDS; ++i)
    msg_p.data[(size_t)i] = (uint16_t)std::clamp(sensor_mv_filt_[(size_t)i], 0.0, 5000.0);
  pub_sensors_->publish(msg_p);

  std_msgs::msg::Float64MultiArray msg_c;
  msg_c.data.assign(current_mv_filt_.begin(), current_mv_filt_.end());
  pub_currents_->publish(msg_c);

  // ── board/analog : boards 17~25 각도 [deg], raw ADC 왕복 후 발행 ─────────
  std_msgs::msg::Float64MultiArray msg_a;
  msg_a.data.assign(NUM_ANALOG, 0.0);
  for (int a = 0; a < num_actuators_ && a < NUM_ANALOG; ++a) {
    const int bid = ANALOG_BOARD_START + a;
    const uint16_t raw = deg_to_raw_adc(bid, actuators_[(size_t)a].theta_deg);
    msg_a.data[(size_t)a] = raw_adc_to_deg(bid, raw);
  }
  pub_analog_->publish(msg_a);

  // 1초마다 상태 로그
  if (tick_ % (1000 / std::max(1, period_ms_)) == 0) {
    std::string ch_s;
    for (int gid = 0; gid < num_total_channels_; ++gid) {
      const bool active = (gid < num_actuators_) ||
                          (gid >= num_positive_channels_ && gid < num_positive_channels_ + num_actuators_);
      if (!active) continue;
      ch_s += "  ch" + std::to_string(gid) + "=" +
              std::to_string((int)std::round(P_ch_kpa_[(size_t)gid]));
    }
    std::string ang_s;
    for (const auto& ac : actuators_)
      ang_s += "  " + std::to_string((int)std::round(ac.theta_deg)) + "deg";

    RCLCPP_INFO(get_logger(),
      "line[pos=%.1f neg=%.1f mac=%.1f macneg=%.1f] ch[%s ] θ[%s ]",
      P_line_pos_kpa_, P_line_neg_kpa_, P_line_macro_kpa_, P_line_macro_neg_kpa_,
      ch_s.c_str(), ang_s.c_str());
  }
}
