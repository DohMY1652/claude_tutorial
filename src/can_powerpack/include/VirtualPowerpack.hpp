#pragma once

// ============================================================================
// VirtualPowerpack — CanBridge 를 대체하는 가상 하드웨어 노드
// ============================================================================
// Kvaser CANlib / CAN 보드 25장이 없는 환경에서 CanBridge 와 **완전히 동일한
// 토픽 인터페이스**를 제공한다. pp_controller 는 이 노드가 CanBridge 인지
// 구분할 수 없다.
//
//   Subscribe : board/pwm_cmd  UInt16MultiArray  [(bid-1)*3 + v] = 0..4095
//   Publish   : board/sensors  UInt16MultiArray  16개, index i = board(i+1), 단위 mV(0~5000)
//               board/currents Float64MultiArray 16*3개, 밸브 전류 [mV]
//               board/analog   Float64MultiArray 9개, board 17~25 엔코더 각도 [deg]
//
// 보드 배정 (powerpack_config.yaml 과 동일)
//   board 1  : P_line_pos    센서 + 양압 라인 PID 밸브(v1)
//   board 2  : P_line_neg    센서 + 음압 라인 PID 밸브(v1)
//   board 3  : P_line_macro  센서
//   board 4  : P_line_macro_neg 센서 + MacroSwitch 밸브(v1)
//   board 5~10  : 양압 채널 gid 0~5   (channel_board_offset = 5)
//   board 11~16 : 음압 채널 gid 6~11
//   board 17~25 : 엔코더 (액추에이터 축 0~8)
//
// 밸브 인덱스 (Controller::on_timer 의 zoh_ 기록 순서)
//   v1 = micro (공급), v2 = atm (배기), v3 = macro (대유량)
//
// 물리 모델 — Controller.cpp 가 내부적으로 가정하는 모델을 그대로 구현
//   1. 13-variable 비례밸브: Bouc-Wen 히스테리시스 → 시그모이드 유효면적
//      → 압축성 유동 Phi(P_in,P_out) → 2차 밸브 동역학
//        (AcadosMpc::update_linearization / compute_input_reference 와 동일 식)
//   2. 챔버 압력: 등온 이상기체
//        dP/dt = (R·T·ṁ - P·V̇) / V
//      (Controller 의 feedforward 가 쓰는 m_dot_pressure / m_dot_volume 두 항과 동일)
//   3. micro 라인은 펌프 한 대로 이어진 닫힌 회로
//        펌프 흡입구 = 음압 라인,  펌프 토출구 = 양압 라인
//      회로가 대기와 통하는 곳은 두 군데뿐이다:
//        board 1 v1 : 양압 라인 → 대기 (방출) — 회로의 유일한 출구
//        board 2 v1 : 대기 → 음압 라인 (유입) — 회로의 유일한 입구
//      둘 다 채널 밸브와 같은 정극성(PWM 4095 = 완전 개방)이다. LinePID 가 (100−u) 로
//      반전 출력하는 것은 밸브 극성이 아니라 "누출 밸브라서 부족하면 닫는다"는
//      배관 위치의 결과다. 정변위 펌프 이송량은 흡입측 밀도에 비례하고 압력비가
//      한계에 닿으면 0 이 되므로, 한쪽 밸브 조작이 반대쪽 라인까지 끌고 다닌다.
//   4. 액추에이터 회전 동역학
//        J·ω̇ = A_piston·r·(P_pos - P_neg) - m·g·L·sin θ - b·ω - τ_coulomb·sgn(ω)
//      부피는 Controller::on_timer 의 각도-부피 식과 동일:
//        V_pos = tank_pos + A·max(0, pos_offset + mm_per_deg·θ)/1000  [mL]
//        V_neg = tank_neg + A·max(0, neg_offset - mm_per_deg·θ)/1000  [mL]
//   5. 센서는 raw ADC 왕복까지 재현 — 각도는 반전증폭 역산(4125, 0.825)을
//      역으로 적용해 uint16 raw 로 양자화한 뒤 CanBridge 와 같은 식으로 되돌린다.
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "PistonPump.hpp"

#include <array>
#include <limits>
#include <mutex>
#include <vector>

class VirtualPowerpack : public rclcpp::Node {
public:
  explicit VirtualPowerpack(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());

private:
  // ── 상수 (CanBridge 와 동일) ──────────────────────────────────────────────
  static constexpr int    NUM_BOARDS         = 25;  // 물리 보드 수
  static constexpr int    PWM_BOARDS         = 16;  // TX/압력센서 보드 (1~16)
  static constexpr int    ANALOG_BOARD_START = 17;  // 엔코더 보드 시작
  static constexpr int    NUM_ANALOG         = NUM_BOARDS - ANALOG_BOARD_START + 1;  // 9
  static constexpr int    PWM_PER_BOARD      = 3;
  static constexpr int    PWM_TOTAL          = NUM_BOARDS * PWM_PER_BOARD;  // 75

  static constexpr double PWM_TO_PCT = 100.0 / 4095.0;
  static constexpr double KAPPA      = 1.4;        // 공기 비열비
  static constexpr double RGAS_AIR   = 287.0;      // [J/(kg·K)]
  static constexpr double TEMP_K     = 293.15;     // [K]
  static constexpr double LPM_TO_KGPS = 0.0002155; // [LPM] → [kg/s]

  // ── 13-variable 비례밸브 파라미터 ────────────────────────────────────────
  struct ValveParams {
    double I_MAX{0.30};
    double A_max{0.2845};
    double k_shape{33.09};
    double C_k{0.0288};
    double C_p{0.00012};
    double C_z{0.0};
    double A_bw{260649.5};
    double beta_bw{179.0};
    double gamma_bw{0.06};
    double alpha_shape{3884.2};
    double wn_up{40.0};
    double zeta_up{1.2};
    double wn_down{45.0};
    double zeta_down{1.0};
  };

  // 밸브 하나의 적분 상태
  struct ValveState {
    double z{0.0};       // Bouc-Wen 히스테리시스 상태
    double x1{0.0};      // 유량 출력 [LPM]
    double x2{0.0};      // 유량 미분 [LPM/s]
    double prev_I{0.0};  // 직전 전류 [A]
    int    dir{0};       // 0=닫힘 방향, 1=열림 방향
  };

  struct BoardCalib {
    double offset{1000.0};   // 압력: raw mV @ 대기압 / 엔코더: orig_mV @ 0deg
    double gain{0.250};      // 압력: kPa/mV        / 엔코더: deg/mV
  };

  // 액추에이터(축) 하나
  struct Actuator {
    int    pos_gid{0};
    int    neg_gid{6};
    double theta_deg{0.0};
    double omega_dps{0.0};
    // 부피 모델 — 피스톤 변위 x = reel_radius · θ_rad (릴 구동)
    double piston_area_mm2{0.0};
    double reel_radius_mm{25.0};
    double pos_offset_mm{40.0};
    double neg_offset_mm{90.0};
    double tank_pos_ml{50.0};
    double tank_neg_ml{50.0};
    // 회전 동역학 — 토크 팔 = 릴 반경 (같은 기하)
    double torque_arm_m{0.025};
    double inertia{0.05};
    double damping{0.30};
    double coulomb_nm{0.50};
    double mass_kg{5.0};
    double link_length_m{0.2};
    double theta_min_deg{0.0};
    double theta_max_deg{105.0};
  };

  // ── 모델 계산 ────────────────────────────────────────────────────────────
  static double get_phi(double P_in, double P_out);
  double step_valve(double u_pct, double P_in, double P_out,
                    const ValveParams& vp, ValveState& vs, double dt_sub) const;

  // 챔버/라인 압력 1스텝: dP/dt = (R·T·ṁ - P·V̇)/V
  static double pressure_derivative(double P_kpa, double net_flow_lpm,
                                    double volume_m3, double vol_dot_m3ps);

  void sim_step();
  void integrate(const std::array<double, PWM_TOTAL>& pwm, double dt_sub);
  void publish_state();

  // 센서 인코딩
  uint16_t kpa_to_raw_mv(int board_id, double kpa) const;
  uint16_t deg_to_raw_adc(int board_id, double deg) const;
  double   raw_adc_to_deg(int board_id, uint16_t raw) const;

  void on_pwm_cmd(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);

  // ── 설정 ────────────────────────────────────────────────────────────────
  int    num_total_channels_{12};
  int    num_positive_channels_{6};
  int    num_actuators_{1};
  int    channel_board_offset_{5};
  int    sim_substeps_{10};
  int    period_ms_{2};
  bool   actuator_connected_{true};
  bool   sensor_lpf_enable_{true};
  double sensor_lpf_alpha_{0.2};        // CanBridge::rx_loop 의 LPF_ALPHA
  double current_mv_per_amp_{11000.0};  // I[A] → 전류센서 출력 [mV]

  int    P_pos_board_id_{1};
  int    P_neg_board_id_{2};
  int    P_macro_board_id_{3};
  int    P_macro_neg_board_id_{4};
  int    line_pos_pwm_index_{0};     // (board1-1)*3 + 0
  int    line_neg_pwm_index_{3};     // (board2-1)*3 + 0
  int    macro_switch_pwm_index_{9}; // (board4-1)*3 + 0

  double P_atm_kpa_{101.325};
  double atm_offset_{101.325};

  // ── micro 라인 펌프 (PistonPump.hpp 공용 슬라이더-크랭크 모델) ──────────
  // 흡입구 = 음압 라인, 토출구 = 양압 라인. 흡입량 = 토출량이라 양압을 높이면
  // 음압이 억눌리는 능력경계가 생긴다. 기동 시 2D 테이블을 만들어 보간해 쓴다.
  pneu::PumpGeom  pump_geom_;
  pneu::PumpTable pump_table_;

  // ── 압축탱크 (board 3): 회복 없는 소진형 자원 ──────────────────────────
  // 213 mL 를 30 MPa 로 충전하고 레귤레이터로 700 kPa gauge 로 낮춰 쓴다.
  // 저장 질량 ≈ 76 g ≈ 표준 63 L — 이젝터(57 LPM)만 돌려도 약 1분이면 바닥난다.
  double tank_volume_m3_{213e-6};
  double tank_charge_kpa_abs_{30000.0};    // 30 MPa
  double tank_reg_kpa_abs_{801.325};       // 700 kPa gauge 출력
  double tank_mass_kg_{0.0};               // 상태 (회복 없음)

  // ── 이젝터 (board 4 라인) : 특성곡선 ──────────────────────────────────
  // 구동압은 board 4 v1(MacroSwitch)이 열렸을 때의 탱크 레귤레이터 출력.
  double ej_suction_lpm_{0.0};             // 진단용
  double ej_consume_lpm_{0.0};
  double ej_reach_kpa_abs_{101.325};    // 현재 구동압에서의 도달 진공
  double ej_floor_kpa_{11.325};         // 최대 구동 시 도달 진공 = 수치 하한

  double ejector_limit_kpa_{11.325};    // 음압 채널 압력 하한 (수치 가드)
  double V_line_pos_ml_{500.0};
  double V_line_neg_ml_{500.0};
  double V_line_macro_ml_{1000.0};
  double V_line_macro_neg_ml_{500.0};
  double default_volume_ml_{1.0};

  // 라인 기생 누설 [LPM/kPa] — 매니폴드/피팅. 제어 경로가 아니라 실제 누설분이므로 작게.
  double line_leak_pos_{0.002};
  double line_leak_neg_{0.002};
  double line_leak_macro_{0.002};
  double line_leak_macro_neg_{0.002};

  // 라인 압력 수치 가드 [kPa]
  double line_pos_min_kpa_{50.0},  line_pos_max_kpa_{800.0};
  double line_neg_min_kpa_{5.0},   line_neg_max_kpa_{110.0};

  ValveParams line_valve_params_;                 // board 1/2/4 라인 밸브
  // [gid][0=micro, 1=atm, 2=macro] — 세 밸브는 오리피스도 상류압도 달라
  // (micro 2.3 mm/레일, macro 1.6 mm/탱크 800 kPa) 같은 파라미터를 쓰면 안 된다.
  // 채널당 하나만 두던 때는 시뮬 안의 macro 가 micro 로 동작해 탱크에서
  // 2233 kPa/s 를 쏟아부었다.
  std::vector<std::array<ValveParams, 3>> channel_valve_params_; // [gid][valve]
  std::vector<double>      channel_tank_ml_;      // [gid] 액추에이터 미연결 시 고정 부피
  std::array<BoardCalib, NUM_BOARDS> calib_{};    // [board_id-1]

  // ── 상태 ────────────────────────────────────────────────────────────────
  double P_line_pos_kpa_{101.325};
  double P_line_neg_kpa_{101.325};
  double P_line_macro_kpa_{101.325};
  double P_line_macro_neg_kpa_{101.325};

  std::vector<double> P_ch_kpa_;            // [gid]
  std::vector<double> V_ch_m3_;             // [gid] 현재 부피
  std::vector<double> Vdot_ch_m3ps_;        // [gid] 부피 변화율
  std::vector<Actuator> actuators_;

  // 밸브 상태: [(board_id-1)*3 + v_idx], v_idx 0=micro 1=atm 2=macro
  std::array<ValveState, PWM_BOARDS * PWM_PER_BOARD> valves_{};
  // 밸브별 최근 유량 [LPM] / 지령 [%] — 라인 수지 계산과 전류 센서용
  std::array<double, PWM_BOARDS * PWM_PER_BOARD> valve_flow_lpm_{};
  std::array<double, PWM_BOARDS * PWM_PER_BOARD> valve_u_pct_{};

  // 센서 출력 LPF 상태 (CanBridge 의 rx_loop LPF 재현)
  std::array<double, PWM_BOARDS>                    sensor_mv_filt_{};
  std::array<double, PWM_BOARDS * PWM_PER_BOARD>    current_mv_filt_{};
  bool sensor_filt_init_{false};

  std::array<double, PWM_TOTAL> pwm_cmd_{};
  std::mutex pwm_mtx_;
  uint64_t tick_{0};

  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr    pub_sensors_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr   pub_currents_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr   pub_analog_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr    pub_analog_raw_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_pwm_cmd_;
  rclcpp::TimerBase::SharedPtr sim_timer_;
};
