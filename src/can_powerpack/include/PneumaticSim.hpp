#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <array>
#include <mutex>
#include <vector>

// Simulates pneumatic actuator dynamics using the 13-parameter physics model:
//   Bouc-Wen hysteresis → sigmoid effective area → compressible Phi → 2nd-order dynamics
//
// Subscribes  : board/pwm_cmd  (UInt16MultiArray, same format as CanBridge)
// Publishes   : board/sensors  (UInt16MultiArray, same format as CanBridge)

class PneumaticSim : public rclcpp::Node {
public:
  explicit PneumaticSim(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());

private:
  // 13-parameter proportional valve model (MATLAB optimization output)
  struct ChannelParams {
    double A_max{0.2845};         // max effective area coefficient
    double k_shape{33.09};        // sigmoid steepness
    double C_k{0.0288};           // current threshold offset [A]
    double C_p{0.00012};          // pressure coupling coefficient
    double C_z{0.0};              // hysteresis state coupling
    double A_bw{260649.5};        // Bouc-Wen tangent stiffness
    double beta_bw{179.0};        // Bouc-Wen parameter
    double gamma_bw{0.06};        // Bouc-Wen parameter
    double alpha_shape{3884.2};   // sigmoid exponent
    double wn_up{40.0};           // 2nd-order natural freq, opening [rad/s]
    double zeta_up{1.2};          // 2nd-order damping, opening
    double wn_down{45.0};         // 2nd-order natural freq, closing [rad/s]
    double zeta_down{1.0};        // 2nd-order damping, closing
    double volume_m3{1.0e-6};
    bool   is_positive{true};
  };

  // Per-valve integration state (one per physical valve: micro/atm/macro)
  struct ValveState {
    double z{0.0};       // Bouc-Wen hysteresis state
    double x1{0.0};      // flow output [LPM]
    double x2{0.0};      // flow derivative [LPM/s]
    double prev_I{0.0};  // previous current [A]
    int    dir{0};       // 0=closing, 1=opening
  };

  struct BoardCalib {
    double offset{1000.0};
    double gain{0.250};
  };

  static constexpr int    NUM_BOARDS    = 17;
  static constexpr int    PWM_PER_BOARD = 3;
  static constexpr double I_MAX         = 0.30;  // [A], maps u=100% → 0.3 A
  static constexpr double KAPPA         = 1.4;   // heat capacity ratio for air

  // Compressible flow function Phi(P_in, P_out)
  static double get_phi(double P_in, double P_out);

  // 13-variable valve model: updates ValveState in-place, returns Q [LPM]
  // u_pct: [0,100], P_in/P_out: kPa (absolute), dt_sub: sub-step size [s]
  double step_valve(double u_pct, double P_in, double P_out,
                    const ChannelParams& cp, ValveState& vs, double dt_sub) const;

  uint16_t kpa_to_raw(int board_id, double kpa) const;

  void on_pwm_cmd(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
  void sim_step();

  // Config
  int    num_total_channels_{12};
  int    num_positive_channels_{6};
  int    channel_board_offset_{5};
  int    sim_substeps_{10};

  double P_line_pos_kpa_{400.0};
  double P_line_neg_kpa_{50.0};
  double P_line_macro_kpa_{400.0};
  double P_macro_neg_kpa_{50.0};
  double P_atm_kpa_{101.325};
  double atm_offset_{101.325};

  std::vector<ChannelParams> channel_params_;
  std::array<BoardCalib, NUM_BOARDS> board_calib_{};

  // State
  std::vector<double> pressure_kpa_;
  // Per-channel valve states: [gid][0]=micro, [1]=atm, [2]=macro
  std::vector<std::array<ValveState, 3>> valve_states_;

  std::array<double, NUM_BOARDS * PWM_PER_BOARD> pwm_pct_{};
  std::mutex pwm_mtx_;

  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr     pub_sensors_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr  sub_pwm_cmd_;
  rclcpp::TimerBase::SharedPtr sim_timer_;
};
