#include "PneumaticSim.hpp"
#include <algorithm>
#include <cmath>
#include <string>

using namespace std::chrono_literals;

static constexpr double LPMTOKG_S  = 0.0002155;
static constexpr double RGAS_AIR   = 287.0;
static constexpr double TEMP_K     = 293.15;
static constexpr double PWM_TO_PCT = 1.0 / 40.95;  // uint16 [0,4095] → percent [0,100]

template <typename T>
static T gp(rclcpp::Node* n, const std::string& k, const T& def) {
    try { return n->declare_parameter<T>(k, def); }
    catch (...) { return n->get_parameter(k).get_value<T>(); }
}

PneumaticSim::PneumaticSim(const rclcpp::NodeOptions& opts)
: rclcpp::Node("pneumatic_sim", opts)
{
    num_total_channels_    = gp<int>(this, "num_total_channels",    12);
    num_positive_channels_ = gp<int>(this, "num_positive_channels",  6);
    channel_board_offset_  = gp<int>(this, "channel_board_offset",   5);
    sim_substeps_          = gp<int>(this, "sim_substeps",           10);

    P_line_pos_kpa_   = gp<double>(this, "P_line_pos_kpa",           400.0);
    P_line_neg_kpa_   = gp<double>(this, "P_line_neg_kpa",            50.0);
    P_line_macro_kpa_ = gp<double>(this, "P_line_macro_kpa",         400.0);
    P_macro_neg_kpa_  = gp<double>(this, "MacroNegLine.pressure_kpa", 50.0);
    P_atm_kpa_        = gp<double>(this, "P_atm_kpa",               101.325);
    atm_offset_       = gp<double>(this, "Sensor_calibration.atm_offset", 101.325);

    const double init_pos = gp<double>(this, "initial_pressure_pos_kpa", P_atm_kpa_);
    const double init_neg = gp<double>(this, "initial_pressure_neg_kpa", P_atm_kpa_);

    for (int bid = 1; bid <= NUM_BOARDS; ++bid) {
        const std::string pre = "Sensor_calibration.boards." + std::to_string(bid) + ".";
        board_calib_[bid-1].offset = gp<double>(this, pre + "offset", 1000.0);
        board_calib_[bid-1].gain   = gp<double>(this, pre + "gain",   0.250);
    }

    const double default_volume_m3 = gp<double>(this, "default_volume_ml", 1.0) * 1e-6;

    channel_params_.resize(num_total_channels_);
    for (int i = 0; i < num_total_channels_; ++i) {
        const std::string pre = "channel_config.ch" + std::to_string(i) + ".";
        auto& cp = channel_params_[i];
        cp.A_max       = gp<double>(this, pre + "A_max",       0.2845);
        cp.k_shape     = gp<double>(this, pre + "k_shape",     33.09);
        cp.C_k         = gp<double>(this, pre + "C_k",         0.0288);
        cp.C_p         = gp<double>(this, pre + "C_p",         0.00012);
        cp.C_z         = gp<double>(this, pre + "C_z",         0.0);
        cp.A_bw        = gp<double>(this, pre + "A_bw",        260649.5);
        cp.beta_bw     = gp<double>(this, pre + "beta_bw",     179.0);
        cp.gamma_bw    = gp<double>(this, pre + "gamma_bw",    0.06);
        cp.alpha_shape = gp<double>(this, pre + "alpha_shape", 3884.2);
        cp.wn_up       = gp<double>(this, pre + "wn_up",       40.0);
        cp.zeta_up     = gp<double>(this, pre + "zeta_up",     1.2);
        cp.wn_down     = gp<double>(this, pre + "wn_down",     45.0);
        cp.zeta_down   = gp<double>(this, pre + "zeta_down",   1.0);
        cp.volume_m3   = gp<double>(this, pre + "volume_ml",   1.0) * 1e-6;
        if (cp.volume_m3 < 1e-12) cp.volume_m3 = default_volume_m3;
        cp.is_positive = (i < num_positive_channels_);
    }

    pressure_kpa_.resize(num_total_channels_);
    valve_states_.resize(num_total_channels_);
    for (int i = 0; i < num_total_channels_; ++i) {
        pressure_kpa_[i] = channel_params_[i].is_positive ? init_pos : init_neg;
        // valve_states_[i] default-initialized to zero
    }

    pwm_pct_.fill(0.0);

    pub_sensors_ = create_publisher<std_msgs::msg::UInt16MultiArray>("board/sensors", 10);
    sub_pwm_cmd_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
        "board/pwm_cmd", 10,
        [this](const std_msgs::msg::UInt16MultiArray::SharedPtr m){ on_pwm_cmd(m); });
    sim_timer_ = create_wall_timer(2ms, [this]{ sim_step(); });

    RCLCPP_INFO(get_logger(),
        "PneumaticSim (13-var model): %d ch (%d pos + %d neg), substeps=%d, "
        "P_pos=%.1f P_neg=%.1f P_mac=%.1f P_mac_neg=%.1f [kPa]",
        num_total_channels_, num_positive_channels_,
        num_total_channels_ - num_positive_channels_,
        sim_substeps_, P_line_pos_kpa_, P_line_neg_kpa_,
        P_line_macro_kpa_, P_macro_neg_kpa_);
}

// ── Compressible flow function Phi(P_in, P_out) ──────────────────────────────
double PneumaticSim::get_phi(double P_in, double P_out)
{
    if (P_in < 1e-9) return 0.0;
    const double kappa = KAPPA;
    const double Pr    = std::clamp(P_out / P_in, 0.0, 1.0);
    const double P_cr  = std::pow(2.0 / (kappa + 1.0), kappa / (kappa - 1.0));

    if (Pr <= P_cr) {
        return std::sqrt(kappa * std::pow(2.0 / (kappa + 1.0), (kappa + 1.0) / (kappa - 1.0)));
    }
    const double t1 = std::sqrt(2.0 * kappa / (kappa - 1.0));
    const double t2 = std::sqrt(std::max(0.0,
        std::pow(Pr, 2.0 / kappa) - std::pow(Pr, (kappa + 1.0) / kappa)));
    return t1 * t2;
}

// ── 13-variable proportional valve model ─────────────────────────────────────
// Mirrors MATLAB simulate_physics_model() for one time step.
// Bouc-Wen hysteresis → sigmoid area → compressible Phi → 2nd-order dynamics.
double PneumaticSim::step_valve(double u_pct, double P_in, double P_out,
                                const ChannelParams& cp, ValveState& vs,
                                double dt_sub) const
{
    u_pct = std::clamp(u_pct, 0.0, 100.0);
    const double I = u_pct / 100.0 * I_MAX;

    // Bouc-Wen hysteresis update
    const double dI     = I - vs.prev_I;
    const double abs_dI = std::abs(dI);
    const double dz     = cp.A_bw * dI
                        - cp.beta_bw  * abs_dI * vs.z
                        - cp.gamma_bw * dI * std::abs(vs.z);
    vs.z = std::clamp(vs.z + dz, -1e6, 1e6);

    // Direction (opening=1 / closing=0) — threshold matches MATLAB 1e-4
    if      (dI >  1e-4) vs.dir = 1;
    else if (dI < -1e-4) vs.dir = 0;

    vs.prev_I = I;

    // Effective area: A_max * sigmoid(k_shape * Force_net)^alpha
    const double Force_net = std::clamp(
        I + cp.C_z * vs.z + cp.C_p * P_in - cp.C_k, -500.0, 500.0);
    const double sigma    = 1.0 / (1.0 + std::exp(-cp.k_shape * Force_net));
    const double Area_eff = cp.A_max * std::pow(sigma, cp.alpha_shape);

    // Static flow Q_static [LPM]
    const double phi      = (P_in > P_out) ? get_phi(P_in, P_out) : 0.0;
    const double Q_static = Area_eff * P_in * phi;

    // 2nd-order dynamics: one Euler sub-step
    const double wn   = (vs.dir == 1) ? cp.wn_up   : cp.wn_down;
    const double zeta = (vs.dir == 1) ? cp.zeta_up : cp.zeta_down;
    const double dx2  = wn * wn * (Q_static - vs.x1) - 2.0 * zeta * wn * vs.x2;
    vs.x1 += dt_sub * vs.x2;
    vs.x2 += dt_sub * dx2;

    return std::max(0.0, vs.x1);
}

// ── Sensor encoding ──────────────────────────────────────────────────────────
uint16_t PneumaticSim::kpa_to_raw(int board_id, double kpa) const
{
    if (board_id < 1 || board_id > NUM_BOARDS) return 0;
    const auto& c = board_calib_[board_id - 1];
    if (std::abs(c.gain) < 1e-12) return 0;
    const double p_mv_raw = (kpa - atm_offset_) / c.gain + c.offset;
    return static_cast<uint16_t>(std::clamp(std::round(p_mv_raw), 0.0, 5000.0));
}

// ── Callbacks ────────────────────────────────────────────────────────────────
void PneumaticSim::on_pwm_cmd(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(pwm_mtx_);
    const int n = std::min((int)msg->data.size(), NUM_BOARDS * PWM_PER_BOARD);
    for (int i = 0; i < n; ++i)
        pwm_pct_[i] = msg->data[i] * PWM_TO_PCT;
}

void PneumaticSim::sim_step()
{
    std::array<double, NUM_BOARDS * PWM_PER_BOARD> pwm;
    {
        std::lock_guard<std::mutex> lk(pwm_mtx_);
        pwm = pwm_pct_;
    }

    const double dt_sub         = 0.002 / sim_substeps_;
    const double dynamics_scale = (RGAS_AIR * TEMP_K) * LPMTOKG_S / 1000.0;
    // dP/dt [kPa/s] = dynamics_scale / volume * net_flow [LPM]

    for (int s = 0; s < sim_substeps_; ++s) {
        for (int gid = 0; gid < num_total_channels_; ++gid) {
            const int bid  = gid + channel_board_offset_;
            const int pw0  = (bid - 1) * PWM_PER_BOARD;
            const auto& cp = channel_params_[gid];
            auto& vs       = valve_states_[gid];  // [0]=micro, [1]=atm, [2]=macro

            const double u_micro = pwm[pw0 + 0];
            const double u_atm   = pwm[pw0 + 1];
            const double u_macro = pwm[pw0 + 2];
            const double P_now   = pressure_kpa_[gid];

            double net_flow;

            if (cp.is_positive) {
                // fill: micro/macro supply → chamber; exhaust: chamber → atm
                const double f_mi = step_valve(u_micro, P_line_pos_kpa_,   P_now,           cp, vs[0], dt_sub);
                const double f_ma = step_valve(u_macro, P_line_macro_kpa_, P_now,           cp, vs[2], dt_sub);
                const double f_at = step_valve(u_atm,   P_now,             P_atm_kpa_,      cp, vs[1], dt_sub);
                net_flow = f_mi + f_ma - f_at;
            } else {
                // fill: atm → chamber; exhaust: chamber → suction/macro-neg
                const double f_at = step_valve(u_atm,   P_atm_kpa_,       P_now,           cp, vs[1], dt_sub);
                const double f_mi = step_valve(u_micro, P_now,             P_line_neg_kpa_, cp, vs[0], dt_sub);
                const double f_ma = step_valve(u_macro, P_now,             P_macro_neg_kpa_,cp, vs[2], dt_sub);
                net_flow = f_at - f_mi - f_ma;
            }

            pressure_kpa_[gid] += (dynamics_scale / cp.volume_m3) * net_flow * dt_sub;

            const double p_min = cp.is_positive ? 50.0  : 10.0;
            const double p_max = cp.is_positive ? 800.0 : 110.0;
            pressure_kpa_[gid] = std::clamp(pressure_kpa_[gid], p_min, p_max);
        }
    }

    // Publish sensor array (same format as CanBridge::sensor_routine)
    std_msgs::msg::UInt16MultiArray msg;
    msg.data.resize(NUM_BOARDS, 0);

    for (int i = 0; i < NUM_BOARDS; ++i)
        msg.data[i] = kpa_to_raw(i + 1, P_atm_kpa_);

    msg.data[0] = kpa_to_raw(1, P_line_pos_kpa_);
    msg.data[1] = kpa_to_raw(2, P_line_neg_kpa_);
    msg.data[2] = kpa_to_raw(3, P_line_macro_kpa_);

    for (int gid = 0; gid < num_total_channels_; ++gid) {
        const int bid = gid + channel_board_offset_;
        if (bid >= 1 && bid <= NUM_BOARDS)
            msg.data[bid - 1] = kpa_to_raw(bid, pressure_kpa_[gid]);
    }

    pub_sensors_->publish(msg);
}
