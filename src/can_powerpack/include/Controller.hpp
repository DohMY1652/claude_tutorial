#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Dense>

#include <deque>
#include <set>
#include <vector>
#include <array>
#include <mutex>
#include <thread>
#include <queue>
#include <condition_variable>
#include <functional>
#include <memory>
#include <atomic>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <string>

#include <qpOASES.hpp>

#ifdef __linux__
  #include <pthread.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <arpa/inet.h>
  #include <fcntl.h>
  #include <unistd.h>
  #include <errno.h>
#endif

// ================================
// Fixed sizes for this project
// ================================
static constexpr int NUM_CAN_BOARDS = 25;   // physical CAN boards (board_id 1..25)
static constexpr int PWM_PER_BOARD  = 3;    // v1, v2, v3 per board
static constexpr int PWM_TOTAL      = NUM_CAN_BOARDS * PWM_PER_BOARD;  // 51

static constexpr int PWM_CLAMP_MIN  = 0;
static constexpr int PWM_CLAMP_MAX  = 4095;

// Event-driven: board TX 500Hz → on_sensor 500Hz → all channels every tick
static constexpr int MPC_PHASES     = 1;   // 500Hz / 1 = 500Hz per channel (parallel via ThreadPool)

static constexpr int MPC_TOTAL      = 24;   // max channel capacity
static constexpr int MPC_OUT_DIM    = 3;

// QP Solver Wrapper
class QP {
public:
    QP(int nv, int nc) : solver_(nv, nc) {
        options_.setToMPC();
        options_.printLevel = qpOASES::PL_NONE;
        solver_.setOptions(options_);
    }

    bool solve(const Eigen::MatrixXf& H, const Eigen::VectorXf& g,
               const Eigen::VectorXf& lb, const Eigen::VectorXf& ub,
               Eigen::VectorXf& solution)
    {
        Eigen::MatrixXd Hd = H.cast<double>();
        Eigen::VectorXd gd = g.cast<double>();
        Eigen::VectorXd lbd = lb.cast<double>();
        Eigen::VectorXd ubd = ub.cast<double>();

        qpOASES::returnValue ret;
        if (!initialized_) {
            // cold start: 첫 호출 또는 이전 실패 후 재시작
            int nWSR = 100;
            ret = solver_.init(Hd.data(), gd.data(), nullptr,
                               lbd.data(), ubd.data(), nullptr, nullptr, nWSR);
            initialized_ = (ret == qpOASES::SUCCESSFUL_RETURN);
        } else {
            // hot start: 이전 active set 재사용, WSR 대폭 절감
            int nWSR = 10;
            ret = solver_.hotstart(Hd.data(), gd.data(), nullptr,
                                   lbd.data(), ubd.data(), nullptr, nullptr, nWSR);
            if (ret != qpOASES::SUCCESSFUL_RETURN) initialized_ = false;
        }

        if (ret == qpOASES::SUCCESSFUL_RETURN) {
            Eigen::VectorXd sol(H.rows());
            solver_.getPrimalSolution(sol.data());
            solution = sol.cast<float>();
            return true;
        }
        return false;
    }

private:
    qpOASES::SQProblem solver_;
    qpOASES::Options options_;
    bool initialized_{false};
};

class ThreadPool {
public:
  explicit ThreadPool(size_t num_threads, const std::vector<int>& pin_cpus = {});
  ~ThreadPool();

  void enqueue(std::function<void()> fn);
  void run_batch_and_wait(std::vector<std::function<void()>>& tasks);

private:
  std::vector<std::thread> workers_;
  std::queue<std::function<void()>> queue_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::atomic<bool> stop_{false};
  std::vector<int> pin_cpus_;
};

class AcadosMpc {
public:
  struct Config {
    int can_board_id{4};   // physical CAN board ID (1-based); sensor = filt_out_[can_board_id-1]
    int global_id{0};
    int   NP{10};
    int   n_x{1};
    int   n_u{3};
    float Ts{0.004f};
    float Q_value{1.0f};
    float R_value{1.0f};
    float A_lin{1.0f};
    std::array<float,3> B_lin{1.0f, 0.0f, 0.0f};
    bool  is_positive{true};
    float pos_ki_micro{0.0f}, pos_ki_macro{0.0f}, pos_ki_atm{0.0f};
    float neg_ki_micro{0.0f}, neg_ki_macro{0.0f}, neg_ki_atm{0.0f};
    float ref_value{0.0f};
    float du_min{-100.0f};
    float du_max{+100.0f};
    float u_abs_min{0.0f};
    float u_abs_max{100.0f};
    float volume_m3{1.0e-5f};
    float prev_vol_m3{1.0e-5f};

    float last_ref_value_ = 101.325f;
    float ejector_k = 0.005f;
    float ejector_p_limit = 11.325f;
    float leakage_u_pos = 0.0f;
    float leakage_u_neg = 0.0f;
    float target_time_constant = 0.2f;
    float macro_threshold = 30.0f;
    float actuating_threshold = 5.0f;
    // 13-variable proportional valve model parameters
    float I_MAX{0.30f};
    float A_max{0.2845f};
    float k_shape{33.09f};
    float C_k{0.0288f};
    float C_p{0.00012f};
    float C_z{0.0f};
    float A_bw{260649.5f};
    float beta_bw{179.0f};
    float gamma_bw{0.06f};
    float alpha_shape{3884.2f};
    float wn_up{40.0f};
    float zeta_up{1.2f};
    float wn_down{45.0f};
    float zeta_down{1.0f};
  };

  explicit AcadosMpc(const Config& cfg);
  void set_qp_solver(std::shared_ptr<QP> qp);
  inline void set_ref_value(float ref_kpa) { cfg_.ref_value = ref_kpa; }
  inline void set_volume(float vol_m3) { cfg_.volume_m3 = std::max(1e-12f, vol_m3); }
  inline void set_prev_volume(float vol_m3) { cfg_.prev_vol_m3 = std::max(1e-12f, vol_m3); }
  void update_linearization(float x_ref, const Eigen::RowVector3f& u_ref);
  void set_AB_sequences(const std::vector<float>& A_seq, const std::vector<Eigen::RowVector3f>& B_seq);
  void set_AB_constant(float A_scalar, const Eigen::RowVector3f& B_row);
  void solve(float dt_ms, std::array<uint16_t, MPC_OUT_DIM>& out3, float current_time_sec);
  const Config& cfg() const { return cfg_; }

  float current_P_now_       = 101.325f;
  float current_P_micro_     = 101.325f;
  float current_P_macro_     = 101.325f;
  float current_P_macro_neg_ = 101.325f;
  float current_P_atm_       = 101.325f;

private:
  // Bouc-Wen hysteresis state estimates for each valve (micro/atm/macro)
  double z_micro_{0.0}, prev_I_micro_{0.0};
  double z_atm_{0.0},   prev_I_atm_{0.0};
  double z_macro_{0.0}, prev_I_macro_{0.0};
  int    dir_micro_{0},  dir_atm_{0},  dir_macro_{0};
  std::array<float,3> compute_input_reference(float P_now, float P_micro, float P_macro, float P_macro_neg, float dt_sec, float current_time_sec);
  void build_mpc_qp(const std::vector<float>& A_seq, const std::vector<Eigen::RowVector3f>& B_seq, float P_now, const std::vector<float>& P_ref, Eigen::MatrixXf& P, Eigen::VectorXf& q, Eigen::MatrixXf& A_con, Eigen::VectorXf& LL, Eigen::VectorXf& UL);
  std::array<float,3> solve_qp_first_step(const Eigen::MatrixXf& P, const Eigen::VectorXf& q, const Eigen::MatrixXf& A_con, const Eigen::VectorXf& LL, const Eigen::VectorXf& UL);

private:
  Config cfg_;
  std::shared_ptr<QP> qp_;
  std::vector<float> P_ref_;
  std::vector<float> A_seq_;
  std::vector<Eigen::RowVector3f> B_seq_;
  Eigen::MatrixXf Q_, R_, Pmat_, Acon_;
  Eigen::VectorXf qvec_, LL_, UL_;
  std::array<float,3> last_u3_{0,0,0};
  float pos_error_integral_{0.0f};
  float neg_error_integral_{0.0f};

  float last_error_{0.0f};

  std::deque<float> vol_dot_buffer_;
  const size_t vol_dot_window_size_ = 5;

  int qp_fail_count_{0};

  Eigen::MatrixXf S_bar_, T_bar_;
  Eigen::VectorXf x0_mpc_, Xref_mpc_, qtmp_, solution_;
};

struct SensorCalib {
  struct Channel { double offset{1.0}; double gain{250.0}; };
  double atm_offset{101.325};
  std::array<Channel, NUM_CAN_BOARDS> boards{};   // boards[0] = board_id 1
  double kpa_atm() const { return atm_offset; }
  double kpa(int board_id, uint16_t raw) const {
    int idx = board_id - 1;
    if (idx < 0 || idx >= (int)boards.size()) return kpa_atm();
    const auto& c = boards[(size_t)idx];
    return (double(raw) - c.offset) * c.gain + kpa_atm();
  }
};

class RefTcpClient {
public:
    struct Config {
        bool enable = false;
        std::string host = "169.254.46.254";
        int port = 2272;
        int expect_n = 12;
        double pressure_scale = 1.0 / 327.675;
    };
    using Callback = std::function<void(const std::vector<double>&)>;

    RefTcpClient(const Config& cfg, Callback cb)
    : cfg_(cfg), cb_(std::move(cb)) {
#ifdef __linux__
        th_ = std::thread([this](){ run_(); });
#else
        (void)cfg_; (void)cb_;
#endif
    }

    ~RefTcpClient() {
        stop_.store(true);
#ifdef __linux__
        if (client_fd_ >= 0) ::shutdown(client_fd_, SHUT_RDWR);
#endif
        if (th_.joinable()) th_.join();
    }

private:
    void run_() {
#ifndef __linux__
        return;
#else
        const size_t NUM_INTEGERS = (size_t)cfg_.expect_n;
        const size_t BYTES_PER_INT = sizeof(uint16_t);
        const size_t BUFFER_SIZE = NUM_INTEGERS * BYTES_PER_INT;

        std::vector<char> buffer(BUFFER_SIZE);
        std::vector<double> out_values(NUM_INTEGERS);

        while (!stop_.load()) {
            client_fd_ = -1;
            try {
                client_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
                if (client_fd_ < 0) throw std::runtime_error("Socket creation failed");

                sockaddr_in server_addr{};
                server_addr.sin_family = AF_INET;
                server_addr.sin_port = htons((uint16_t)cfg_.port);

                if (::inet_pton(AF_INET, cfg_.host.c_str(), &server_addr.sin_addr) <= 0)
                    throw std::runtime_error("Invalid address/ Address not supported");

                RCLCPP_INFO(rclcpp::get_logger("RefTcpClient"), "Connecting to reference server %s:%d...", cfg_.host.c_str(), cfg_.port);

                if (::connect(client_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
                    throw std::runtime_error("Connection Failed");

                RCLCPP_INFO(rclcpp::get_logger("RefTcpClient"), "Reference server connected.");

                while (!stop_.load()) {
                    size_t total_recd = 0;
                    while (total_recd < BUFFER_SIZE && !stop_.load()) {
                        ssize_t n = ::recv(client_fd_, buffer.data() + total_recd, BUFFER_SIZE - total_recd, 0);
                        if (n == 0) throw std::runtime_error("Server disconnected");
                        if (n < 0) {
                            if (errno == EINTR) continue;
                            throw std::runtime_error(std::string("Recv error: ") + strerror(errno));
                        }
                        total_recd += (size_t)n;
                    }

                    if (total_recd == BUFFER_SIZE) {
                        const char* ptr = buffer.data();
                        for (size_t i = 0; i < NUM_INTEGERS; ++i) {
                            uint16_t net_val;
                            std::memcpy(&net_val, ptr, BYTES_PER_INT);
                            ptr += BYTES_PER_INT;
                            uint16_t host_val = ntohs(net_val);
                            out_values[i] = static_cast<double>(host_val) * cfg_.pressure_scale;
                        }
                        cb_(out_values);
                    }
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("RefTcpClient"), "%s", e.what());
                if (client_fd_ >= 0) { ::close(client_fd_); client_fd_ = -1; }
                if (!stop_.load()) {
                    RCLCPP_INFO(rclcpp::get_logger("RefTcpClient"), "Reconnecting in 5 seconds...");
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                }
            }
        }
        if (client_fd_ >= 0) ::close(client_fd_);
#endif
    }

    Config cfg_;
    Callback cb_;
    std::thread th_;
    std::atomic<bool> stop_{false};
    int client_fd_ = -1;
};

class RefTcpServer {
public:
    struct Config {
        bool enable = false;
        int port = 2293;
        int pos_gid = 0;
        int neg_gid = 6;
    };
    using Callback = std::function<void(double pos_kpa, double neg_kpa)>;

    RefTcpServer(const Config& cfg, Callback cb);
    ~RefTcpServer();

private:
    void run_();
    Config cfg_;
    Callback cb_;
    std::thread th_;
    std::atomic<bool> stop_{false};
    std::atomic<int> server_fd_{-1};
    std::atomic<int> client_fd_{-1};
};

class Controller : public rclcpp::Node {
public:
  explicit Controller(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());
  ~Controller() override;

private:
  void on_sensor(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
  void on_volume(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void on_timer();
  void build_mpcs();
  void on_zero_calibration(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res);
  void inner_loop_1khz(float dt_ms);
  inline uint16_t clamp_pwm(int v) const { return static_cast<uint16_t>( std::min(std::max(v, PWM_CLAMP_MIN), PWM_CLAMP_MAX) ); }
  void publish_cmds();

  // 위치 제어: 엔코더 각도 → 압력 레퍼런스 변환 (on_timer 내 호출)
  void run_position_control(double dt_sec);

private:
  double sensor_filter_alpha_{1.0};
  std::vector<double> filt_state_;                      // [NUM_CAN_BOARDS]
  std::array<double,   NUM_CAN_BOARDS> filt_out_{};     // kPa, indexed by board_id-1
  bool filter_initialized_{false};

  std::vector<double> ref_snapshot_;
  std::vector<double> final_active_vols_ml_;

  int period_ms_{2};
  bool enable_thread_pinning_{true};
  std::vector<int64_t> cpu_pins_param_;

  int num_positive_channels_{8};
  int num_total_channels_{12};

  int channel_board_offset_{4};   // board_id = gid + channel_board_offset
  int P_pos_board_id_{1};         // board carrying P_line_pos sensor
  int P_neg_board_id_{2};         // board carrying P_line_neg sensor
  int P_macro_board_id_{3};       // board carrying P_line_macro sensor
  int P_macro_neg_board_id_{4};   // board carrying P_line_macro_neg sensor

  struct ChannelConfig {
    double pos_ki_micro{0.0};
    double pos_ki_macro{0.0};
    double pos_ki_atm{0.0};
    double neg_ki_micro{0.0};
    double neg_ki_macro{0.0};
    double neg_ki_atm{0.0};
    // 13-variable proportional valve model parameters
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

  std::vector<ChannelConfig> channel_configs_;

  double default_volume_ml_{1.0};
  bool   actuator_connected_{true};
  double tank_volume_pos_ml_{750.0};
  double tank_volume_neg_ml_{400.0};

  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_sensors_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_volumes_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_analog_;
  std::array<double, 9> encoder_angles_{};   // boards 17..25 [deg], index 0 = board 17
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_pwm_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_mpc_refs_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_active_vols_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_kpa_all_;

  std::unique_ptr<ThreadPool> pool_;
  std::mutex sensors_mtx_;
  std::array<uint16_t, NUM_CAN_BOARDS> sensors_raw_{};   // indexed by board_id-1

  std::array<uint16_t, PWM_TOTAL> zoh_{};    // [(board_id-1)*3 + v_idx]
  std::array<int,      PWM_TOTAL> inner_{};
  std::array<uint16_t, PWM_TOTAL> cmds_{};

  std::vector<std::unique_ptr<AcadosMpc>> mpcs_;
  uint64_t tick_{0};
  SensorCalib sensor_;

  struct MpcYaml {
    int   NP{5}; int n_x{1}; int n_u{3}; double Ts{0.01}; double Q_value{10.0}; double R_value{1.0};
    double ejector_k{0.005};
    double ejector_p_limit{11.325};
    double leakage_u_pos{0.0};
    double leakage_u_neg{0.0};
    double target_tc{0.2};
    double macro_threshold{30.0};
    double actuating_threshold{5.0};
  } mpc_;

  std::vector<double> vol_ml_;

  bool sys_valve_operate_{false};
  RefTcpClient::Config ref_client_cfg_;
  std::unique_ptr<RefTcpClient> ref_client_;
  RefTcpServer::Config ref_server_cfg_;
  std::unique_ptr<RefTcpServer> ref_server_;
  std::mutex mpc_ref_mtx_;
  std::vector<double> mpc_ref_kpa_;

  struct PidGains { double kp{0.5}, ki{0.0}, kd{0.0}, ref{150.0}; };
  struct PidState { double integ{0.0}; double prev_err{0.0}; bool has_prev{false}; };
  PidGains pid_pos_;
  PidState pid_pos_state_;
  double pid_out_min_{0.0}, pid_out_max_{100.0};
  int    pid_pos_pwm_index_{0};    // flat index into zoh_: (board_id-1)*3 + v_idx
  PidGains pid_neg_;
  PidState pid_neg_state_;
  int    pid_neg_pwm_index_{3};    // flat index into zoh_

  int    macro_switch_pwm_index_{3};   // flat index into zoh_

  // ──────────────────────────────────────────
  // 위치 제어기
  // ──────────────────────────────────────────
  int control_mode_{0};   // 0: 압력 제어, 1: 위치 제어

  struct PositionCtrlConfig {
    // PID 게인
    double kp{3.0};              // [kPa/deg]
    double ki{0.05};             // [kPa/(deg·s)]
    double kd{0.02};             // [kPa·s/deg]
    // 중력 피드포워드
    double kff_gravity{10.0};    // [kPa/(N·m)]
    double mass_kg{1.0};
    double link_length_m{0.2};
    // 마찰 보상
    double friction_kpa{2.0};
    double vel_deadband_dps{0.5};
    // 바이어스 압력
    double p_bias_pos_kpa{120.0};
    double p_bias_neg_kpa{90.0};
    double neg_coupling{0.5};
    // 출력 제한
    double p_pos_max_kpa{165.0};
    double p_pos_min_kpa{101.325};
    double p_neg_max_kpa{101.325};
    double p_neg_min_kpa{70.0};
    // 채널 매핑
    int actuator_idx{0};
    int pos_gid{0};
    int neg_gid{6};
    // 필터 / 초기값
    double vel_filter_alpha{0.05};
    double default_angle_deg{0.0};
    double integral_limit_kpa{20.0};
  } pos_ctrl_cfg_;

  struct PositionCtrlState {
    double integral{0.0};
    double prev_angle{0.0};
    double vel_filt{0.0};    // 필터된 각속도 [deg/s]
    bool   initialized{false};
  } pos_ctrl_state_;

  // TCP에서 수신한 목표 각도 (mpc_ref_mtx_ 로 보호)
  double target_angle_deg_{0.0};
  bool   pos_tcp_received_{false};

  // 위치 제어 디버그 토픽
  // data: [angle, angle_ref, p_pos_ref, p_neg_ref, p_pid, p_ff, p_friction, vel_dps]
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pos_dbg_;

  int log_channel_id_{-1};
  std::ofstream log_file_;

  std::vector<double> prev_vol_m3_;
  std::set<int> active_channels_;

  std::chrono::steady_clock::time_point start_time_;
  double elapsed_time_sec_ = 0.0;

  // Sensor zero-calibration at startup
  static constexpr int ZERO_SAMPLES = 250;   // ~0.5 sec at 500 Hz
  bool   sensor_zeroed_{true};   // true = use YAML offsets directly (no auto-calib at startup)
  int    sensor_zero_tick_{0};
  std::array<double, NUM_CAN_BOARDS> sensor_zero_sum_{};
  std::array<int,    NUM_CAN_BOARDS> sensor_zero_cnt_{};

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_calib_srv_;

  // Over-pressure safety: positive channels only
  // Hysteresis: latch ON at >= limit, release only when P < (limit - hysteresis_kpa)
  double pressure_safety_limit_kpa_{170.0};
  double pressure_safety_hysteresis_kpa_{10.0};
  std::array<bool, 12> safety_latched_{};   // per positive channel (gid 0..11)
};
