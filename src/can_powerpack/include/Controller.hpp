#pragma once

// DMY 빠른 클래스 지도
//   Controller   : ROS 입출력과 한 제어 틱을 소유하는 최상위 노드
//   PressureCtrl : 채널별 압력 PID (PressureCtrl.hpp)
//                   제어기 **밖**에서 하드웨어로 나가는 명령에 적용한다
//   QP           : qpOASES 박스 QP 래퍼 (PressureRefGen 이 쓴다)
//   ThreadPool   : 활성 채널의 제어 계산을 제어 틱 안에서 병렬 실행
//
// 이 브랜치는 제어기 내부를 비운 스켈레톤이다. 통신(CanBridge/토픽/TCP), 센서
// 환산, 목표압 수신, 위치·힘 외부 루프(PressureRefGen), 라인 PID, 과압
// 세이프티, PWM 발행은 그대로다. 비어 있는 것은 PressureCtrl::compute() 뿐이다.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Dense>

#include "PressureRefGen.hpp"

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
#include "PressureCtrl.hpp"

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

// QP Solver Wrapper (박스 제약 전용 — A_con 미사용)
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

        auto extract = [&](int nrows) {
            Eigen::VectorXd sol(nrows);
            solver_.getPrimalSolution(sol.data());
            solution = sol.cast<float>();
        };

        ++n_calls_;

        // hot start 를 먼저 시도하고, 실패하면 **같은 틱에서** cold start 로 재시도한다.
        //
        // 예전에는 hot start 실패를 그대로 반환해 그 틱의 Δu 를 버렸다 (그 채널의 MPC 가
        // 그 틱만 피드포워드로 퇴화하고, 다음 틱에나 cold start 로 복구). 그런데 이 MPC 는
        // 매 틱 수치 야코비안으로 A·B 를 다시 만들고 u_ref 가 바뀌면 박스 경계(LL/UL)도
        // 전부 바뀌므로, active set 이 크게 달라져 hot start 가 실패하는 것은 **정상**이다.
        // 버릴 이유가 없고, 실패한 틱을 버리는 쪽이 오히려 제어를 간헐적으로 열어버린다.
        if (initialized_) {
            int nWSR = HOT_WSR;
            if (solver_.hotstart(Hd.data(), gd.data(), nullptr,
                                 lbd.data(), ubd.data(), nullptr, nullptr, nWSR)
                == qpOASES::SUCCESSFUL_RETURN) {
                extract((int)H.rows());
                return true;
            }
            initialized_ = false;
            ++n_hot_fail_;
        }

        int nWSR = COLD_WSR;
        initialized_ = (solver_.init(Hd.data(), gd.data(), nullptr,
                                     lbd.data(), ubd.data(), nullptr, nullptr, nWSR)
                        == qpOASES::SUCCESSFUL_RETURN);
        if (initialized_) { extract((int)H.rows()); return true; }

        ++n_hard_fail_;
        return false;
    }

    // 진단용 (건강하면 둘 다 0 에 가깝다). hot start 실패는 cold start 로 즉시 복구되어
    // 성능에 드러나지 않으므로 계측 없이는 보이지 않는다.
    struct Stats { int64_t calls, hot_fail, hard_fail; };
    Stats take_stats() {
        Stats s{n_calls_, n_hot_fail_, n_hard_fail_};
        n_calls_ = n_hot_fail_ = n_hard_fail_ = 0;
        return s;
    }

private:
    // hot start 는 직전 active set 을 재사용하므로 적은 반복으로 끝나야 정상이지만,
    // 매 틱 재선형화 때문에 10 회로는 자주 부족했다 (실측 실패율 참조). cold start 는
    // 처음부터 푸는 경로라 넉넉히 준다.
    static constexpr int HOT_WSR  = 30;
    static constexpr int COLD_WSR = 100;

    qpOASES::SQProblem solver_;
    qpOASES::Options options_;
    bool initialized_{false};
    int64_t n_calls_{0}, n_hot_fail_{0}, n_hard_fail_{0};
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
        bool all_channels = false;  // pressure mode: true면 2개가 아니라 전체 채널 double 수신
        int num_values = 2;   // doubles per TCP message (2 = pressure mode, N = N-axis angle mode)
    };
    using Callback = std::function<void(const std::vector<double>&)>;

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
  void build_ctrls();
  void on_zero_calibration(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res);
  void inner_loop_1khz(float dt_ms);
  inline uint16_t clamp_pwm(int v) const { return static_cast<uint16_t>( std::min(std::max(v, PWM_CLAMP_MIN), PWM_CLAMP_MAX) ); }
  void publish_cmds();

  // 위치 제어: 엔코더 각도 → 압력 레퍼런스 변환 (on_timer 내 호출)
  void run_position_control(double dt_sec);

  // control_mode 2: 위치 PID → 목표 토크 → 최적화 생성기 → 12개 목표 압력
  void run_optimized_pressure_ref(double dt_sec);

private:
  double sensor_filter_alpha_{1.0};
  std::vector<double> filt_state_;                      // [NUM_CAN_BOARDS]
  std::array<double,   NUM_CAN_BOARDS> filt_out_{};     // kPa, indexed by board_id-1
  // 필터 **전** 압력. 측정 경로에 LPF 가 직렬 2단(브리지 α=0.2 + 여기 α=0.2, 각 τ≈9 ms)
  // 걸려 있어 filt_out_ 은 약 18 ms 낡았다. 지연 없는 값이 필요한 제어기를 위해
  // 생값도 함께 들고 가서 PressureCtrl::Input.P_meas_raw_kpa 로 넘긴다.
  std::array<double,   NUM_CAN_BOARDS> raw_out_{};
  bool filter_initialized_{false};

  std::vector<double> ref_snapshot_;
  std::vector<double> final_active_vols_ml_;

  int period_ms_{2};
  bool enable_thread_pinning_{true};
  std::vector<int64_t> cpu_pins_param_;

  int num_positive_channels_{8};
  int num_total_channels_{12};
  int num_actuators_{1};   // 액추에이터(축) 수 → 압력채널/엔코더/위치제어기 모두 이 값만큼 활성화

  int channel_board_offset_{4};   // board_id = gid + channel_board_offset
  int P_pos_board_id_{1};         // board carrying P_line_pos sensor
  int P_neg_board_id_{2};         // board carrying P_line_neg sensor
  int P_macro_board_id_{3};       // board carrying P_line_macro sensor
  int P_macro_neg_board_id_{4};   // board carrying P_line_macro_neg sensor

  // 13-variable 비례밸브 모델 파라미터 한 세트.
  //
  // **밸브마다 다르다.** `RUNBOOK.md` 의 밸브 피팅은 채널당 3개(micro/atm/macro)를 따로
  // 맞추고 `valve_fit_solve.py` 가 `channel_config.chN.{micro,atm,macro}.*` 로 쓴다.
  // 예전에는 로더가 채널당 한 세트만 읽어 세 밸브에 같은 값을 써서 **피팅 결과를 쓸 수
  // 없었다** (README 8.8 의 "밸브별 파라미터 로더 미완"). 이제 세 세트를 읽고, 없으면
  // 평면 `chN.*` 로 폴백한다.
  struct Valve13 {
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

  struct ChannelConfig {
    // 밸브별 13-parameter. 인덱스는 PressureCtrl::ValveIdx 와 동일 (0=micro, 1=macro, 2=atm).
    std::array<Valve13, 3> v{};
    double chamber_volume_ml{-1.0};   // 피팅으로 구한 챔버 부피 (<0 이면 미측정)
    bool   per_valve_loaded{false};   // 진단용 — 피팅 파일이 실제로 로드됐는지
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
  // 채널 PID 내부 — 항별로 찍어야 적분 포화·표 오차·피드포워드를 분리할 수 있다.
  // 채널당 CH_DBG_N 개 × 12 채널. 순서는 Controller.cpp 의 발행부 주석 참조.
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_chan_dbg_;
  static constexpr int CH_DBG_N = 12;
  std::vector<std::array<double, CH_DBG_N>> chan_dbg_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_active_vols_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_kpa_all_;

  std::unique_ptr<ThreadPool> pool_;
  size_t pool_threads_{2};
  std::mutex sensors_mtx_;
  std::array<uint16_t, NUM_CAN_BOARDS> sensors_raw_{};   // indexed by board_id-1

  std::array<uint16_t, PWM_TOTAL> zoh_{};    // [(board_id-1)*3 + v_idx]
  std::array<int,      PWM_TOTAL> inner_{};
  std::array<uint16_t, PWM_TOTAL> cmds_{};

  // 채널(gid)마다 하나. 활성 채널만 만든다.
  std::vector<std::unique_ptr<PressureCtrl>> ctrls_;
  uint64_t tick_{0};
  double wall_elapsed_sec_{0.0};   // 실제 벽시계 경과 — 틱 간격 진단용 (제어에는 쓰지 않는다)
  SensorCalib sensor_;

  // 크래킹 임계의 정의값 — "닫힘" 으로 볼 유효면적 비율 (A_eff/A_max).
  // ── 밸브 데드존 (죽은 구간) 표 ────────────────────────────────────────────
  // 밸브 모델은 제어에 쓰지 않는다. 이 값은 **실측**이다: 각 밸브가 열리기 시작하는
  // 지령 [%]. scripts/valve_deadzone.py 가 실기에서 재서 yaml 에 적는다.
  //
  //   u_hw = deadzone(차압) + u_pid       (죽은 구간을 지나간 지점에서 PID 가 출발)
  //
  // **차압의 함수**다. 상류압이 스풀을 여는 방향으로 밀기 때문에 차압이 크면 더 낮은
  // 지령에서 열린다 (실측: 챔버압 10 kPa 변화 → 임계 0.5 %p 이동). 상수 하나로
  // 보상하면 사인파 반주기는 덜 열고 반주기는 더 열게 되므로 표로 보간한다.
  //
  // 모델 역산을 쓰던 예전 방식은 버렸다 — 13-parameter 가 6채널 공용이라 실제 임계와
  // 채널별로 최대 2 %p 어긋났고, 양압 micro 가 임계 **아래**에 놓여 간헐 펄스로만
  // 열렸다 (20260908_171520: 양압 RMSE 2.0~2.7 / 음압 0.9).
  bool dz_enable_{true};

  // 표에서 **빼는** 안전 여유 [%p]. 0 이면 표를 그대로 쓴다.
  //
  // 과보상이 부족보상보다 훨씬 나쁘기 때문에 둔다. 표가 실제 임계보다 높으면
  // u_pid 가 0 을 조금만 넘어도 밸브가 이미 임계 위에서 열려 **유량 0 을 만드는
  // 지령이 아예 없다**. 그러면 루프가 릴레이가 되어 반드시 진동한다 —
  // 실측 20260909_215513: 양압 ch0 에 과보상 +2.68 %p 가 걸려 5.70 Hz,
  // p-p 30~40 kPa 로 떨렸고 micro↔atm 이 초당 11.5 회 번갈았다 (동시 열림 0%).
  // 반대로 부족보상이면 u_pid 가 여유만큼 커져야 열리기 시작할 뿐, 그 구간은
  // 적분이 메운다. 표의 채널 간 편차가 2~4 %p 이므로 그 정도를 빼 두는 것이 안전하다.
  double dz_margin_pct_{0.0};

  // 채널별 여유 [%p] — channel_config.chN.deadzone.margin_pct (없으면 위 전역값).
  // 표의 정확도가 채널마다 다르므로(측정 sd 0.4~1.7 %p) 문제 채널만 깊게 뺄 수 있어야
  // 한다. 전역값을 올리면 멀쩡한 채널까지 죽은 구간이 넓어져 느려진다.
  // [채널][밸브] 여유 [%p]. channel_config.chN.deadzone.margin_{micro,atm,macro}_pct
  //  → 없으면 chN.deadzone.margin_pct → 없으면 전역 valve_deadzone.margin_pct.
  //
  // 밸브별로 나눈 이유: 같은 채널 안에서도 표의 오차가 밸브마다 반대다. 실측
  // 20260910 (표 대비 유량 0 교차 x0, x0<0 = 표가 높다 = 과보상):
  //     ch3 micro −0.46 / atm +3.44,  ch5 micro −0.07 / atm +3.60
  // 채널 하나로 묶으면 micro 를 맞추면 atm 이 4 %p 죽고, atm 을 맞추면 micro 가
  // 릴레이가 된다. 실제로 ch3·ch5 의 하강이 상승보다 2~3 배 느렸던 원인이다.
  std::vector<std::array<double, 3>> dz_margin_ch_;

  // ── 밸브 파킹 ─────────────────────────────────────────────────────────────
  // 쉬는 밸브를 0 으로 두지 않고 (표 최솟값 − park_below_pct) 에 걸어 둔다.
  //
  // 왜: 솔레노이드 코일은 인덕턴스가 있어 0 에서 지령을 주면 전류가 붙는 데
  // 시간이 걸린다. 미리 흘려 두면 열어야 할 때 그 시간이 빠진다. 표 최솟값보다
  // 더 낮게 두므로 **어떤 차압에서도 유량은 0** 이다 (데드존은 차압이 커질수록
  // 낮아지니 최솟값이 최악의 경우다).
  bool   dz_park_enable_{false};
  double dz_park_below_pct_{5.0};
  // [채널][밸브] 실제 파킹 지령 [%] — 기동 때 한 번 계산한다.
  std::vector<std::array<double, 3>> dz_park_ch_;

  // 표가 없는 밸브에 쓰는 상수 [%] — valve_deadzone.{micro,macro,atm}_pct
  std::array<double, 3> dz_pct_{{0.0, 0.0, 0.0}};

  // macro(부스트) 밸브를 쓰나. false = 지령을 항상 0 으로 내고 파킹도 안 한다.
  bool use_macro_{false};

  // 차압 → 데드존 지령. 표가 비어 있으면 flat 을 상수로 쓴다.
  struct DzTable {
    std::vector<double> dp_kpa;   // 오름차순 (로드할 때 검사한다)
    std::vector<double> u_pct;    // 같은 길이
    double flat{0.0};             // 표가 없을 때의 상수

    // 표 전체에서 가장 낮은 값 [%] — 파킹 기준. 어떤 차압에서도 이 값 아래면
    // 확실히 닫혀 있다 (데드존은 차압이 커질수록 낮아지므로 최솟값이 최악의 경우다).
    double u_min() const {
      if (u_pct.empty()) return flat;
      double m = u_pct.front();
      for (double v : u_pct) m = std::min(m, v);
      return m;
    }

    // 선형보간. 표 밖은 **클램프한다** — 외삽하면 잰 적 없는 차압에서 밸브를 활짝
    // 열어 버릴 수 있고, 데드존은 차압에 대해 완만한 포화 곡선이라 끝값이 더 안전하다.
    double at(double dp) const {
      const size_t n = dp_kpa.size();
      if (n == 0 || u_pct.size() != n) return flat;
      if (n == 1 || dp <= dp_kpa.front()) return u_pct.front();
      if (dp >= dp_kpa.back()) return u_pct.back();
      size_t k = 1;
      while (k < n && dp_kpa[k] < dp) ++k;
      const double d0 = dp_kpa[k - 1], d1 = dp_kpa[k];
      const double w = (d1 > d0) ? (dp - d0) / (d1 - d0) : 0.0;
      return u_pct[k - 1] + w * (u_pct[k] - u_pct[k - 1]);
    }
  };

  // [채널][밸브] — 밸브 인덱스는 PressureCtrl::V_MICRO/V_MACRO/V_ATM.
  std::vector<std::array<DzTable, 3>> dz_ch_;
  std::vector<std::array<float, 3>> u_hw_pct_;

  std::vector<double> vol_ml_;

  // 채널별 부피 배율 (channel_config.chN.volume_scale). 기하 모델 값에 곱한다.

  std::vector<double> vol_scale_;

  bool valve_operate_{false};   // system_parameters.valve_operate — false 면 PWM 을 내지 않는다
  RefTcpClient::Config ref_client_cfg_;
  std::unique_ptr<RefTcpClient> ref_client_;
  RefTcpServer::Config ref_server_cfg_;
  std::unique_ptr<RefTcpServer> ref_server_;
  std::mutex mpc_ref_mtx_;
  std::vector<double> mpc_ref_kpa_;

  // ── 채널 압력 PID 게인 로더 ───────────────────────────────────────────────
  // 채널 × 양/음압 × 상승/하강 을 **모두 개별 튜닝**할 수 있게 6단으로 겹쳐 읽는다.
  // 뒤 단계가 앞 단계를 덮어쓰고, yaml 에 없는 키는 앞 단계 값을 그대로 쓴다.
  //
  //   1  ChannelPID.*                          전 채널·전 방향
  //   2  ChannelPID.{up,down}.*                방향별 (전 채널)
  //   3  ChannelPID.{pos,neg}.*                측별   (전 방향)
  //   4  ChannelPID.{pos,neg}.{up,down}.*      측별·방향별
  //   5  channel_config.chN.pid.*              채널별 (전 방향)
  //   6  channel_config.chN.pid.{up,down}.*    채널별·방향별
  //
  // up = 압력을 올리는 방향, down = 내리는 방향 (채널 종류와 무관한 물리 방향).
  PressureCtrl::Gains load_gains(const std::string& prefix, PressureCtrl::Gains g);
  PressureCtrl::Gains gains_for(int gid, bool is_positive, const char* dir);

  // ── 아래는 공유 레일(라인) PID — 채널 PID 와 다른 루프다 ──────────────────
  // i_limit: 적분 '항'(출력 %p 단위) 상한. 레일은 펌프 충전률이 한계인 구간이 있어
  //          벤트를 다 닫아도 오차가 안 줄어든다. 그때 적분이 출력 상한까지 차면
  //          목표를 지난 뒤 되돌리는 데 수십 초가 걸린다(실측 레일 정착 50 s).
  struct PidGains { double kp{0.5}, ki{0.0}, kd{0.0}, ref{150.0}, i_limit{100.0}; };
  struct PidState { double integ{0.0}; double prev_err{0.0}; bool has_prev{false}; };

  // ══════════════════════════════════════════════════════════════════════
  //  레일 피드포워드 (RailFF) — 20260912 실측 맵의 역함수
  // ══════════════════════════════════════════════════════════════════════
  // 레일 둘은 펌프 하나로 이어진 **닫힌 회로**라 서로 독립이 아니다. 방출을
  // 열면 회로에서 기체가 빠져 P+ 만이 아니라 P− 도 내려가고, 유입을 열면 P− 만이
  // 아니라 P+ 도 오른다. SISO 두 개(예전 LinePID)는 서로의 작용을 외란으로만 봐서
  // 160 목표에서 정착에 120 초가 걸렸고 40~50 초 주기로 왕복했다.
  //
  // 실측(rail_map.py, 21점)에서 구조가 거의 **삼각형**임이 드러났다:
  //   · P− 는 유입이 거의 단독 결정 — 유입 +1.27 kPa/%p 대 방출 −0.32
  //   · P+ 는 그 유입 아래에서 방출이 결정
  // 그래서 순차적으로 푼다: 유입 = g(P−_ref), 방출 = h(P+_ref, 유입).
  //
  // **적분이 동작점을 만들 필요가 없어지는 것**이 요점이다. 예전에는 u 의 거의
  // 전부를 적분이 만들었고(정착 시 u 46 중 적분 46) 그래서 동작점이 바뀔 때마다
  // 적분이 처음부터 다시 쌓여야 했다. 채널 쪽 kv 피드포워드와 같은 구조다.
  struct RailFF {
    bool enable{false};
    std::vector<double> admit_pneg, admit_u;                 // P− → 유입 %
    std::vector<double> vent_admit;                          // 곡선의 유입 수준
    std::vector<std::vector<double>> vent_ppos, vent_u;      // 곡선별 P+ → 방출 %
    // 국소 이득 |dP+/d방출| 가 1.67~9.88 kPa/%p 로 **5.9 배** 달라진다 (최대 이득
    // 위치가 유입에 따라 움직인다). 고정 게인으로는 한쪽이 과하고 다른 쪽이 무의미하다.
    double gain_ref{0.0}, gain_min{0.3}, gain_max{3.0};

    static double interp(const std::vector<double>& xs,
                         const std::vector<double>& ys, double x);
    double admit_at(double p_neg) const;
    double vent_at(double p_pos, double admit) const;
    double vent_slope(double p_pos, double admit) const;     // |dP+/d방출|
    bool ok() const {
      return enable && admit_pneg.size() >= 2 && !vent_admit.empty();
    }
    // 표가 덮는 범위. **밖을 목표로 주면 ff 가 끝값에 붙고 PID 와 싸운다** —
    // 20260912 에 P− 목표 62 (표 상한 56.5 밖) 를 줬더니 ff 는 유입 100 을,
    // PID 는 "유입을 닫아라" 를 동시에 내서 개도가 0 이 됐다. 유입은 펌프
    // 흡입구라 0 이면 펌프가 굶어 P+ 도 못 오른다.
    double pneg_min() const { return admit_pneg.front(); }
    double pneg_max() const { return admit_pneg.back(); }
    double ppos_min() const;
    double ppos_max() const;
  };
  RailFF rail_ff_;
  double rail_pp_prev_{0.0};   // 진단: 직전 주기의 P+ (펌프 정지 감지)
  double rail_u_pos_{0.0}, rail_u_neg_{0.0}, rail_gs_pos_{1.0};  // 진단용 스냅샷
  // 레일 상태를 토픽으로도 낸다. 모드 0 에서는 pressure_ref_dbg 가 안 나와서
  // pp_monitor 의 레일 Ref 칸이 **비어 있었다** — 목표가 바뀌었는지 화면으로
  // 확인할 방법이 없었다.
  //   controller/rail_dbg : [P+목표, P−목표, ff방출, ff유입, 개도방출, 개도유입]
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_rail_dbg_;

  // ── 레일 목표를 런타임에 받는다 ────────────────────────────────────────
  //   controller/rail_ref_kpa : [P+ 목표, P− 목표]  (Float64MultiArray, kPa abs)
  // yaml 의 LinePID.{pos,neg}.ref 를 덮어쓴다. **control_mode 2 에서는 무시된다** —
  // 그때는 PressureRefGen 이 매 틱 레일 셋포인트를 다시 쓰므로 외부 값이 곧
  // 지워지고, 두 주인이 싸우는 상태가 된다.
  // NaN 이면 "아직 안 받았다" = yaml 값을 그대로 쓴다.
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_rail_ref_;
  std::atomic<double> rail_ref_pos_{std::numeric_limits<double>::quiet_NaN()};
  std::atomic<double> rail_ref_neg_{std::numeric_limits<double>::quiet_NaN()};
  double rail_ref_pos_min_{101.325}, rail_ref_pos_max_{250.0};
  double rail_ref_neg_min_{10.0},    rail_ref_neg_max_{101.325};
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

  // 축(actuator) 하나의 위치 제어 설정.
  //
  // mode 1(휴리스틱)과 mode 2(최적화 생성기)가 같은 구조체를 공유하지만 쓰는 필드가
  // 다르다. 어느 값이 살아 있는지 yaml 만 보고 알 수 없었으므로 mode 1 전용 게인을
  // `m1` 하위 구조체로 분리했다. mode 2 는 `m1` 을 전혀 읽지 않는다 — PID 는
  // TorquePID.axis*, 압력 분배는 PressureRefGen 의 슬루 박스 최적화가 담당한다.
  struct PositionCtrlConfig {
    // ── 공용 (mode 1 / 2) ──────────────────────────────────────────────
    // 채널 / 엔코더 매핑
    int    actuator_idx{0};      // 엔코더 인덱스 (0 = board 17)
    int    pos_gid{0};
    int    neg_gid{6};
    // 중력 피드포워드용 부하 (mode 1 은 kPa 로 환산, mode 2 는 N·m 로 직접 사용)
    double mass_kg{1.0};
    double link_length_m{0.2};
    // 채널 압력 정격 [kPa abs] — mode 1 은 출력 클램프, mode 2 는 생성기 슬루 박스의
    // 상/하한으로 쓴다 (Controller 생성자에서 게이지 Pa 로 변환해 주입).
    double p_pos_max_kpa{165.0};
    double p_neg_min_kpa{70.0};
    // 각속도 추정 LPF / 초기 목표각
    double vel_filter_alpha{0.05};
    double default_angle_deg{0.0};

    // ── mode 1 (휴리스틱 위치 제어) 전용 ────────────────────────────────
    struct Mode1 {
      // PID 게인 (각도 오차 → 압력 보정)
      double kp{3.0};              // [kPa/deg]
      double ki{0.05};             // [kPa/(deg·s)]
      double kd{0.02};             // [kPa·s/deg]
      double integral_limit_kpa{20.0};
      // 중력 FF 환산 계수 [kPa/(N·m)] — mode 2 는 환산이 불필요해 쓰지 않는다
      double kff_gravity{10.0};
      // 쿨롱 마찰 보상
      double friction_kpa{2.0};
      // 바이어스(평형점) 압력 + 음압 차동 비율
      double p_bias_pos_kpa{120.0};
      double p_bias_neg_kpa{90.0};
      double neg_coupling{0.5};
      // 출력 제한 중 mode 1 만 쓰는 쪽 (반대쪽 한계는 공용 정격)
      double p_pos_min_kpa{101.325};
      double p_neg_max_kpa{101.325};
      // 압력 레퍼런스 슬루레이트 제한 [kPa/s]. 목표압력이 한 번에 점프하면 밸브모델-실제
      // 불일치로 안전한계에 부딪히는 릴레이 진동이 난다 (20260818).
      // mode 2 는 생성기의 슬루 박스가 같은 역할을 물리적으로 하므로 불필요하다.
      double ref_slew_kpa_per_s{3.0};
    } m1;
  };

  struct PositionCtrlState {
    double integral{0.0};
    double prev_angle{0.0};
    double vel_filt{0.0};    // 필터된 각속도 [deg/s]
    bool   initialized{false};
    double p_pos_ref_filt{101.325};   // 슬루레이트 제한 후 마지막 P+ 레퍼런스 [kPa]
    double p_neg_ref_filt{101.325};   // 슬루레이트 제한 후 마지막 P- 레퍼런스 [kPa]
  };

  // 축(actuator)별 위치 제어기 설정/상태. 크기 = num_actuators_
  std::vector<PositionCtrlConfig> pos_ctrl_cfg_;
  std::vector<PositionCtrlState>  pos_ctrl_state_;

  // TCP에서 수신한 축별 목표 각도 (mpc_ref_mtx_ 로 보호). 크기 = num_actuators_
  std::vector<double> target_angle_deg_;
  std::vector<double> target_angle_slewed_;   // 슬루 제한을 지난 목표 (제어가 쓰는 값)
  double target_slew_dps_{0.0};               // 각도 목표 슬루 [deg/s], 0=끔
  // 목표가 측정각보다 앞설 수 있는 최대 오차 [deg]. 0 이하면 끔.
  // 한 방향 힘 시스템이라 하강은 중력에 맡길 수밖에 없다 — 목표가
  // 달아나면 τ_ref 가 0 으로 떨어져 자유낙하한다 (S-30 참조).
  double target_follow_band_deg_{5.0};
  // 목표 슬루 속도 [deg/s] — D 항의 피드포워드. slew_targets() 가 매 틱 채운다.
  std::vector<double> target_slew_rate_;
  // D 항이 목표 속도를 얼마나 빼고 볼지 (0=예전 −kd·vel, 1=완전 피드포워드).
  double kd_vel_ff_{1.0};
  // 추종 오차가 밴드에 붙어 있는 연속 틱 수 (축별). 오래 붙으면 경고한다.
  std::vector<int> band_sat_ticks_;
  // 슬루 상태를 현재 각도에서 한 번 출발시켰나 (기동 계단 방지).
  bool slew_seeded_{false};
  int  slew_seed_ticks_{0};
  void slew_targets(double dt_sec);
  bool   pos_tcp_received_{false};

  // 위치 제어 디버그 토픽
  // data: 축마다 8개씩 이어붙임 [angle, angle_ref, p_pos_ref, p_neg_ref, p_pid, p_ff, p_friction, vel_dps] × num_actuators_
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pos_dbg_;

  // ──────────────────────────────────────────
  // control_mode 2: 최적화 기반 압력 레퍼런스 생성기
  // ──────────────────────────────────────────
  std::unique_ptr<PressureRefGen> refgen_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_refgen_dbg_;

  int      gen_period_ms_{20};       // 생성기 주기 (제어 tick 여러 개마다 1회)
  // 생성기가 낸 압력 레퍼런스의 슬루 제한 [kPa/s]. 0 이하면 끔.
  // 모드 1 의 ref_slew_kpa_per_s 와 같은 역할인데 모드 2 에는 없었다.
  double   gen_ref_slew_kpa_s_{150.0};
  uint64_t gen_tick_{0};
  bool     gen_use_ej_meas_{true};   // board 4 측정 음압을 이젝터 하류압으로 사용

  // 액추에이터 기하 (단일 출처). 부피식·토크 환산이 모두 여기서 나온다.
  double piston_area_mm2_{M_PI * 25.0 * 25.0};   // Ø50 mm
  double reel_radius_mm_{25.0};                  // 조인트 ~ 피스톤 로드 (= 부피식 mm/rad)
  double vol_offset_pos_mm_{40.0};
  double vol_offset_neg_mm_{90.0};

  // 축별 토크 PID (mode 2). 게인 단위는 N·m/deg 계열.
  struct TorquePid {
    double kp{0.0786}, ki{0.0295}, kd{0.0049};
    double integ_limit_nm{2.0};
    double friction_nm{0.30};
    // 마찰 보상 밴드 [deg]. 오차가 이 안이면 friction_nm 을 선형으로 준다.
    // 0 에 가까우면 예전의 하드 sign 과 같아져 목표 근처에서 ±friction_nm 이
    // 계단으로 뒤집힌다 (S-29 참조).
    double friction_band_deg{1.0};
    // 중력 피드포워드 배율. 액추에이터 미연결 시험에서 목표 압력을 낮추는 데 쓴다.
    // 목표 압력에 거의 선형으로 반영된다. 액추에이터를 붙이면 1.0 으로 되돌릴 것.
    double tau_ff_gain{1.0};
  };
  std::vector<TorquePid>  tau_pid_;
  std::vector<double>     tau_integ_;

  // macro 게이트 임계 — 생성기의 축별 **유량 부족률** [0,1] 이 이 값을 넘으면 macro 를 연다.
  // 무차원이라 "레일이 이번 스텝 수요의 몇 %를 못 대면 부스트를 부른다"로 읽힌다.
  std::vector<double> gen_starve_pos_, gen_starve_neg_;   // 진단용 [%]

  // gid → 채널 제어기 조회
  PressureCtrl* ctrl_for_gid(int gid) const;

  // 생성기 결과 ZOH (생성기 주기 사이 유지)
  std::vector<double> gen_pos_ref_kpa_, gen_neg_ref_kpa_;
  double gen_rail_pos_sp_kpa_{155.0}, gen_rail_neg_sp_kpa_{30.0};
  bool   gen_has_result_{false};

  int log_channel_id_{-1};
  std::ofstream log_file_;

  std::vector<double> prev_vol_m3_;
  std::set<int> active_channels_;

  std::chrono::steady_clock::time_point start_time_;
  double elapsed_time_sec_ = 0.0;

  // Sensor zero-calibration at startup
  static constexpr int ZERO_SAMPLES = 250;   // ~0.5 sec at 500 Hz
  // 0점 재보정이 yaml 기준에서 얼마나 벗어났는지 알리기 위해 원본을 보관한다.
  std::array<double, NUM_CAN_BOARDS> yaml_offset_{};
  double zero_tolerance_kpa_{8.0};
  // 액추에이터 미연결이면 엔코더도 미연결이다 — 각도를 0° 로 고정한다.
  bool encoder_zero_when_disconnected_{true};

  // ── 실측 제어 주기 ──────────────────────────────────────────────────
  // 제어 루프는 board/sensors 도착에 물려 돈다. 컨트롤러가 그 속도를 못 따라가면
  // 실제 틱 간격이 period_ms 보다 길어지는데, 예전에는 dt 를 **항상 period_ms 로**
  // 썼다 (실기 계측: 가정 2.0 ms, 실측 3.09 ms — 1.55배). 그 괴리는 특히
  // advance_valve_estimate 가 밸브 상태를 실시간 대비 1.55배 느리게 전진시켜,
  // 모델이 밸브를 굼뜨다고 보고 과도하게 명령하게 만든다 (첫스텝 포화 99%).
  // 실측 간격을 EMA 로 잡아 dt 로 쓴다. 스파이크는 클램프로 막는다.
  bool   use_measured_dt_{true};
  double dt_meas_sec_{-1.0};
  std::chrono::steady_clock::time_point last_tick_time_{};
  double dt_ctrl_sec_{0.002};        // 이번 틱에 실제로 쓰는 dt
  bool   sensor_zeroed_{true};   // true = use YAML offsets directly (no auto-calib at startup)
  // 보드별 "프레임을 한 번이라도 받았나". 전부 받기 전에는 제어를 시작하지 않는다.
  std::array<bool, 16> sensor_seen_{};
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
