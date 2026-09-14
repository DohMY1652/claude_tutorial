#include "Controller.hpp"

// ============================================================================
// DMY 코드 읽기 안내 — 이 파일은 "제어 계층을 연결하는 곳"이다.
//
// 이 브랜치는 **제어기 내부를 비운 스켈레톤**이다. 아래 사슬에서 마지막 채널
// 압력 제어기만 플레이스홀더(PressureCtrl::compute)이고 나머지는 그대로다.
//
// 위치 제어(control_mode=2)의 호출 사슬:
//   on_timer
//     -> slew_targets                         사용자 목표각에 속도 제한
//     -> run_optimized_pressure_ref           각도오차 -> 토크 -> 힘
//        -> PressureRefGen::step              힘 -> 양/음압 챔버 목표 + 레일 목표
//     -> PressureCtrl::compute (활성 채널별 병렬)   ★ 챔버 목표압 -> 밸브 3개 명령
//     -> PressureCtrl::to_pwm                 % -> 보드 PWM 슬롯 (0..4095)
//     -> LinePID, MacroSwitch, Safety         공유 레일과 최종 안전 처리
//     -> publish_cmds                         board/pwm_cmd 발행
//
// 압력 제어(control_mode=0)에서는 위 두 단계가 없고, RefTcpServer/RefTcpClient 로
// 받은 축별 목표압이 mpc_ref_kpa_ 에 그대로 들어가 PressureCtrl 로 간다.
//
// 압력 단위도 계층마다 다르다. 이 파일과 PressureCtrl 은 kPa absolute,
// PressureRefGen 은 Pa gauge 를 쓴다. 변환 경계는 run_optimized_pressure_ref 에
// 모여 있다.
// 전체 그림과 추천 읽기 순서는 저장소 루트의 DMY_MPPI_CODE_READING_GUIDE.md 참조.
// ============================================================================

#include <chrono>
#include <algorithm>
#include <cmath>
#include <optional>
#include <limits>
#include <set>
#include <fstream>

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

using std::placeholders::_1;
using namespace std::chrono_literals;

// ================================
// RefTcpServer
// ================================
RefTcpServer::RefTcpServer(const Config& cfg, Callback cb)
: cfg_(cfg), cb_(std::move(cb))
{
#ifdef __linux__
    if (cfg_.enable)
        th_ = std::thread([this](){ run_(); });
#endif
}

RefTcpServer::~RefTcpServer()
{
    stop_.store(true);
    int sfd = server_fd_.exchange(-1);
    int cfd = client_fd_.exchange(-1);
#ifdef __linux__
    if (sfd >= 0) ::close(sfd);
    if (cfd >= 0) ::close(cfd);
#endif
    if (th_.joinable()) th_.join();
}

void RefTcpServer::run_()
{
#ifndef __linux__
    return;
#else
    int sfd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sfd < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("RefTcpServer"), "socket() failed");
        return;
    }
    server_fd_.store(sfd);

    int opt = 1;
    ::setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons((uint16_t)cfg_.port);

    if (::bind(sfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("RefTcpServer"), "bind() failed on port %d", cfg_.port);
        ::close(sfd); server_fd_.store(-1);
        return;
    }
    ::listen(sfd, 1);
    RCLCPP_INFO(rclcpp::get_logger("RefTcpServer"),
                "Listening for refs on port %d  [%d doubles per message]",
                cfg_.port, cfg_.num_values);

    while (!stop_.load()) {
        fd_set fds; FD_ZERO(&fds); FD_SET(sfd, &fds);
        struct timeval tv{1, 0};
        if (::select(sfd + 1, &fds, nullptr, nullptr, &tv) <= 0) continue;

        sockaddr_in cli{}; socklen_t cli_len = sizeof(cli);
        int cfd = ::accept(sfd, (struct sockaddr*)&cli, &cli_len);
        if (cfd < 0) continue;
        client_fd_.store(cfd);

        // 1s recv timeout so the inner loop can check stop_
        struct timeval rtv{1, 0};
        ::setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &rtv, sizeof(rtv));
        RCLCPP_INFO(rclcpp::get_logger("RefTcpServer"), "Client connected.");

        const size_t MSG = (size_t)cfg_.num_values * sizeof(double);
        std::vector<uint8_t> buf(MSG);
        bool ok = true;

        while (!stop_.load() && ok) {
            size_t total = 0;
            while (total < MSG && !stop_.load()) {
                ssize_t n = ::recv(cfd, buf.data() + total, MSG - total, 0);
                if (n == 0) { ok = false; break; }
                if (n < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                    ok = false; break;
                }
                total += (size_t)n;
            }
            if (!ok || total < MSG) break;

            std::vector<double> vals((size_t)cfg_.num_values);
            for (size_t i = 0; i < vals.size(); ++i)
                std::memcpy(&vals[i], buf.data() + i * sizeof(double), sizeof(double));
            cb_(vals);
        }

        ::close(cfd); client_fd_.store(-1);
        RCLCPP_INFO(rclcpp::get_logger("RefTcpServer"), "Client disconnected.");
    }
    ::close(sfd); server_fd_.store(-1);
#endif
}

// yaml 파라미터를 읽되, 없으면 기본값을 쓴다.
//
// 주의: yaml 에 소수점 없이 적힌 값(예: pos_sp_max_kpa: 150)은 int 로 파싱되어
// declare_parameter<double>() 이 InvalidParameterTypeException 을 던진다. 예전 구현은
// 그 예외를 삼키고 **기본값을 조용히 반환**했다 — yaml 을 고쳐도 반영되지 않는 함정이라
// 튜닝 중에 실제로 물렸다. 이제 예외 후 get_parameter 로 실제 타입에 맞춰 읽고,
// 산술 타입이면 정수 → 실수 변환까지 구제한다.
// (CanBridge::declare_double_flexible / VirtualPowerpack::gpd 와 같은 처리)
template <typename T>
static T get_param_or(rclcpp::Node* node, const std::string& name, const T& defv) {
  if (!node->has_parameter(name)) {
    try {
      // **동적 타이핑으로 선언한다.** 정적 타입으로 선언하면 yaml 에 `3` (정수) 로
      // 적힌 값을 double 로 받을 때 InvalidParameterTypeException 이 나고, 그러면
      // 파라미터가 아예 선언되지 않아 아래 get_parameter 도 실패해 기본값이 조용히
      // 쓰인다. 실제로 valve_deadzone.margin_pct: 3 이 그렇게 무시됐다
      // (기동 로그에 "여유 0.0 %p" 로 찍혔다).
      rcl_interfaces::msg::ParameterDescriptor d;
      d.dynamic_typing = true;
      node->declare_parameter(name, rclcpp::ParameterValue(defv), d);
    } catch (...) {
      // 이미 선언됐거나 선언할 수 없다 — 아래에서 읽는다
    }
  }
  T out = defv;
  if (node->get_parameter(name, out)) return out;

  if constexpr (std::is_arithmetic_v<T>) {
    rclcpp::Parameter p;
    if (node->get_parameter(name, p)) {
      if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        return static_cast<T>(p.as_int());
      if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        return static_cast<T>(p.as_double());
      if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
        return static_cast<T>(p.as_bool());
    }
  }
  return defv;
}

// yaml 에 **실제로 적혀 있을 때만** 값을 돌려준다 (없으면 nullopt).
//
// get_param_or 로 겹쳐 읽으면 안 되는 이유: 그 함수는 없는 이름을 "그때의 기본값"으로
// 선언해 버리고, 그 선언이 남아서 **다음 호출을 오염시킨다**. 게인 6단 우선순위가
// 이것 때문에 통째로 망가져 있었다 —
//   gains_for(ch0,"up")  의 5단계가 channel_config.ch0.pid.kp 를 1.0 으로 선언
//   gains_for(ch0,"down") 의 4단계가 ChannelPID.pos.down.kp = 3.0 을 제대로 읽지만
//   이어지는 5단계가 위에서 선언된 1.0 을 읽어 **3.0 을 덮어썼다**.
//   실측: yaml 의 pos.down kp 3.0 / kp_far 8.0 이 한 번도 적용된 적이 없다
//   (기동 로그 "하강 kp=1.000(far 2.500)").
// NaN 을 기본값으로 선언해 "없음" 을 표시하면 다시 물어도 없음이 유지된다.
static std::optional<double> get_param_opt(rclcpp::Node* node, const std::string& name) {
  if (!node->has_parameter(name)) {
    try {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.dynamic_typing = true;
      node->declare_parameter(
          name, rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN()), d);
    } catch (...) {
      return std::nullopt;
    }
  }
  rclcpp::Parameter p;
  if (!node->get_parameter(name, p)) return std::nullopt;
  double v;
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)       v = p.as_double();
  else if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) v = (double)p.as_int();
  else return std::nullopt;
  if (std::isnan(v)) return std::nullopt;
  return v;
}

// ================================
// ThreadPool
// ================================
ThreadPool::ThreadPool(size_t num_threads, const std::vector<int>& pin_cpus)
: pin_cpus_(pin_cpus)
{
  workers_.reserve(num_threads);
  for (size_t i = 0; i < num_threads; ++i) {
    workers_.emplace_back([this, i](){
#ifdef __linux__
      if (!pin_cpus_.empty()) {
        int cpu = pin_cpus_[i % pin_cpus_.size()];
        cpu_set_t set; CPU_ZERO(&set); CPU_SET(cpu, &set);
        pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
      }
#endif
      while (true) {
        std::function<void()> task;
        {
          std::unique_lock<std::mutex> lk(mtx_);
          cv_.wait(lk, [&]{ return stop_.load() || !queue_.empty(); });
          if (stop_.load()) return;
          task = std::move(queue_.front()); queue_.pop();
        }
        if (task) task();
      }
    });
  }
}

ThreadPool::~ThreadPool() {
  stop_.store(true);
  cv_.notify_all();
  for (auto& t : workers_) if (t.joinable()) t.join();
}

void ThreadPool::enqueue(std::function<void()> fn) {
  {
    std::lock_guard<std::mutex> lk(mtx_);
    queue_.push(std::move(fn));
  }
  cv_.notify_one();
}

void ThreadPool::run_batch_and_wait(std::vector<std::function<void()>>& tasks) {
  if (tasks.empty()) return;
  std::atomic<size_t> done{0};
  const size_t N = tasks.size();
  for (auto& f : tasks) {
    enqueue([&done, f](){ f(); done.fetch_add(1, std::memory_order_relaxed); });
  }
  while (done.load(std::memory_order_acquire) < N) {
    std::this_thread::yield();
  }
}

// ================================
// Controller
// ================================
Controller::Controller(const rclcpp::NodeOptions& opts)
: rclcpp::Node("pp_controller", opts)
{
  period_ms_ = this->declare_parameter<int>("period_ms", 2);  // 2ms default (500Hz event-driven)
  enable_thread_pinning_ = this->declare_parameter<bool>("enable_thread_pinning", true);
  cpu_pins_param_ = this->declare_parameter<std::vector<int64_t>>("cpu_pins", std::vector<int64_t>{0,1,2,3});

  num_positive_channels_ = this->declare_parameter<int>("num_positive_channels", 8);
  num_total_channels_   = this->declare_parameter<int>("num_total_channels", 12);
  num_actuators_        = this->declare_parameter<int>("num_actuators", 1);

  // 채널 수에 맞게 동적 벡터 초기화
  channel_configs_.resize(num_total_channels_);
  mpc_ref_kpa_.assign(num_total_channels_, 101.325);
  ref_snapshot_.assign(num_total_channels_, 0.0);
  final_active_vols_ml_.assign(num_total_channels_, 0.0);

  sensor_.atm_offset = get_param_or<double>(this, "Sensor_calibration.atm_offset", 101.325);
  zero_tolerance_kpa_ = get_param_or<double>(this, "Sensor_calibration.zero_tolerance_kpa", 8.0);
  encoder_zero_when_disconnected_ =
      get_param_or<bool>(this, "encoder_zero_when_disconnected", true);
  use_measured_dt_ = get_param_or<bool>(this, "use_measured_dt", true);

  sensor_filter_alpha_ = this->declare_parameter<double>("sensor_filter_alpha", 1.0);

  if (sensor_filter_alpha_ <= 0.0) sensor_filter_alpha_ = 0.01;
  if (sensor_filter_alpha_ > 1.0)  sensor_filter_alpha_ = 1.0;

  RCLCPP_INFO(get_logger(), "Sensor Filter Alpha applied: %.3f", sensor_filter_alpha_);

  // Flat per-board calibration: Sensor_calibration.boards."N".offset/gain
  for (int bid = 1; bid <= NUM_CAN_BOARDS; ++bid) {
    const std::string base = "Sensor_calibration.boards." + std::to_string(bid);
    auto& ch = sensor_.boards[(size_t)(bid - 1)];
    ch.offset = get_param_or<double>(this, base + ".offset", ch.offset);
    if (bid - 1 >= 0 && bid - 1 < NUM_CAN_BOARDS)
      yaml_offset_[(size_t)(bid - 1)] = ch.offset;   // 0점 재보정 이탈 경고 기준
    ch.gain   = get_param_or<double>(this, base + ".gain",   ch.gain);
  }

  channel_board_offset_ = get_param_or<int>(this, "channel_board_offset", 4);
  P_pos_board_id_       = get_param_or<int>(this, "line_pressure_boards.pos",       1);
  P_neg_board_id_       = get_param_or<int>(this, "line_pressure_boards.neg",       2);
  P_macro_board_id_     = get_param_or<int>(this, "line_pressure_boards.macro",     3);
  P_macro_neg_board_id_ = get_param_or<int>(this, "line_pressure_boards.macro_neg", 4);

  RCLCPP_INFO(this->get_logger(), "================ PARAMETER DIAGNOSIS ================");
  RCLCPP_INFO(this->get_logger(), "Loaded parameter [Sensor_calibration.boards.4.offset]: %f", sensor_.boards[3].offset);
  RCLCPP_INFO(this->get_logger(), "channel_board_offset=%d  P_pos_board=%d  P_neg_board=%d  P_macro_board=%d",
              channel_board_offset_, P_pos_board_id_, P_neg_board_id_, P_macro_board_id_);
  RCLCPP_INFO(this->get_logger(), "=====================================================");


  use_macro_ = get_param_or<bool>(this, "use_macro_valves", false);

  // ── 밸브 데드존 (실측 상수) ────────────────────────────────────────────────
  // 밸브 모델은 여기서 쓰지 않는다. 열리기 시작하는 지령을 실기에서 재서 적은
  // 값이다 (scripts/valve_deadzone.py). 채널별 오버라이드는 build_ctrls 앞에서
  // channel_config.chN.deadzone.* 로 읽는다.
  dz_enable_ = get_param_or<bool>(this, "valve_deadzone.enable", true);
  dz_pct_[PressureCtrl::V_MICRO] = get_param_or<double>(this, "valve_deadzone.micro_pct", 0.0);
  dz_pct_[PressureCtrl::V_MACRO] = get_param_or<double>(this, "valve_deadzone.macro_pct", 0.0);
  dz_pct_[PressureCtrl::V_ATM]   = get_param_or<double>(this, "valve_deadzone.atm_pct",   0.0);
  dz_margin_pct_ = get_param_or<double>(this, "valve_deadzone.margin_pct", 0.0);
  dz_park_enable_    = get_param_or<bool>  (this, "valve_deadzone.park_enable",    false);
  dz_park_below_pct_ = get_param_or<double>(this, "valve_deadzone.park_below_pct", 5.0);
  u_hw_pct_.assign((size_t)num_total_channels_, {0.0f, 0.0f, 0.0f});
  RCLCPP_INFO(get_logger(),
    "밸브 데드존 보상: %s — u_hw = (deadzone(차압) − 여유 %.1f %%p) + u_pid. "
    "표가 없는 밸브의 상수 [%%] micro=%.1f macro=%.1f atm=%.1f. macro 밸브: %s.",
    dz_enable_ ? "켜짐" : "꺼짐", dz_margin_pct_,
    dz_pct_[PressureCtrl::V_MICRO], dz_pct_[PressureCtrl::V_MACRO],
    dz_pct_[PressureCtrl::V_ATM],
    use_macro_ ? "사용 (파킹 포함)" : "사용 안 함 — 지령을 항상 0 으로 낸다");

  default_volume_ml_  = get_param_or<double>(this, "default_volume_ml",    1.0);
  actuator_connected_ = get_param_or<bool>  (this, "actuator_connected",   true);
  tank_volume_pos_ml_ = get_param_or<double>(this, "tank_volume_pos_ml", 750.0);
  tank_volume_neg_ml_ = get_param_or<double>(this, "tank_volume_neg_ml", 400.0);

  vol_ml_.resize(num_total_channels_);
  vol_scale_.assign(num_total_channels_, 1.0);
  for (int i = 0; i < num_total_channels_; ++i) {
    if (!actuator_connected_)
      vol_ml_[i] = (i < num_positive_channels_) ? tank_volume_pos_ml_ : tank_volume_neg_ml_;
    else
      vol_ml_[i] = default_volume_ml_;

    // 채널별 부피 오버라이드 — channel_config.chN.volume_ml
    //
    // 챔버 부피는 요구 유량에 그대로 곱해진다 (m_dot = P_dot·V/(R·T)). 전 채널을
    // 같은 값으로 두면 실제 부피가 큰 채널은 필요한 유량의 일부만 요구해 느리고,
    // 작은 채널은 과요구해 진동한다.
    //   실기 20260829_142216 (6축, 공급 정상): 같은 전류·차압에서 dP/dt 를 비교하면
    //   유효 부피가 채널 간 수 배 차이났다. ch1·ch2 가 크고(정착 8~22 초, 밸브
    //   통전 8~12%), ch3·ch4 가 작았다(ch4 는 p-p 33~45 kPa 로 진동).
    //   크기의 절대값은 그 로그로 확정하지 못했지만(창을 좁히면 추정이 흩어진다)
    //   순서는 일관됐다. 채널을 하나씩 돌려 재면 확정된다.
    const double v_ov = get_param_or<double>(
        this, "channel_config.ch" + std::to_string(i) + ".volume_ml", 0.0);
    if (v_ov > 0.0) vol_ml_[i] = v_ov;

    // 배율 — 액추에이터 연결 시 기하 모델 값에 곱한다 (volume_ml 은 그때 덮어써진다).
    vol_scale_[(size_t)i] = std::max(1e-3, get_param_or<double>(
        this, "channel_config.ch" + std::to_string(i) + ".volume_scale", 1.0));
  }
  {
    std::string vs;
    for (int i = 0; i < num_total_channels_; ++i) {
      char b[48];
      if (std::abs(vol_scale_[(size_t)i] - 1.0) > 1e-6)
        snprintf(b, sizeof(b), " ch%d=%.0f(x%.2f)", i, vol_ml_[(size_t)i], vol_scale_[(size_t)i]);
      else snprintf(b, sizeof(b), " ch%d=%.0f", i, vol_ml_[(size_t)i]);
      vs += b;
    }
    RCLCPP_INFO(get_logger(), "[채널 부피 mL]%s", vs.c_str());
  }
  prev_vol_m3_.resize(num_total_channels_);
  for (int i = 0; i < num_total_channels_; ++i)
    prev_vol_m3_[i] = vol_ml_[i] * 1.0e-6;

  RCLCPP_INFO(get_logger(), "Actuator: %s | vol_pos=%.0f mL, vol_neg=%.0f mL",
              actuator_connected_ ? "CONNECTED" : "DISCONNECTED",
              tank_volume_pos_ml_, tank_volume_neg_ml_);

  for(int i = 0; i < num_total_channels_; ++i) {
    std::string prefix = "channel_config.ch" + std::to_string(i) + ".";

    // ── 밸브별 13-parameter ─────────────────────────────────────────────────
    // 우선순위: chN.<role>.<param>  (valve_fit_solve.py 가 쓰는 형태)
    //        → chN.<param>          (예전 평면 형태 = 세 밸브 공용)
    //        → 하드코딩 기본값
    // 이렇게 하면 피팅 파일을 병합하는 순간 밸브별 값이 실제로 쓰이고, 없으면
    // 기존 동작과 완전히 같다.
    {
      auto flat = [&](const char* n, double dflt) {
        return get_param_or<double>(this, prefix + n, dflt);
      };
      // 평면 값(또는 기본값)을 먼저 읽어 세 밸브의 폴백으로 쓴다.
      Valve13 base;
      base.I_MAX       = flat("I_MAX",       0.30);
      base.A_max       = flat("A_max",       0.2845);
      base.k_shape     = flat("k_shape",     33.09);
      base.C_k         = flat("C_k",         0.0288);
      base.C_p         = flat("C_p",         0.00012);
      base.C_z         = flat("C_z",         0.0);
      base.A_bw        = flat("A_bw",        260649.5);
      base.beta_bw     = flat("beta_bw",     179.0);
      base.gamma_bw    = flat("gamma_bw",    0.06);
      base.alpha_shape = flat("alpha_shape", 3884.2);
      base.wn_up       = flat("wn_up",       40.0);
      base.zeta_up     = flat("zeta_up",     1.2);
      base.wn_down     = flat("wn_down",     45.0);
      base.zeta_down   = flat("zeta_down",   1.0);

      // PressureCtrl::ValveIdx 와 같은 순서: 0=micro, 1=macro, 2=atm
      static const char* kRole[3] = {"micro", "macro", "atm"};
      bool any_per_valve = false;
      for (int j = 0; j < 3; ++j) {
        const std::string rp = prefix + kRole[j] + ".";
        auto pv = [&](const char* n, double dflt) {
          const double v = get_param_or<double>(this, rp + n, dflt);
          if (v != dflt) any_per_valve = true;   // 피팅 파일에서 실제로 읽혔다는 신호
          return v;
        };
        auto& t = channel_configs_[i].v[(size_t)j];
        t.I_MAX       = pv("I_MAX",       base.I_MAX);
        t.A_max       = pv("A_max",       base.A_max);
        t.k_shape     = pv("k_shape",     base.k_shape);
        t.C_k         = pv("C_k",         base.C_k);
        t.C_p         = pv("C_p",         base.C_p);
        t.C_z         = pv("C_z",         base.C_z);
        t.A_bw        = pv("A_bw",        base.A_bw);
        t.beta_bw     = pv("beta_bw",     base.beta_bw);
        t.gamma_bw    = pv("gamma_bw",    base.gamma_bw);
        t.alpha_shape = pv("alpha_shape", base.alpha_shape);
        t.wn_up       = pv("wn_up",       base.wn_up);
        t.zeta_up     = pv("zeta_up",     base.zeta_up);
        t.wn_down     = pv("wn_down",     base.wn_down);
        t.zeta_down   = pv("zeta_down",   base.zeta_down);
      }
      channel_configs_[i].per_valve_loaded = any_per_valve;
      channel_configs_[i].chamber_volume_ml =
          get_param_or<double>(this, prefix + "chamber_volume_ml", -1.0);
    }
  }

  valve_operate_       = get_param_or<bool>(this, "system_parameters.valve_operate",   false);

  macro_switch_pwm_index_     = get_param_or<int>(this, "MacroSwitch.pwm_index", 9);

  // 채널 압력 PID 게인은 build_ctrls() 에서 채널·방향별로 읽는다 (gains_for).

  // ── 채널별 데드존 표 ──────────────────────────────────────────────────────
  // channel_config.chN.deadzone.{micro,macro,atm}_dp_kpa / _u_pct 가 표,
  // 없으면 같은 곳의 _pct 스칼라, 그것도 없으면 valve_deadzone.{role}_pct.
  // 표는 scripts/valve_deadzone.py 가 실기에서 재서 자동으로 적는다.
  dz_ch_.assign((size_t)num_total_channels_, std::array<DzTable, 3>{});
  dz_margin_ch_.assign((size_t)num_total_channels_,
      std::array<double, 3>{dz_margin_pct_, dz_margin_pct_, dz_margin_pct_});
  dz_park_ch_.assign((size_t)num_total_channels_, std::array<double, 3>{});
  {
    static const char* kRole[3] = {"micro", "macro", "atm"};
    int n_ch = 0, n_side = 0;
    for (int i = 0; i < num_total_channels_; ++i) {
      const std::string pre  = "channel_config.ch" + std::to_string(i) + ".deadzone.";
      {   // 채널 공통 → 밸브별 순으로 좁혀 읽는다
        const double m_ch = get_param_or<double>(this, pre + "margin_pct", dz_margin_pct_);
        static const char* kR[3] = {"micro", "macro", "atm"};
        for (int j = 0; j < 3; ++j)
          dz_margin_ch_[(size_t)i][(size_t)j] =
              get_param_or<double>(this, pre + "margin_" + kR[j] + "_pct", m_ch);
      }
      // 측별 기본 표. 채널 간 차이가 작으면 한두 채널만 재서 그 측 전체에 쓴다
      // (scripts/valve_deadzone.py --write-scope side). 양압과 음압은 데드존이
      // 크게 다르므로(실측 micro 56 vs 44) 전역 하나가 아니라 측별이 맞는 단위다.
      const std::string side = std::string("valve_deadzone.")
                             + (i < num_positive_channels_ ? "pos." : "neg.");
      for (int j = 0; j < 3; ++j) {
        DzTable& t = dz_ch_[(size_t)i][(size_t)j];
        const std::string role = kRole[j];
        // 상수 기본값: 채널 → 측 → 전역
        t.flat = get_param_or<double>(this, pre + role + "_pct",
                 get_param_or<double>(this, side + role + "_pct", dz_pct_[(size_t)j]));
        // 표: 채널이 있으면 채널, 없으면 측.
        t.dp_kpa = get_param_or<std::vector<double>>(
            this, pre + role + "_dp_kpa", std::vector<double>{});
        t.u_pct  = get_param_or<std::vector<double>>(
            this, pre + role + "_u_pct",  std::vector<double>{});
        const bool from_channel = !t.dp_kpa.empty();
        if (!from_channel) {
          t.dp_kpa = get_param_or<std::vector<double>>(
              this, side + role + "_dp_kpa", std::vector<double>{});
          t.u_pct  = get_param_or<std::vector<double>>(
              this, side + role + "_u_pct",  std::vector<double>{});
        }
        // 길이가 다르거나 차압이 오름차순이 아니면 표를 통째로 버린다. 반쯤 맞는
        // 표로 보간하면 엉뚱한 지령이 조용히 나가므로, 상수로 떨어뜨리고 경고한다.
        bool ok = (t.dp_kpa.size() == t.u_pct.size());
        for (size_t k = 1; ok && k < t.dp_kpa.size(); ++k)
          ok = (t.dp_kpa[k] > t.dp_kpa[k - 1]);
        if (!ok) {
          RCLCPP_WARN(get_logger(),
            "ch%d %s 데드존 표(%s)가 잘못됐다 (길이 %zu/%zu, 차압 오름차순?) — "
            "상수 %.1f %% 로 대체한다", i, role.c_str(),
            from_channel ? "채널별" : "측별", t.dp_kpa.size(), t.u_pct.size(), t.flat);
          t.dp_kpa.clear(); t.u_pct.clear();
        }
        if (!t.dp_kpa.empty()) { if (from_channel) ++n_ch; else ++n_side; }
      }
    }
    // 파킹 지령: 표 최솟값에서 park_below 만큼 더 내린다 (0 아래로는 안 간다).
    for (int i = 0; i < num_total_channels_; ++i)
      for (int j = 0; j < 3; ++j)
        dz_park_ch_[(size_t)i][(size_t)j] = dz_park_enable_
            ? std::max(0.0, dz_ch_[(size_t)i][(size_t)j].u_min() - dz_park_below_pct_)
            : 0.0;
    if (dz_park_enable_) {
      std::string ps;
      for (int gid : {0, num_positive_channels_}) {
        if (gid >= num_total_channels_) continue;
        char b2[64];
        snprintf(b2, sizeof(b2), " ch%d %.1f/%.1f", gid,
                 dz_park_ch_[(size_t)gid][PressureCtrl::V_MICRO],
                 dz_park_ch_[(size_t)gid][PressureCtrl::V_ATM]);
        ps += b2;
      }
      RCLCPP_INFO(get_logger(),
        "밸브 파킹: 켜짐 — 쉬는 밸브를 (표 최솟값 − %.1f %%p) 에 걸어 둔다 "
        "(코일 전류 상승 시간을 없앤다). 예 micro/atm:%s",
        dz_park_below_pct_, ps.c_str());
    }

    RCLCPP_INFO(get_logger(),
      "데드존 차압 표: 채널별 %d 개 + 측별 기본 %d 개 (채널×밸브)%s",
      n_ch, n_side,
      (n_ch + n_side) == 0
        ? " — 전부 상수를 쓴다. scripts/valve_deadzone.py 로 재라" : "");
  }

  pid_pos_.kp  = get_param_or<double>(this, "LinePID.pos.kp",  0.5);
  pid_pos_.ki  = get_param_or<double>(this, "LinePID.pos.ki",  0.0);
  pid_pos_.kd  = get_param_or<double>(this, "LinePID.pos.kd",  0.0);
  pid_pos_.ref = get_param_or<double>(this, "LinePID.pos.ref", 150.0);
  pid_pos_.i_limit = get_param_or<double>(this, "LinePID.pos.i_limit", 60.0);
  pid_out_min_ = get_param_or<double>(this, "LinePID.out_min", 0.0);
  pid_out_max_ = get_param_or<double>(this, "LinePID.out_max", 100.0);
  // flat index: (P_pos_board_id-1)*3 + 0 = (1-1)*3+0 = 0
  pid_pos_pwm_index_ = get_param_or<int>(this, "LinePID.pos.pwm_index", 0);

  pid_neg_.kp  = get_param_or<double>(this, "LinePID.neg.kp",  0.5);
  pid_neg_.ki  = get_param_or<double>(this, "LinePID.neg.ki",  0.0);
  pid_neg_.kd  = get_param_or<double>(this, "LinePID.neg.kd",  0.0);
  pid_neg_.ref = get_param_or<double>(this, "LinePID.neg.ref", 20.0);
  pid_neg_.i_limit = get_param_or<double>(this, "LinePID.neg.i_limit", 80.0);
  // flat index: (P_neg_board_id-1)*3 + 0 = (2-1)*3+0 = 3
  pid_neg_pwm_index_ = get_param_or<int>(this, "LinePID.neg.pwm_index", 3);

  // ── 레일 피드포워드 표 (rail_map.py --invert 가 생성) ──────────────────
  {
    auto dvec = [&](const std::string& name) {
      return this->declare_parameter<std::vector<double>>(name, std::vector<double>{});
    };
    rail_ff_.enable = get_param_or<bool>(this, "RailFF.enable", false);
    rail_ff_.admit_pneg = dvec("RailFF.admit.p_neg_kpa");
    rail_ff_.admit_u    = dvec("RailFF.admit.u_pct");
    rail_ff_.vent_admit = dvec("RailFF.vent.admit_pct");
    for (double a : rail_ff_.vent_admit) {
      char key[64];
      // yaml 의 "60": 은 **따옴표가 이름에 안 들어간다** — 파라미터 이름은
      // RailFF.vent.curves.60.p_pos_kpa 다 (TeensyEncoder.channels.0.* 과 같은 규칙).
      // 따옴표를 넣으면 영영 안 맞아 빈 벡터가 되고, RailFF 가 조용히 꺼진다.
      snprintf(key, sizeof(key), "RailFF.vent.curves.%.0f.", a);
      rail_ff_.vent_ppos.push_back(dvec(std::string(key) + "p_pos_kpa"));
      rail_ff_.vent_u.push_back(dvec(std::string(key) + "u_pct"));
    }
    rail_ff_.gain_ref = get_param_or<double>(this, "RailFF.gain_ref_kpa_per_pct", 0.0);
    rail_ff_.gain_min = get_param_or<double>(this, "RailFF.gain_scale_min", 0.3);
    rail_ff_.gain_max = get_param_or<double>(this, "RailFF.gain_scale_max", 3.0);

    // 표가 깨져 있으면 **끈다.** 반쯤 로드된 표로 돌면 엉뚱한 개도가 나가는데,
    // 그건 예전 동작(ff=100)보다 훨씬 나쁘다.
    bool bad = rail_ff_.admit_pneg.size() != rail_ff_.admit_u.size() ||
               rail_ff_.admit_pneg.size() < 2;
    for (size_t k = 0; k < rail_ff_.vent_admit.size() && !bad; ++k)
      bad = rail_ff_.vent_ppos[k].size() != rail_ff_.vent_u[k].size() ||
            rail_ff_.vent_ppos[k].size() < 2;
    if (rail_ff_.enable && (bad || rail_ff_.vent_admit.empty())) {
      RCLCPP_ERROR(get_logger(),
        "RailFF 표가 불완전하다 — **끈다** (예전처럼 개도 100 을 원점으로 쓴다). "
        "rail_map.py --invert 출력을 그대로 붙여 넣었는지 확인할 것.");
      rail_ff_.enable = false;
    }
    RCLCPP_INFO(get_logger(),
      "레일 목표 주인: %s",
      control_mode_ == 2
        ? "PressureRefGen (control_mode 2) — controller/rail_ref_kpa 는 무시된다"
        : "yaml LinePID.{pos,neg}.ref, controller/rail_ref_kpa 로 런타임 변경 가능");
    if (rail_ff_.ok()) {
      const double a0 = rail_ff_.admit_at(pid_neg_.ref);
      RCLCPP_INFO(get_logger(),
        "레일 피드포워드 ON — 목표 P+ %.1f / P− %.1f 에서 방출 %.1f %% · 유입 %.1f %% "
        "(곡선 %zu개, 이득기준 %.2f kPa/%%p)",
        pid_pos_.ref, pid_neg_.ref, rail_ff_.vent_at(pid_pos_.ref, a0), a0,
        rail_ff_.vent_admit.size(), rail_ff_.gain_ref);
    } else {
      RCLCPP_WARN(get_logger(), "레일 피드포워드 OFF — 개도 100 %% 를 원점으로 쓴다 "
                  "(적분이 동작점을 통째로 만들어야 한다)");
    }
  }

  ref_client_cfg_.enable        = get_param_or<bool>(this,  "RefTcp.enable",        false);
  ref_client_cfg_.host = get_param_or<std::string>(this, "RefTcp.host", "169.254.46.254");
  ref_client_cfg_.port          = get_param_or<int>(this,   "RefTcp.port",          2292);
  
  ref_client_cfg_.expect_n       = get_param_or<int>(this,   "RefTcp.expect_n",       num_total_channels_);
  ref_client_cfg_.pressure_scale = get_param_or<double>(this,"RefTcp.pressure_scale", 1.0 / 327.675);
  
  log_channel_id_ = this->declare_parameter<int>("log_channel_id", -1);

  if (ref_client_cfg_.enable) {
    ref_client_ = std::make_unique<RefTcpClient>(
      ref_client_cfg_,
      [this](const std::vector<double>& arr){
        if ((int)arr.size() < num_total_channels_) return;

        std::lock_guard<std::mutex> lk(mpc_ref_mtx_);

        for(int i = 0; i < num_total_channels_; ++i) {
            mpc_ref_kpa_[i] = arr[i];
        }
      }
    );
}

  std::fill(mpc_ref_kpa_.begin(), mpc_ref_kpa_.end(), 101.325);

  auto reliable = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)).reliable().keep_last(5);
  sub_sensors_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
      "board/sensors", reliable, std::bind(&Controller::on_sensor, this, _1));

  pub_pwm_cmd_   = create_publisher<std_msgs::msg::UInt16MultiArray>("board/pwm_cmd", 5);
  pub_mpc_refs_  = create_publisher<std_msgs::msg::Float64MultiArray>("controller/mpc_refs_kpa", 10);
  pub_chan_dbg_  = create_publisher<std_msgs::msg::Float64MultiArray>("controller/channel_dbg", 10);
  pub_rail_dbg_  = create_publisher<std_msgs::msg::Float64MultiArray>("controller/rail_dbg", 10);

  // ── 레일 목표 수신 ────────────────────────────────────────────────────
  // 범위를 벗어난 값은 **자르고 경고한다.** 오타 하나로 레일 목표가 400 이 되면
  // 방출 밸브가 끝까지 닫힌 채 릴리프만 계속 때리게 된다.
  rail_ref_pos_min_ = get_param_or<double>(this, "RailRef.pos_min_kpa", 101.325);
  rail_ref_pos_max_ = get_param_or<double>(this, "RailRef.pos_max_kpa", 250.0);
  rail_ref_neg_min_ = get_param_or<double>(this, "RailRef.neg_min_kpa",  10.0);
  rail_ref_neg_max_ = get_param_or<double>(this, "RailRef.neg_max_kpa", 101.325);
  sub_rail_ref_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "controller/rail_ref_kpa", 5,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
        // 모드 2 는 PressureRefGen 이 매 틱 pid_*_.ref 를 다시 쓴다. 여기서 받아
        // 둬도 한 틱 만에 지워지므로, **조용히 무시하지 말고** 이유를 알린다.
        if (control_mode_ == 2) {
          RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 3000,
            "레일 목표를 받았지만 **control_mode 2 라 무시한다** — PressureRefGen 이 "
            "레일 셋포인트를 매 틱 덮어쓴다. 레일 시험은 control_mode:=0 으로 띄울 것.");
          return;
        }
        if (m->data.size() >= 1 && std::isfinite(m->data[0])) {
          const double v = std::clamp(m->data[0], rail_ref_pos_min_, rail_ref_pos_max_);
          if (v != m->data[0])
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
              "레일 양압 목표 %.1f 을 [%.1f, %.1f] 로 잘랐다",
              m->data[0], rail_ref_pos_min_, rail_ref_pos_max_);
          rail_ref_pos_.store(v, std::memory_order_relaxed);
        }
        if (m->data.size() >= 2 && std::isfinite(m->data[1])) {
          const double v = std::clamp(m->data[1], rail_ref_neg_min_, rail_ref_neg_max_);
          if (v != m->data[1])
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
              "레일 음압 목표 %.1f 을 [%.1f, %.1f] 로 잘랐다",
              m->data[1], rail_ref_neg_min_, rail_ref_neg_max_);
          rail_ref_neg_.store(v, std::memory_order_relaxed);
        }
      });
  chan_dbg_.assign(12, {});
  pub_active_vols_ = create_publisher<std_msgs::msg::Float64MultiArray>("controller/active_volumes_ml", 1);
  pub_kpa_all_   = create_publisher<std_msgs::msg::Float64MultiArray>("controller/sensors_kpa", 10);
  pub_pos_dbg_   = create_publisher<std_msgs::msg::Float64MultiArray>("controller/position_dbg", 10);

  sub_volumes_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "actuator/volumes_ml", 10, std::bind(&Controller::on_volume, this, _1));

  sub_analog_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "board/analog", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(sensors_mtx_);
          // 액추에이터가 안 붙어 있으면 엔코더도 안 붙어 있다. 그때 board/analog 는
          // 전압이 안 잡히는 채널에 대해 0 raw 를 주는데, 그 값을 반전증폭 역산에
          // 넣으면 (4125−0)/0.825 = 5000 mV → offset·gain 에 따라 **엉뚱한 큰 각도**가
          // 나온다. 그 값이 속도 추정과 디버그 토픽으로 흘러 들어가 오해를 만든다.
          // 연결 전에는 0° 로 고정한다 (encoder_zero_when_disconnected 로 끌 수 있다).
          const bool zero = encoder_zero_when_disconnected_ && !actuator_connected_;
          const size_t n = std::min(msg->data.size(), encoder_angles_.size());
          for (size_t i = 0; i < n; ++i)
              encoder_angles_[i] = zero ? 0.0 : msg->data[i];
      });

  size_t nth = std::max<size_t>(2, std::min<size_t>(
      (size_t)num_total_channels_,
      std::thread::hardware_concurrency()));
  std::vector<int> pins; if (enable_thread_pinning_) for (auto v: cpu_pins_param_) pins.push_back((int)v);
  pool_threads_ = nth;
  pool_ = std::make_unique<ThreadPool>(nth, pins);

  build_ctrls();

  if (log_channel_id_ >= 0 && log_channel_id_ < num_total_channels_) {
    log_file_.open("mpc_log.csv", std::ios::out | std::ios::trunc);
    if (log_file_.is_open()) {
      RCLCPP_INFO(get_logger(), "Logging data for MPC channel %d to mpc_log.csv", log_channel_id_);
      log_file_ << "tick,reference_kpa,sensed_kpa\n";
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to open log file mpc_log.csv");
      log_channel_id_ = -1; 
    }
  }

  RCLCPP_INFO(get_logger(),
  "RefTcp: enable=%d host=%s port=%d expect_n=%d pressure_scale=%.6f",
  (int)ref_client_cfg_.enable, ref_client_cfg_.host.c_str(),
  ref_client_cfg_.port, ref_client_cfg_.expect_n, ref_client_cfg_.pressure_scale);

  // ──────────────────────────────────────────
  // 제어 모드 (0: 압력, 1: 위치)
  // ──────────────────────────────────────────
  control_mode_ = get_param_or<int>(this, "control_mode", 0);
  RCLCPP_INFO(get_logger(), "Control mode: %d (%s)",
              control_mode_, control_mode_ == 1 ? "POSITION" : "PRESSURE");

  // ──────────────────────────────────────────
  // 위치 제어기 파라미터 로드 (축마다 PositionController.axis<i>.* , 크기 = num_actuators_)
  // ──────────────────────────────────────────
  pos_ctrl_cfg_.assign(num_actuators_, PositionCtrlConfig{});
  pos_ctrl_state_.assign(num_actuators_, PositionCtrlState{});
  target_angle_deg_.assign(num_actuators_, 0.0);
  target_angle_slewed_.assign(num_actuators_, 0.0);
  target_slew_dps_ = get_param_or<double>(this, "PositionController.target_slew_deg_per_s", 0.0);
  target_follow_band_deg_ = get_param_or<double>(this,
      "PositionController.target_follow_band_deg", 5.0);
  kd_vel_ff_ = get_param_or<double>(this,
      "PositionController.kd_vel_ff", 1.0);
  target_slew_rate_.assign(std::max(1, num_actuators_), 0.0);
  band_sat_ticks_.assign(std::max(1, num_actuators_), 0);

  for (int a = 0; a < num_actuators_; ++a) {
    const std::string prefix = "PositionController.axis" + std::to_string(a) + ".";
    auto& c = pos_ctrl_cfg_[(size_t)a];

    // 공용 (mode 1 / 2)
    c.actuator_idx      = get_param_or<int>   (this, prefix + "actuator_idx",      a);
    c.pos_gid           = get_param_or<int>   (this, prefix + "pos_gid",           a);
    c.neg_gid           = get_param_or<int>   (this, prefix + "neg_gid",           num_positive_channels_ + a);
    c.mass_kg           = get_param_or<double>(this, prefix + "mass_kg",           1.0);
    c.link_length_m     = get_param_or<double>(this, prefix + "link_length_m",     0.2);
    c.p_pos_max_kpa     = get_param_or<double>(this, prefix + "p_pos_max_kpa",     165.0);
    c.p_neg_min_kpa     = get_param_or<double>(this, prefix + "p_neg_min_kpa",     70.0);
    c.vel_filter_alpha  = get_param_or<double>(this, prefix + "vel_filter_alpha",  0.05);
    c.default_angle_deg = get_param_or<double>(this, prefix + "default_angle_deg", 0.0);

    // mode 1 전용 (control_mode 2 에서는 읽히지 않는다)
    auto& h = c.m1;
    h.kp                 = get_param_or<double>(this, prefix + "mode1.kp",                 3.0);
    h.ki                 = get_param_or<double>(this, prefix + "mode1.ki",                 0.05);
    h.kd                 = get_param_or<double>(this, prefix + "mode1.kd",                 0.02);
    h.integral_limit_kpa = get_param_or<double>(this, prefix + "mode1.integral_limit_kpa", 20.0);
    h.kff_gravity        = get_param_or<double>(this, prefix + "mode1.kff_gravity",         10.0);
    h.friction_kpa       = get_param_or<double>(this, prefix + "mode1.friction_kpa",        2.0);
    h.p_bias_pos_kpa     = get_param_or<double>(this, prefix + "mode1.p_bias_pos_kpa",     120.0);
    h.p_bias_neg_kpa     = get_param_or<double>(this, prefix + "mode1.p_bias_neg_kpa",      90.0);
    h.neg_coupling       = get_param_or<double>(this, prefix + "mode1.neg_coupling",         0.5);
    h.p_pos_min_kpa      = get_param_or<double>(this, prefix + "mode1.p_pos_min_kpa",      101.325);
    h.p_neg_max_kpa      = get_param_or<double>(this, prefix + "mode1.p_neg_max_kpa",      101.325);
    h.ref_slew_kpa_per_s = get_param_or<double>(this, prefix + "mode1.ref_slew_kpa_per_s",   3.0);

    target_angle_deg_[(size_t)a] = c.default_angle_deg;

    RCLCPP_INFO(get_logger(),
      "[PosCtrl axis%d] 정격 P+≤%.1f / P-≥%.1f kPa | m=%.1fkg L=%.3fm | gid: pos=%d neg=%d enc=%d",
      a, c.p_pos_max_kpa, c.p_neg_min_kpa, c.mass_kg, c.link_length_m,
      c.pos_gid, c.neg_gid, c.actuator_idx);
    if (control_mode_ == 1)
      RCLCPP_INFO(get_logger(),
        "[PosCtrl axis%d] mode1 PID: kp=%.2f ki=%.3f kd=%.3f | kff=%.1f | bias P+=%.1f P-=%.1f",
        a, c.m1.kp, c.m1.ki, c.m1.kd, c.m1.kff_gravity,
        c.m1.p_bias_pos_kpa, c.m1.p_bias_neg_kpa);
  }

  // ──────────────────────────────────────────
  // 액추에이터 기하 (부피식 + 토크 환산의 단일 출처)
  // ──────────────────────────────────────────
  {
    const double dia_mm = get_param_or<double>(this, "Geometry.piston_dia_mm", 50.0);
    piston_area_mm2_    = M_PI * dia_mm * dia_mm / 4.0;
    reel_radius_mm_     = get_param_or<double>(this, "Geometry.reel_radius_mm",   25.0);
    vol_offset_pos_mm_  = get_param_or<double>(this, "Geometry.vol_offset_pos_mm", 40.0);
    vol_offset_neg_mm_  = get_param_or<double>(this, "Geometry.vol_offset_neg_mm", 90.0);
    RCLCPP_INFO(get_logger(),
      "Geometry: piston Ø%.1f mm (A=%.1f mm²), reel %.1f mm, offsets %.0f/%.0f mm",
      dia_mm, piston_area_mm2_, reel_radius_mm_, vol_offset_pos_mm_, vol_offset_neg_mm_);
  }

  // ──────────────────────────────────────────
  // 최적화 기반 압력 레퍼런스 생성기 (control_mode 2)
  // ──────────────────────────────────────────
  gen_period_ms_    = get_param_or<int>(this,  "PressureRefGen.period_ms", 20);
  gen_ref_slew_kpa_s_ = get_param_or<double>(this, "PressureRefGen.ref_slew_kpa_per_s", 150.0);
  gen_use_ej_meas_  = get_param_or<bool>(this, "PressureRefGen.use_ejector_measurement", true);
  gen_pos_ref_kpa_.assign(num_actuators_, sensor_.kpa_atm());
  gen_neg_ref_kpa_.assign(num_actuators_, sensor_.kpa_atm());
  gen_starve_pos_.assign(num_actuators_, 0.0);
  gen_starve_neg_.assign(num_actuators_, 0.0);

  tau_pid_.assign(num_actuators_, TorquePid{});
  tau_integ_.assign(num_actuators_, 0.0);
  for (int a = 0; a < num_actuators_; ++a) {
    const std::string pre = "TorquePID.axis" + std::to_string(a) + ".";
    auto& tp = tau_pid_[(size_t)a];
    tp.kp             = get_param_or<double>(this, pre + "kp",             0.0786);
    tp.ki             = get_param_or<double>(this, pre + "ki",             0.0295);
    tp.kd             = get_param_or<double>(this, pre + "kd",             0.0049);
    tp.integ_limit_nm = get_param_or<double>(this, pre + "integ_limit_nm", 2.0);
    tp.friction_nm    = get_param_or<double>(this, pre + "friction_nm",    0.30);
    tp.tau_ff_gain    = get_param_or<double>(this, pre + "tau_ff_gain",    1.0);
    tp.friction_band_deg = get_param_or<double>(this, pre + "friction_band_deg", 1.0);
  }

  {
    PressureRefGen::Params gp;
    gp.N  = num_actuators_;
    gp.dt = std::max(1e-3, gen_period_ms_ / 1000.0);
    gp.smooth_anchor_ref = get_param_or<bool>(this, "PressureRefGen.smooth_anchor_ref", true);
    const double A_m2 = piston_area_mm2_ * 1e-6;
    gp.Apos.assign(num_actuators_, A_m2);
    gp.Aneg.assign(num_actuators_, A_m2);

    // 채널 정격은 위치 제어기의 보수적 한계를 게이지 Pa 로 변환해 그대로 쓴다
    const double atm = sensor_.kpa_atm();
    const double pos_max_abs = pos_ctrl_cfg_.empty() ? 185.0   : pos_ctrl_cfg_[0].p_pos_max_kpa;
    const double neg_min_abs = pos_ctrl_cfg_.empty() ?  27.0   : pos_ctrl_cfg_[0].p_neg_min_kpa;
    gp.Pch_pos_max   = (pos_max_abs - atm) * 1000.0;
    gp.Pch_neg_min   = (neg_min_abs - atm) * 1000.0;
    // 레일 음압 셋포인트의 최대 깊이. 기본값은 채널 정격과 같지만 **별도 파라미터**다 —
    // 챔버가 필요한 깊이보다 레일을 더 깊게 요구하면 펌프 하나로 리저버+6챔버를 그
    // 깊이까지 뽑는 데 시간만 더 걸린다 (6축 정착시간의 지배 요인).
    gp.Pneg_cap_deep = get_param_or<double>(this, "PressureRefGen.rail.neg_sp_deep_kpa",
                                            neg_min_abs - atm) * 1000.0;

    gp.n_ch     = get_param_or<double>(this, "PressureRefGen.n_chamber", 1.4);
    gp.n_rail   = get_param_or<double>(this, "PressureRefGen.n_rail",    1.0);
    gp.Hpreview = get_param_or<int>   (this, "PressureRefGen.preview_steps", 1);
    gp.Pneg_shallow = get_param_or<double>(this, "PressureRefGen.rail.neg_shallow_kpa", -30.0) * 1000.0;
    gp.Ppos_sp_min  = get_param_or<double>(this, "PressureRefGen.rail.pos_sp_min_kpa",   30.0) * 1000.0;
    gp.Ppos_sp_max  = get_param_or<double>(this, "PressureRefGen.rail.pos_sp_max_kpa",  400.0) * 1000.0;
    gp.Fmax_ref     = get_param_or<double>(this, "PressureRefGen.rail.demand_ref_N",    150.0);
    gp.rail_pos_headroom = get_param_or<double>(this, "PressureRefGen.rail.pos_headroom_kpa", 60.0) * 1000.0;
    gp.rail_sp_decay_tau  = get_param_or<double>(this, "PressureRefGen.rail.sp_decay_tau_s", 2.0);
    gp.chamber_neg_headroom = get_param_or<double>(this, "PressureRefGen.rail.chamber_neg_headroom_kpa", 15.0) * 1000.0;
    gp.chamber_pos_headroom = get_param_or<double>(this, "PressureRefGen.rail.chamber_pos_headroom_kpa", 15.0) * 1000.0;
    gp.supply_filter_tau_s  = get_param_or<double>(this, "PressureRefGen.rail.supply_filter_tau_s", 0.5);
    gp.P_tank_stop  = get_param_or<double>(this, "PressureRefGen.tank_stop_kpa",        450.0) * 1000.0;

    gp.wtrack   = get_param_or<double>(this, "PressureRefGen.weights.track",  100.0);
    gp.w_flow   = get_param_or<double>(this, "PressureRefGen.weights.flow",     0.3);
    gp.w_smooth = get_param_or<double>(this, "PressureRefGen.weights.smooth",   0.5);
    gp.w_tank   = get_param_or<double>(this, "PressureRefGen.weights.tank",    15.0);
    gp.w_eject  = get_param_or<double>(this, "PressureRefGen.weights.eject",   25.0);
    gp.max_iter = get_param_or<int>   (this, "PressureRefGen.sqp_max_iter",     12);

    gp.Cd = get_param_or<double>(this, "PressureRefGen.Cd", 0.8);
    gp.valve_open_eta = get_param_or<double>(this, "PressureRefGen.valve_open_eta", 1.0);

    // 펌프 기하 — 키 이름을 시뮬(Virtual.pump.*)과 같게 맞춰 pump_params.yaml 하나로
    // 두 소비자를 동시에 갱신할 수 있게 한다. 이 블록이 없으면 생성기는 PistonPump.hpp
    // 하드코딩(예전 펌프)을 쓰고, 시뮬만 yaml 을 따라 **아무 경고 없이 어긋난다**.
    gp.pump.delta  = get_param_or<double>(this, "PressureRefGen.pump.delta_m",         gp.pump.delta);
    gp.pump.r      = get_param_or<double>(this, "PressureRefGen.pump.crank_m",         gp.pump.r);
    gp.pump.l      = get_param_or<double>(this, "PressureRefGen.pump.rod_m",           gp.pump.l);
    gp.pump.Spis   = get_param_or<double>(this, "PressureRefGen.pump.piston_area_m2",  gp.pump.Spis);
    gp.pump.Cb_out = get_param_or<double>(this, "PressureRefGen.pump.cb_out_m2",       gp.pump.Cb_out);
    gp.pump.Cb_in  = get_param_or<double>(this, "PressureRefGen.pump.cb_in_m2",        gp.pump.Cb_in);
    gp.pump.Npis   = get_param_or<int>   (this, "PressureRefGen.pump.n_piston",        gp.pump.Npis);
    gp.pump.omega  = get_param_or<double>(this, "PressureRefGen.pump.rpm",
                                          gp.pump.omega * 60.0 / (2.0 * M_PI)) * 2.0 * M_PI / 60.0;
    gp.pump_grid_n = get_param_or<int>(this, "PressureRefGen.pump_grid_n", gp.pump_grid_n);
    gp.set_orifices(
      get_param_or<double>(this, "PressureRefGen.orifice_mm.fill",   2.3),
      get_param_or<double>(this, "PressureRefGen.orifice_mm.vent",   4.0),
      get_param_or<double>(this, "PressureRefGen.orifice_mm.boost",  1.6),
      get_param_or<double>(this, "PressureRefGen.orifice_mm.suck",   4.0),
      get_param_or<double>(this, "PressureRefGen.orifice_mm.admit",  4.0),
      get_param_or<double>(this, "PressureRefGen.orifice_mm.eject",  4.0));

    refgen_ = std::make_unique<PressureRefGen>(gp);
    RCLCPP_INFO(get_logger(),
      "PressureRefGen: N=%d, dt=%.0f ms, 정격 P⁺≤%.1f kPa / P⁻≥%.1f kPa (gauge), "
      "Cd=%.2f eta=%.2f, F_max=%.1f N → τ_max=%.2f N·m",
      gp.N, gp.dt * 1e3, gp.Pch_pos_max / 1e3, gp.Pch_neg_min / 1e3,
      gp.Cd, gp.valve_open_eta,
      gp.Pch_pos_max * A_m2 + std::abs(gp.Pch_neg_min) * A_m2,
      (gp.Pch_pos_max + std::abs(gp.Pch_neg_min)) * A_m2 * reel_radius_mm_ * 1e-3);

    RCLCPP_INFO(get_logger(),
      "펌프 기하: delta=%.4f m r=%.4f l=%.4f Spis=%.4e Cb_out=%.3e Cb_in=%.3e "
      "rpm=%.0f Npis=%d  (소기량 %.2f mL, 사구간 %.3f mL, 압축비 %.1f)",
      gp.pump.delta, gp.pump.r, gp.pump.l, gp.pump.Spis, gp.pump.Cb_out, gp.pump.Cb_in,
      gp.pump.omega * 60.0 / (2.0 * M_PI), gp.pump.Npis,
      gp.pump.Spis * 2.0 * gp.pump.r * 1e6,
      gp.pump.Spis * (gp.pump.delta - 2.0 * gp.pump.r) * 1e6,
      (gp.pump.delta - 2.0 * gp.pump.r) > 1e-9
        ? gp.pump.delta / (gp.pump.delta - 2.0 * gp.pump.r) : -1.0);
    RCLCPP_INFO(get_logger(), "펌프 능력 테이블 계산 중...");
    const auto t0 = std::chrono::steady_clock::now();
    refgen_->build_pump_table();

    // 실측 능력경계(pump_fit_solve.py Phase F) — 기하 피팅보다 우선한다. 기하는
    // 5-파라미터 슬라이더-크랭크라 소기량×Cb_in 축퇴가 남아 데드헤드 외삽 오차가
    // 크다(자기검증 ~15%); 측정 구간 안은 직접 측정으로 덮어쓰고 밖은 기하 외삽을
    // 그대로 둔다. pump_frontier_measured 가 없으면(빈 벡터) 기하 테이블 그대로.
    {
      const auto pneg_kpa = get_param_or<std::vector<double>>(this,
          "PressureRefGen.pump_frontier_measured.pneg_kpa_gauge", {});
      const auto ppos_kpa = get_param_or<std::vector<double>>(this,
          "PressureRefGen.pump_frontier_measured.ppos_max_kpa_gauge", {});
      if (!pneg_kpa.empty() && pneg_kpa.size() == ppos_kpa.size()) {
        std::vector<double> pneg_pa(pneg_kpa.size()), ppos_pa(ppos_kpa.size());
        for (size_t i = 0; i < pneg_kpa.size(); ++i) {
          pneg_pa[i] = pneg_kpa[i] * 1000.0;
          ppos_pa[i] = ppos_kpa[i] * 1000.0;
        }
        refgen_->apply_measured_frontier(pneg_pa, ppos_pa);
        RCLCPP_INFO(get_logger(), "PressureRefGen: 실측 능력경계 %zu 점으로 cap_ppos 덮어씀 (Phase F)",
                    pneg_kpa.size());
      }
    }

    RCLCPP_INFO(get_logger(), "펌프 테이블 완료 (%.2f s). 능력경계: 음압 %.1f kPa → 양압 %.1f kPa",
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(),
      gp.Pneg_cap_deep / 1e3, refgen_->cap_ppos(gp.Pneg_cap_deep) / 1e3);
  }

  pub_refgen_dbg_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "controller/pressure_ref_dbg", 10);

  ref_server_cfg_.enable  = get_param_or<bool>(this, "RefTcpServer.enable",  false);
  ref_server_cfg_.port    = get_param_or<int> (this, "RefTcpServer.port",    2293);
  ref_server_cfg_.pos_gid = get_param_or<int> (this, "RefTcpServer.pos_gid", 0);
  ref_server_cfg_.neg_gid = get_param_or<int> (this, "RefTcpServer.neg_gid", num_positive_channels_);
  ref_server_cfg_.all_channels = get_param_or<bool>(this, "RefTcpServer.all_channels", false);

  if (ref_server_cfg_.enable) {
    if (control_mode_ == 1 || control_mode_ == 2) {
      // 위치 제어 모드: TCP가 축 개수(num_actuators_)만큼의 angle_ref_deg 를 수신
      ref_server_cfg_.num_values = num_actuators_;
      ref_server_ = std::make_unique<RefTcpServer>(
        ref_server_cfg_,
        [this](const std::vector<double>& angles) {
          {
            std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
            for (size_t i = 0; i < angles.size() && i < target_angle_deg_.size(); ++i)
              target_angle_deg_[i] = angles[i];
            pos_tcp_received_ = true;
          }
          std::string s;
          for (double a : angles) s += (s.empty() ? "" : ", ") + std::to_string(a);
          RCLCPP_INFO(rclcpp::get_logger("RefTcpServer"), "[PosCtrl] angle_ref = [%s] deg", s.c_str());
        }
      );
      RCLCPP_INFO(get_logger(),
        "RefTcpServer [POSITION mode]: port %d — expects [%d doubles: angle_ref_deg per axis]",
        ref_server_cfg_.port, num_actuators_);
    } else {
      if (ref_server_cfg_.all_channels) {
        // 6축 자동 스윕용 전체 채널 입력. 순서는 mpc_ref_kpa_의 global id와 동일하다:
        // [P+ axis1..6, P- axis1..6], 모두 kPa absolute의 little-endian double.
        ref_server_cfg_.num_values = num_total_channels_;
        ref_server_ = std::make_unique<RefTcpServer>(
          ref_server_cfg_,
          [this](const std::vector<double>& v) {
            if ((int)v.size() < num_total_channels_) return;
            std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
            for (int gid = 0; gid < num_total_channels_; ++gid)
              mpc_ref_kpa_[(size_t)gid] = v[(size_t)gid];
          }
        );
        RCLCPP_INFO(get_logger(),
          "RefTcpServer [PRESSURE ALL mode]: port %d — expects [%d doubles: P+ axes then P- axes, kPa abs]",
          ref_server_cfg_.port, num_total_channels_);
      } else {
        // 기존 수동 시험 호환: 지정한 양압/음압 채널 한 쌍만 받는다.
        ref_server_cfg_.num_values = 2;
        ref_server_ = std::make_unique<RefTcpServer>(
          ref_server_cfg_,
          [this](const std::vector<double>& v) {
            if (v.size() < 2) return;
            std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
            const int pg = ref_server_cfg_.pos_gid;
            const int ng = ref_server_cfg_.neg_gid;
            if (pg >= 0 && pg < (int)mpc_ref_kpa_.size()) mpc_ref_kpa_[pg] = v[0];
            if (ng >= 0 && ng < (int)mpc_ref_kpa_.size()) mpc_ref_kpa_[ng] = v[1];
          }
        );
        RCLCPP_INFO(get_logger(),
          "RefTcpServer [PRESSURE mode]: port %d (pos_gid=%d, neg_gid=%d) — expects [double pos_kpa, double neg_kpa]",
          ref_server_cfg_.port, ref_server_cfg_.pos_gid, ref_server_cfg_.neg_gid);
      }
    }
  }

  filt_state_.assign(NUM_CAN_BOARDS, 101.325);

  zero_calib_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/zero_calibration",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
           std_srvs::srv::Trigger::Response::SharedPtr res) {
      on_zero_calibration(req, res);
    });

  pressure_safety_limit_kpa_      = get_param_or<double>(this, "pressure_safety_limit_kpa",           170.0);
  pressure_safety_hysteresis_kpa_ = get_param_or<double>(this, "pressure_safety_hysteresis_kpa",       10.0);
  RCLCPP_INFO(get_logger(), "Over-pressure safety: limit=%.1f kPa, hysteresis=%.1f kPa (release at %.1f kPa)",
    pressure_safety_limit_kpa_, pressure_safety_hysteresis_kpa_,
    pressure_safety_limit_kpa_ - pressure_safety_hysteresis_kpa_);

  start_time_ = std::chrono::steady_clock::now();
  elapsed_time_sec_ = 0.0;

  RCLCPP_INFO(this->get_logger(), "Controller node initialization complete.");
}

void Controller::on_zero_calibration(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  sensor_zero_sum_.fill(0.0);
  sensor_zero_cnt_.fill(0);
  sensor_zero_tick_ = 0;
  sensor_zeroed_    = false;
  RCLCPP_INFO(get_logger(), "Sensor zero-calibration re-triggered (current values → offset).");
  res->success = true;
  res->message = "Zero-calibration started. Offsets will update in ~0.5 sec.";
}

Controller::~Controller()
{
  if (log_file_.is_open()) {
    log_file_.close();
    RCLCPP_INFO(get_logger(), "Log file mpc_log.csv closed.");
  }
}

// ============================================================================
// 게인 로더 — 한 접두어의 키를 읽어 없는 것은 넘겨받은 값을 유지한다.
// ============================================================================
PressureCtrl::Gains Controller::load_gains(const std::string& pre, PressureCtrl::Gains g) {
  // yaml 에 **있는 키만** 덮어쓴다 (get_param_opt 주석 참조 — 없는 키를 기본값으로
  // 선언하면 그 선언이 뒤 단계를 오염시켜 우선순위가 무너진다).
  auto set = [&](const char* key, float& dst) {
    if (auto v = get_param_opt(this, pre + key)) dst = (float)*v;
  };
  set("kp",                  g.kp);
  set("kp_far",              g.kp_far);
  set("kp_break_kpa",        g.kp_break_kpa);
  set("ki",                  g.ki);
  set("ki_far",              g.ki_far);
  set("kd",                  g.kd);
  set("i_limit_pct",         g.i_limit_pct);
  set("i_deadband_kpa",      g.i_deadband_kpa);   // 예전에 빠져 있어 yaml 이 무시됐다
  set("band_kpa",            g.band_kpa);
  set("i_reset_on_step_kpa", g.i_reset_on_step_kpa);
  set("gain_dp_ref_kpa",     g.gain_dp_ref_kpa);
  set("gain_scale_min",      g.gain_scale_min);
  set("gain_scale_max",      g.gain_scale_max);
  set("kv",                  g.kv);                 // 레퍼런스 속도 피드포워드
  return g;
}

// 채널 × 측(양/음압) × 방향(상승/하강) 을 모두 개별 튜닝할 수 있게 6단으로 겹친다.
// 뒤 단계가 앞을 덮어쓰고, yaml 에 없는 키는 앞 단계 값이 남는다.
PressureCtrl::Gains Controller::gains_for(int gid, bool is_positive, const char* dir) {
  const std::string side = is_positive ? "pos" : "neg";
  const std::string ch   = "channel_config.ch" + std::to_string(gid) + ".pid.";
  PressureCtrl::Gains g{};                                    // 코드 기본값
  g = load_gains("ChannelPID.", g);                           // 1 전 채널·전 방향
  g = load_gains(std::string("ChannelPID.") + dir + ".", g);  // 2 방향별
  g = load_gains("ChannelPID." + side + ".", g);              // 3 측별
  g = load_gains("ChannelPID." + side + "." + dir + ".", g);  // 4 측별·방향별
  g = load_gains(ch, g);                                      // 5 채널별
  g = load_gains(ch + dir + ".", g);                          // 6 채널별·방향별
  return g;
}

// ============================================================================
// build_ctrls — 활성 채널마다 PressureCtrl 인스턴스 하나
//
// 활성 채널은 **축별 gid 설정**에서 온다. 예전에는 0..num_actuators-1 로
// 하드코딩돼 있어서, PositionController.axisN.pos_gid 를 바꿔도 그 채널의
// 제어기가 만들어지지 않아 무시됐다.
//
// 이 덕분에 축 하나로 임의의 물리 채널을 돌릴 수 있다 (채널별 부피·밸브를
// 하나씩 재려면 필수다 — 여럿을 같이 돌리면 레일을 나눠 쓰느라 차압이 흔들려
// 측정이 흩어진다).
//   예: ch2 만 → num_actuators=1, axis0.pos_gid=2, axis0.neg_gid=8,
//               axis0.actuator_idx=2   (control.launch.py 의 axis:=2 가 이걸 한다)
//
// 주의: pos_ctrl_cfg_ 는 이 함수보다 **뒤에** 로드되므로 파라미터를 직접 읽는다.
// ============================================================================
void Controller::build_ctrls() {
  active_channels_.clear();
  for (int i = 0; i < num_actuators_; ++i) {
    const std::string pfx = "PositionController.axis" + std::to_string(i) + ".";
    const int pg = get_param_or<int>(this, pfx + "pos_gid", i);
    const int ng = get_param_or<int>(this, pfx + "neg_gid", num_positive_channels_ + i);
    if (pg >= 0 && pg < num_total_channels_) active_channels_.insert(pg);
    else RCLCPP_ERROR(get_logger(), "axis%d.pos_gid=%d 가 범위 밖이다 (0~%d)",
                      i, pg, num_total_channels_ - 1);
    if (ng >= 0 && ng < num_total_channels_) active_channels_.insert(ng);
    else RCLCPP_ERROR(get_logger(), "axis%d.neg_gid=%d 가 범위 밖이다 (0~%d)",
                      i, ng, num_total_channels_ - 1);
  }
  {
    std::string s;
    for (int gid : active_channels_) { char b[16]; snprintf(b, sizeof(b), " %d", gid); s += b; }
    RCLCPP_INFO(get_logger(), "[활성 채널 gid]%s  (보드 = gid + %d)",
                s.c_str(), channel_board_offset_);
  }

  ctrls_.clear();
  ctrls_.reserve(active_channels_.size());

  for (int gid = 0; gid < num_total_channels_; ++gid) {
    if (active_channels_.find(gid) == active_channels_.end()) continue;

    const bool is_pos = (gid < num_positive_channels_);

    PressureCtrl::Config cfg;
    cfg.can_board_id = gid + channel_board_offset_;   // 예: gid 0 → board 5
    cfg.global_id    = gid;
    cfg.is_positive  = is_pos;
    cfg.u_min_pct    = 0.0f;
    cfg.u_max_pct    = 100.0f;
    cfg.ref_rate_tau_s = (float)get_param_or<double>(this, "ChannelPID.ref_rate_tau_s", 0.3);
    cfg.ff_limit_pct   = (float)get_param_or<double>(this, "ChannelPID.ff_limit_pct",  15.0);

    // PID 게인: 양압/음압 공통 기본값 → 채널별 오버라이드
    // 상승/하강 방향 게인을 따로 읽는다 (6단 우선순위 — gains_for 주석 참조)
    cfg.g_up   = gains_for(gid, is_pos, "up");
    cfg.g_down = gains_for(gid, is_pos, "down");

    ctrls_.emplace_back(std::make_unique<PressureCtrl>(cfg));
  }

  RCLCPP_INFO(get_logger(), "채널 압력 PID %zu개 생성 (활성 채널마다 하나)", ctrls_.size());
  // 데드존은 표(차압별)일 수도 상수일 수도 있다. 무엇이 실제로 적용됐는지 한 줄로
  // 보여 준다 — yaml 만 보고는 표가 로드됐는지 알 수 없다.
  auto dz_desc = [](const DzTable& t) {
    char b[64];
    if (t.dp_kpa.empty()) { snprintf(b, sizeof(b), "%.1f%%(상수)", t.flat); return std::string(b); }
    snprintf(b, sizeof(b), "%.1f~%.1f%%(dp %.0f~%.0f, %zu점)",
             t.u_pct.back(), t.u_pct.front(), t.dp_kpa.front(), t.dp_kpa.back(),
             t.dp_kpa.size());
    return std::string(b);
  };
  // 채널마다 방향별 게인과 대기 깊이를 한 줄씩 찍는다 — 무엇이 실제로 적용됐는지
  // yaml 만 보고는 알 수 없다 (6단 우선순위 + 채널 오버라이드).
  for (const auto& cc : ctrls_) {
    const auto& k = cc->cfg();
    const int g = k.global_id;
    RCLCPP_INFO(get_logger(),
      "  ch%-2d %s | 상승 kp=%.3f(far %.3f, 경계 %.1f) ki=%.3f(far %.3f) kd=%.4f i_lim=%.0f%%"
      " | 하강 kp=%.3f(far %.3f, 경계 %.1f) ki=%.3f(far %.3f) kd=%.4f i_lim=%.0f%%"
      " | i_db=%.2f 스텝리셋>%.1f kPa"
      " | 여유 mi %.2f / at %.2f %%p, 데드존 micro %s atm %s",
      g, k.is_positive ? "양압" : "음압",
      (double)k.g_up.kp, (double)k.g_up.kp_far, (double)k.g_up.kp_break_kpa,
      (double)k.g_up.ki, (double)k.g_up.ki_far, (double)k.g_up.kd, (double)k.g_up.i_limit_pct,
      (double)k.g_down.kp, (double)k.g_down.kp_far, (double)k.g_down.kp_break_kpa,
      (double)k.g_down.ki, (double)k.g_down.ki_far, (double)k.g_down.kd,
      (double)k.g_down.i_limit_pct,
      (double)k.g_up.i_deadband_kpa, (double)k.g_up.i_reset_on_step_kpa,
      dz_margin_ch_[(size_t)g][PressureCtrl::V_MICRO],
      dz_margin_ch_[(size_t)g][PressureCtrl::V_ATM],
      dz_desc(dz_ch_[(size_t)g][PressureCtrl::V_MICRO]).c_str(),
      dz_desc(dz_ch_[(size_t)g][PressureCtrl::V_ATM]).c_str());
  }

  zoh_.fill(0);
}

PressureCtrl* Controller::ctrl_for_gid(int gid) const {
  for (const auto& c : ctrls_)
    if (c && c->cfg().global_id == gid) return c.get();
  return nullptr;
}

void Controller::on_sensor(const std_msgs::msg::UInt16MultiArray::SharedPtr m) {
  {
    std::lock_guard<std::mutex> lk(sensors_mtx_);
    const size_t n = std::min(m->data.size(), sensors_raw_.size());
    for (size_t i = 0; i < n; ++i) sensors_raw_[i] = m->data[i];
  }
  on_timer();
}

// 각도 목표를 슬루 제한으로 램프시킨다.
//
// TCP 로 들어온 목표를 계단으로 주면 레퍼런스 생성기가 즉시 큰 힘을 요구하고
// 액추에이터가 그만큼 세게 튄다. 목표를 램프시키면 압력 레퍼런스도 따라서
// 완만해진다 — 액추에이터를 보수적으로 움직일 때 여기부터 조인다.
// target_slew_deg_per_s <= 0 이면 계단 그대로다.
void Controller::slew_targets(double dt_sec) {
  // ── 기동 시: 슬루 상태를 **현재 각도에서 한 번만** 출발시킨다 ────────────
  //
  // 목표 자체는 처음부터 default_angle_deg (= 0) 다. 시작하면 모든 축이 0° 로
  // 내려온다. 다만 **출발점**은 지금 팔이 있는 각도여야 한다 — 슬루 상태를 0 에서
  // 시작하면 첫 틱에 angle_ref 가 측정각에서 0 으로 계단 점프한다.
  // 실기 20260829_165306: t=10.79 에 각도 19.9° 인데 목표가 0.45° 로 떨어졌다
  // (−19.4° 계단) — 액추에이터에 그대로 충격으로 간다.
  //
  // 한 번 씨앗을 심고 나면 평범하게 target_slew_deg_per_s 로 0 까지 램프한다.
  // (예전에는 TCP 명령이 올 때까지 계속 측정각에 붙여 뒀다. 그러면 목표가 팔을
  //  따라다니기만 하고 0 으로 안 갔다.)
  if (!slew_seeded_) {
    std::array<double, 9> ang;
    { std::lock_guard<std::mutex> lk(sensors_mtx_); ang = encoder_angles_; }
    // 엔코더가 아직 안 들어왔으면(전부 0) 다음 틱에 다시 시도한다 — 0 에서
    // 출발했다가 진짜 각도가 들어오는 순간 계단이 되는 것을 막는다.
    bool have = false;
    for (size_t i = 0; i < pos_ctrl_cfg_.size() && !have; ++i) {
      const int enc = std::clamp(pos_ctrl_cfg_[i].actuator_idx, 0, (int)ang.size() - 1);
      if (std::abs(ang[(size_t)enc]) > 1e-9) have = true;
    }
    if (target_angle_slewed_.size() != target_angle_deg_.size())
      target_angle_slewed_.assign(target_angle_deg_.size(), 0.0);
    for (size_t i = 0; i < target_angle_slewed_.size(); ++i) {
      const int enc = (i < pos_ctrl_cfg_.size())
          ? std::clamp(pos_ctrl_cfg_[i].actuator_idx, 0, (int)ang.size() - 1) : (int)i;
      target_angle_slewed_[i] = ang[(size_t)enc];
    }
    target_slew_rate_.assign(target_angle_slewed_.size(), 0.0);
    if (have || slew_seed_ticks_++ > 500) {   // 500 틱(=1 s @500 Hz) 지나면 0 으로 확정
      slew_seeded_ = true;
      std::string vs;
      for (size_t i = 0; i < target_angle_slewed_.size(); ++i) {
        char b[32]; snprintf(b, sizeof(b), " ax%zu=%.1f°", i, target_angle_slewed_[i]); vs += b;
      }
      RCLCPP_INFO(get_logger(),
        "기동 목표: 전 축 %.1f° 로 내려간다 (%.0f deg/s). 현재 각도에서 출발:%s",
        target_angle_deg_.empty() ? 0.0 : target_angle_deg_[0], target_slew_dps_, vs.c_str());
    }
    return;
  }

  if (target_slew_dps_ <= 0.0 || dt_sec <= 0.0) {
    target_angle_slewed_ = target_angle_deg_;
    target_slew_rate_.assign(target_angle_slewed_.size(), 0.0);   // 계단이면 속도 FF 없음
    return;
  }
  if (target_angle_slewed_.size() != target_angle_deg_.size())
    target_angle_slewed_ = target_angle_deg_;
  if (target_slew_rate_.size() != target_angle_slewed_.size())
    target_slew_rate_.assign(target_angle_slewed_.size(), 0.0);
  const std::vector<double> prev = target_angle_slewed_;
  const double step = target_slew_dps_ * dt_sec;
  for (size_t i = 0; i < target_angle_deg_.size(); ++i) {
    const double d = target_angle_deg_[i] - target_angle_slewed_[i];
    target_angle_slewed_[i] += std::clamp(d, -step, step);
  }

  // 목표 슬루 **속도** 를 남긴다. 제어기의 D 항이 이 값을 피드포워드로 쓴다 —
  // 그래야 D 가 "명령한 움직임" 이 아니라 "명령에서 벗어난 만큼" 만 억제한다.
  for (size_t i = 0; i < target_angle_slewed_.size(); ++i)
    target_slew_rate_[i] = (target_angle_slewed_[i] - prev[i]) / dt_sec;
}

// 토픽: actuator/volumes_ml  (Float64MultiArray, num_total_channels_ 개, 단위 mL)
// 활성 채널만 업데이트. 비활성 채널은 default_volume_ml_ 유지.
void Controller::on_volume(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (!actuator_connected_) return;
  const int n = std::min((int)msg->data.size(), num_total_channels_);
  for (int i = 0; i < n; ++i) {
    if (active_channels_.count(i) == 0) continue;
    if (msg->data[i] > 0.0) vol_ml_[i] = msg->data[i];
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  RailFF — 실측 레일 맵의 역함수
// ════════════════════════════════════════════════════════════════════════════
double Controller::RailFF::interp(const std::vector<double>& xs,
                                  const std::vector<double>& ys, double x) {
  if (xs.empty() || ys.size() != xs.size()) return 0.0;
  if (x <= xs.front()) return ys.front();
  if (x >= xs.back())  return ys.back();
  for (size_t i = 1; i < xs.size(); ++i) {
    if (xs[i] >= x) {
      const double d = xs[i] - xs[i - 1];
      const double f = (d > 1e-9) ? (x - xs[i - 1]) / d : 0.0;
      return ys[i - 1] + f * (ys[i] - ys[i - 1]);
    }
  }
  return ys.back();
}

double Controller::RailFF::ppos_min() const {
  double v = 1e18;
  for (const auto& c : vent_ppos) if (!c.empty()) v = std::min(v, c.front());
  return (v > 1e17) ? 0.0 : v;
}

double Controller::RailFF::ppos_max() const {
  double v = -1e18;
  for (const auto& c : vent_ppos) if (!c.empty()) v = std::max(v, c.back());
  return (v < -1e17) ? 1e6 : v;
}

double Controller::RailFF::admit_at(double p_neg) const {
  return interp(admit_pneg, admit_u, p_neg);
}

double Controller::RailFF::vent_at(double p_pos, double admit) const {
  // 유입 수준별 곡선에서 각각 방출 개도를 뽑은 뒤, 그 값을 유입으로 보간한다.
  std::vector<double> per;
  per.reserve(vent_admit.size());
  for (size_t k = 0; k < vent_admit.size(); ++k)
    per.push_back(interp(vent_ppos[k], vent_u[k], p_pos));
  return interp(vent_admit, per, admit);
}

double Controller::RailFF::vent_slope(double p_pos, double admit) const {
  // |dP+/d방출| — 게인 스케줄용. 역함수의 기울기를 수치로 뒤집는다.
  const double du = 1.0;
  const double a = vent_at(p_pos - 5.0, admit);
  const double b = vent_at(p_pos + 5.0, admit);
  const double d_u = std::abs(b - a);
  return (d_u > 1e-6) ? (10.0 / d_u) : 0.0;   // ΔP+ / Δ방출
  (void)du;
}

void Controller::on_timer() {
  // DMY 한 틱 지도:
  //   (0) 필요한 CAN 센서 수신 확인 및 시작 영점 보정
  //   (1) ADC raw -> kPa absolute 변환과 저역통과 필터
  //   (2) 목표각 slew 후 control_mode에 맞는 압력 레퍼런스 생성
  //   (3) 엔코더 각도로 각 챔버의 현재 부피 계산
  //   (4) 채널별 압력 PID 를 병렬 실행해 밸브 PWM 생성
  //   (5) 공유 레일 PID와 macro switch 결정
  //   (6) 과압/센서 이상 안전 로직 적용 후 CAN bridge로 publish
  // 이 함수가 길어도 위 순서대로 구역이 나뉘어 있으므로, 먼저 여기서 데이터의
  // 수명을 따라가고 세부 수식은 호출된 함수로 내려가면 된다.
  // 경과 시간을 **틱 카운트**에서 만든다. 벽시계로 만들면 5초 안에 제어 틱이 몇 번
  // 들어가는지가 실행마다 달라져, 아래 두 게이트(밸브 잠금 해제 / 적분 리셋 해제)가
  // 서로 다른 시점에 풀린다 — 같은 빌드 반복 실행에서 정상상태 밸브 개방률이
  // 0/32/100% 로 흩어진 원인 중 하나다 (README 0절).
  // 틱 기준이면 같은 입력에 항상 같은 궤적이 나오고, 오프라인 하네스로 옮길 때도
  // 시간 주입점이 여기 한 곳으로 끝난다.
  elapsed_time_sec_ = (double)tick_ * (double)std::max(1, period_ms_) / 1000.0;
  const auto tick_now = std::chrono::steady_clock::now();
  wall_elapsed_sec_ = std::chrono::duration<double>(tick_now - start_time_).count();

  // ── 이번 틱의 dt ────────────────────────────────────────────────────
  // elapsed_time_sec_ 는 게이트 재현성을 위해 **틱 기반**으로 남기고(README 0절),
  // 물리 계산에 쓰는 dt 만 실측으로 바꾼다. 둘의 역할이 다르다:
  // 전자는 "몇 번째 틱인가", 후자는 "얼마나 시간이 흘렀는가" 다.
  {
    const double nom = (double)std::max(1, period_ms_) / 1000.0;
    if (tick_ > 0) {
      const double raw = std::chrono::duration<double>(tick_now - last_tick_time_).count();
      // 스케줄러 스파이크가 dt 를 오염시키지 않게 공칭의 [0.25, 4] 배로 자른다.
      const double d = std::clamp(raw, 0.25 * nom, 4.0 * nom);
      dt_meas_sec_ = (dt_meas_sec_ <= 0.0) ? d : (0.02 * d + 0.98 * dt_meas_sec_);
    }
    last_tick_time_ = tick_now;
    dt_ctrl_sec_ = (use_measured_dt_ && dt_meas_sec_ > 0.0) ? dt_meas_sec_ : nom;
  }

  std::array<uint16_t, NUM_CAN_BOARDS> snap_sensors;
  {
    std::lock_guard<std::mutex> lk(sensors_mtx_);
    snap_sensors = sensors_raw_;
  }

  // ----------------------------------------------------------------
  // 0-. **센서가 유효해지기 전에는 밸브를 절대 건드리지 않는다**
  //
  // raw 0 = 그 보드 프레임을 한 번도 못 받았다는 뜻이다. 예전에는 그걸 그대로
  // 환산해 (0 − 1112) × 0.25 + 101.325 = **−176.7 kPa** 를 진짜 압력으로 믿었다.
  // 컨트롤러는 "챔버가 대기압보다 278 kPa 낮다" 로 보고 micro·macro 를 활짝 열고,
  // 그 지령이 CAN 송신 큐에 쌓였다. CAN 이 8.5 초 뒤 살아나자 backlog 가 순서대로
  // 쏟아져 탱크 580 kPa 가 챔버로 들어갔고(228 kPa) 팔이 스토퍼를 넘었다 —
  // 20260829_201659, 액추에이터 파손.
  //
  // 0 kPa 로 붙잡아 두는 것도 안 된다 (그것도 진공이라 똑같이 채우려 든다).
  // 필요한 보드(라인 4 개 + 활성 채널) 가 **하나라도** 아직 안 왔으면 제어를
  // 시작하지 않는다. 여기서 return 하면 PWM 이 안 나가고, 브리지의 PWM 워치독이
  // 200 ms 뒤 안전 상태(채널 폐쇄 + 라인 전개)로 간다.
  // ----------------------------------------------------------------
  {
    std::string bad;
    for (int bid = 1; bid <= 16; ++bid) {
      const int idx = bid - 1;
      if (snap_sensors[idx] > 0) sensor_seen_[(size_t)idx] = true;
      if (sensor_seen_[(size_t)idx]) continue;
      const int gid = bid - channel_board_offset_;
      const bool is_line = (bid == P_pos_board_id_ || bid == P_neg_board_id_ ||
                            bid == P_macro_board_id_ || bid == P_macro_neg_board_id_);
      const bool needed = is_line ||
          (gid >= 0 && gid < num_total_channels_ && active_channels_.count(gid) > 0);
      if (needed) bad += (bad.empty() ? "" : ", ") + std::to_string(bid);
    }
    if (!bad.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "센서 대기: board %s 의 프레임이 **한 번도** 안 왔다 (raw=0). 제어를 "
        "시작하지 않는다 — 이 상태로 돌면 압력을 −176.7 kPa 로 읽고 밸브를 활짝 "
        "연다 (20260829 액추에이터 파손 경로). CAN 배선·보드 전원을 확인할 것.",
        bad.c_str());
      return;
    }
  }

  // ----------------------------------------------------------------
  // 0. Sensor zero-calibration: boards 1~16 초기화 (0.5초간 평균)
  // ----------------------------------------------------------------
  if (!sensor_zeroed_) {
    for (int i = 0; i < 16; ++i) {
      if (snap_sensors[i] > 0) {
        sensor_zero_sum_[i] += snap_sensors[i];
        sensor_zero_cnt_[i]++;
      }
    }
    sensor_zero_tick_++;

    if (sensor_zero_tick_ >= ZERO_SAMPLES) {
      for (int i = 0; i < 16; ++i) {
        if (sensor_zero_cnt_[i] == 0) continue;
        const double meas = sensor_zero_sum_[i] / sensor_zero_cnt_[i];
        // 0점 보정은 "지금 이 압력이 대기압이다" 라고 선언하는 것이다. 계통에 잔압이
        // 남은 채 부르면 그 채널의 영점이 잔압만큼 통째로 밀리고, **과압 세이프티도
        // 같이 밀린다** (190 kPa 트립이 실제 210 kPa 에서 걸리는 식). 조용히 넘어가면
        // 안 되므로 yaml 값과의 차이를 kPa 로 환산해 경고한다.
        const double dev_kpa = (meas - yaml_offset_[(size_t)i]) * sensor_.boards[i].gain;
        if (std::abs(dev_kpa) > zero_tolerance_kpa_)
          RCLCPP_WARN(get_logger(),
            "0점 보정: board %d 가 yaml 기준에서 %+.1f kPa 벗어났다 "
            "(offset %.1f → %.1f). 계통에 잔압이 남아 있지 않은지 확인할 것 — "
            "영점이 밀리면 과압 세이프티도 같이 밀린다.",
            i + 1, dev_kpa, yaml_offset_[(size_t)i], meas);
        sensor_.boards[i].offset = meas;
      }
      sensor_zeroed_ = true;
      RCLCPP_INFO(get_logger(), "=== Sensor zero-calibration complete ===");
      for (int bid = 1; bid <= 16; ++bid)
        RCLCPP_INFO(get_logger(), "  Board %2d: offset=%.1f", bid, sensor_.boards[bid-1].offset);
    }
    return;   // 초기화 중에는 제어 출력 없음
  }

  // ----------------------------------------------------------------
  // 1. Raw Data -> kPa 변환 및 LPF 적용 (보드별 flat 루프)
  // ----------------------------------------------------------------
  for (int bid = 1; bid <= NUM_CAN_BOARDS; ++bid) {
    int idx = bid - 1;

    // 엔코더 보드 (17~22): **여기서 각도를 만들지 않는다.**
    //
    // board/sensors 는 보드 1~16 만 싣는다 (CanBridge 가 PWM_BOARDS=16 개만 낸다).
    // 그래서 snap_sensors[16..21] 은 언제나 0 이고, 그걸 반전증폭 역산에 넣으면
    // (4125−0)/0.825 = 5000 mV 라 보정에 따라 −85° 니 +247° 니 하는 상수가 나온다.
    // 실제 각도는 CanBridge 가 board/analog 로 내고 encoder_angles_ 로 들어온다.
    //
    // 이 죽은 경로 때문에 Sensor_calibration 에 엔코더 보정 사본을 두게 됐고,
    // 그 사본이 EncoderCalibration 과 어긋나 두 번 사고를 냈다 (S-28 board 17,
    // S-30 board 18). 사본은 지웠고 여기서는 0 을 낸다 — controller/sensors_kpa
    // 를 보는 쪽에 "이 슬롯엔 압력이 없다" 가 분명해진다.
    if (bid >= 17 && bid <= 22) {
      filt_out_[idx] = 0.0;
      continue;
    }

    // 라인 압력 보드 or 활성 채널 보드만 처리
    bool is_line_board = (bid == P_pos_board_id_ || bid == P_neg_board_id_ || bid == P_macro_board_id_ || bid == P_macro_neg_board_id_);
    int gid = bid - channel_board_offset_;
    if (!is_line_board && (gid < 0 || gid >= num_total_channels_ || active_channels_.count(gid) == 0)) {
      filt_out_[idx] = sensor_.kpa_atm();
      raw_out_[idx]  = sensor_.kpa_atm();
      continue;
    }
    // raw 0 = 그 보드 프레임을 **못 받았다** (CAN 두절·보드 전원). 이걸 그대로
    // 환산하면 (0 − 1112) × 0.25 + 101.325 = **−176.7 kPa** 라는 물리적으로
    // 불가능한 값이 나오고, 컨트롤러는 "대기압보다 278 kPa 낮다" 로 믿어 밸브를
    // 활짝 연다. 20260829_201659 의 액추에이터 파손이 정확히 그 경로였다.
    // 직전 값을 유지하고 소리를 낸다 (브리지의 RX 워치독가 별도로 안전 상태로 간다).
    if (snap_sensors[idx] == 0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "board %d raw=0 — 프레임이 없다. 압력을 직전 값(%.1f kPa)으로 유지한다. "
        "그대로 환산하면 −176.7 kPa 가 되어 밸브가 활짝 열린다.", bid, filt_out_[idx]);
      continue;
    }
    double raw_kpa = sensor_.kpa(bid, snap_sensors[idx]);
    if (!filter_initialized_) filt_state_[idx] = raw_kpa;
    filt_state_[idx] = sensor_filter_alpha_ * raw_kpa + (1.0 - sensor_filter_alpha_) * filt_state_[idx];
    filt_out_[idx]   = filt_state_[idx];
    raw_out_[idx]    = raw_kpa;              // 예측기 초기 상태 비교용 (지연 없음, 잡음 있음)
  }
  filter_initialized_ = true;

  // 채널 PID 진단 — 2초마다 한 줄. 오차·적분 누적·명령을 같이 봐야
  // "게인이 낮아 못 가는 것" 과 "포화·크래킹에 걸린 것" 을 구분할 수 있다.
  if (tick_ > 0 && (tick_ % 1000) == 0 && !ctrls_.empty()) {
    std::string line;
    for (auto& c : ctrls_) {
      char buf[96];
      const int g = c->cfg().global_id;
      const bool ok = (g >= 0 && g < (int)u_hw_pct_.size());
      const std::array<float,3> u = ok ? u_hw_pct_[(size_t)g] : std::array<float,3>{};
      snprintf(buf, sizeof(buf),
               " ch%d[e=%+.1f i=%+.0f%% u=%.0f/%.0f/%.0f]",
               g, (double)c->error_kpa(), (double)c->integ_pct(),
               (double)u[0], (double)u[1], (double)u[2]);
      line += buf;
    }
    RCLCPP_INFO(get_logger(),
      "[PID e=오차kPa i=적분%% u=하드웨어명령%% (micro/macro/atm)]%s", line.c_str());

    // ── 레일 한 줄 ────────────────────────────────────────────────────
    // 채널만 찍고 레일을 안 찍으면, 레일 목표가 바뀌었는지·피드포워드가 무슨 개도를
    // 내고 있는지를 **로그만 보고는 알 수 없다.** 실제로 그래서 "레퍼런스가 안
    // 들어온다" 를 원인까지 못 좁혔다 (20260912).
    {
      const double pp = filt_out_[P_pos_board_id_ - 1];
      const double pn = filt_out_[P_neg_board_id_ - 1];
      char ff[96] = "피드포워드 OFF";
      if (rail_ff_.ok()) {
        const double a0 = rail_ff_.admit_at(pid_neg_.ref);
        snprintf(ff, sizeof(ff), "ff 방출 %.1f%% 유입 %.1f%%",
                 rail_ff_.vent_at(pid_pos_.ref, a0), a0);
      }
      // 개도가 한쪽 끝에 붙고 적분도 한계인데 압력이 안 움직이면 **플랜트가 없다.**
      // 거의 항상 펌프가 꺼져 있는 경우다 (20260912 에 이걸로 한참 헤맸다).
      const bool stuck =
          std::abs(pp - pid_pos_.ref) > 10.0 &&
          std::abs(pid_pos_state_.integ * pid_pos_.ki) >= pid_pos_.i_limit - 0.5 &&
          std::abs(pp - rail_pp_prev_) < 0.5;
      rail_pp_prev_ = pp;
      if (stuck)
        RCLCPP_ERROR(get_logger(),
          "레일이 **전혀 안 움직인다** — 오차 %.1f kPa 인데 적분은 한계이고 압력 변화가 "
          "0.5 kPa 미만이다. 펌프가 도는지 먼저 확인할 것 (밸브만으로는 레일을 못 만든다).",
          pp - pid_pos_.ref);
      RCLCPP_INFO(get_logger(),
        "[레일] P+ 목표 %.1f 실측 %.1f (e %+.1f) · P− 목표 %.1f 실측 %.1f (e %+.1f) "
        "| 개도 방출 %.1f%% 유입 %.1f%% | %s | 적분 %+.1f/%+.1f",
        pid_pos_.ref, pp, pp - pid_pos_.ref,
        pid_neg_.ref, pn, pn - pid_neg_.ref,
        zoh_[(size_t)pid_pos_pwm_index_] / 40.95,
        zoh_[(size_t)pid_neg_pwm_index_] / 40.95,
        ff, pid_pos_state_.integ * pid_pos_.ki, pid_neg_state_.integ * pid_neg_.ki);
    }
  }

  // ----------------------------------------------------------------
  // 2. 필터링된 값을 Topic으로 Publish
  // ----------------------------------------------------------------
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(filt_out_.begin(), filt_out_.end());
    pub_kpa_all_->publish(msg);
  }

  const double P_line_pos_kPa       = filt_out_[P_pos_board_id_ - 1];
  const double P_line_neg_kPa       = filt_out_[P_neg_board_id_ - 1];
  const double P_line_macro_kPa     = filt_out_[P_macro_board_id_ - 1];
  const double P_line_macro_neg_kPa = filt_out_[P_macro_neg_board_id_ - 1];
  const double P_atm_kPa            = sensor_.kpa_atm();

  // ----------------------------------------------------------------
  // 위치 제어: filt_out_ 갱신 이후, ref_snapshot 이전에 실행
  // 각도 → 압력 레퍼런스 변환 후 mpc_ref_kpa_ 에 기록
  // ----------------------------------------------------------------
  slew_targets(std::max(1e-6, dt_ctrl_sec_));
  if (control_mode_ == 1) {
    run_position_control(std::max(1e-6, dt_ctrl_sec_));
  } else if (control_mode_ == 2) {
    run_optimized_pressure_ref(std::max(1e-6, dt_ctrl_sec_));
  }

  {
    std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
    ref_snapshot_ = mpc_ref_kpa_;
  }
  if (pub_mpc_refs_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(ref_snapshot_.begin(), ref_snapshot_.end());
    pub_mpc_refs_->publish(msg);

    // 레일 상태 — [P+목표, P−목표, ff방출, ff유입, 개도방출, 개도유입]
    if (pub_rail_dbg_) {
      double ffv = 100.0, ffa = 100.0;
      if (rail_ff_.ok()) {
        ffa = rail_ff_.admit_at(pid_neg_.ref);
        ffv = rail_ff_.vent_at(pid_pos_.ref, ffa);
      }
      // 레일 루프 내부를 **항별로** 남긴다. 개도만 보면 무엇이 흔드는지 못 가른다 —
      // 20260912 에 "kd 가 리플을 키운다" 로 잘못 짚었다가 kd 0 으로 돌려도
      // 리플이 그대로여서 처음부터 다시 봐야 했다.
      std_msgs::msg::Float64MultiArray rd;
      rd.data = {pid_pos_.ref, pid_neg_.ref, ffv, ffa,
                 zoh_[(size_t)pid_pos_pwm_index_] / 40.95,
                 zoh_[(size_t)pid_neg_pwm_index_] / 40.95,
                 filt_out_[P_pos_board_id_ - 1], filt_out_[P_neg_board_id_ - 1],
                 rail_u_pos_, pid_pos_state_.integ * pid_pos_.ki, rail_gs_pos_,
                 rail_u_neg_, pid_neg_state_.integ * pid_neg_.ki};
      pub_rail_dbg_->publish(rd);
    }

    // 채널 PID 내부. 채널당 12 개 × 12 채널 = 144.
    // [u_pid, P, I, D, FF, I상태(0정상/1클램프/2정지), 오차, 목표변화율,
    //  이득배율, dz_micro, dz_atm, dp_micro]
    {
      std_msgs::msg::Float64MultiArray cd;
      cd.data.reserve(12 * CH_DBG_N);
      for (const auto& g_ : chan_dbg_)
        cd.data.insert(cd.data.end(), g_.begin(), g_.end());
      pub_chan_dbg_->publish(cd);
    }
  }

  // ----------------------------------------------------------------
  // 액추에이터 연결 시 엔코더 각도로 부피 계산 (actuator_connected=true)
  //
  //   A     = π × (piston_dia/2)²  = π × 25²  [mm²]   (Ø50 mm 피스톤)
  //   reel  = 25 mm  — 조인트에서 피스톤 로드까지의 릴 반경
  //
  //   피스톤 변위 x = reel × θ  이고 θ 는 **라디안**이다. 엔코더는 도(deg)로 오므로
  //   반드시 변환해야 한다. (도를 그대로 넣으면 45°에서 V⁺가 2338 mL 이 되어
  //   Ø50 mm 실린더로 불가능한 값이 나온다 — 원래 있던 단위 버그.)
  //
  //   양압: V_pos = tank_pos_mL + A × max(0,  40 + reel×θ_rad) / 1000  [mL]
  //   음압: V_neg = tank_neg_mL + A × max(0,  90 - reel×θ_rad) / 1000  [mL]
  //
  //   actuator i  →  encoder board (17+i)  →  ang[i]  [deg]
  // ----------------------------------------------------------------
  if (actuator_connected_) {
      std::array<double, 9> ang;
      {
          std::lock_guard<std::mutex> lk(sensors_mtx_);
          ang = encoder_angles_;
      }
      const double A = piston_area_mm2_;
      for (int i = 0; i < num_positive_channels_; ++i) {
          if (active_channels_.count(i) == 0) continue;
          const double x_mm    = reel_radius_mm_ * ang[i] * M_PI / 180.0;   // 피스톤 변위
          const int    neg_gid = num_positive_channels_ + i;

          // 기하 모델에 **채널별 배율**을 곱한다.
          //
          // 절대 오버라이드(channel_config.chN.volume_ml)는 여기서 매 틱 덮어써져
          // 소용이 없다 — 액추에이터가 붙어 있으면 부피가 각도의 함수이기 때문이다.
          // 배율이면 각도 의존성은 그대로 두고 크기만 고친다.
          //
          // 실기 20260829_195910: 세 축이 같은 게인·질량·기하인데 axis1 만 진동했다
          // (각도 p-p: ax0 4.6° / ax2 3.2° / **ax1 22.0°**). 같은 지령·차압에서
          // dP/dt 를 재니 ch1 이 ch0 의 4.97 배였다. dP/dt = ṁ·R·T/V 이므로
          // 유효 부피가 1/5 라는 뜻이고, 기하 모델이 146 mL 로 보는 동안 MPC 는
          // 5 배 유량을 요구해 과개방하고 있었다.
          vol_ml_[i] = vol_scale_[(size_t)i] * (tank_volume_pos_ml_
              + A * std::max(0.0, vol_offset_pos_mm_ + x_mm) / 1000.0);

          if (neg_gid < num_total_channels_ && active_channels_.count(neg_gid))
              vol_ml_[neg_gid] = vol_scale_[(size_t)neg_gid] * (tank_volume_neg_ml_
                  + A * std::max(0.0, vol_offset_neg_mm_ - x_mm) / 1000.0);
      }
  }

  std::fill(final_active_vols_ml_.begin(), final_active_vols_ml_.end(), 0.0);
  for(int i = 0; i < num_total_channels_; ++i) {
      if (active_channels_.count(i) == 0) continue;
      final_active_vols_ml_[i] = vol_ml_[i];
  }

  if (pub_active_vols_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(final_active_vols_ml_.begin(), final_active_vols_ml_.end());
    pub_active_vols_->publish(msg);
  }

  // ── 채널 압력 PID ────────────────────────────────────────────────────────
  // 활성 채널마다 PressureCtrl::compute 를 한 번 부른다. 채널끼리 상태를 공유하지
  // 않으므로 ThreadPool 로 병렬 실행한다 (500 Hz 를 맞추려면 필요하다).
  {
    const int phase = static_cast<int>(tick_ % MPC_PHASES);
    std::vector<std::function<void()>> tasks;

    for (auto& ctrl : ctrls_) {
      if ((ctrl->cfg().global_id % MPC_PHASES) != phase) continue;

      PressureCtrl* c = ctrl.get();   // 람다마다 자기 제어기를 잡게 한다
      tasks.emplace_back([this, c,
                          P_line_pos_kPa, P_line_neg_kPa,
                          P_line_macro_kPa, P_line_macro_neg_kPa, P_atm_kPa]() {
        const int  brd_idx  = c->cfg().can_board_id - 1;
        const int  gid      = c->cfg().global_id;
        const bool pos_side = c->cfg().is_positive;

        PressureCtrl::Input in;

        // ★ 이 채널의 목표 압력. control_mode 에 따라 만든 주인이 다르지만
        //   (TCP 직접 / 위치 PID / PressureRefGen) 여기서는 모두 같은 배열이다.
        in.P_ref_kpa = (gid >= 0 && gid < (int)ref_snapshot_.size())
                     ? (float)ref_snapshot_[(size_t)gid] : (float)sensor_.kpa_atm();

        in.P_meas_kpa     = (float)filt_out_[brd_idx];
        in.P_meas_raw_kpa = (float)raw_out_[brd_idx];

        in.P_supply_kpa     = (float)(pos_side ? P_line_pos_kPa : P_line_neg_kPa);
        in.P_macro_kpa      = (float)P_line_macro_kPa;
        in.P_macro_neg_kpa  = (float)P_line_macro_neg_kPa;
        in.P_atm_kpa        = (float)P_atm_kPa;

        in.volume_m3      = (float)(final_active_vols_ml_[(size_t)gid] * 1e-6);
        in.prev_volume_m3 = (float)prev_vol_m3_[(size_t)gid];

        in.rail_rate_kpa_s = 0.0f;   // 레일 변화율 추정은 이 브랜치에 없다
        in.dt_sec = (float)dt_ctrl_sec_;
        in.t_sec  = (float)elapsed_time_sec_;
        in.safety_latched = (gid >= 0 && gid < (int)safety_latched_.size())
                          ? safety_latched_[(size_t)gid] : false;

        // ── 밸브 데드존 보상 (제어기 **밖**) ───────────────────────────────
        // 각 밸브는 어떤 지령까지는 전혀 열리지 않는다 (스프링 예압). 그 구간에서
        // 만든 제어량은 전부 버려지므로, PID 출력의 원점을 죽은 구간 끝으로 옮긴다.
        //
        //   u_hw = deadzone + u_pid
        //
        // deadzone 은 **실측 상수**다 (yaml, scripts/valve_deadzone.py). 밸브 모델로
        // 매 틱 역산하던 예전 방식은 버렸다 — 13-parameter 가 6채널 공용이라 실제
        // 임계와 채널별로 최대 2 %p 어긋났고, 양압 micro 가 임계 아래에 놓여 간헐
        // 펄스로만 열렸다 (20260908_171520: 양압 RMSE 2.0~2.7 / 음압 0.9).
        //
        // anti-windup 에 실효 상한(100 − deadzone)을 먼저 알려 준다. 이것을 넘겨
        // 주지 않으면 제어기는 100 까지 여유가 있다고 보고, 밸브가 이미 활짝 열린
        // 뒤에도 적분을 계속 쌓는다 (도달 후 오버슛의 원인).
        // 이 밸브가 실제로 받고 있는 차압(상류 − 하류)으로 표를 조회한다.
        //   양압 micro 레일→챔버   양압 atm 챔버→대기
        //   음압 micro 챔버→레일   음압 atm 대기→챔버
        std::array<float, PressureCtrl::N_VALVE> dz{};
        if (dz_enable_) {
          const float dp[3] = {
            pos_side ? (in.P_supply_kpa - in.P_meas_kpa)     // micro
                     : (in.P_meas_kpa   - in.P_supply_kpa),
            pos_side ? (in.P_macro_kpa  - in.P_meas_kpa)     // macro
                     : (in.P_meas_kpa   - in.P_macro_neg_kpa),
            pos_side ? (in.P_meas_kpa   - in.P_atm_kpa)      // atm
                     : (in.P_atm_kpa    - in.P_meas_kpa),
          };
          // 표에서 여유를 뺀다. 과보상은 유량 0 을 만들 수 없게 해 릴레이 진동을
          // 낳으므로, 표가 틀렸을 때 **부족한 쪽으로** 틀리게 만든다
          // (Controller.hpp 의 dz_margin_pct_ 주석 참조).
          const auto& mgn = dz_margin_ch_[(size_t)gid];
          for (int j = 0; j < PressureCtrl::N_VALVE; ++j)
            dz[(size_t)j] = (float)std::max(
                0.0, dz_ch_[(size_t)gid][(size_t)j].at((double)dp[j]) - mgn[(size_t)j]);
          // 동작점 이득 보정에 쓸 차압 (Gains::gain_dp_ref_kpa)
          in.dp_micro_kpa = dp[PressureCtrl::V_MICRO];
          in.dp_atm_kpa   = dp[PressureCtrl::V_ATM];
          in.u_limit_up_pct   = std::clamp(100.0f - dz[PressureCtrl::V_MICRO], 1.0f, 100.0f);
          in.u_limit_down_pct = std::clamp(100.0f - dz[PressureCtrl::V_ATM],   1.0f, 100.0f);
        }

        // ★ 제어기 호출: 목표 압력 in → 밸브 명령 out
        PressureCtrl::Output out = c->compute(in);

        // u_pid = 0 인 밸브는 0 으로 둔다 — 데드존에 대기시키면 정확히 임계라
        // 온도·차압이 조금만 흔들려도 반대 방향으로 새어 나간다. PID 가 요구할
        // 때만 죽은 구간을 건너뛴다.
        if (dz_enable_) {
          for (int j = 0; j < PressureCtrl::N_VALVE; ++j) {
            float& u = out.u_pct[(size_t)j];
            if (u <= 0.0f) continue;
            u = std::clamp(dz[(size_t)j] + u, 0.0f, 100.0f);
          }
        }
        // ── 쉬는 밸브 파킹 ────────────────────────────────────────────────
        // 0 으로 끄지 않고 표 최솟값 아래에 걸어 둔다. 코일 전류가 이미 흐르고
        // 있으므로 열어야 할 때 전류가 붙는 시간이 빠진다. 파킹값은 어떤 차압의
        // 데드존보다도 낮으므로 유량은 0 이다 (Controller.hpp 주석 참조).
        if (dz_park_enable_) {
          const auto& pk = dz_park_ch_[(size_t)gid];
          for (int j = 0; j < PressureCtrl::N_VALVE; ++j)
            if (out.u_pct[(size_t)j] <= 0.0f)
              out.u_pct[(size_t)j] = (float)pk[(size_t)j];
        }
        // 안 쓰는 macro 는 어떤 경로로도 열리지 않게 마지막에 한 번 더 0 으로 못 박는다.
        if (!use_macro_) out.u_pct[PressureCtrl::V_MACRO] = 0.0f;

        if (gid >= 0 && gid < (int)u_hw_pct_.size())
          u_hw_pct_[(size_t)gid] = out.u_pct;        // 진단용 — 실제 나간 명령

        // ── 채널 PID 내부 스냅샷 ──────────────────────────────────────────
        // u_hw = (표(dp) − 여유) + u_pid 이므로, 밸브 지령만 로그하면 표가 틀렸을 때
        // 그 오차가 전부 적분처럼 보인다. 항을 나눠 찍어야 분해가 된다.
        if (gid >= 0 && gid < (int)chan_dbg_.size()) {
          auto& g_ = chan_dbg_[(size_t)gid];
          g_[0]  = c->u_pid_pct();        // 합산 지령 (데드존 더하기 전)
          g_[1]  = c->p_term_pct();
          g_[2]  = c->i_term_pct();       // ★ 적분 누적
          g_[3]  = c->d_term_pct();
          g_[4]  = c->ff_term_pct();
          g_[5]  = (double)c->integ_state();   // 0 정상 / 1 클램프 / 2 정지
          g_[6]  = c->error_kpa();
          g_[7]  = c->ref_rate_kpa_s();
          g_[8]  = c->gain_scale();
          g_[9]  = dz[PressureCtrl::V_MICRO];   // 적용된 데드존 (여유 뺀 값)
          g_[10] = dz[PressureCtrl::V_ATM];
          g_[11] = in.dp_micro_kpa;
        }

        if (gid == log_channel_id_ && log_file_.is_open()) {
          log_file_ << tick_ << "," << in.P_ref_kpa << "," << in.P_meas_kpa << "\n";
        }

        // % → PWM. 슬롯 순서 {v1=micro, v2=atm, v3=macro} 는 to_pwm 이 맞춘다.
        const auto pwm = PressureCtrl::to_pwm(out);
        const int pwm_base = brd_idx * PWM_PER_BOARD;
        zoh_[pwm_base + 0] = pwm[0];
        zoh_[pwm_base + 1] = pwm[1];
        zoh_[pwm_base + 2] = pwm[2];
      });
    }

    pool_->run_batch_and_wait(tasks);
  }

  // MacroSwitch: 음압 macro 라인 솔레노이드. macro 를 안 쓰면 같이 닫아 둔다.
  // (macro 지령이 파킹으로 0 보다 컸을 때는 이 스위치가 항상 열린 채였다.)
  if (!use_macro_ && macro_switch_pwm_index_ >= 0 && macro_switch_pwm_index_ < PWM_TOTAL) {
    zoh_[(size_t)macro_switch_pwm_index_] = 0;
  } else if (macro_switch_pwm_index_ >= 0 && macro_switch_pwm_index_ < PWM_TOTAL) {
    bool any_neg_macro_active = false;
    for (int gid = num_positive_channels_; gid < num_total_channels_; ++gid) {
      int pwm_macro_idx = (gid + channel_board_offset_ - 1) * PWM_PER_BOARD + 2;  // v3 = macro
      if (pwm_macro_idx < PWM_TOTAL && zoh_[pwm_macro_idx] > 0) {
        any_neg_macro_active = true;
        break;
      }
    }
    zoh_[(size_t)macro_switch_pwm_index_] = any_neg_macro_active ? 4095 : 0;
  }

  // ── 외부 레일 목표 반영 ──────────────────────────────────────────────
  // control_mode 2 는 PressureRefGen 이 매 틱 pid_*_.ref 를 다시 쓰므로 건드리지
  // 않는다 (두 주인이 싸우면 외부 값이 한 틱 만에 지워진다).
  if (control_mode_ != 2) {
    const double rp = rail_ref_pos_.load(std::memory_order_relaxed);
    const double rn = rail_ref_neg_.load(std::memory_order_relaxed);
    if (std::isfinite(rp)) pid_pos_.ref = rp;
    if (std::isfinite(rn)) pid_neg_.ref = rn;
  }
  // ── 목표를 측정 맵 범위로 자른다 ──────────────────────────────────────
  // 밖을 목표로 주면 ff 가 표 끝값에 붙고 PID 가 반대로 밀어 **둘이 싸운다.**
  // 특히 P− 목표가 상한 밖이면 ff 는 유입 100 을, PID 는 폐쇄를 동시에 내서
  // 개도가 0 이 된다 — 유입은 펌프 흡입구라 그러면 펌프가 통째로 굶는다.
  if (rail_ff_.ok()) {
    const double rp0 = pid_pos_.ref, rn0 = pid_neg_.ref;
    pid_pos_.ref = std::clamp(pid_pos_.ref, rail_ff_.ppos_min(), rail_ff_.ppos_max());
    pid_neg_.ref = std::clamp(pid_neg_.ref, rail_ff_.pneg_min(), rail_ff_.pneg_max());
    if (pid_pos_.ref != rp0 || pid_neg_.ref != rn0)
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 3000,
        "레일 목표가 측정 맵 밖이라 잘랐다: P+ %.1f→%.1f [%.1f~%.1f] · "
        "P− %.1f→%.1f [%.1f~%.1f]. 그 범위를 쓰려면 rail_map.py 로 다시 뜰 것.",
        rp0, pid_pos_.ref, rail_ff_.ppos_min(), rail_ff_.ppos_max(),
        rn0, pid_neg_.ref, rail_ff_.pneg_min(), rail_ff_.pneg_max());
  }

  const double dt = std::max(1e-6, dt_ctrl_sec_);   // LinePID

  // -------------------------------------------------------------
  // [양압 라인 PID] (Positive Line)
  // -------------------------------------------------------------
  {
    const double err = pid_pos_.ref - P_line_pos_kPa;
    
    pid_pos_state_.integ += err * dt;
    if (pid_pos_.ki > 1e-6) {
      const double ilim = std::abs(pid_pos_.i_limit) / pid_pos_.ki;
      pid_pos_state_.integ = std::clamp(pid_pos_state_.integ, -ilim, ilim);
    }
    double deriv = 0.0;
    if (pid_pos_state_.has_prev) deriv = (err - pid_pos_state_.prev_err) / dt;

    double u = pid_pos_.kp * err + pid_pos_.ki * pid_pos_state_.integ + pid_pos_.kd * deriv;

    pid_pos_state_.prev_err = err;
    pid_pos_state_.has_prev = true;

    // ⚠ u 를 [0,100] 으로 자르면 안 된다. 개도 = ff − gs·u 이므로 u<0 이 곧
    //   "피드포워드보다 **더 열어라**" 다. 예전 코드는 그것을 막아서 레일이
    //   ff 예측 아래로는 영영 못 내려갔다 — 20260912 실측에서 올라가는 계단
    //   (145/160)은 8~14 초에 정착했는데 내려오는 계단(170/145)은 +6.7/+11.3 kPa
    //   정상오차가 남았다. anti-windup 은 아래에서 **개도 포화** 기준으로 한다.

    // ── 피드포워드 ────────────────────────────────────────────────────
    // pwm 은 밸브 **개도**다. 예전에는 개도 = 100 − u 였다 — 즉 "활짝 열린 상태"를
    // 원점으로 두고 적분이 거기서부터 동작점을 만들었다. 실측 역맵이 있으면
    // 그 원점을 **목표 압력쌍에 맞는 개도**로 바꿔 주면 된다. 그러면 PID 는
    // 잔차만 맡고, 동작점이 바뀌어도 적분을 처음부터 쌓지 않는다.
    // (RailFF 가 꺼져 있으면 ff = 100 이라 예전과 완전히 같다.)
    double ff_vent = pid_out_max_;
    double gs_rail = 1.0;
    rail_u_pos_ = u;
    if (rail_ff_.ok()) {
      const double ff_admit = rail_ff_.admit_at(pid_neg_.ref);
      ff_vent = std::clamp(rail_ff_.vent_at(pid_pos_.ref, ff_admit),
                           pid_out_min_, pid_out_max_);
      if (rail_ff_.gain_ref > 0.0) {
        const double sl = rail_ff_.vent_slope(pid_pos_.ref, ff_admit);
        if (sl > 1e-6)
          gs_rail = std::clamp(rail_ff_.gain_ref / sl,
                               rail_ff_.gain_min, rail_ff_.gain_max);
      }
    }
    rail_gs_pos_ = gs_rail;
    // ── anti-windup: 개도가 잘린 만큼만 적분을 되돌린다 ────────────────
    // 실제로 하드웨어가 받는 것은 개도이므로, 포화 판정도 개도로 해야 한다.
    const double open_raw = ff_vent - gs_rail * u;
    const double inverted_u = std::clamp(open_raw, pid_out_min_, pid_out_max_);
    if (inverted_u != open_raw && gs_rail > 1e-9 && std::abs(pid_pos_.ki) > 1e-6) {
      const double u_eff = (ff_vent - inverted_u) / gs_rail;   // 개도가 실제로 뜻하는 u
      pid_pos_state_.integ -= (u - u_eff) / pid_pos_.ki;
      if (pid_pos_.ki > 1e-6) {
        const double ilim = std::abs(pid_pos_.i_limit) / pid_pos_.ki;
        pid_pos_state_.integ = std::clamp(pid_pos_state_.integ, -ilim, ilim);
      }
    }
    // [수정] 4095 스케일 (40.95 = 4095/100)
    const uint16_t pwm = static_cast<uint16_t>( std::round(inverted_u * 40.95) );

    if (pid_pos_pwm_index_ >= 0 && pid_pos_pwm_index_ < PWM_TOTAL) {
      zoh_[(size_t)pid_pos_pwm_index_] = pwm;
    }
  }

  // -------------------------------------------------------------
  // [음압 라인 PID] (Negative Line)
  // -------------------------------------------------------------
  {
    const double err = P_line_neg_kPa - pid_neg_.ref; 
    
    pid_neg_state_.integ += err * dt;
    if (pid_neg_.ki > 1e-6) {
      const double ilim = std::abs(pid_neg_.i_limit) / pid_neg_.ki;
      pid_neg_state_.integ = std::clamp(pid_neg_state_.integ, -ilim, ilim);
    }
    double deriv = 0.0;
    if (pid_neg_state_.has_prev) deriv = (err - pid_neg_state_.prev_err) / dt;

    double u = pid_neg_.kp * err + pid_neg_.ki * pid_neg_state_.integ + pid_neg_.kd * deriv;

    pid_neg_state_.prev_err = err;
    pid_neg_state_.has_prev = true;

    // 양압과 같은 이유로 u 를 자르지 않는다 (개도 = ff − u).

    // 유입 개도는 P− 목표가 거의 단독으로 정한다 (실측 +1.27 kPa/%p 대 방출 −0.32).
    const double ff_admit = rail_ff_.ok()
        ? std::clamp(rail_ff_.admit_at(pid_neg_.ref), pid_out_min_, pid_out_max_)
        : pid_out_max_;
    rail_u_neg_ = u;
    const double open_raw_n = ff_admit - u;
    const double inverted_u = std::clamp(open_raw_n, pid_out_min_, pid_out_max_);
    if (inverted_u != open_raw_n && std::abs(pid_neg_.ki) > 1e-6) {
      const double u_eff = ff_admit - inverted_u;
      pid_neg_state_.integ -= (u - u_eff) / pid_neg_.ki;
      if (pid_neg_.ki > 1e-6) {
        const double ilim = std::abs(pid_neg_.i_limit) / pid_neg_.ki;
        pid_neg_state_.integ = std::clamp(pid_neg_state_.integ, -ilim, ilim);
      }
    }
    // [수정] 4095 스케일 (40.95 = 4095/100)
    const uint16_t pwm = static_cast<uint16_t>( std::round(inverted_u * 40.95) );

    if (pid_neg_pwm_index_ >= 0 && pid_neg_pwm_index_ < PWM_TOTAL) {
      zoh_[(size_t)pid_neg_pwm_index_] = pwm;
    }
  }

  inner_loop_1khz(static_cast<float>(period_ms_));

  if (tick_ > 0 && (tick_ % 5000) == 0) {
    const double ratio = wall_elapsed_sec_ / std::max(1e-9, elapsed_time_sec_);
    if (ratio < 0.97 || ratio > 1.03)
      RCLCPP_WARN(get_logger(),
        "틱 간격: 가정 %.1f ms, 실측 평균 %.2f ms (비 %.3f). dt 는 %s. "
        "period_ms 를 실측에 맞추면 게이트 시간(elapsed)도 함께 맞는다.",
        (double)period_ms_, (double)period_ms_ * ratio, ratio,
        use_measured_dt_ ? "실측값을 쓴다 (모델 오차 없음)"
                         : "period_ms 를 쓴다 — 이 괴리가 그대로 모델 오차다");
  }

  if (valve_operate_ && elapsed_time_sec_ >= 5.0) {
    for (int i = 0; i < PWM_TOTAL; ++i)
      cmds_[i] = clamp_pwm(static_cast<int>(zoh_[i]) + inner_[i]);
  } else {
    std::fill(cmds_.begin(), cmds_.end(), 0);
  }

  // ----------------------------------------------------------------
  // Over-pressure safety: positive channels (hysteresis latch)
  // Latch ON  when P >= limit            → force exhaust fully open
  // Latch OFF when P <  limit - hyst     → return control to MPC
  // ----------------------------------------------------------------
  for (int gid = 0; gid < num_positive_channels_; ++gid) {
    if (active_channels_.count(gid) == 0) continue;
    int bid     = gid + channel_board_offset_;
    int brd_idx = bid - 1;
    if (brd_idx < 0 || brd_idx >= NUM_CAN_BOARDS) continue;

    const double P = filt_out_[brd_idx];
    const double release_threshold = pressure_safety_limit_kpa_ - pressure_safety_hysteresis_kpa_;

    if (P >= pressure_safety_limit_kpa_) {
      safety_latched_[gid] = true;
    } else if (P < release_threshold) {
      safety_latched_[gid] = false;
    }

    if (safety_latched_[gid]) {
      int base        = brd_idx * PWM_PER_BOARD;
      cmds_[base + 0] = 0;     // micro valve: closed
      cmds_[base + 1] = 4095;  // exhaust valve: fully open
      cmds_[base + 2] = 0;     // macro valve: closed
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
        "[SAFETY] ch%d (board %d) P=%.1f kPa, latched (limit=%.1f, release=%.1f)",
        gid, bid, P, pressure_safety_limit_kpa_, release_threshold);
    }
  }

  publish_cmds();

  for(int i = 0; i < num_total_channels_; ++i) {
      prev_vol_m3_[i] = final_active_vols_ml_[i] * 1.0e-6;
  }

  ++tick_;
}

// ================================
// run_position_control
// ================================
// 외층 위치 PID + 중력 피드포워드 + 마찰 보상
// 결과를 mpc_ref_kpa_ 에 기록하면 이후 MPC 내층이 압력 추종
//
// 중력 토크: τ = m·g·L·cos(90°-θ) = m·g·L·sin(θ_rad)
// P_ff = kff_gravity × τ          [kPa]
//
// 마찰 (쿨롱):
//   운동 중 → sign(vel) × friction_kpa   (운동 방향 반대로 보상)
//   정지 근처 → sign(error) × friction_kpa (정지마찰 극복)
//
// 출력:
//   P_pos_ref = p_bias_pos + (P_pid + P_ff + P_friction)
//   P_neg_ref = p_bias_neg - (P_pid + P_ff + P_friction) × neg_coupling
//
// actuator_connected_=false (액추에이터 미연결, 순수 압력추종 테스트):
//   엔코더 각도가 고정돼 있어 PID/마찰 보상은 의미가 없으므로 끄고,
//   중력 FF만 목표각(angle_ref) 기준으로 계산해 목표압력을 만든다.
//   → position_ref_client.py로 보낸 각도(30°, 45°, ...)마다 서로 다른
//     목표압력이 생성되고, 그 압력을 MPC(단일 채널)가 추종하는지 확인 가능.
void Controller::run_position_control(double dt_sec)
{
  const int n = (int)pos_ctrl_cfg_.size();
  std::vector<double> dbg_all;
  dbg_all.reserve((size_t)n * 8);

  for (int a = 0; a < n; ++a) {
    auto& cfg   = pos_ctrl_cfg_[(size_t)a];
    auto& state = pos_ctrl_state_[(size_t)a];

    // 엔코더 각도: board/analog 토픽 → encoder_angles_[] (sensors_mtx_ 보호)
    // board/sensors raw(filt_out_) 는 데이터가 없으면 0이라 사용 불가
    const int enc_idx = cfg.actuator_idx;
    if (enc_idx < 0 || enc_idx >= (int)encoder_angles_.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[PosCtrl axis%d] actuator_idx=%d out of range", a, enc_idx);
      dbg_all.insert(dbg_all.end(), 8, 0.0);
      continue;
    }
    double angle;
    {
      std::lock_guard<std::mutex> lk(sensors_mtx_);
      angle = encoder_angles_[(size_t)enc_idx];
    }

    // 목표 각도: TCP 수신 전까지는 현재 각도 유지 (급격한 움직임 방지)
    double angle_ref;
    {
      std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
      // 항상 슬루된 목표를 쓴다. 예전에는 TCP 수신 전에 측정각을 그대로 넣어
      // 오차를 0 으로 두었는데, 그러면 기동 목표(0°)가 적용되지 않았다.
      angle_ref = target_angle_slewed_[(size_t)a];
    }

    // 최초 진입: 속도 추정기 초기화만 하고 제어 출력은 건너뜀
    if (!state.initialized) {
      state.prev_angle    = angle;
      state.vel_filt      = 0.0;
      state.integral      = 0.0;
      state.p_pos_ref_filt = cfg.m1.p_bias_pos_kpa;
      state.p_neg_ref_filt = cfg.m1.p_bias_neg_kpa;
      state.initialized   = true;
      dbg_all.insert(dbg_all.end(), {angle, angle_ref, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
      continue;
    }

    // ── 각속도 추정 (유한차분 + LPF) ──
    const double vel_raw = (angle - state.prev_angle) / dt_sec;   // [deg/s]
    state.vel_filt = cfg.vel_filter_alpha * vel_raw
                    + (1.0 - cfg.vel_filter_alpha) * state.vel_filt;
    state.prev_angle = angle;
    const double vel = state.vel_filt;

    // ── PID ──
    // actuator_connected_=false 이면 엔코더가 실제로 움직이지 않아 error가 절대 해소되지
    // 않는다. 이 상태에서 PID를 그대로 돌리면 kp*error(비례항은 클램프 없음)가 각도 명령에
    // 비례해 무한정 커져 45°만 넘어도 p_pos_max_kpa에 곧장 포화되고, 45/60/90°가 전부 같은
    // 압력으로 뭉개진다 (20260818, 액추에이터 미연결 압력추종 테스트 중 확인).
    // → 액추에이터 미연결 시에는 PID를 끄고, 아래 중력 FF만으로 목표압력을 생성한다.
    // 모드 2 와 같은 처리 — 목표는 건드리지 않고 **제어 오차만** 밴드로 자른다.
    const double error_raw = angle_ref - angle;
    const double error = (target_follow_band_deg_ > 0.0)
        ? std::clamp(error_raw, -target_follow_band_deg_, target_follow_band_deg_)
        : error_raw;
    // D 항은 목표 속도를 빼고 본다 (명령한 움직임은 억제하지 않는다).
    const double vel_ref_m1 = (a < (int)target_slew_rate_.size() && pos_tcp_received_)
        ? target_slew_rate_[(size_t)a] : 0.0;
    const double vel_err_m1 = vel - kd_vel_ff_ * vel_ref_m1;
    double p_pid = 0.0;

    if (actuator_connected_) {
      // 적분 (부호 반전 시 리셋: 목표 반대 방향으로 쌓인 적분이 오버슈트를 유발하지 않도록)
      if ((error > 0.0 && state.integral < 0.0) ||
          (error < 0.0 && state.integral > 0.0)) {
        state.integral = 0.0;
      }
      // 적분 (와인드업 방지: 포화 전 클램핑)
      state.integral += error * dt_sec;
      const double integ_limit = (std::abs(cfg.m1.ki) > 1e-9)
                                 ? cfg.m1.integral_limit_kpa / cfg.m1.ki
                                 : 0.0;
      state.integral = std::clamp(state.integral, -integ_limit, integ_limit);

      p_pid = cfg.m1.kp * error
            + cfg.m1.ki * state.integral
            - cfg.m1.kd * vel_err_m1;   // 미분: 측정 − 목표 속도
    }

    // ── 중력 피드포워드 ──
    // τ = m·g·L·cos(90°-angle) = m·g·L·sin(angle_rad)
    // actuator_connected_=false: 실제 각도가 고정돼 있으므로 목표각(angle_ref) 기준으로
    // 계산해야 각도 명령이 실제로 서로 다른 목표압력에 매핑된다.
    const double ff_angle = actuator_connected_ ? angle : angle_ref;
    const double angle_rad = ff_angle * M_PI / 180.0;
    const double tau_gravity = cfg.mass_kg * 9.81
                              * cfg.link_length_m
                              * std::sin(angle_rad);          // [N·m]
    const double p_ff = cfg.m1.kff_gravity * tau_gravity;   // [kPa]

    // ── 마찰 보상 (쿨롱) ── error 방향으로 보상 (vel 방향은 수축 필요 시 역방향 힘을 줌)
    // 액추에이터 미연결 시에는 실제 움직임이 없으므로 마찰 보상도 의미가 없어 생략.
    double p_friction = 0.0;
    if (actuator_connected_ && std::abs(error) > 0.3) {
      p_friction = cfg.m1.friction_kpa * (error > 0.0 ? 1.0 : -1.0);
    }

    // ── 합산 및 압력 레퍼런스 생성 ──
    const double delta = p_pid + p_ff + p_friction;

    const double p_pos_unsat = cfg.m1.p_bias_pos_kpa + delta;
    double p_pos = std::clamp(p_pos_unsat, cfg.m1.p_pos_min_kpa, cfg.p_pos_max_kpa);

    double p_neg = std::clamp(
      cfg.m1.p_bias_neg_kpa - delta * cfg.m1.neg_coupling,
      cfg.p_neg_min_kpa, cfg.m1.p_neg_max_kpa);

    // P+ 포화 & 연장 방향 오차 시 음압 독립 구동
    // neg_coupling은 delta에 비례하므로 P+가 천장(anti-windup으로 delta 동결)에
    // 걸리면 P-도 같이 멈춤. error>0인 동안 P-를 p_neg_min으로 독립 구동해
    // 차압을 최대화한다.
    if (p_pos_unsat > cfg.p_pos_max_kpa && error > 0.0) {
      p_neg = cfg.p_neg_min_kpa;
    }

    // 포화 시 적분 되돌리기 (back-calculation anti-windup)
    // 오차 방향과 같은 방향으로 포화된 경우에만 취소:
    //   - 연장 필요(error>0)인데 p_pos가 최대에 걸림 → 더 밀어봤자 의미없음
    //   - 수축 필요(error<0)인데 p_pos가 최소에 걸림 → 중력이 이미 수축 중, 적분은 계속 쌓음
    const bool sat_same_dir = (p_pos_unsat > cfg.p_pos_max_kpa && error > 0.0) ||
                              (p_pos_unsat < cfg.m1.p_pos_min_kpa && error < 0.0);
    if (actuator_connected_ && sat_same_dir && std::abs(cfg.m1.ki) > 1e-9) {
      state.integral -= error * dt_sec;   // 이번 적분 취소
    }

    // ── 압력 레퍼런스 슬루레이트 제한 ──
    // p_pos/p_neg가 한 tick에서 크게 점프하면(예: 위치명령 변경) 밸브모델-실제 불일치로
    // 큰 실압력 스파이크가 발생할 수 있으므로, MPC에 넘기는 레퍼런스 자체를
    // ref_slew_kpa_per_s 로 제한된 램프로 바꿔 서서히 목표에 도달하게 한다.
    const double max_step = cfg.m1.ref_slew_kpa_per_s * dt_sec;
    state.p_pos_ref_filt += std::clamp(p_pos - state.p_pos_ref_filt, -max_step, max_step);
    state.p_neg_ref_filt += std::clamp(p_neg - state.p_neg_ref_filt, -max_step, max_step);
    p_pos = state.p_pos_ref_filt;
    p_neg = state.p_neg_ref_filt;

    // mpc_ref_kpa_ 에 기록 (MPC 내층이 이 값을 압력 레퍼런스로 사용)
    {
      std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
      const int pg = cfg.pos_gid;
      const int ng = cfg.neg_gid;
      if (pg >= 0 && pg < (int)mpc_ref_kpa_.size()) mpc_ref_kpa_[pg] = p_pos;
      if (ng >= 0 && ng < (int)mpc_ref_kpa_.size()) mpc_ref_kpa_[ng] = p_neg;
    }

    dbg_all.insert(dbg_all.end(), {angle, angle_ref, p_pos, p_neg, p_pid, p_ff, p_friction, vel});

    // 500Hz × 250 = 0.5초마다 출력
    if (tick_ % 250 == 0) {
      RCLCPP_INFO(get_logger(),
        "[PosCtrl axis%d] θ=%.2f°  ref=%.2f°  err=%+.2f°  vel=%+.1fdps | "
        "pid=%+.1f ff=%.1f fric=%+.1f → P+=%.1f  P-=%.1f kPa",
        a, angle, angle_ref, error, vel, p_pid, p_ff, p_friction, p_pos, p_neg);
    }
  }

  // 디버그 토픽 발행 (500Hz → 구독 측에서 다운샘플 권장)
  // 축마다 8개씩 이어붙임: [angle, angle_ref, p_pos_ref, p_neg_ref, p_pid, p_ff, p_friction, vel_dps] × n
  if (pub_pos_dbg_) {
    std_msgs::msg::Float64MultiArray dbg;
    dbg.data = dbg_all;
    pub_pos_dbg_->publish(dbg);
  }
}

// ================================
// run_optimized_pressure_ref  (control_mode 2)
// ================================
// 위치 PID → 목표 토크 → PressureRefGen → 12개 목표 압력 + 적응 레일 셋포인트
//
// mode 1 과의 차이:
//   - PID 출력이 kPa 가 아니라 **토크(N·m)** 다. 중력 FF 가 이미 m·g·L·sinθ 로 토크를
//     계산하고 있었으므로 kff_gravity 곱셈만 빼면 그대로 정확한 FF 가 된다
//     (기존 kff=3.0 은 물리값 20.4 의 1/6.8 이라 90°에서 중력의 29% 만 보상했다).
//   - 양압/음압 분배가 고정 bias±delta 가 아니라, 그 순간 밸브에 실제 흐르는 유량으로
//     만든 슬루 박스 안에서 최적화로 결정된다.
//   - 라인압 셋포인트가 고정 상수가 아니라 수요에 따라 능력경계 위에서 재배분된다.
//
// 생성기는 gen_period_ms_ (기본 20 ms) 마다 한 번만 돌리고, 그 사이에는 결과를 유지한다
// (레일은 초 단위로 느리고 챔버는 20 ms 안에 수십 kPa 움직이므로 계층 분리가 성립).
void Controller::run_optimized_pressure_ref(double dt_sec)
{
  // DMY 위치 외부 루프(control_mode=2)의 책임:
  //   theta error --PID+중력+마찰--> tau_ref [N*m]
  //   tau_ref / reel_radius        --> F_ref [N]
  //   PressureRefGen::step         --> P+ref, P-ref [Pa gauge]
  //   + P_atm, /1000               --> mpc_ref_kpa_ [kPa absolute]
  // 이 함수는 밸브 PWM을 직접 계산하지 않는다. 여기서 생성한 2N개의 압력 목표를
  // 같은 틱 뒤쪽의 채널 PID가 각각 추종한다. 오버슈트를 볼 때는 먼저
  // "목표 힘/압력이 늦게 내려오는가"와 "목표는 내려왔는데 실제 압력만 늦는가"를
  // 나누면 외부 루프와 내부 루프를 구분할 수 있다.
  if (!refgen_) return;
  const int N = num_actuators_;
  const double atm = sensor_.kpa_atm();
  auto to_gauge_pa = [atm](double abs_kpa) { return (abs_kpa - atm) * 1000.0; };
  auto to_abs_kpa  = [atm](double gauge_pa) { return gauge_pa / 1000.0 + atm; };

  // ── 1. 축별 목표 토크 (위치 PID + 중력 FF + 마찰) ───────────────────
  std::array<double, 9> ang;
  {
    std::lock_guard<std::mutex> lk(sensors_mtx_);
    ang = encoder_angles_;
  }
  std::vector<double> F_ref((size_t)N, 0.0), tau_ref((size_t)N, 0.0);
  std::vector<double> dbg_tau_pid((size_t)N, 0.0), dbg_tau_ff((size_t)N, 0.0);
  std::vector<double> dbg_angle((size_t)N, 0.0), dbg_angle_ref((size_t)N, 0.0);
  std::vector<double> dbg_vel((size_t)N, 0.0);

  const double reel_m = reel_radius_mm_ * 1e-3;

  for (int a = 0; a < N; ++a) {
    auto& cfg   = pos_ctrl_cfg_[(size_t)a];
    auto& state = pos_ctrl_state_[(size_t)a];
    auto& tp    = tau_pid_[(size_t)a];

    const int enc = std::clamp(cfg.actuator_idx, 0, (int)ang.size() - 1);
    const double angle = ang[(size_t)enc];
    double angle_ref;
    {
      std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
      // 항상 슬루된 목표를 쓴다. 예전에는 TCP 수신 전에 측정각을 그대로 넣어
      // 오차를 0 으로 두었는데, 그러면 기동 목표(0°)가 적용되지 않았다.
      angle_ref = target_angle_slewed_[(size_t)a];
    }

    if (!state.initialized) {
      state.prev_angle = angle; state.vel_filt = 0.0; state.initialized = true;
      tau_integ_[(size_t)a] = 0.0;
    }
    const double vel_raw = (angle - state.prev_angle) / dt_sec;
    state.vel_filt = cfg.vel_filter_alpha * vel_raw + (1.0 - cfg.vel_filter_alpha) * state.vel_filt;
    state.prev_angle = angle;
    const double vel = state.vel_filt;

    const double err_raw = angle_ref - angle;

    // ── 추종 오차 제한 ────────────────────────────────────────────────
    // **목표 자체는 건드리지 않는다.** 예전에는 target_angle_slewed_ 를 측정각
    // ±밴드로 묶었는데, 그러면 팔이 멈춘 자리에서 목표까지 같이 얼어붙는다 —
    // 80° 에서 0° 를 명령하면 팔이 14° 에 서고 목표는 9° 에서 멈췄다
    // (실기 20260829_193558: angle − target 이 정확히 +5.00 에 붙어 있었다).
    // 목표는 명령까지 끝까지 가야 한다. 묶어야 하는 것은 **제어기에 들어가는
    // 오차** 뿐이다 — 적분 와인드업과 도달 못 할 큰 수요를 막는 것이 목적이니
    // 여기서 자르면 충분하다.
    const double err = (target_follow_band_deg_ > 0.0)
        ? std::clamp(err_raw, -target_follow_band_deg_, target_follow_band_deg_)
        : err_raw;

    // ── 오차가 밴드에 계속 붙어 있으면 알린다 ────────────────────────────
    //
    // 밴드에 붙으면 kp·err 이 상수가 되고, 적분도 integ_limit_nm 에서 멈추고,
    // 마찰항도 포화한다 — **목표 각도가 더 이상 압력 레퍼런스에 실리지 않는다.**
    // 45° 든 90° 든 같은 토크를 요구하게 된다.
    //
    // 액추에이터를 떼고 시험할 때 특히 그렇다: 팔이 안 움직이니 오차가 절대
    // 안 줄고, 0.75 s (= (integ_limit/ki)/band) 만에 전부 포화한다. 실기
    // 20260829_220606 (actuator_connected:=true, 액추에이터 미연결):
    //     45° → τ 1.90 N·m,  90° → τ 1.92 N·m   (구분이 사라졌다)
    // 그때는 **actuator_connected:=false** 로 돌려야 한다 — PID 를 끄고 중력 FF 를
    // 목표각으로 계산하므로 각도 명령이 그대로 압력에 실린다.
    if (actuator_connected_ && target_follow_band_deg_ > 0.0 &&
        std::abs(err_raw) > target_follow_band_deg_ * 1.5) {
      if (++band_sat_ticks_[(size_t)a] > (int)(3.0 / std::max(1e-6, dt_sec))) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "[axis%d] 추종 오차가 3 초 넘게 밴드(%.1f°)에 붙어 있다 — 실제 %.1f°. "
          "이 상태에서는 kp·오차·적분·마찰이 모두 포화해 **목표 각도가 압력 "
          "레퍼런스에 반영되지 않는다** (45° 와 90° 가 같은 토크가 된다). "
          "액추에이터가 안 붙어 있다면 actuator_connected:=false 로 돌릴 것 — "
          "그러면 중력 FF 를 목표각으로 계산해 각도가 그대로 압력에 실린다.",
          a, target_follow_band_deg_, err_raw);
        band_sat_ticks_[(size_t)a] = 0;
      }
    } else {
      band_sat_ticks_[(size_t)a] = 0;
    }

    // 적분 (부호 반전 시 리셋 + 클램프)
    double& I = tau_integ_[(size_t)a];
    if ((err > 0.0 && I < 0.0) || (err < 0.0 && I > 0.0)) I = 0.0;
    if (actuator_connected_) I += err * dt_sec;
    const double I_lim = (std::abs(tp.ki) > 1e-12) ? tp.integ_limit_nm / tp.ki : 0.0;
    I = std::clamp(I, -I_lim, I_lim);

    // ── D 항: 목표 속도 피드포워드 ────────────────────────────────────
    //
    // 예전에는 −kd·vel 이라 **명령한 움직임까지 억제**했다. 목표가 −15 deg/s 로
    // 내려가라고 하는데 팔이 그대로 내려가면 D 가 +0.3 N·m 로 붙잡는다. 그래서
    // 팔이 못 내려가다가, 중력이 이겨 −45 deg/s 로 미끄러지면 D 가 뒤늦게
    // +0.9 로 튀어 붙잡는다 — 계단의 정체다.
    //
    // 실기 20260829_193558 하강(t 8~18): 목표는 15 deg/s 인데 실제 속도가
    // −63.8 ~ +14.1 deg/s 로 요동했고 **15% 는 하강 중에 위로 되튀었다**.
    // 그 순간 τ 는 1.2 → 3.5 N·m 로 튀었는데 챔버는 128 kPa 에서 못 따라와
    // (목표 156) 붙잡기가 늦고 약했다.
    //
    // vel 대신 (vel − 목표속도) 를 쓰면 D 는 **명령에서 벗어난 만큼만** 억제한다.
    // 명령한 속도로 내려가는 동안은 0 이고, −45 로 미끄러지면 그 초과분만 잡는다.
    // kd_vel_ff_ 0 이면 예전 동작, 1 이면 완전 피드포워드.
    const double vel_ref = (a < (int)target_slew_rate_.size() && pos_tcp_received_)
        ? target_slew_rate_[(size_t)a] : 0.0;
    const double vel_err = vel - kd_vel_ff_ * vel_ref;

    const double tau_pid = actuator_connected_
        ? (tp.kp * err + tp.ki * I - tp.kd * vel_err) : 0.0;

    // 중력 피드포워드. tau_ff_gain 으로 크기를 줄일 수 있다.
    //
    // 액추에이터를 떼고 압력만 시험할 때는 실제로 들 하중이 없다. 그런데 기하값
    // (5 kg × 0.15 m)이 그대로 들어가면 45° 에서 5.20 N·m → 208 N 을 요구하고,
    // 그 힘은 양압 정격(185 kPa abs) 단독으로 못 내므로 생성기가 음압까지 30 kPa
    // 까지 끌어내린다. 시험용으로는 과하다.
    // 게인은 **목표 압력에 거의 선형**으로 반영된다 (F = P⁺·A − P⁻·A).
    // 액추에이터를 붙이면 반드시 1.0 으로 되돌릴 것 — 그때는 중력을 실제로 들어야 한다.
    const double ff_angle = actuator_connected_ ? angle : angle_ref;
    const double tau_grav = tp.tau_ff_gain * cfg.mass_kg * 9.81 * cfg.link_length_m
                          * std::sin(ff_angle * M_PI / 180.0);

    // 마찰 보상. **하드 sign 은 err 이 0 을 지날 때마다 ±friction_nm 를 통째로
    // 뒤집는다** — friction_nm 0.48 이면 0.96 N·m 계단이고, 이는 2 kg·150 mm 의
    // 중력 최대치(2.94 N·m)의 33% 다. 목표 근처에서 이게 매 틱 진동한다.
    // 밴드 안에서 선형으로 준다 (밴드 밖에서는 예전과 같은 ±friction_nm).
    const double fb = std::max(1e-6, tp.friction_band_deg);
    const double tau_fric = actuator_connected_
        ? tp.friction_nm * std::clamp(err / fb, -1.0, 1.0) : 0.0;

    // 이 시스템은 한 방향 힘만 낸다 → 목표는 항상 ≥ 0
    tau_ref[(size_t)a] = std::max(0.0, tau_pid + tau_grav + tau_fric);
    F_ref[(size_t)a]   = tau_ref[(size_t)a] / std::max(1e-6, reel_m);

    dbg_tau_pid[(size_t)a] = tau_pid;
    dbg_tau_ff[(size_t)a]  = tau_grav;
    dbg_angle[(size_t)a]   = angle;
    dbg_angle_ref[(size_t)a] = angle_ref;
    dbg_vel[(size_t)a]     = vel;
  }

  // ── 2. 생성기 주기마다 최적화 1회 ───────────────────────────────────
  const int decim = std::max(1, gen_period_ms_ / std::max(1, period_ms_));
  const bool run_gen = (gen_tick_ % (uint64_t)decim == 0) || !gen_has_result_;
  ++gen_tick_;

  PressureRefGen::Result r;
  if (run_gen) {
    // 공급원: boards 1~4 측정값
    PressureRefGen::SupplyState sup;
    sup.P_rail_pos = to_gauge_pa(filt_out_[P_pos_board_id_ - 1]);
    sup.P_rail_neg = to_gauge_pa(filt_out_[P_neg_board_id_ - 1]);
    sup.P_tank     = to_gauge_pa(filt_out_[P_macro_board_id_ - 1]);
    sup.P_ej       = to_gauge_pa(filt_out_[P_macro_neg_board_id_ - 1]);
    sup.use_ej_meas = gen_use_ej_meas_;
    // MacroSwitch(board4 v1) 개방 여부 = 이젝터 구동 중
    sup.ej_running  = (macro_switch_pwm_index_ >= 0 && macro_switch_pwm_index_ < PWM_TOTAL)
                      ? (zoh_[(size_t)macro_switch_pwm_index_] > 0) : false;

    // 축 상태: 챔버 압력 + 부피 + 부피 변화율
    std::vector<PressureRefGen::AxisState> axes((size_t)N);
    for (int a = 0; a < N; ++a) {
      const auto& cfg = pos_ctrl_cfg_[(size_t)a];
      const int pos_bid = cfg.pos_gid + channel_board_offset_;
      const int neg_bid = cfg.neg_gid + channel_board_offset_;
      auto& ax = axes[(size_t)a];
      ax.P_pos = to_gauge_pa(filt_out_[std::clamp(pos_bid - 1, 0, NUM_CAN_BOARDS - 1)]);
      ax.P_neg = to_gauge_pa(filt_out_[std::clamp(neg_bid - 1, 0, NUM_CAN_BOARDS - 1)]);
      ax.V_pos = std::max(1e-9, vol_ml_[(size_t)cfg.pos_gid] * 1e-6);
      ax.V_neg = std::max(1e-9, vol_ml_[(size_t)cfg.neg_gid] * 1e-6);
      // V̇ = A · reel · ω  (신장 방향이 양압 챔버를 키운다)
      const double omega_rad = dbg_vel[(size_t)a] * M_PI / 180.0;
      const double dV = piston_area_mm2_ * reel_radius_mm_ * omega_rad * 1e-9;  // mm³/s → m³/s
      ax.dVdt_pos =  dV;
      ax.dVdt_neg = -dV;
    }

    r = refgen_->step(F_ref, axes, sup);

    // ── 압력 레퍼런스 슬루 제한 ────────────────────────────────────────
    //
    // 모드 1 에는 ref_slew_kpa_per_s 가 있었는데 모드 2 에는 없었다. 생성기 출력이
    // 그대로 MPC 로 갔고, 힘 수요가 조금만 떨리면 레퍼런스가 통째로 튀었다.
    // 실기 20260829_165306: 40 ms 사이에 P⁻ 레퍼런스가 101.3 → 51.3 → 101.0 kPa
    // (≈1250 kPa/s), P⁺ 가 101.5 → 122.1 → 105.4 였다. 챔버가 따라갈 수 없는
    // 명령이라 밸브만 두들기고 액추에이터에는 충격으로 간다.
    //
    // 0 이하면 끔(예전 동작).
    const double gen_dt = std::max(1e-3, gen_period_ms_ / 1000.0);
    const double max_step = gen_ref_slew_kpa_s_ * gen_dt;
    for (int a = 0; a < N; ++a) {
      const double want_p = to_abs_kpa(r.P_pos_ref[(size_t)a]);
      const double want_n = to_abs_kpa(r.P_neg_ref[(size_t)a]);
      if (gen_ref_slew_kpa_s_ > 0.0 && gen_has_result_) {
        gen_pos_ref_kpa_[(size_t)a] +=
            std::clamp(want_p - gen_pos_ref_kpa_[(size_t)a], -max_step, max_step);
        gen_neg_ref_kpa_[(size_t)a] +=
            std::clamp(want_n - gen_neg_ref_kpa_[(size_t)a], -max_step, max_step);
      } else {
        gen_pos_ref_kpa_[(size_t)a] = want_p;
        gen_neg_ref_kpa_[(size_t)a] = want_n;
      }
    }
    // ── 공급이 없어 레퍼런스를 못 만드는 상태를 **직접** 알린다 ──────────
    //
    // 힘은 요구되는데 챔버 상한(ub⁺)이 대기압에 붙어 있으면 레퍼런스가 대기압에서
    // 못 움직인다. 로그에는 "P⁺ 레퍼런스가 101.3 고정" 으로만 보여서 제어 버그처럼
    // 읽힌다 — 실제로는 **레일에 공기가 없다는** 뜻이다.
    // 20260829_222722 에서 탱크가 t=4.84 s 에 585 → 97 kPa 로 빠졌고(그 순간 부스트
    // 0.000 g/s, 이젝터 0.000 g/s — 컨트롤러는 탱크를 안 쓰고 있었다) 그 뒤 모든
    // 실험에서 레퍼런스가 대기압에 붙었다.
    {
      const double atm_g = 0.0;   // gauge 기준 대기압
      double want = 0.0, head = 1e18;
      for (int a = 0; a < N; ++a) {
        want = std::max(want, tau_ref[(size_t)a]);
        head = std::min(head, r.ub_pos[(size_t)a] - atm_g);
      }
      if (want > 0.2 && head < 2000.0) {   // 힘은 요구되는데 상한이 대기압 +2 kPa 미만
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
          "레퍼런스를 못 만든다: 토크 %.2f N·m 가 요구되는데 챔버 상한이 %.1f kPa abs "
          "(대기압) 다. 양압레일 %.1f / 탱크 %.1f kPa abs — **공급에 공기가 없다.** "
          "레퍼런스 생성기는 레일이 못 주는 압력을 목표로 삼지 않는다(정상 동작). "
          "컴프레서·탱크 배관을 확인할 것.",
          want, to_abs_kpa(head + atm_g),
          filt_out_[P_pos_board_id_ - 1], filt_out_[P_macro_board_id_ - 1]);
      }
    }

    gen_rail_pos_sp_kpa_ = to_abs_kpa(r.rail_pos_sp);
    gen_rail_neg_sp_kpa_ = to_abs_kpa(r.rail_neg_sp);
    gen_has_result_ = true;

    // 부족률은 **진단값**이다 (MATLAB 도 usage 를 버린다: `[rail_next, ~] = ...`).
    // macro 개방은 여기서 정하지 않는다 — 내층이 매 틱 유량을 나눠 저절로 결정한다.
    for (int a = 0; a < N; ++a) {
      gen_starve_pos_[(size_t)a] = r.starve_pos[(size_t)a] * 100.0;
      gen_starve_neg_[(size_t)a] = r.starve_neg[(size_t)a] * 100.0;
    }

    // ── 3. 적응 레일 셋포인트를 LinePID 에 넘긴다 ────────────────────
    pid_pos_.ref = gen_rail_pos_sp_kpa_;
    pid_neg_.ref = gen_rail_neg_sp_kpa_;

    // 디버그 토픽: 축마다 10개 + 말미 공용 6개
    if (pub_refgen_dbg_) {
      std_msgs::msg::Float64MultiArray m;
      m.data.reserve((size_t)N * 12 + 6);
      for (int a = 0; a < N; ++a) {
        m.data.push_back(dbg_angle[(size_t)a]);
        m.data.push_back(dbg_angle_ref[(size_t)a]);
        m.data.push_back(tau_ref[(size_t)a]);
        m.data.push_back(r.F_achieved[(size_t)a] * reel_m);      // 달성 토크 [N·m]
        m.data.push_back(gen_pos_ref_kpa_[(size_t)a]);
        m.data.push_back(gen_neg_ref_kpa_[(size_t)a]);
        m.data.push_back(to_abs_kpa(r.ub_pos[(size_t)a]));       // 슬루 상한 P⁺
        m.data.push_back(to_abs_kpa(r.lb_pos[(size_t)a]));
        m.data.push_back(to_abs_kpa(r.lb_neg[(size_t)a]));       // 슬루 하한 P⁻
        m.data.push_back(to_abs_kpa(r.ub_neg[(size_t)a]));
        m.data.push_back(gen_starve_pos_[(size_t)a]);             // 양압 유량 부족률 [%]
        m.data.push_back(gen_starve_neg_[(size_t)a]);             // 음압 유량 부족률 [%]
      }
      m.data.push_back(gen_rail_pos_sp_kpa_);
      m.data.push_back(gen_rail_neg_sp_kpa_);
      m.data.push_back(filt_out_[P_macro_board_id_ - 1]);        // 탱크 압력 [kPa abs]
      m.data.push_back(r.tank_low ? 1.0 : 0.0);
      m.data.push_back(r.m_boost * 1e3);                        // 부스트 [g/s]
      m.data.push_back(r.m_eject * 1e3);                        // 이젝터 [g/s]
      pub_refgen_dbg_->publish(m);
    }

    // ── 공급 부족 경고 ────────────────────────────────────────────────
    //
    // 공급이 모자라면 밸브를 아무리 열어도 챔버가 안 찬다. 그 상태의 로그로
    // 밸브를 피팅하면 실제의 수십분의 1 로 잡혀 컨트롤러가 전 채널을 100% 로
    // 내게 된다 — 실기 20260828_181748 에서 탱크가 98 kPa(정상 ~600)까지
    // 비어 있었는데 " LOW" 가 INFO 줄 끝에 붙어 있을 뿐이라 알아채지 못했다.
    // 터미널에서 바로 보이게 ERROR 로 올린다 (2 초마다 한 번).
    {
      const double tank_abs = filt_out_[P_macro_board_id_ - 1];
      const double rail_abs = filt_out_[P_pos_board_id_ - 1];
      const double rail_neg_abs = filt_out_[P_neg_board_id_ - 1];
      double ref_pos_max = 0.0, ref_neg_min = 1e9;
      for (int a = 0; a < N; ++a) {
        ref_pos_max = std::max(ref_pos_max, gen_pos_ref_kpa_[(size_t)a]);
        ref_neg_min = std::min(ref_neg_min, gen_neg_ref_kpa_[(size_t)a]);
      }
      if (r.tank_low) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
          "[공급 부족] 탱크 %.0f kPa — 운전 하한 %.0f kPa 미만이다. macro 부스트를 쓸 수 없다. "
          "컴프레서/펌프를 확인할 것. 이 상태의 로그는 밸브 피팅에 쓰면 안 된다.",
          tank_abs, get_param_or<double>(this, "PressureRefGen.tank_stop_kpa", 450.0) + 101.325);
      }
      // 레일이 챔버 목표보다 낮으면 micro 로는 채울 방법이 없다.
      // 대기압 근처 목표(=사실상 수요 없음)에서는 알리지 않는다. 기동 직후
      // 챔버·레일이 모두 101 kPa 일 때 "여유 +1 kPa" 라고 뜨는 오탐이 있었다.
      if (ref_pos_max > 101.325 + 5.0 && rail_abs < ref_pos_max + 3.0) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
          "[공급 부족] 양압레일 %.0f kPa 로는 챔버 목표 최대 %.0f kPa 를 채울 수 없다 "
          "(여유 %+.0f kPa). 수요가 macro 로 몰린다. 레일 셋포인트는 %.0f kPa 다. "
          "이 라인 밸브는 배기(레일→대기)라 레일을 올릴 수 없다 — 펌프 능력을 확인할 것.",
          rail_abs, ref_pos_max, rail_abs - ref_pos_max, gen_rail_pos_sp_kpa_);
      }
      // 여유가 **양수면 정상**이다. 문턱을 3 kPa 로 두어 진짜 부족할 때만 알린다.
      // (10 kPa 로 두었더니 여유 8.9 kPa 인 정상 상태에도 "깊지 않다" 고 떠서
      //  오탐이었다.)
      if (ref_neg_min < 101.325 - 5.0 && rail_neg_abs > ref_neg_min - 3.0) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
          "[공급 부족] 음압레일 %.1f kPa 로는 챔버 목표 최저 %.1f kPa 에 못 미친다 "
          "(여유 %+.1f kPa). 레일 셋포인트는 %.1f kPa 다.",
          rail_neg_abs, ref_neg_min, ref_neg_min - rail_neg_abs, gen_rail_neg_sp_kpa_);
      }
    }

    if (tick_ % 250 == 0) {
      RCLCPP_INFO(get_logger(),
        "[RefGen] θ=%.2f→%.2f°  τ=%.2f/%.2f N·m (pid %.2f ff %.2f) | "
        "P⁺=%.1f[%.1f~%.1f] P⁻=%.1f[%.1f~%.1f] | rail SP %.1f/%.1f | "
        "tank %.0f%s starve %.0f/%.0f%% [%s%s] it=%d",
        dbg_angle[0], dbg_angle_ref[0], r.F_achieved[0] * reel_m, tau_ref[0],
        dbg_tau_pid[0], dbg_tau_ff[0],
        gen_pos_ref_kpa_[0], to_abs_kpa(r.lb_pos[0]), to_abs_kpa(r.ub_pos[0]),
        gen_neg_ref_kpa_[0], to_abs_kpa(r.lb_neg[0]), to_abs_kpa(r.ub_neg[0]),
        gen_rail_pos_sp_kpa_, gen_rail_neg_sp_kpa_,
        filt_out_[P_macro_board_id_ - 1], r.tank_low ? " LOW" : "",
        gen_starve_pos_[0], gen_starve_neg_[0],
        r.starve_pos[0] > 0.0 ? "B" : "-",
        r.starve_neg[0] > 0.0 ? "E" : "-",
        r.sqp_iters);
    }
  }

  // ── 4. 결과를 MPC 레퍼런스로 (생성기 주기 사이에는 ZOH) ─────────────
  {
    std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
    for (int a = 0; a < N; ++a) {
      const auto& cfg = pos_ctrl_cfg_[(size_t)a];
      if (cfg.pos_gid >= 0 && cfg.pos_gid < (int)mpc_ref_kpa_.size())
        mpc_ref_kpa_[(size_t)cfg.pos_gid] = gen_pos_ref_kpa_[(size_t)a];
      if (cfg.neg_gid >= 0 && cfg.neg_gid < (int)mpc_ref_kpa_.size())
        mpc_ref_kpa_[(size_t)cfg.neg_gid] = gen_neg_ref_kpa_[(size_t)a];
    }
  }
}

void Controller::inner_loop_1khz(float /*dt_ms*/) {
  std::fill(inner_.begin(), inner_.end(), 0);
}

void Controller::publish_cmds() {
  std_msgs::msg::UInt16MultiArray m;
  m.data.assign(cmds_.begin(), cmds_.end());
  pub_pwm_cmd_->publish(m);
}
