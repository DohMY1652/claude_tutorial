#include "CanBridge.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#define CMD_ID_GRP1 0x100
#define CMD_ID_GRP2 0x101

// Boards 17..25: angle encoders (0~5V=360°, range 1.7~3.16V). No TX, RX = 2 bytes.
// Same signal conditioning as pressure: 1~5V → 3.3V~0V inverted, ADC 12-bit.
// Conversion: V_mv = 5000 - (raw * 4000/4095), angle_deg = V_mv / 5000 * 360
static constexpr int PWM_BOARDS = ANALOG_BOARD_START - 1;  // 16

// raw ADC(0~4095) → orig_mV (반전증폭 역산), can_monitor.py의 calc_original_voltage_mv()와 동일
static double raw_to_orig_mv(double raw_adc) {
  double adc_mv = std::clamp(raw_adc * (3300.0 / 4095.0), 0.0, 3300.0);
  return (4125.0 - adc_mv) / 0.825;
}

// double 파라미터를 선언하되, yaml에 소수점 없이 정수로 적혀 있어도(예: raw_0deg: 1200)
// rclcpp의 엄격한 타입 검사로 노드가 죽지 않도록 int로도 허용해서 double로 변환.
// (ParameterDescriptor::dynamic_typing은 ROS2 Foxy에 없으므로 예외 처리로 대체)
//
// 주의: declare_parameter<double>()이 타입 불일치로 던질 때 rclcpp는 이미 파라미터를
// 등록한 상태다. 따라서 int로 다시 declare하면 ParameterAlreadyDeclared로 노드가 죽는다.
// → 예외를 삼키고 get_parameter()로 실제 타입에 맞춰 읽는다.
static double declare_double_flexible(rclcpp::Node* node, const std::string& name, double default_value) {
  if (!node->has_parameter(name)) {
    try {
      node->declare_parameter<double>(name, default_value);
    } catch (const rclcpp::exceptions::InvalidParameterTypeException&) {
      // yaml 값이 int — 아래 get_parameter에서 정수로 읽는다
    }
  }
  rclcpp::Parameter p;
  if (node->get_parameter(name, p)) {
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)  return p.as_double();
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) return static_cast<double>(p.as_int());
  }
  return default_value;
}

using namespace std::chrono_literals;

CanBridge::CanBridge(const rclcpp::NodeOptions & options)
: Node("can_bridge", options), hnd_(-1), running_(false)
{
  channel_num_   = this->declare_parameter<int>("can_channel",    0);
  current_mode_  = (uint8_t)this->declare_parameter<int>("current_mode", 1);  // 1=Debug
  control_type_  = (uint8_t)this->declare_parameter<int>("control_type", 1);  // 1=PWM

  int num_actuators = this->declare_parameter<int>("num_actuators", 1);
  for (int i = 0; i < num_actuators; ++i)
    active_encoder_boards_.insert(ANALOG_BOARD_START + i);

  double enc_offset_default = declare_double_flexible(this, "encoder_offset", 1740.0);
  double enc_gain_default   = declare_double_flexible(this, "encoder_gain",   105.0 / (3127.0 - 1740.0));
  enc_offset_.fill(enc_offset_default);
  enc_gain_.fill(enc_gain_default);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (int bid = ANALOG_BOARD_START; bid <= NUM_BOARDS; ++bid) {
    const std::string base = "EncoderCalibration.boards." + std::to_string(bid);
    // raw_0deg/raw_90deg 실측값이 있으면 offset/gain을 자동 계산 (can_monitor.py의
    // calib_from_raw_2pt()와 동일 로직). 없으면 offset/gain 직접 override로 폴백.
    double raw_0deg  = declare_double_flexible(this, base + ".raw_0deg",  nan);
    double raw_90deg = declare_double_flexible(this, base + ".raw_90deg", nan);
    if (!std::isnan(raw_0deg) && !std::isnan(raw_90deg)) {
      double orig_mv_0deg  = raw_to_orig_mv(raw_0deg);
      double orig_mv_90deg = raw_to_orig_mv(raw_90deg);
      enc_offset_[bid] = orig_mv_0deg;
      enc_gain_[bid]   = 90.0 / (orig_mv_90deg - orig_mv_0deg);
      enc_measured_[bid] = true;
    } else {
      enc_offset_[bid] = declare_double_flexible(this, base + ".offset", enc_offset_[bid]);
      enc_gain_[bid]   = declare_double_flexible(this, base + ".gain",   enc_gain_[bid]);
      enc_measured_[bid] = false;
    }
  }

  // ── 캘리브레이션 안 된 엔코더 경고 ──────────────────────────────────────
  // **부호가 문제다.** 실측된 board 17·18·19 는 gain 이 모두 **음수**다
  // (−0.0725 / −0.0808 / −0.0886 deg/mV — raw 가 커지면 orig_mV 가 줄어드는 반전증폭).
  // 반면 기본값 `encoder_gain` 은 +0.0757 로 **부호가 반대**다. 즉 실측값이 없는 보드는
  // 각도가 **반대 방향으로** 읽힌다. 위치 제어(control_mode 2)에서 각도 부호가 뒤집히면
  // 오차 부호가 뒤집혀 목표에서 멀어지는 방향으로 구동된다 — 포화까지 간다.
  // 그래서 활성 엔코더 중 미측정 보드가 있으면 강하게 경고한다.
  {
    std::string bad;
    for (int bid : active_encoder_boards_)
      if (bid >= ANALOG_BOARD_START && bid <= NUM_BOARDS && !enc_measured_[bid])
        bad += (bad.empty() ? "" : ", ") + std::to_string(bid);
    if (!bad.empty()) {
      RCLCPP_ERROR(get_logger(),
        "엔코더 캘리브레이션 없음: board %s — 일반 기본값(offset %.1f, gain %+.6f)으로 돈다. "
        "실측된 보드들은 gain 이 모두 **음수**인데 이 기본값은 **양수**다 → 각도가 "
        "반대 방향으로 읽힐 수 있다. 위치 제어 전에 "
        "scripts/encoder_calib.py 로 2점 캘리브레이션할 것 (RUNBOOK.md 0.5절).",
        bad.c_str(), enc_offset_default, enc_gain_default);
    }
    for (int bid : active_encoder_boards_)
      if (bid >= ANALOG_BOARD_START && bid <= NUM_BOARDS && enc_measured_[bid])
        RCLCPP_INFO(get_logger(), "엔코더 board %d: offset %.2f mV, gain %+.6f deg/mV (실측)",
                    bid, enc_offset_[bid], enc_gain_[bid]);
  }

  targets_.resize(NUM_BOARDS + 1);
  // 워치독: 0 이면 끔. 실기에서는 반드시 켤 것.
  node_start_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  wd_timeout_ms_  = this->declare_parameter<int>("pwm_watchdog_ms", 200);
  rx_timeout_ms_  = this->declare_parameter<int>("can_rx_watchdog_ms", 200);
  wd_vent_index_  = this->declare_parameter<int>("pwm_watchdog_vent_index",  0);   // board1 v1
  wd_admit_index_ = this->declare_parameter<int>("pwm_watchdog_admit_index", 3);   // board2 v1

  // **초기값을 안전 상태로 둔다.** targets_ 는 0 으로 시작하는데, 0 은 릴리프 밸브가
  // **닫힌** 상태다 — 컨트롤러가 한 번도 붙지 않은 채 펌프가 돌면 양압 레일이 무한정
  // 오른다 (엔코더 캘리브레이션처럼 브리지만 띄우는 작업에서 실제로 그렇게 된다).
  // 워치독의 안전 상태와 같은 곳에서 시작한다: 채널 폐쇄 + 라인 밸브 전개.
  apply_safe_state();
  RCLCPP_INFO(get_logger(),
    "PWM 워치독: %d ms (0=끔). 시한 초과 시 채널 밸브 폐쇄 + 라인 밸브(idx %d, %d) 전개",
    wd_timeout_ms_, wd_vent_index_, wd_admit_index_);
  sensors_n_  = (size_t)(PWM_BOARDS + 1);
  currents_n_ = (size_t)(PWM_BOARDS + 1) * 3;
  analog_n_   = (size_t)(NUM_BOARDS - ANALOG_BOARD_START + 1);      // 9 values (boards 17..25)
  sensors_snapshot_ = std::make_unique<std::atomic<uint16_t>[]>(sensors_n_);
  current_snapshot_ = std::make_unique<std::atomic<float>[]>(currents_n_);
  analog_snapshot_  = std::make_unique<std::atomic<uint16_t>[]>(analog_n_);
  for (size_t i = 0; i < sensors_n_;  ++i) sensors_snapshot_[i].store(0, std::memory_order_relaxed);
  for (size_t i = 0; i < currents_n_; ++i) current_snapshot_[i].store(0.0f, std::memory_order_relaxed);
  for (size_t i = 0; i < analog_n_;   ++i) analog_snapshot_[i].store(0, std::memory_order_relaxed);
  sensors_filt_.assign(PWM_BOARDS + 1, 0.0);

  // Single flat sensor publisher: board/sensors
  // Index i (0-based) = board_id (i+1)
  pub_sensors_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("board/sensors",  10);
  pub_currents_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("board/currents", 10);
  pub_analog_   = this->create_publisher<std_msgs::msg::Float64MultiArray>("board/analog",    10);
  // 엔코더 **raw ADC**. board/analog 은 캘리브레이션이 적용된 각도라 캘리브레이션 자체에는
  // 쓸 수 없다. can_monitor.py 처럼 CAN 을 직접 읽으면 can_bridge_node 와 핸들이 충돌하므로
  // 여기서 토픽으로 낸다 → scripts/encoder_calib.py 가 이것을 받는다.
  pub_analog_raw_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("board/analog_raw", 10);

  // Single flat PWM subscriber: board/pwm_cmd
  // Index (bid-1)*3 .. (bid-1)*3+2 = board bid v1/v2/v3
  auto qos = rclcpp::QoS(10);
  sub_pwm_cmd_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
    "board/pwm_cmd", qos, std::bind(&CanBridge::on_cmd_pwm, this, std::placeholders::_1));

  try {
    init_can();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(this->get_logger(), "CAN Init Failed: %s", e.what());
    exit(1);
  }

  // TX 는 두 경로로 나간다: on_cmd_pwm(컨트롤러 명령마다, 실측 500 Hz) + 이 타이머.
  // 타이머는 컨트롤러가 조용할 때 보드를 살려 두는 keepalive 이자 워치독 구동원이다.
  // 명령이 오는 동안에는 중복 발행이라 CAN 버스만 더 쓴다 — 실기에서 버스 점유가
  // ID 높은 보드(board14+)의 응답을 밀어낸 전례가 있어(RUNBOOK/HANDOFF) 조정할 수
  // 있게 파라미터로 뺀다. 기본값은 기존 동작 그대로 4 ms 다.
  tx_fallback_ms_     = this->declare_parameter<int>("can_tx_fallback_ms", 4);
  tx_min_interval_ms_ = this->declare_parameter<int>("can_tx_min_interval_ms", 0);
  diag_period_s_      = this->declare_parameter<double>("can_diag_period_s", 5.0);
  const int tx_ms = tx_fallback_ms_;
  tx_timer_     = this->create_wall_timer(std::chrono::milliseconds(std::max(1, tx_ms)),
                                          std::bind(&CanBridge::tx_routine,     this));
  // **이 타이머가 제어 주기다.** 컨트롤러의 제어 틱은 board/sensors 구독 콜백에서
  // 돌기 때문에(Controller.cpp: sub_sensors_ → on_sensor), 여기서 발행하는 주기가
  // 그대로 제어 루프 주기가 된다. 예전에는 2 ms 로 박혀 있어 pp_controller 의
  // period_ms 를 바꿔도 실제 주기는 500 Hz 그대로였다 — period_ms 는 공칭 dt 로만
  // 쓰였다. **둘을 반드시 같은 값으로 맞출 것.**
  sensor_period_ms_ = this->declare_parameter<int>("sensor_period_ms", 2);
  sensor_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(std::max(1, sensor_period_ms_)),
      std::bind(&CanBridge::sensor_routine, this));
  if (diag_period_s_ > 0.0)
    diag_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)std::lround(diag_period_s_ * 1000.0)),
        std::bind(&CanBridge::diag_routine, this));

  running_ = true;
  rx_thread_ = std::thread(&CanBridge::rx_loop, this);

  RCLCPP_INFO(this->get_logger(),
    "=== Kvaser CanBridge Running (Ch %d, 5Mbps) | current_mode=%d control_type=%d ===",
    channel_num_, current_mode_, control_type_);
}

CanBridge::~CanBridge() {
  running_ = false;
  if (rx_thread_.joinable()) rx_thread_.join();
  close_can();
}

// 안전 상태 — 채널 밸브 폐쇄 + 라인 밸브 2개 전개.
// **"전부 0" 은 안전이 아니다**: board1 v1 이 0 이면 릴리프가 닫혀 펌프가 양압 레일을
// 무한정 올린다. 초기화와 워치독 트립이 같은 상태를 쓴다.
// 호출자가 cmd_mtx_ 를 잡고 있거나(워치독) 아직 스레드가 없을 때(생성자) 부른다.
void CanBridge::apply_safe_state() {
  for (int bid = 1; bid <= PWM_BOARDS; ++bid) {
    targets_[bid].v1 = 0; targets_[bid].v2 = 0; targets_[bid].v3 = 0;
  }
  auto open_slot = [&](int flat) {
    if (flat < 0) return;
    const int bid = flat / 3 + 1, v = flat % 3;
    if (bid < 1 || bid > PWM_BOARDS) return;
    const uint16_t FULL = 4095;
    if (v == 0) targets_[bid].v1 = FULL;
    else if (v == 1) targets_[bid].v2 = FULL;
    else targets_[bid].v3 = FULL;
  };
  open_slot(wd_vent_index_);
  open_slot(wd_admit_index_);
}

void CanBridge::init_can() {
  canInitializeLibrary();
  hnd_ = canOpenChannel(channel_num_, canOPEN_CAN_FD);
  if (hnd_ < 0) {
    char err_msg[64]; canGetErrorText((canStatus)hnd_, err_msg, sizeof(err_msg));
    RCLCPP_ERROR(this->get_logger(), "Open Failed: %s (%d)", err_msg, hnd_);
    throw std::runtime_error("Open Failed");
  }

  canStatus stat = canSetBusParams(hnd_, canBITRATE_1M, 0, 0, 0, 0, 0);
  if (stat != canOK) RCLCPP_ERROR(this->get_logger(), "Set Nominal Bitrate Failed");

  stat = canSetBusParamsFd(hnd_, 5000000, 11, 4, 4);
  if (stat != canOK) {
    char err_msg[64]; canGetErrorText(stat, err_msg, sizeof(err_msg));
    RCLCPP_ERROR(this->get_logger(), "Set Data Bitrate Failed: %s", err_msg);
    throw std::runtime_error("Bitrate Set Failed");
  }

  stat = canBusOn(hnd_);
  if (stat != canOK) {
    char err_msg[64]; canGetErrorText(stat, err_msg, sizeof(err_msg));
    throw std::runtime_error("BusOn Failed");
  }
}

void CanBridge::close_can() {
  if (hnd_ >= 0) {
    canBusOff(hnd_);
    canClose(hnd_);
    hnd_ = -1;
  }
}

// Single flat PWM command: data[i*3+0..2] → board (i+1) v1/v2/v3
void CanBridge::on_cmd_pwm(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
  // CAN 수신이 끊긴 동안에는 명령을 **통째로** 무시한다. 컨트롤러는 얼어붙은 센서값을
  // 보고 있으므로 그 명령을 신뢰할 수 없고, 받아서 targets_ 에 넣으면 수신 워치독이
  // 넣어 둔 안전 상태를 매 틱 덮어써 무효가 된다.
  //
  // **last_cmd_ 도 갱신하지 않는다.** 갱신하면 타이머 폴백이 "방금 명령으로 보냈다"고
  // 판단해 조기 반환하고, 그러면 안전 상태가 실제로 보드에 송신되지 않는다.
  // 갱신하지 않으면 PWM 워치독도 함께 걸리는데, 둘 다 같은 안전 상태를 가리키므로 맞다.
  if (rx_stale_.load(std::memory_order_relaxed)) return;
  {
    std::lock_guard<std::mutex> lk(cmd_mtx_);
    last_cmd_ = std::chrono::steady_clock::now();
    cmd_seen_ = true;
    if (wd_tripped_.exchange(false, std::memory_order_relaxed))
      RCLCPP_INFO(get_logger(), "PWM 워치독 복구 — 명령 수신 재개");
    const int n = std::min((int)msg->data.size() / 3, PWM_BOARDS);
    for (int i = 0; i < n; ++i) {
      int bid = i + 1;
      targets_[bid].v1 = msg->data[i*3+0];
      targets_[bid].v2 = msg->data[i*3+1];
      targets_[bid].v3 = msg->data[i*3+2];
    }
  }
  // 명령마다 즉시 송신한다. can_tx_min_interval_ms 로 이 속도를 낮출 수 있다 —
  // 밸브 대역이 wn≈40 rad/s(6.4 Hz)라 500 Hz 송신은 78배 과잉이고, 버스가 빡빡하면
  // 여기를 줄이는 것이 저우선순위 보드를 살리는 가장 큰 수단이다 (500→100 Hz 로
  // 우리 TX 점유율 12.6% → 2.5%).
  if (tx_min_interval_ms_ > 0) {
    const auto now = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last;
    { std::lock_guard<std::mutex> lk(cmd_mtx_); last = last_tx_; }
    if (last.time_since_epoch().count() != 0 &&
        std::chrono::duration<double, std::milli>(now - last).count()
            < (double)tx_min_interval_ms_)
      return;                                   // 다음 타이머 폴백이 보낸다
  }
  tx_send();
}

void CanBridge::diag_routine() {
  std::string alive, dead;
  for (int bid = 1; bid <= NUM_BOARDS; ++bid) {
    const uint32_t now = rx_count_[(size_t)bid].load(std::memory_order_relaxed);
    const uint32_t d = now - rx_count_prev_[(size_t)bid];
    rx_count_prev_[(size_t)bid] = now;
    if (d == 0) {
      if (bid <= PWM_BOARDS || active_encoder_boards_.count(bid))
        dead += (dead.empty() ? "" : ", ") + std::to_string(bid);
    } else {
      char buf[32];
      snprintf(buf, sizeof(buf), "%d:%.0fHz", bid, d / diag_period_s_);
      alive += (alive.empty() ? "" : " ") + std::string(buf);
    }
  }
  RCLCPP_INFO(get_logger(), "CAN 수신 [%s]", alive.c_str());

  // ── 같은 토픽에 다른 퍼블리셔가 있나 ────────────────────────────────
  // virtual.launch.py 의 시뮬레이터(virtual_powerpack)는 `name='can_bridge'` 로
  // **이 노드 이름을 그대로 뺏어 쓴다.** 그래서 시뮬레이터를 안 내리고 실기를 띄우면
  // 두 노드가 같은 토픽에 번갈아 퍼블리시하고, 구독자는 실기값과 시뮬값을 섞어 받는다.
  // ROS 는 이걸 오류로 보지 않는다 — 조용히 절반씩 섞인다. 20260829 에 20 분 동안
  // 이 상태로 실험 4 회를 날렸다: 엔코더가 0°↔105° 로 튀고, 대기압인 board 5 가
  // 305 kPa 로 읽혔다. can_monitor.py 는 CAN 을 직접 읽어 멀쩡했기에 더 헷갈렸다.
  {
    const int n = count_publishers(pub_sensors_->get_topic_name());
    if (n > 1)
      RCLCPP_ERROR(get_logger(),
        "토픽 %s 에 퍼블리셔가 **%d 개**다 — 이 노드 말고 다른 놈이 같은 토픽에 쓰고 있다. "
        "십중팔구 virtual.launch.py 시뮬레이터가 안 죽고 남아 있는 것이다 (그놈도 노드 "
        "이름이 can_bridge 다). 이 상태의 데이터는 실기값과 시뮬값이 **번갈아 섞인** "
        "쓰레기다. `pkill -f virtual_powerpack` 로 내리고 다시 띄울 것.",
        pub_sensors_->get_topic_name(), n);
  }

  if (!dead.empty())
    RCLCPP_ERROR(get_logger(),
      "CAN 수신 없음: board %s — 프레임이 **0** 이다. 0 이면 배선·전원·펌웨어 문제이고, "
      "다른 보드보다 주파수만 낮으면 버스 경합이다 (ID 가 높을수록 우선순위가 낮다: "
      "board N = 0x%03X).", dead.c_str(), 0x120 + 20);
}

void CanBridge::rx_loop() {
  const double TO_MV   = 3300.0 / 4095.0;
  const double LPF_ALPHA = 0.2;

  while (running_ && rclcpp::ok()) {
    long id;
    uint8_t data[64];
    unsigned int dlc, flags;
    unsigned long timestamp;

    canStatus stat = canReadWait(hnd_, &id, data, &dlc, &flags, &timestamp, 100);

    if (stat == canOK && id >= 0x121 && id <= (0x120 + NUM_BOARDS)) {
      int bid = id - 0x120;
      if (bid >= 0 && bid <= NUM_BOARDS)
        rx_count_[(size_t)bid].fetch_add(1, std::memory_order_relaxed);
      last_rx_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count(),
          std::memory_order_relaxed);
      if (rx_stale_.exchange(false, std::memory_order_relaxed))
        RCLCPP_INFO(get_logger(), "CAN 수신 워치독 복구 — 보드 프레임 재개");

      if (bid >= ANALOG_BOARD_START) {
        // Encoder boards (17..25): 8-byte payload, raw[3] (bytes 6-7) = PA7 angle sensor
        if (dlc >= 8 && (active_encoder_boards_.empty() || active_encoder_boards_.count(bid))) {
          uint16_t raw_a;
          memcpy(&raw_a, &data[6], 2);  // bytes 6-7 = raw[3] = PA7
          const size_t ai = (size_t)(bid - ANALOG_BOARD_START);
          if (ai < analog_n_) analog_snapshot_[ai].store(raw_a, std::memory_order_relaxed);
        }
      } else if (dlc >= 8) {
        // MPC/PID boards (1..18): 8-byte payload
        uint16_t raw_i1, raw_i2, raw_i3, raw_p;
        memcpy(&raw_i1, &data[0], 2);
        memcpy(&raw_i2, &data[2], 2);
        memcpy(&raw_i3, &data[4], 2);
        memcpy(&raw_p,  &data[6], 2);


        double p_mv_raw = 5000.0 - ((double)raw_p * 4000.0 / 4095.0);
        if (p_mv_raw < 0) p_mv_raw = 0;
        if (p_mv_raw > 5000) p_mv_raw = 5000;

        // **LPF 상태를 double 로 들고 있어야 한다.** 예전에는 sensors_snapshot_
        // (uint16) 에 곧바로 되먹여, 매 스텝 소수부가 절단됐다. alpha=0.2 이면
        // 증분이 0.2·Δ 라 **Δ가 5 mV 미만이면 증분이 1 미만이라 통째로 사라진다** —
        // 필터가 그 자리에 멈춘다. 계측: 상승 신호에서 정상상태 −4 mV 로 수렴하고
        // (gain 0.250 → **−1.0 kPa 영구 편향**), 하강에서는 0 이라 비대칭이다.
        // 과압 세이프티도 이 값을 보므로 실제보다 1 kPa 늦게 트립한다.
        double& p_state = sensors_filt_[bid];
        p_state = (p_state == 0.0) ? p_mv_raw
                                   : (p_state * (1.0 - LPF_ALPHA) + p_mv_raw * LPF_ALPHA);
        sensors_snapshot_[bid] = (uint16_t)std::lround(p_state);

        double c1_raw = (double)raw_i1 * TO_MV;
        double c2_raw = (double)raw_i2 * TO_MV;
        double c3_raw = (double)raw_i3 * TO_MV;

          const size_t ci = (size_t)bid * 3;
          if (ci + 2 < currents_n_) {
            const double raw3[3] = {c1_raw, c2_raw, c3_raw};
            const float p0 = current_snapshot_[ci + 0].load(std::memory_order_relaxed);
            const float p1 = current_snapshot_[ci + 1].load(std::memory_order_relaxed);
            const float p2 = current_snapshot_[ci + 2].load(std::memory_order_relaxed);
            const bool  first = (p0 == 0.0f && p1 == 0.0f && p2 == 0.0f);
            const float prev[3] = {p0, p1, p2};
            for (int v = 0; v < 3; ++v) {
              const double nv = first ? raw3[v]
                  : (double)prev[v] * (1.0 - LPF_ALPHA) + raw3[v] * LPF_ALPHA;
              current_snapshot_[ci + v].store((float)nv, std::memory_order_relaxed);
            }
          }
      }
    }
  }
}

void CanBridge::sensor_routine() {
  // 락 없이 원소별로 읽는다 (쓰는 쪽은 rx_loop 하나뿐, relaxed 로 충분).
  std::vector<uint16_t> p_raw(sensors_n_);
  std::vector<std::array<double, 3>> c_raw(sensors_n_);
  std::vector<uint16_t> a_raw(analog_n_);
  for (size_t i = 0; i < sensors_n_; ++i)
    p_raw[i] = sensors_snapshot_[i].load(std::memory_order_relaxed);
  for (size_t i = 0; i < sensors_n_; ++i)
    for (size_t v = 0; v < 3; ++v)
      c_raw[i][v] = (double)current_snapshot_[i * 3 + v].load(std::memory_order_relaxed);
  for (size_t i = 0; i < analog_n_; ++i)
    a_raw[i] = analog_snapshot_[i].load(std::memory_order_relaxed);

  // Publish pressure: boards 1..18, index i = board (i+1)
  std_msgs::msg::UInt16MultiArray msg_p;
  msg_p.data.resize(PWM_BOARDS);
  for (int i = 0; i < PWM_BOARDS; ++i)
    msg_p.data[i] = p_raw[i + 1];
  pub_sensors_->publish(msg_p);

  // Publish currents: boards 1..18, index i*3+0..2 = board (i+1)
  std_msgs::msg::Float64MultiArray msg_c;
  msg_c.data.resize(PWM_BOARDS * 3);
  for (int i = 0; i < PWM_BOARDS; ++i) {
    msg_c.data[i*3+0] = c_raw[i+1][0];
    msg_c.data[i*3+1] = c_raw[i+1][1];
    msg_c.data[i*3+2] = c_raw[i+1][2];
  }
  pub_currents_->publish(msg_c);

  // Publish encoder angles [deg]: boards 17..25, raw[3](PA7) → inverting amp recovery → calibration
  // Circuit: 1~5V → 3.3V~0V. orig_mV = (4125 - adc_mv) / 0.825. angle = (orig_mV - offset)*gain
  std_msgs::msg::Float64MultiArray msg_a;
  msg_a.data.resize(a_raw.size(), 0.0);
  for (size_t i = 0; i < a_raw.size(); ++i) {
    int board_id = ANALOG_BOARD_START + (int)i;
    if (!active_encoder_boards_.empty() && !active_encoder_boards_.count(board_id)) continue;
    // raw 0 = 그 보드 프레임을 **한 번도 못 받았다** (배선/전원/펌웨어). 그대로
    // 역산하면 (4125−0)/0.825 = 5000 mV 라 기본 보정에서 246.8° 같은 큰 각도가
    // 나온다. 그 값이 위치 제어로 흘러가면 오차가 통째로 뒤집힌다 — 0° 로 둔다.
    // (프레임이 정말 0 인 보드는 diag_routine 이 매 주기 ERROR 로 따로 짖는다.)
    if (a_raw[i] == 0) { msg_a.data[i] = 0.0; continue; }
    double adc_mv = std::clamp((double)a_raw[i] * (3300.0 / 4095.0), 0.0, 3300.0);
    double orig_mv = (4125.0 - adc_mv) / 0.825;
    msg_a.data[i] = (orig_mv - enc_offset_[board_id]) * enc_gain_[board_id];
  }
  pub_analog_->publish(msg_a);

  // raw ADC 도 함께 낸다 (활성 여부와 무관하게 전부 — 캘리브레이션 대상이 아직
  // 활성화되지 않았을 수 있다).
  std_msgs::msg::UInt16MultiArray msg_ar;
  msg_ar.data.assign(a_raw.begin(), a_raw.end());
  pub_analog_raw_->publish(msg_ar);
}

// TX: boards 1..18 packed into two CAN FD frames. Boards 19..25 are analog-only (no TX).
void CanBridge::tx_routine() {
  // ── PWM 워치독 ───────────────────────────────────────────────────────────
  // targets_ 는 영구 래치다 — pp_controller 가 죽거나 Ctrl-C 되면 마지막 PWM 이 CAN
  // 주기로 계속 나간다. 밸브가 열린 채로 남으면 실기에서 위험하다.
  //
  // 그리고 **"전부 0" 은 안전 상태가 아니다**: board1 v1(양압 릴리프)이 0 이면 릴리프가
  // **닫혀** 펌프가 양압 레일을 무한정 올린다. 펌프 피팅에서 확립한 안전 상태와 같이
  //   · 채널 밸브 → 0 (닫힘, 챔버 압력 동결)
  //   · 라인 밸브 2개 → **전개** (레일을 대기로 되돌린다)
  //   · MacroSwitch → 0 (이젝터 정지)
  // 로 간다.
  // ── CAN 수신 워치독 ─────────────────────────────────────────────────────
  // 수신이 끊기면 압력이 마지막 값에 얼어붙는데, 컨트롤러는 그걸 모르고 계속
  // 밸브를 연다 (과압 세이프티도 같은 얼어붙은 값을 본다). 여기서 끊어 준다.
  if (rx_timeout_ms_ > 0) {
    const long long now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    // last_rx_ns_ 가 0 이면 **아직 한 프레임도 못 받았다**. 예전에는 그 경우를
    // 통째로 건너뛰어(if (last != 0)) 워치독이 꺼져 있었다 — CAN 이 늦게 올라오는
    // 시작 구간이 무방비였다. 20260829_201659 에서 8.5 초 동안 그 상태로 컨트롤러가
    // 밸브를 열었고, 그 지령이 나중에 한꺼번에 나가 액추에이터를 부쉈다.
    // 기준을 노드 기동 시각으로 두면 "처음부터 안 온다" 도 똑같이 잡힌다.
    const long long last_raw = last_rx_ns_.load(std::memory_order_relaxed);
    const long long last = (last_raw != 0) ? last_raw : node_start_ns_;
    {
      const double age_ms = (double)(now_ns - last) / 1e6;
      if (age_ms > (double)rx_timeout_ms_) {
        if (!rx_stale_.exchange(true, std::memory_order_relaxed))
          RCLCPP_ERROR(get_logger(),
            "CAN 수신 워치독: %d ms 동안 보드 프레임이 %s — %s. "
            "채널 밸브 폐쇄 + 라인 밸브 전개. 배선·전원·버스를 확인할 것.",
            rx_timeout_ms_,
            (last_raw == 0) ? "**한 번도** 없었다" : "없다",
            (last_raw == 0) ? "컨트롤러가 센서를 못 읽는 채로 돈다"
                            : "센서값이 얼어붙었다");
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        apply_safe_state();
      }
    }
  }

  if (wd_timeout_ms_ > 0) {
    bool trip = false;
    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      if (cmd_seen_) {
        const double age_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - last_cmd_).count();
        trip = (age_ms > (double)wd_timeout_ms_);
      }
      if (trip) apply_safe_state();
    }
    if (trip && !wd_tripped_.exchange(true, std::memory_order_relaxed)) {
      RCLCPP_ERROR(get_logger(),
        "PWM 워치독: %d ms 동안 board/pwm_cmd 가 없다 — 채널 밸브 폐쇄 + 라인 밸브 전개. "
        "pp_controller 가 죽었는지 확인할 것.", wd_timeout_ms_);
    }
  }

  // 명령이 최근에 왔으면 그 콜백이 이미 보냈다 — 여기서 또 보내면 **순수 중복**이다.
  // 실측 버스 점유율이 79% 라 (보드 25개 × 500 Hz + 우리 TX 750/s) 저우선순위 프레임이
  // 굶는 영역이고, 우리 명령 ID(0x100/0x101)는 최고 우선순위라 그 손해를 전부 보드가 진다.
  // 타이머는 컨트롤러가 조용할 때의 keepalive 로만 남긴다.
  {
    std::lock_guard<std::mutex> lk(cmd_mtx_);
    if (cmd_seen_) {
      const double age_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - last_cmd_).count();
      if (age_ms < (double)tx_fallback_ms_) return;   // 방금 명령으로 보냈다
    }
  }
  tx_send();
}

void CanBridge::tx_send() {
  std::lock_guard<std::mutex> lk(cmd_mtx_);
  last_tx_ = std::chrono::steady_clock::now();

  heartbeat_cnt_++;

  // Group 1: boards 1..10 (64 bytes)
  uint8_t payload_g1[64];
  memset(payload_g1, 0, 64);
  int offset = 0;
  for (int i = 1; i <= 10; ++i) {
    memcpy(&payload_g1[offset], &targets_[i].v1, 2); offset += 2;
    memcpy(&payload_g1[offset], &targets_[i].v2, 2); offset += 2;
    memcpy(&payload_g1[offset], &targets_[i].v3, 2); offset += 2;
  }
  payload_g1[60] = current_mode_;
  payload_g1[61] = control_type_;
  payload_g1[63] = heartbeat_cnt_;

  // ── **묵은 지령을 절대 큐에 남기지 않는다** ───────────────────────────
  //
  // canWrite 는 비동기다 — 드라이버 송신 큐에 넣고 바로 돌아온다. 버스가 죽어
  // 있으면(보드 미기동·전원·비트레이트 불일치) 프레임이 ACK 를 못 받고 큐에
  // 쌓이기만 하다가, 버스가 살아나는 순간 **밀린 것이 순서대로 전부 나간다.**
  // 보드는 그 옛날 지령을 차례로 실행한다.
  //
  // 20260829_201659 에서 실제로 액추에이터가 부서졌다. CAN 이 8.5 초 늦게
  // 올라왔고, 그동안 컨트롤러는 센서를 −167 kPa 로 읽어 "세게 채워라" 를 계속
  // 냈다. 버스가 살아나자 그 backlog 가 쏟아졌다:
  //
  //     t=8.5~10.7  지령 macro 62%  → 실측 전류 0.3 mA   (아무것도 안 나갔다)
  //     t=10.7~11.5 지령 macro  0%  → 실측 전류 155 mA   (= 62%, 2 초 전 지령)
  //
  // 그 사이 과압 세이프티가 소프트웨어에서는 제대로 래치됐는데(배기 100%),
  // 그 지령도 같은 큐 뒤에 서 있어서 0.8 초 늦게 도착했다. 탱크 580 kPa 가
  // 챔버로 쏟아져 228 kPa 까지 올랐고 팔이 스토퍼를 넘어 129°/136°/276° 로 갔다.
  //
  // 이 스트림은 **주기적 setpoint** 다. 늦게 도착한 setpoint 는 쓸모가 없는
  // 정도가 아니라 위험하다. 매번 큐를 비우고 최신 것만 실어 보낸다.
  // **대책 — 버스를 건드리지 않고 backlog 자체를 못 만들게 한다.**
  //
  // 처음에는 매 주기 canIOCTL_FLUSH_TX_BUFFER 로 큐를 비웠다. **그게 버스를
  // 죽였다.** 500 Hz 로 전송 중인 64 바이트 FD 프레임을 중간에 잘라내면 에러
  // 프레임이 쏟아진다. 계측(브리지 단독, 같은 배선):
  //     FLUSH 있음 → RX 31 Hz → 다음 창에서 완전 사망
  //     FLUSH 없음 → RX 265~372 Hz, 안정
  // canWrite 는 계속 canOK 를 반환했다 — 드라이버는 받아들이는데 버스가 죽는다.
  //
  // 그래서 큐를 비우는 대신 **버스가 안 살아 있으면 아예 넣지 않는다.**
  // backlog 는 버스가 죽은 동안 계속 밀어 넣어서 생긴다. 수신이 끊긴 상태
  // (rx_stale_) 에서 송신을 멈추면 밀리는 양이 워치독 시한(기본 200 ms)으로
  // 묶인다 — 사고 때의 8.5 초가 아니라.
  //
  // 큐 잔량도 같이 본다 (읽기 전용이라 버스에 영향이 없다). 쌓이기 시작하면
  // 그것만으로 이미 지령이 늦고 있다는 뜻이라 소리를 낸다.
  if (rx_stale_.load(std::memory_order_relaxed)) {
    if (!tx_paused_.exchange(true, std::memory_order_relaxed))
      RCLCPP_ERROR(get_logger(),
        "CAN 수신이 끊겼다 — **밸브 지령 송신을 멈춘다.** 버스가 죽은 채로 계속 "
        "밀어 넣으면 드라이버 큐에 쌓였다가 버스가 살아나는 순간 묵은 지령이 "
        "한꺼번에 쏟아진다 (20260829 액추에이터 파손 경로). 수신이 복구되면 "
        "자동으로 재개한다.");
    return;
  }
  if (tx_paused_.exchange(false, std::memory_order_relaxed))
    RCLCPP_INFO(get_logger(), "CAN 수신 복구 — 밸브 지령 송신 재개.");

  {
    unsigned int lvl = 0;
    // 임계 100 프레임. 500 Hz × 2 프레임 = 1000 프레임/s 이므로 100 ≈ **100 ms** 다.
    // 정상 버퍼링은 9 프레임(≈9 ms) 수준이라 8 로 두면 매번 헛짖는다.
    // 사고 때 문제가 된 backlog 는 2 초(≈2000 프레임)였다.
    if (canIoCtl(hnd_, canIOCTL_GET_TX_BUFFER_LEVEL, &lvl, sizeof(lvl)) == canOK && lvl > 100)
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 500,
        "CAN 송신 큐가 %u 프레임(≈%.0f ms) 밀렸다 — 지령이 그만큼 늦게 도착한다. "
        "버스 부하·비트레이트를 확인할 것.", lvl, lvl / 1.0);
  }

  tx_check(canWrite(hnd_, CMD_ID_GRP1, payload_g1, 64,
                    canMSG_STD | canFDMSG_FDF | canFDMSG_BRS), 1);

  // Group 2: boards 11..17 (7 boards × 6 bytes = 42 bytes PWM + meta → 48 bytes)
  // Layout matches can_control.py: mode@42, type@43, heartbeat@47
  uint8_t payload_g2[48];
  memset(payload_g2, 0, 48);
  offset = 0;
  for (int i = 11; i <= 17; ++i) {
    memcpy(&payload_g2[offset], &targets_[i].v1, 2); offset += 2;
    memcpy(&payload_g2[offset], &targets_[i].v2, 2); offset += 2;
    memcpy(&payload_g2[offset], &targets_[i].v3, 2); offset += 2;
  }
  payload_g2[42] = current_mode_;
  payload_g2[43] = control_type_;
  payload_g2[47] = heartbeat_cnt_;
  tx_check(canWrite(hnd_, CMD_ID_GRP2, payload_g2, 48,
                    canMSG_STD | canFDMSG_FDF | canFDMSG_BRS), 2);
}

// canWrite 의 반환값을 **반드시** 본다. 예전에는 버렸다 — 버스가 죽어 큐가
// 넘치는 동안(canERR_TXBUFOFL) 아무 신호도 없었고, 조용히 backlog 를 쌓았다.
void CanBridge::tx_check(canStatus st, int grp) {
  if (st == canOK) { tx_err_streak_ = 0; return; }
  ++tx_err_streak_;
  RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 500,
    "CAN 송신 실패 (그룹 %d, status %d, 연속 %d 회) — 밸브 지령이 보드에 "
    "도달하지 않는다. 버스·전원·비트레이트를 확인할 것. 큐를 비우므로 묵은 "
    "지령이 나중에 쏟아지지는 않는다.", grp, (int)st, tx_err_streak_);
}
