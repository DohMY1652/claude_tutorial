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
  sensors_snapshot_.assign(PWM_BOARDS + 1, 0);
  sensors_filt_.assign(PWM_BOARDS + 1, 0.0);
  current_snapshot_.resize(PWM_BOARDS + 1, {0.0, 0.0, 0.0});
  analog_snapshot_.assign(NUM_BOARDS - ANALOG_BOARD_START + 1, 0);  // 9 values (boards 17..25)

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
  const int tx_ms = this->declare_parameter<int>("can_tx_period_ms", 4);
  tx_timer_     = this->create_wall_timer(std::chrono::milliseconds(std::max(1, tx_ms)),
                                          std::bind(&CanBridge::tx_routine,     this));
  sensor_timer_ = this->create_wall_timer(2ms, std::bind(&CanBridge::sensor_routine, this));

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
  {
    std::lock_guard<std::mutex> lk(cmd_mtx_);
    last_cmd_ = std::chrono::steady_clock::now();
    cmd_seen_ = true;
    if (wd_tripped_) {
      wd_tripped_ = false;
      RCLCPP_INFO(get_logger(), "PWM 워치독 복구 — 명령 수신 재개");
    }
    // CAN 수신이 끊긴 동안에는 컨트롤러 명령을 받지 않는다 — 받으면 위에서 넣은
    // 안전 상태를 매 틱 덮어써 무효가 된다. 컨트롤러는 얼어붙은 센서값을 보고 있으므로
    // 그 명령을 신뢰할 수 없다.
    if (rx_stale_.load(std::memory_order_relaxed)) return;
    const int n = std::min((int)msg->data.size() / 3, PWM_BOARDS);
    for (int i = 0; i < n; ++i) {
      int bid = i + 1;
      targets_[bid].v1 = msg->data[i*3+0];
      targets_[bid].v2 = msg->data[i*3+1];
      targets_[bid].v3 = msg->data[i*3+2];
    }
  }
  tx_routine();
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
          std::lock_guard<std::mutex> lk(sensor_mtx_);
          analog_snapshot_[bid - ANALOG_BOARD_START] = raw_a;
        }
      } else if (dlc >= 8) {
        // MPC/PID boards (1..18): 8-byte payload
        uint16_t raw_i1, raw_i2, raw_i3, raw_p;
        memcpy(&raw_i1, &data[0], 2);
        memcpy(&raw_i2, &data[2], 2);
        memcpy(&raw_i3, &data[4], 2);
        memcpy(&raw_p,  &data[6], 2);

        std::lock_guard<std::mutex> lk(sensor_mtx_);

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

        auto& cs = current_snapshot_[bid];
        if (cs[0] == 0.0 && cs[1] == 0.0 && cs[2] == 0.0) {
          cs = {c1_raw, c2_raw, c3_raw};
        } else {
          cs[0] = cs[0] * (1.0 - LPF_ALPHA) + c1_raw * LPF_ALPHA;
          cs[1] = cs[1] * (1.0 - LPF_ALPHA) + c2_raw * LPF_ALPHA;
          cs[2] = cs[2] * (1.0 - LPF_ALPHA) + c3_raw * LPF_ALPHA;
        }
      }
    }
  }
}

void CanBridge::sensor_routine() {
  std::vector<uint16_t> p_raw;
  std::vector<std::array<double, 3>> c_raw;
  std::vector<uint16_t> a_raw;
  {
    std::lock_guard<std::mutex> lk(sensor_mtx_);
    p_raw = sensors_snapshot_;
    c_raw = current_snapshot_;
    a_raw = analog_snapshot_;
  }

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
    const long long last = last_rx_ns_.load(std::memory_order_relaxed);
    if (last != 0) {
      const double age_ms = (double)(now_ns - last) / 1e6;
      if (age_ms > (double)rx_timeout_ms_) {
        if (!rx_stale_.exchange(true, std::memory_order_relaxed))
          RCLCPP_ERROR(get_logger(),
            "CAN 수신 워치독: %d ms 동안 보드 프레임이 없다 — 센서값이 얼어붙었다. "
            "채널 밸브 폐쇄 + 라인 밸브 전개. 배선·전원·버스를 확인할 것.",
            rx_timeout_ms_);
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
    if (trip && !wd_tripped_) {
      wd_tripped_ = true;
      RCLCPP_ERROR(get_logger(),
        "PWM 워치독: %d ms 동안 board/pwm_cmd 가 없다 — 채널 밸브 폐쇄 + 라인 밸브 전개. "
        "pp_controller 가 죽었는지 확인할 것.", wd_timeout_ms_);
    }
  }

  std::lock_guard<std::mutex> lk(cmd_mtx_);

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
  canWrite(hnd_, CMD_ID_GRP1, payload_g1, 64, canMSG_STD | canFDMSG_FDF | canFDMSG_BRS);

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
  canWrite(hnd_, CMD_ID_GRP2, payload_g2, 48, canMSG_STD | canFDMSG_FDF | canFDMSG_BRS);
}
