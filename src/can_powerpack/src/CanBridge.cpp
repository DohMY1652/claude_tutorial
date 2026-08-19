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
    } else {
      enc_offset_[bid] = declare_double_flexible(this, base + ".offset", enc_offset_[bid]);
      enc_gain_[bid]   = declare_double_flexible(this, base + ".gain",   enc_gain_[bid]);
    }
  }

  targets_.resize(NUM_BOARDS + 1);
  sensors_snapshot_.assign(PWM_BOARDS + 1, 0);
  current_snapshot_.resize(PWM_BOARDS + 1, {0.0, 0.0, 0.0});
  analog_snapshot_.assign(NUM_BOARDS - ANALOG_BOARD_START + 1, 0);  // 9 values (boards 17..25)

  // Single flat sensor publisher: board/sensors
  // Index i (0-based) = board_id (i+1)
  pub_sensors_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("board/sensors",  10);
  pub_currents_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("board/currents", 10);
  pub_analog_   = this->create_publisher<std_msgs::msg::Float64MultiArray>("board/analog",    10);

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

  tx_timer_     = this->create_wall_timer(4ms, std::bind(&CanBridge::tx_routine,     this));
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

        double p_prev = (double)sensors_snapshot_[bid];
        double p_filtered = (p_prev == 0.0) ? p_mv_raw
                          : (p_prev * (1.0 - LPF_ALPHA) + p_mv_raw * LPF_ALPHA);
        sensors_snapshot_[bid] = (uint16_t)p_filtered;

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
}

// TX: boards 1..18 packed into two CAN FD frames. Boards 19..25 are analog-only (no TX).
void CanBridge::tx_routine() {
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
