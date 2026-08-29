#ifndef CAN_BRIDGE_HPP_
#define CAN_BRIDGE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <vector>
#include <array>
#include <set>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>
#include <canlib.h>

// CAN FD constants missing from older canlib headers
#ifndef canOPEN_CAN_FD
#define canOPEN_CAN_FD   0x0400
#endif
#ifndef canFDMSG_FDF
#define canFDMSG_FDF     0x010000
#endif
#ifndef canFDMSG_BRS
#define canFDMSG_BRS     0x020000
#endif

// canSetBusParamsFd declaration missing from older canlib headers
#ifdef __cplusplus
extern "C" {
#endif
canStatus canSetBusParamsFd(canHandle hnd, long freq_brs,
                             unsigned int tseg1_brs, unsigned int tseg2_brs,
                             unsigned int sjw_brs);
#ifdef __cplusplus
}
#endif

#define NUM_BOARDS        25   // physical CAN boards (board_id 1..25)
#define ANALOG_BOARD_START 17  // boards 17..25: angle encoder (0~5V=360°, range 1.7~3.16V)

class CanBridge : public rclcpp::Node {
public:
  explicit CanBridge(const rclcpp::NodeOptions & options);
  virtual ~CanBridge();

private:
  canHandle hnd_;
  int channel_num_;

  // === Command (TX) ===
  struct BoardCmd {
    uint16_t v1{0};
    uint16_t v2{0};
    uint16_t v3{0};
  };
  std::vector<BoardCmd> targets_;   // targets_[bid], bid = 1..NUM_BOARDS
  std::mutex cmd_mtx_;
  // PWM 워치독 — targets_ 가 영구 래치라 컨트롤러가 죽으면 마지막 명령이 계속 나간다.
  std::chrono::steady_clock::time_point last_cmd_{};
  bool cmd_seen_{false};
  // wd_tripped_ 는 tx_routine(타이머 스레드)이 쓰고 on_cmd_pwm(콜백)이 지운다 — atomic.
  std::atomic<bool> wd_tripped_{false};
  int  wd_timeout_ms_{200}, wd_vent_index_{0}, wd_admit_index_{3};
  void apply_safe_state();     // 채널 폐쇄 + 라인 밸브 전개 (초기화·워치독 공용)

  // === CAN 수신 워치독 ===
  // PWM 워치독은 "컨트롤러가 죽었다"를 막지만 그 반대는 못 막는다: CAN 수신이
  // 끊기면 sensors_snapshot_ 이 마지막 값에 얼어붙고 sensor_routine 은 그 값을
  // 2 ms 마다 계속 발행한다. 컨트롤러는 살아 있으므로 PWM 워치독도 안 걸리고,
  // **얼어붙은 압력을 보며 밸브를 계속 연다.** 과압 세이프티도 같은 값을 보므로
  // 트립하지 않는다. 그래서 수신 쪽에도 워치독이 필요하다.
  std::atomic<long long> last_rx_ns_{0};      // steady_clock, 마지막 유효 프레임
  std::atomic<bool> rx_stale_{false};
  int rx_timeout_ms_{200};

  // === 보드별 수신 진단 ===
  // "보드가 안 붙는다"의 원인을 가르는 유일한 방법이다. 프레임이 **0** 이면 배선·전원·
  // 펌웨어 문제이고, 프레임은 오는데 기대 주파수보다 낮으면 버스 경합이다
  // (CAN 은 ID 가 낮을수록 우선이라 board 20~22 = 0x134~0x136 이 가장 먼저 굶는다).
  std::array<std::atomic<uint32_t>, NUM_BOARDS + 1> rx_count_{};
  std::array<uint32_t, NUM_BOARDS + 1> rx_count_prev_{};
  rclcpp::TimerBase::SharedPtr diag_timer_;
  double diag_period_s_{5.0};
  void diag_routine();

  // TX 중복 억제 — 타이머는 명령이 없을 때만 보내는 폴백이다.
  int tx_fallback_ms_{4};
  int tx_min_interval_ms_{0};                 // 0 = 명령마다 송신
  std::chrono::steady_clock::time_point last_tx_{};
  void tx_send();                             // 실제 CAN 기록 (cmd_mtx_ 를 잡는다)
  void tx_check(canStatus st, int grp);       // canWrite 반환값 확인 (조용한 실패 금지)
  int  tx_err_streak_{0};
  std::atomic<bool> tx_paused_{false};        // 수신 두절 중에는 송신을 멈춘다
  long long node_start_ns_{0};                // RX 워치독 기준 (한 번도 못 받은 경우)

  uint8_t current_mode_{0};
  uint8_t control_type_{0};
  uint8_t heartbeat_cnt_{0};

  // === Sensor (RX) ===
  std::vector<uint16_t> sensors_snapshot_;                  // [bid], bid = 0..PWM_BOARDS (발행용)
  std::vector<double>   sensors_filt_;                      // [bid] LPF 상태 (절단 방지)
  std::vector<std::array<double, 3>> current_snapshot_;     // [bid][0..2], bid 0..18
  std::vector<uint16_t> analog_snapshot_;                   // [0..8] → board 17..25
  std::set<int> active_encoder_boards_;                     // board IDs to read (empty = all)
  // 보드별 엔코더 캘리브레이션 (index = board_id, [0]은 미사용). 기본값은 encoder_offset/encoder_gain,
  // EncoderCalibration.boards.<id> 로 보드별 override 가능.
  std::array<double, NUM_BOARDS + 1> enc_offset_{};   // orig_mV at 0 degrees
  std::array<double, NUM_BOARDS + 1> enc_gain_{};
  // 실측 2점(raw_0deg/raw_90deg)으로 캘리브레이션됐는지 — 기동 경고에 쓴다
  std::array<bool, NUM_BOARDS + 1> enc_measured_{};     // deg/mV
  std::mutex sensor_mtx_;

  // board/sensors  : boards 1..18 pressure (18 values, index 0 = board 1)
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_sensors_;
  // board/currents : boards 1..18 currents (18*3 values)
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_currents_;
  // board/analog   : boards 17..25 encoder angle [deg] (9 values, index 0 = board 17)
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_analog_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_analog_raw_;
  // board/pwm_cmd  : boards 1..18 PWM (18*3 values, index (bid-1)*3 = board bid)
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_pwm_cmd_;

  rclcpp::TimerBase::SharedPtr tx_timer_;
  rclcpp::TimerBase::SharedPtr sensor_timer_;
  std::thread rx_thread_;
  std::atomic<bool> running_;

  void init_can();
  void close_can();
  void rx_loop();
  void tx_routine();
  void sensor_routine();
  void on_cmd_pwm(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
};

#endif
