#ifndef CAN_BRIDGE_HPP_
#define CAN_BRIDGE_HPP_

// DMY 읽기 안내
// 이 클래스의 책임은 ROS 배열 메시지와 물리 CAN 프레임 사이의 변환이다.
// 제어 계산은 Controller에 있고, 여기에는 RX 스레드, 주기적 센서 publish,
// PWM 명령 래치/TX, 엔코더 보정, 통신 watchdog이 있다. 공유 자료는 RX용
// sensor_mtx_와 TX용 cmd_mtx_로 나뉜다. 파손 방지 관점에서는 생성자 초기 상태,
// apply_safe_state(), on_cmd_pwm(), tx_routine() 순서로 읽는 것이 가장 중요하다.

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
#include "TeensyFrame.hpp"

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
  // CAN 을 못 열었을 때 죽을지. 기본 true(= 종전 동작).
  // false 는 **벤치 점검 전용**이다 — Teensy 엔코더만 확인할 때 쓴다.
  // 이때 압력·전류는 아예 발행하지 않는다 (아래 can_ok_ 주석 참조).
  bool can_required_{true};
  bool can_ok_{false};

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
  // ── 수신율을 토픽으로도 낸다 ───────────────────────────────────────────
  // 보드별 수신율은 **브리지만 안다** (CAN 프레임을 직접 세는 유일한 곳).
  // 모니터가 CAN 핸들을 따로 열면 호스트 부하가 늘고 실수로 송신할 위험도 생기니,
  // 여기서 1 Hz 로 발행해 pp_monitor 가 받아 쓰게 한다.
  //   board/rx_hz : [보드1..보드16 Hz, Teensy Hz]  (길이 17)
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_rx_hz_;
  rclcpp::TimerBase::SharedPtr rate_timer_;
  std::array<uint32_t, NUM_BOARDS + 1> rate_prev_{};
  uint32_t rate_teensy_prev_{0};
  std::chrono::steady_clock::time_point rate_last_tp_{};
  void rate_routine();
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
  // **이 값이 제어 루프 주기다.** 컨트롤러의 제어 틱은 board/sensors 구독 콜백에서
  // 돌기 때문에(Controller.cpp: sub_sensors_ → on_sensor), 여기서 발행하는 주기가
  // 그대로 제어 주기가 된다. pp_controller 의 period_ms 는 공칭 dt·데시메이션에만
  // 쓰이므로 **둘을 반드시 같은 값으로 맞출 것.**
  int sensor_period_ms_{5};
  std::vector<std::array<double, 3>> current_snapshot_;     // [bid][0..2], bid 0..18
  std::vector<uint16_t> analog_snapshot_;                   // [0..8] → board 17..25
  std::set<int> active_encoder_boards_;                     // board IDs to read (비면 CAN 엔코더 없음)

  // ══════════════════════════════════════════════════════════════════════
  //  엔코더 소스 — CAN 보드 17~25 또는 Teensy USB CDC
  // ══════════════════════════════════════════════════════════════════════
  // "teensy"(기본) 또는 "can". board/analog 를 누가 채우는지 정한다.
  // "teensy" 면 CAN 보드 17~25 는 아예 없는 것으로 취급한다 — 파싱도 진단도 안 한다.
  // 토픽 이름·타입은 두 경우가 같으므로 **구독자는 전원 무수정**이다
  // (Controller, pp_logger, pp_monitor, encoder_calib ...).
  std::string encoder_source_{"teensy"};
  bool enc_from_can_{false};

  // ── Teensy 엔코더 (USB CDC 시리얼) ───────────────────────────────────
  // 20260903 에 엔코더 6채널을 CAN 에서 떼어 Teensy 4.0 + ADS1115×3 으로 옮겼다.
  //
  // **왜 옮겼나.** CAN 실측에서 지령 프레임 1개당 보드 프레임 약 9.5개가 사라진다
  // (보드가 수신 처리로 ~1.2 ms 멈춘다). 비용이 바이트가 아니라 **프레임당**이라
  // 페이로드를 줄여도 안 듣는다. 그런데 엔코더 보드는 CAN ID 가 가장 높아
  // (0x131~0x136) 우선순위 최하위였고, 지령을 100 Hz 로 보내는 순간 **1 Hz 까지
  // 굶었다.** 위치 제어가 눈을 잃는 것이라 가장 위험한 실패였다.
  // Teensy 는 USB 로 확정 공급하므로 CAN 경합과 무관해진다.
  //
  // 프레임 포맷(24 B)과 해독은 include/TeensyFrame.hpp 에 있다 — ROS·CAN 에
  // 의존하지 않아 test/test_teensy_frame.cpp 가 실기 캡처로 그대로 검증한다.
  //
  // **스트리밍은 자동으로 시작되지 않는다** — 'r' 을 보내야 하고 'x' 로 멈춘다.
  // 빠뜨리면 포트는 열리는데 한 바이트도 안 온다 (실기에서 확인된 동작이다).
  static constexpr int TEENSY_FRAME_LEN = teensy::FRAME_LEN;
  static constexpr int TEENSY_NCH       = teensy::NCH;

  bool        teensy_enable_{true};
  std::string teensy_port_;                 // 비면 자동탐색
  std::string teensy_port_used_;            // 실제로 연 포트 (진단 출력용)
  std::string teensy_open_err_;             // 마지막 열기 실패 사유
  int         teensy_watchdog_ms_{100};
  int         teensy_fd_{-1};
  std::thread teensy_thread_;

  // raw 스냅샷 — teensy_thread_ 만 쓰고 sensor_routine 만 읽는다 (relaxed 로 충분).
  std::array<std::atomic<int32_t>, TEENSY_NCH> teensy_raw_{};
  std::atomic<bool>      teensy_seen_{false};     // 한 번이라도 유효 프레임을 받았나
  std::atomic<bool>      teensy_stale_{true};
  std::atomic<long long> teensy_last_ns_{0};
  std::atomic<uint32_t>  teensy_frames_{0};       // 누적 유효 프레임
  std::atomic<uint32_t>  teensy_lost_{0};         // seq 불연속으로 센 유실
  std::atomic<uint32_t>  teensy_crc_err_{0};
  std::atomic<uint32_t>  teensy_status_{0};       // 마지막 status 워드
  uint32_t               teensy_frames_prev_{0};  // diag_routine 전용

  // 2점 보정. ADS1115 는 센서를 직접 읽으므로 CAN 엔코더의 반전앰프 역산이
  // **필요 없다** — 단순 선형이다: deg = (raw − raw_0deg) × 90 / (raw_90deg − raw_0deg)
  std::array<double, TEENSY_NCH> tenc_raw0_{};
  std::array<double, TEENSY_NCH> tenc_scale_{};   // deg per raw count
  std::array<bool,   TEENSY_NCH> tenc_measured_{};

  // ── 엔코더 두절 페일세이프 (3 단계) ──────────────────────────────────
  // 각도가 없으면 위치 제어는 눈을 잃는다. 그 상태로 발행을 **끊으면** 컨트롤러의
  // encoder_angles_ 가 마지막 값에 얼어붙고, 컨트롤러는 그걸 현재 자세로 믿은 채
  // 계속 민다. 그래서 끊는 대신 정해진 각도로 **갈아끼워** 스스로 감압하게 한다.
  //   1) 두절 즉시     board/analog 을 teensy_failsafe_angle_deg 로 고정 발행
  //   2) +hold_ms      하드 안전상태(채널 폐쇄 + 레일 배기), 이후 지령 무시
  //   3) +vent_ms      노드 종료 — 보드에 남는 마지막 지령이 안전상태다
  //
  // **각도 선택 근거.** tau_ref = max(0, kp·err + I + 마찰 + m·g·L·sin(angle)),
  // err = clamp(angle_ref − angle, ±5°) 라 두 힘이 동시에 걸린다. 각도를 올리면
  // PID 는 압력을 빼지만 err 이 밴드에 잘려 1.62 N·m 에서 포화하고, 동시에 중력
  // FF 가 **실측각**을 쓰므로 sin(angle) 이 커진다. 각도별 tau_ref:
  //   0°→1.62(오히려 밀어 올린다)  60°→0.93  90°→1.32  120°→0.93  150°→0  180°→0
  // 150° 이상이라야 모든 목표에서 정확히 0 이다. 기본 180° 는 기계 범위(120°) 밖이라
  // 로그에서 "이건 실측이 아니다" 가 바로 보이는 이점도 있다.
  double teensy_failsafe_angle_deg_{180.0};
  int    teensy_failsafe_hold_ms_{3000};
  int    teensy_failsafe_vent_ms_{1500};
  bool   teensy_failsafe_shutdown_{true};
  std::atomic<bool>      failsafe_latched_{false};   // on_cmd_pwm 을 막는다
  std::atomic<long long> failsafe_since_ns_{0};
  rclcpp::TimerBase::SharedPtr failsafe_timer_;
  void failsafe_tick();

  void teensy_loop();
  bool teensy_open();
  void teensy_close();
  std::string teensy_find_port() const;
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
  // board/analog   : 엔코더 각도 [deg]. encoder_source 에 따라 길이가 다르다 —
  //                  teensy: 6 개 (index = Teensy 채널 = axis 의 actuator_idx)
  //                  can   : 9 개 (index 0 = board 17)
  //                  Controller 는 std::min(msg.size(), 9) 로 받으므로 둘 다 안전하다.
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
