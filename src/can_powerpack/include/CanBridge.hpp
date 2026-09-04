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
  bool can_required_{true};   // false = CAN 없이도 뜬다 (벤치 점검 전용)
  bool can_ok_{false};        // init_can 이 성공했나

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
  // 진단 주파수는 **실제 경과 시간**으로 나눈다 (공칭으로 나누면 타이머
  // 지연이 1 % 하향 편차가 되어 200 Hz 가 198 Hz 로 보인다).
  std::chrono::steady_clock::time_point diag_last_tp_{};

  // ── 수신율을 토픽으로도 낸다 ───────────────────────────────────────────
  // 보드별 수신율은 **브리지만 안다** (CAN 프레임을 직접 세는 유일한 곳).
  // 모니터가 CAN 핸들을 따로 열면 호스트 부하가 늘고 실수로 송신할 위험도 생기니,
  // 여기서 1 Hz 로 발행해 pp_monitor·pp_check 가 받아 쓰게 한다.
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
  // ── 수신 스냅샷은 **락 없이** 쓴다 ─────────────────────────────────────
  //
  // 예전에는 이 셋을 sensor_mtx_ 로 감쌌다. 그런데 쓰는 쪽(rx_loop)은 **초당
  // 11000 프레임**이고 읽는 쪽(sensor_routine)은 500 Hz 로 벡터 3 개를 통째로
  // 복사하며 같은 락을 잡았다. 그 경합에 수신이 밀려 드라이버 큐가 넘쳤고,
  // 보드가 ID 순서로 순환 송신하므로 **뒤처진 수신기가 매 주기 꼬리(높은 ID)를
  // 놓쳤다** — 보드 16~22 만 1~3 Hz 로 굶던 것이 이것이다.
  //   계측 20260902: 같은 송신율에서 파이썬 시험 6284 f/s(균일 286 Hz) 대
  //   C++ 브리지 2300 f/s(1~213 Hz). 파이썬보다 2.7 배 못 받고 있었다.
  //
  // 쓰는 쪽은 rx_loop 하나뿐이고 읽는 쪽은 값을 한 번에 하나씩만 보면 되므로
  // 원소별 원자 접근(relaxed)으로 충분하다. 원소 간 시점이 살짝 어긋날 수 있는데,
  // 어차피 보드마다 도착 시각이 다르므로 락이 있어도 같은 성질이다.
  std::unique_ptr<std::atomic<uint16_t>[]> sensors_snapshot_;   // [bid] 0..PWM_BOARDS
  std::unique_ptr<std::atomic<float>[]>    current_snapshot_;   // [bid*3 + v]
  std::unique_ptr<std::atomic<uint16_t>[]> analog_snapshot_;    // [0..8] → board 17..25
  size_t sensors_n_{0}, currents_n_{0}, analog_n_{0};
  int sensor_period_ms_{2};   // board/sensors 발행 주기 = 제어 루프 주기
  // LPF 상태 — rx_loop 만 만진다 (공유 아님).
  std::vector<double>   sensors_filt_;
  std::set<int> active_encoder_boards_;                     // board IDs to read (비면 CAN 엔코더 없음)
  // "teensy" (기본) 또는 "can". board/analog 를 누가 채우는지 정한다.
  // "teensy" 면 CAN 보드 17~25 는 아예 없는 것으로 취급한다 — 파싱도 진단도 하지 않는다.
  std::string encoder_source_{"teensy"};
  bool enc_from_can_{false};

  // ══════════════════════════════════════════════════════════════════════
  //  Teensy 엔코더 (USB CDC 시리얼)
  // ══════════════════════════════════════════════════════════════════════
  // 20260903: 엔코더 6채널을 CAN 에서 떼어 Teensy 4.0 + ADS1115×3 으로 옮겼다.
  //
  // **왜 옮겼나.** CAN 실측에서 지령 프레임 1개당 보드 프레임 약 9.5개가 사라진다
  // (보드가 수신 처리로 ~1.2 ms 멈춘다). 비용은 바이트가 아니라 **프레임당**이라
  // 페이로드를 줄여도 안 듣는다. 그런데 엔코더 보드는 CAN ID 가 가장 높아
  // (0x131~0x136) 우선순위가 최하위였고, 지령을 100 Hz 로 보내는 순간 1 Hz 까지
  // 굶었다. 위치 제어가 눈을 잃는 것이라 가장 위험한 실패였다.
  //
  // Teensy 는 USB 로 200 Hz 를 확정 공급한다. CAN 경합과 무관해졌다.
  //
  // **프레임 포맷** (24 B, scripts/teensy_monitor.py 와 동일, 실기 검증 완료):
  //   [0..1]   SYNC 0xAA 0x55
  //   [2..3]   seq      uint16   (유실 검출)
  //   [4..7]   t_us     uint32   (Teensy micros())
  //   [8..19]  ch0~ch5  int16×6  (ADS1115 raw 카운트)
  //   [20..21] status   uint16   (비트 c = 칩 c I2C 오류)
  //   [22..23] CRC16-CCITT (poly 0x1021, init 0xFFFF, 바이트 0..21)
  //
  // **스트리밍은 자동으로 시작되지 않는다** — 'r' 을 보내야 하고 'x' 로 멈춘다.
  // 이걸 빠뜨리면 포트는 열리는데 한 바이트도 안 온다 (실기에서 확인함).
  //
  // 주의: scripts/arduino/ads1115_6ch/ads1115_6ch.ino 는 **오래된 소스다.**
  // 그 파일은 ASCII CSV 를 뱉는다고 돼 있지만 실기는 위 바이너리를 보낸다.
  // 실기 펌웨어 소스를 확보하기 전까지 .ino 를 신뢰하지 말 것.
  // 프레임 해독은 include/TeensyFrame.hpp 에 있다 (ROS·CAN 비의존 → 단위 테스트 가능).
  // test/test_teensy_frame.cpp 가 실기 캡처로 그 코드를 그대로 돌린다.
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
  // **필요 없다** — 단순 선형이다:  deg = (raw - raw_0deg) * 90 / (raw_90deg - raw_0deg)
  std::array<double, TEENSY_NCH> tenc_raw0_{};
  std::array<double, TEENSY_NCH> tenc_scale_{};   // deg per raw count
  std::array<bool,   TEENSY_NCH> tenc_measured_{};

  // ── 엔코더 두절 시 페일세이프 ────────────────────────────────────────
  // 각도가 없으면 위치 제어는 눈을 잃는다. 그 상태로 유지하면 컨트롤러는
  // **얼어붙은 각도를 현재 자세로 믿고** 계속 밀거나 당긴다. 그래서:
  //   1) 정해진 각도를 대신 발행해 컨트롤러가 스스로 감압하게 만들고
  //      (컨트롤러가 ref_slew 로 천천히 줄인다 — 밸브를 직접 때리는 것보다 부드럽다)
  //   2) teensy_failsafe_hold_ms 뒤에 하드 안전상태(채널 폐쇄 + 레일 배기)를 걸고
  //   3) 노드를 내린다.
  //
  // **각도 선택**. 요구 토크는
  //     tau_ref = max(0, kp·err + I + fric·sgn(err) + m·g·L·sin(angle))
  //     err     = clamp(angle_ref − angle, ±target_follow_band_deg)   (Controller.cpp:3403)
  //     tau_grav 는 **실측각**을 쓴다 (Controller.cpp:3495, actuator_connected 일 때)
  // 이라 두 가지가 동시에 걸린다:
  //   · 각도를 올리면 err 이 음수가 되어 PID 가 압력을 뺀다 — 다만 err 이 밴드(±5°)로
  //     잘리므로 빼는 힘은 **kp·5 + 적분한계 + 마찰 = 1.62 N·m 에서 포화**한다.
  //   · 동시에 tau_grav = 2.94·sin(angle) 이 커진다. 90° 에서 최대다.
  // 그래서 각도별 tau_ref 는 (목표 무관, 각도가 목표+5° 를 넘은 뒤):
  //     0°→1.62   60°→0.93   90°→1.32   120°→0.93   150°→0.00   180°→0.00
  //   (0° 는 err 이 **양수**가 되어 오히려 밀어 올린다 — 직관과 반대다)
  // 기계적 최대 120° 는 0.93 N·m 가 남고, **150° 이상이면 sin 이 충분히 작아져
  // 모든 목표에서 정확히 0** 이 된다. 그래서 기본값은 180° 다 — 기계 범위 밖의
  // 값이라 로그에서 "이건 실측이 아니다" 가 바로 보이는 것도 이점이다.
  //
  // 참고: actuator_connected:=false 면 tau_pid·tau_fric 가 0 이고 중력 FF 가
  // **목표각**을 쓰므로, 이 가짜 각도는 아무 효과가 없다. 그때는 2 단계(하드
  // 안전상태)만이 실제 보호다 — 그래서 hold_ms 를 짧게 두는 편이 낫다.
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

  // board/sensors  : boards 1..18 pressure (18 values, index 0 = board 1)
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_sensors_;
  // board/currents : boards 1..18 currents (18*3 values)
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_currents_;
  // board/analog     : 엔코더 각도 [deg]. encoder_source 에 따라 길이가 다르다 —
  //                    teensy: 6 개 (index = Teensy 채널 = axis 의 actuator_idx)
  //                    can   : 9 개 (index 0 = board 17)
  //                    Controller 는 std::min(msg.size(), 9) 로 받으므로 둘 다 안전하다.
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_analog_;
  // board/analog_raw : 위와 같은 순서의 **보정 전 raw**. 2점 보정을 잡을 때 쓴다.
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
