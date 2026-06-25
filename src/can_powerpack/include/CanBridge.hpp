#ifndef CAN_BRIDGE_HPP_
#define CAN_BRIDGE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <vector>
#include <array>
#include <mutex>
#include <thread>
#include <atomic>
#include <canlib.h>

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

  uint8_t current_mode_{0};
  uint8_t control_type_{0};
  uint8_t heartbeat_cnt_{0};

  // === Sensor (RX) ===
  std::vector<uint16_t> sensors_snapshot_;                  // [bid], bid = 0..18
  std::vector<std::array<double, 3>> current_snapshot_;     // [bid][0..2], bid 0..18
  std::vector<uint16_t> analog_snapshot_;                   // [0..6] → board 19..25
  std::mutex sensor_mtx_;

  // board/sensors  : boards 1..18 pressure (18 values, index 0 = board 1)
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_sensors_;
  // board/currents : boards 1..18 currents (18*3 values)
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_currents_;
  // board/analog   : boards 17..25 encoder angle [deg] (9 values, index 0 = board 17)
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_analog_;
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
