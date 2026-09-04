cmake_minimum_required(VERSION 3.8)
project(can_powerpack)

# =========================================
# 1. 패키지 및 라이브러리 찾기
# =========================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(qpoases_vendor REQUIRED)

# [Kvaser CANlib 찾기]
# 시스템에 설치된 libcanlib.so 파일을 찾습니다.
find_library(CANLIB_LIBRARY canlib)

include_directories(include)

# =========================================
# 2. Controller 노드 빌드 설정
# =========================================
add_executable(pp_controller
  src/Controller.cpp
  src/PressureRefGen.cpp
  src/Mppi.cpp
  src/MppiSystem.cpp
  src/controller_main.cpp
)

# ROS 관련 의존성
ament_target_dependencies(pp_controller
  rclcpp
  std_msgs
  std_srvs
  rcl_interfaces
  qpoases_vendor
)

# Eigen3와 qpOASES 라이브러리 링크 (qpoases_vendor_LIBRARIES = installed .a path)
target_link_libraries(pp_controller
  Eigen3::Eigen
  ${qpoases_vendor_LIBRARIES}
)

# 실시간 제어용 최적화 (Debug 빌드 제외)
if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(pp_controller PRIVATE -O3 -march=native)
endif()

# =========================================
# 3. CanBridge 노드 빌드 설정
# =========================================
if(CANLIB_LIBRARY)
  add_executable(can_bridge_node
    src/CanBridge.cpp
    src/can_bridge_main.cpp
  )

  ament_target_dependencies(can_bridge_node
    rclcpp
    std_msgs
    rcl_interfaces
  )

  target_link_libraries(can_bridge_node ${CANLIB_LIBRARY})
else()
  message(WARNING "Kvaser CANlib not found — can_bridge_node will not be built.")
endif()

# =========================================
# 4. Pneumatic Simulator 빌드 설정
# =========================================
add_executable(pneumatic_sim
  src/PneumaticSim.cpp
  src/sim_main.cpp
)

ament_target_dependencies(pneumatic_sim
  rclcpp
  std_msgs
  rcl_interfaces
)

if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(pneumatic_sim PRIVATE -O3 -march=native)
endif()

# =========================================
# 4-1. PressureRefGen 단독 검증 실행파일
# =========================================
add_executable(pressure_ref_test
  src/PressureRefGen.cpp
  src/pressure_ref_test.cpp
)

ament_target_dependencies(pressure_ref_test
  rclcpp
  std_msgs
  std_srvs
  rcl_interfaces
  qpoases_vendor
)

target_link_libraries(pressure_ref_test
  Eigen3::Eigen
  ${qpoases_vendor_LIBRARIES}
)

# =========================================
# 4-2. Virtual Powerpack (CanBridge 대체 가상 하드웨어)
# =========================================
add_executable(virtual_powerpack
  src/VirtualPowerpack.cpp
  src/virtual_powerpack_main.cpp
)

ament_target_dependencies(virtual_powerpack
  rclcpp
  std_msgs
  rcl_interfaces
)

if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(virtual_powerpack PRIVATE -O3 -march=native)
endif()

# =========================================
# 5. 설치 설정
# =========================================

# (1) C++ 노드들 설치
install(TARGETS
  pp_controller
  pneumatic_sim
  virtual_powerpack
  pressure_ref_test
  DESTINATION lib/${PROJECT_NAME}
)

if(CANLIB_LIBRARY)
  install(TARGETS can_bridge_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Teensy 프레임 해독기 검증 — 실기 캡처(test/data/teensy_5s.bin)로 돌린다.
# ROS·CAN 에 의존하지 않으므로 하드웨어 없이 언제든 돌릴 수 있다.
#   ./build/can_powerpack/test_teensy_frame src/can_powerpack/test/data/teensy_5s.bin
add_executable(test_teensy_frame test/test_teensy_frame.cpp)
target_include_directories(test_teensy_frame PRIVATE include)

# (2) Python 스크립트 설치
install(PROGRAMS
  scripts/dashboard.py
  scripts/ctrl_eval.py
  scripts/preflight.sh
  scripts/encoder_calib.py
  scripts/can_monitor.py
  scripts/teensy_monitor.py
  scripts/can_control.py
  scripts/can_control_auto.py
  scripts/crack_from_log.py
  scripts/pp_check.py
  scripts/can_test.py
  scripts/log_topics.py
  scripts/self_test_ref_server.py
  scripts/diagnostic_check.py
  scripts/live_control.py
  scripts/pp_monitor.py
  scripts/pp_logger.py
  scripts/pp_results.py
  scripts/valve_fit_from_log.py
  scripts/chamber_volume_from_log.py
  scripts/position_ref_client.py
  scripts/valve_fit_model.py
  scripts/valve_fit_record.py
  scripts/valve_fit_solve.py
  scripts/valve_fit_selftest.py
  scripts/pump_fit_model.py
  scripts/pump_fit_record.py
  scripts/pump_fit_solve.py
  scripts/pump_fit_selftest.py
  scripts/run_powerpack.sh
  DESTINATION lib/${PROJECT_NAME}
)

# (3) 헤더 파일, 런치 파일, 설정 파일 설치
install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()
