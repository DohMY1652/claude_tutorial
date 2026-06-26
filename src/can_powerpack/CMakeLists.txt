cmake_minimum_required(VERSION 3.8)
project(can_powerpack)

# =========================================
# 1. 패키지 및 라이브러리 찾기
# =========================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
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
  src/controller_main.cpp
)

# ROS 관련 의존성
ament_target_dependencies(pp_controller
  rclcpp
  std_msgs
  rcl_interfaces
  qpoases_vendor
)

# Eigen3와 qpOASES 라이브러리 링크
target_link_libraries(pp_controller
  Eigen3::Eigen
  qpOASES
)

# 실시간 제어용 최적화 (Debug 빌드 제외)
if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(pp_controller PRIVATE -O3 -march=native)
endif()

# =========================================
# 3. CanBridge 노드 빌드 설정
# =========================================
add_executable(can_bridge_node
  src/CanBridge.cpp
  src/can_bridge_main.cpp
)

ament_target_dependencies(can_bridge_node
  rclcpp
  std_msgs
  rcl_interfaces
)

# [CANlib 링크 설정]
if(CANLIB_LIBRARY)
  # find_library로 찾은 경우 해당 경로로 링크
  target_link_libraries(can_bridge_node ${CANLIB_LIBRARY})
else()
  # 못 찾았을 경우, 시스템 표준 경로의 canlib를 강제로 링크
  target_link_libraries(can_bridge_node canlib)
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
# 5. 설치 설정
# =========================================

# (1) C++ 노드들 설치
install(TARGETS
  pp_controller
  can_bridge_node
  pneumatic_sim
  DESTINATION lib/${PROJECT_NAME}
)

# (2) Python 스크립트 설치
install(PROGRAMS
  scripts/dashboard.py
  scripts/can_monitor.py
  scripts/can_control.py
  scripts/log_topics.py
  scripts/self_test_ref_server.py
  scripts/diagnostic_check.py
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
