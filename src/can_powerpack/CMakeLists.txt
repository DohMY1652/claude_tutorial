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
# 5. 설치 설정
# =========================================

# (1) C++ 노드들 설치
install(TARGETS
  pp_controller
  pneumatic_sim
  DESTINATION lib/${PROJECT_NAME}
)

if(CANLIB_LIBRARY)
  install(TARGETS can_bridge_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# (2) Python 스크립트 설치
install(PROGRAMS
  scripts/dashboard.py
  scripts/can_monitor.py
  scripts/can_control.py
  scripts/can_test.py
  scripts/log_topics.py
  scripts/self_test_ref_server.py
  scripts/diagnostic_check.py
  scripts/live_control.py
  scripts/pp_monitor.py
  scripts/pp_logger.py
  scripts/position_ref_client.py
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
