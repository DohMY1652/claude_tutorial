# 명령 모음

## 빌드

빌드
```bash
cd ~/claude_tutorial && colcon build --packages-select can_powerpack --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**config 만 바꿨을 때** — colcon 이 설치를 건너뛰는 일이 있다. 반드시 확인할 것
```bash
cd ~/claude_tutorial && grep -n "바꾼항목" install/can_powerpack/share/can_powerpack/config/powerpack_config.yaml
```

안 바뀌었으면 강제로
```bash
cd ~/claude_tutorial && touch src/can_powerpack/CMakeLists.txt && colcon build --packages-select can_powerpack --cmake-args -DCMAKE_BUILD_TYPE=Release
```

환경 설정 (새 터미널마다)
```bash
cd ~/claude_tutorial && source install/setup.bash
```

Teensy 프레임 파서 테스트 (하드웨어 없이)
```bash
cd ~/claude_tutorial && ./build/can_powerpack/test_teensy_frame src/can_powerpack/test/data/teensy_5s.bin
```

---

## 점검 (아무것도 안 띄운 상태)

통합 모니터 — CAN + Teensy 한 화면
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/pp_check.py
```

통합 모니터, 20초만
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/pp_check.py --seconds 20
```

통합 모니터, CAN 안 열고 엔코더만
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/pp_check.py --no-can
```

통합 모니터, Teensy 포트 직접 지정
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/pp_check.py --port /dev/ttyACM0
```

엔코더 2점 보정 잡기 (0°에서 `0`, 90°에서 `9`, 출력 `p`, 종료 `q`)
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/pp_check.py --calib
```

CAN만 보기
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/can_monitor.py
```

엔코더만 보기
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/teensy_monitor.py
```

엔코더 CSV 로깅
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/teensy_monitor.py --csv enc.csv
```

엔코더 그래프
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/teensy_monitor.py --plot
```

---

## 통신 부하 시험 (제어기 끈 상태)

200 Hz로 랜덤 제어 명령 보내기
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/can_control_auto.py --hz 200 --pattern random
```

200 Hz로 값 0 고정 송신 (밸브 안 움직임)
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/can_control_auto.py --hz 200 --pattern zero --seconds 20
```

macro 배관 물려 있을 때 (v2 = macro를 0에 묶음)
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/can_control_auto.py --hz 200 --pattern random --no-macro
```

값 상한 걸고 송신
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/can_control_auto.py --hz 200 --pattern random --max 1500
```

송신 주파수 쓸어보기
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/can_control_auto.py --sweep 5,25,50,100,200 --pattern random --per-board
```

pp_check로 송신 부하 주며 확인
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/pp_check.py --tx 200 --seconds 20
```

밸브 수동 조작
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/can_control.py
```

---

## 제어기 실행

제어기 실행 (액추에이터 없이 = 압력 제어)
```bash
cd ~/claude_tutorial && source install/setup.bash && ros2 launch can_powerpack control.launch.py actuator_connected:=false
```

제어기 실행 (액추에이터 연결)
```bash
cd ~/claude_tutorial && source install/setup.bash && ros2 launch can_powerpack control.launch.py actuator_connected:=true
```

특정 축만
```bash
cd ~/claude_tutorial && source install/setup.bash && ros2 launch can_powerpack control.launch.py axis:=0,1 actuator_connected:=true
```

파라미터 덮어쓰기
```bash
cd ~/claude_tutorial && source install/setup.bash && ros2 launch can_powerpack control.launch.py overrides:="MPC_parameters.mppi_lambda=0.2"
```

시뮬레이터
```bash
cd ~/claude_tutorial && source install/setup.bash && ros2 launch can_powerpack virtual.launch.py
```

브리지만 (CAN 없이 Teensy만 점검)
```bash
cd ~/claude_tutorial && source install/setup.bash && ros2 run can_powerpack can_bridge_node --ros-args --params-file install/can_powerpack/share/can_powerpack/config/powerpack_config.yaml -r __ns:=/pack2 -p can_required:=false
```

---

## 제어기 켠 상태에서 확인

통합 모니터 (자동으로 ROS 모드)
```bash
cd ~/claude_tutorial && source install/setup.bash && python3 src/can_powerpack/scripts/pp_check.py
```

제어 루프 주기
```bash
cd ~/claude_tutorial && source install/setup.bash && ros2 topic hz /pack2/board/sensors
```

엔코더 주기
```bash
cd ~/claude_tutorial && source install/setup.bash && ros2 topic hz /pack2/board/analog
```

지령 송신 주기
```bash
cd ~/claude_tutorial && source install/setup.bash && ros2 topic hz /pack2/board/pwm_cmd
```

보드별·엔코더 수신율
```bash
cd ~/claude_tutorial && source install/setup.bash && ros2 topic echo /pack2/board/rx_hz
```

엔코더 raw 값
```bash
cd ~/claude_tutorial && source install/setup.bash && ros2 topic echo /pack2/board/analog_raw --once
```

엔코더 각도
```bash
cd ~/claude_tutorial && source install/setup.bash && ros2 topic echo /pack2/board/analog --once
```

압력 (kPa)
```bash
cd ~/claude_tutorial && source install/setup.bash && ros2 topic echo /pack2/controller/sensors_kpa --once
```

진단 화면
```bash
cd ~/claude_tutorial && source install/setup.bash && python3 src/can_powerpack/scripts/diagnostic_check.py
```

---

## 목표 각도 주기

TCP 서버 모드
```bash
cd ~/claude_tutorial && source install/setup.bash && python3 src/can_powerpack/scripts/position_ref_client.py
```

한 번만 보내기 (축 순서대로)
```bash
cd ~/claude_tutorial && source install/setup.bash && python3 src/can_powerpack/scripts/position_ref_client.py --once 30 30 30 30 30 30
```

축 수 직접 지정해서 한 번 보내기
```bash
cd ~/claude_tutorial && source install/setup.bash && python3 src/can_powerpack/scripts/position_ref_client.py --axes 6 --once 30 30 30 30 30 30
```

전 축 0°
```bash
cd ~/claude_tutorial && source install/setup.bash && python3 src/can_powerpack/scripts/position_ref_client.py --once 0 0 0 0 0 0
```

압력 레퍼런스 직접 주기
```bash
cd ~/claude_tutorial && source install/setup.bash && python3 src/can_powerpack/scripts/pressure_ref_client.py
```

---

## 로깅·결과

로깅 (`~/result/<타임스탬프>/` 에 저장)
```bash
cd ~/claude_tutorial && source install/setup.bash && python3 src/can_powerpack/scripts/pp_logger.py
```

실행 목록
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/pp_results.py list
```

한 실행 상세
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/pp_results.py show <타임스탬프>
```

인덱스 갱신
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/pp_results.py index
```

빈 실행 정리
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/pp_results.py prune --apply
```

제어 성능 평가
```bash
cd ~/claude_tutorial && source install/setup.bash && python3 src/can_powerpack/scripts/ctrl_eval.py
```

---

## 종료·정리

제어기 종료
```bash
# 런치 터미널에서 Ctrl-C
```

강제 종료
```bash
pkill -f "ros2 launch"; pkill -f can_bridge_node; pkill -f pp_controller; pkill -f pp_logger; pkill -f pp_monitor
```

남은 프로세스 확인
```bash
ps -eo pid,cmd | grep -E "can_bridge_node|pp_controller|pp_monitor|pp_logger" | grep -v grep
```

---

## 하드웨어 확인

Kvaser 연결 확인
```bash
lsusb | grep -i kvaser
```

Teensy 연결 확인
```bash
ls -l /dev/ttyACM* && lsusb | grep 16c0
```

Teensy 고정 경로
```bash
ls -l /dev/serial/by-id/
```

---

## 챔버 부피 곡선 측정 (밸브 닫고 손으로 각도 움직이기)

기록 + 피팅 (1축)
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/volume_probe.py --axes 0
```

전 축
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/volume_probe.py
```

기록해 둔 CSV 만 다시 피팅
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/volume_probe.py --fit ~/result/volprobe_<타임스탬프>.csv
```

라인 릴리프도 닫기 (**펌프 반드시 끌 것**)
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/scripts/volume_probe.py --axes 0 --hold-rail-closed
```

---

## 보정

압력 센서 대기압 재영점 — 현재 raw 읽기
```bash
cd ~/claude_tutorial && source install/setup.bash && ros2 topic echo /pack2/controller/sensors_kpa --once
```

엔코더 보정값 확인
```bash
cd ~/claude_tutorial && grep -A8 "TeensyEncoder:" src/can_powerpack/config/powerpack_config.yaml
```

압력 보정값 확인
```bash
cd ~/claude_tutorial && grep -A20 "Sensor_calibration:" src/can_powerpack/config/powerpack_config.yaml
```

설정 편집
```bash
cd ~/claude_tutorial && nano src/can_powerpack/config/powerpack_config.yaml
```

---

## 실기 없이 시험

가상 Teensy 띄우기
```bash
cd ~/claude_tutorial && python3 src/can_powerpack/test/fake_teensy.py /tmp/tlink
```

가상 Teensy에 브리지 물리기
```bash
cd ~/claude_tutorial && source install/setup.bash && ros2 run can_powerpack can_bridge_node --ros-args --params-file install/can_powerpack/share/can_powerpack/config/powerpack_config.yaml -r __ns:=/pack2 -p teensy_port:=/tmp/tlink -p can_required:=false
```
