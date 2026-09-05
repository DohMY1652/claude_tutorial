# codex-mppi 1축 실기 시험

이 브랜치는 오버슛/한계진동 원인을 한 번에 큰 gain으로 덮지 않고 다음 순서로 분리했다.

- 압력 PI `u_trim` 중복 적용 제거
- 목표각을 속도/가속도/jerk 제한 quintic S-curve로 생성
- `J * theta_ref_ddot` 관성 feedforward 추가
- 최종 post-slew 압력 지령으로 적용 토크를 다시 계산하는 outer anti-windup 추가
- 5 ms 실제 제어 주기와 MPPI rollout을 `8 x 5 ms = 40 ms`로 일치
- 목표 근처는 저대역 PD, 큰 오차에서만 제한된 P를 보태는 gain scheduling
- 첫 실기에서는 `ki=0`; 안정성 확인 뒤 작은 I를 단계적으로 추가

## 0. 안전 전제

기계식 스토퍼와 독립 릴리프를 준비하고, 첫 시험은 반드시 `axis:=0` 한 축만 한다.
엔코더 방향, 0점, 양/음압 채널 매핑이 맞지 않으면 시작하지 않는다.

```bash
cd ~/claude_tutorial
git switch codex-mppi
src/can_powerpack/scripts/preflight.sh
colcon build --packages-select can_powerpack --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 1. 하드웨어 없이 확인

터미널 A:

```bash
cd ~/claude_tutorial && source install/setup.bash
ros2 launch can_powerpack virtual.launch.py control_mode:=2 solver:=mppi axis:=0 actuator_connected:=true
```

터미널 B:

```bash
cd ~/claude_tutorial && source install/setup.bash
python3 src/can_powerpack/scripts/position_ref_client.py --axes 1 --once 10
python3 src/can_powerpack/scripts/position_ref_client.py --axes 1 --once 20
python3 src/can_powerpack/scripts/position_ref_client.py --axes 1 --once 10
```

기동 로그에서 아래 값이 보여야 한다.

```text
Control mode: 2 (POSITION_REFGEN)
NP=8, Ts=5.0 ms (지평 40 ms)
S-curve 25.0 deg/s, 60.0 deg/s², 300.0 deg/s³
외층 wc≈1.20 rad/s (0.19 Hz), PM≈44°
```

## 2. 실기 1축, I=0

먼저 별도 터미널에서 모니터를 확인한다.

```bash
cd ~/claude_tutorial && source install/setup.bash
python3 src/can_powerpack/scripts/pp_check.py
```

제어기:

```bash
cd ~/claude_tutorial && source install/setup.bash
ros2 launch can_powerpack control.launch.py control_mode:=2 solver:=mppi axis:=0 actuator_connected:=true
```

목표는 작은 범위부터 한 번씩 보낸다.

```bash
python3 src/can_powerpack/scripts/position_ref_client.py --axes 1 --once 5
python3 src/can_powerpack/scripts/position_ref_client.py --axes 1 --once 10
python3 src/can_powerpack/scripts/position_ref_client.py --axes 1 --once 5
```

다음 조건이면 즉시 중지한다.

- 각도 또는 압력 진폭이 연속해서 증가
- 유지 중 반대 밸브가 번갈아 crack point를 넘음
- startup 로그에 캐스케이드/PM 오류가 표시됨
- pressure reference와 실제 압력 차이가 8 kPa 이상인 상태가 지속
- over-pressure latch 또는 runaway protection 동작

## 3. I를 넣기 전 판정

같은 스텝을 최소 3회 반복해 진동이 없을 때만 진행한다. I=0에서 1~2° 정지 오차는
정지마찰을 보수적으로 처리한 의도된 결과다. 이 단계에서 P/D를 먼저 올리지 않는다.

정상상태 오차가 반복 가능하고 같은 부호일 때만 axis0의 시작값을 다음처럼 바꾼다.

```yaml
TorquePID:
  axis0:
    ki: 0.0001
    integ_limit_nm: 0.10
```

`ki=0.0001`에서 startup 추정 PM은 약 42°다. 이후에는 한 번에 2배 이상 올리지 않고,
`controller/tau_debug`의 마지막 필드를 bitmask로 본다.

- bit 1: pressure inner loop 지연으로 적분 정지
- bit 2: error band 포화로 적분 정지
- bit 4: post-slew 적용 토크 기반 back-calculation 동작

## 4. 가상 시험 기준 결과

2026-09-04 단일 축 MPPI 가상 시험에서 다음을 확인했다.

| 명령 | 최대/최소 각도 | 마지막 5초 중앙값 | 마지막 5초 p-p |
|---|---:|---:|---:|
| 0 -> 30 deg | max 31.32 deg | 31.25 deg | 0.00 deg |
| 30 -> 10 deg | min 10.23 deg | 10.23 deg | 0.00 deg |

가상 하네스는 실기 밸브/마찰과 동일하지 않으므로 이 수치는 합격 보장이 아니라
코드 경로, 방향, 정착 여부를 확인한 smoke test로만 사용한다.
