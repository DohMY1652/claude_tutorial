# DMY MPPI 3축 제어 코드 읽기 가이드

이 문서는 `DMY_mppi` 브랜치의 3축 위치 제어 코드를 직접 읽기 위한 길잡이다.
브랜치의 시작점은 3축 위치 제어 실험에 사용했던 커밋 `3d60148`이며, 이 브랜치에서
추가한 것은 설명 주석과 이 문서뿐이다. 제어식, 파라미터 값, 실행 순서에는 손대지 않았다.

> 안전 주의: 이 코드는 고압 공압 하드웨어를 움직인다. 과거 실험 기록에 액추에이터
> 파손 경로가 남아 있다. 소스를 이해하기 위한 브랜치이지, 무인 운전의 안전을 보증하는
> 브랜치가 아니다. 실기에서는 기계식 압력 제한, 비상 정지, 작업자 감시를 별도로 둔다.

## 1. 가장 먼저 볼 전체 흐름

위치 제어 모드(`control_mode: 2`)의 한 제어 틱은 다음 순서다.

```text
CAN 압력 보드(1~16) ──> CanBridge ──> board/sensors ──> Controller::on_sensor
CAN 엔코더(17~22)   ──> CanBridge ──> board/analog  ──> encoder_angles_
                                                              │
사용자 목표각 ──> slew_targets() ──> run_optimized_pressure_ref()
                                      │
                                      ├─ 위치 오차 → 목표 토크
                                      ├─ 중력·마찰 보상
                                      ├─ 토크 / 릴 반경 → 목표 힘
                                      └─ PressureRefGen::step()
                                               │
                                               ├─ 도달 가능한 압력 경계
                                               ├─ P+ / P- 목표 압력
                                               └─ 양압/음압 레일 목표
                                                        │
각 챔버 현재압·목표압 ──> AcadosMpc::solve() ──> 밸브 3개 명령[%]
                            │
                            ├─ 역모델 피드포워드 u_ref
                            ├─ MPPI 롤아웃 보정 Δu
                            └─ 테이퍼·크래킹·LPF·포화
                                                        │
라인 PID + 안전 검사 ──> 0..4095 PWM ──> board/cmd_pwm ──> CanBridge ──> CAN
```

핵심은 위치 제어기가 밸브를 바로 움직이지 않는다는 점이다. 바깥 루프가 목표 각도를
목표 힘으로 바꾸고, `PressureRefGen`이 그 힘을 만들 양·음압 챔버 목표를 정한다.
안쪽의 채널별 `AcadosMpc`가 실제 챔버 압력을 그 목표에 맞추도록 밸브 명령을 계산한다.

## 2. 추천 읽기 순서

1. `config/powerpack_config.yaml`: 실제 숫자와 보드 배치를 먼저 확인한다.
2. `launch/control.launch.py`: YAML, 자동 생성 피팅 파일, 실행 인자가 어떤 순서로
   병합되는지 확인한다.
3. `Controller::on_timer()`: 한 틱의 전체 호출 순서를 읽는다.
4. `Controller::run_optimized_pressure_ref()`: 각도에서 힘, 압력 목표로 변환되는 과정을 읽는다.
5. `PressureRefGen::step()`: P+/P- 압력 목표와 레일 목표를 만드는 상위 최적화를 읽는다.
6. `AcadosMpc::compute_input_reference()`와 `AcadosMpc::solve()`: 각 챔버의 밸브 명령을 읽는다.
7. `Mppi.cpp`: 롤아웃 플랜트와 샘플 가중 평균을 읽는다.
8. `CanBridge.cpp`: 최종 PWM이 CAN으로 나가고 센서가 돌아오는 경계를 읽는다.

`MppiSystem.cpp`는 `solver:=mppi_system`일 때만 쓰는 중앙집중형 실험 경로다. 기본
채널별 `solver:=mppi`를 이해하려면 나중에 읽어도 된다.

## 3. 파일별 책임

| 파일 | 책임 | 먼저 찾을 함수/구조체 |
|---|---|---|
| `Controller.cpp/.hpp` | ROS 노드, 제어 틱, 위치 외부 루프, 채널 MPC 연결, 안전 처리 | `on_timer`, `run_optimized_pressure_ref`, `AcadosMpc::solve` |
| `PressureRefGen.cpp/.hpp` | 목표 힘을 도달 가능한 P+/P- 목표와 레일 목표로 변환 | `step`, `build_slew_box`, `objective` |
| `Mppi.cpp/.hpp` | 채널 1개의 비선형 플랜트 모델과 MPPI 솔버 | `mppi::step`, `rollout_cost`, `Solver::solve` |
| `MppiSystem.cpp/.hpp` | 모든 채널과 공유 레일을 한 번에 예측하는 선택 경로 | `sys_step`, `SystemSolver::solve` |
| `PneumaticFlow.hpp` | 압축성 오리피스 유량 등 공통 물리식 | `orifice_phi`, `valve_phys_kgps` |
| `PistonPump.hpp` | 양압/음압 레일의 펌프 능력 경계 | `PumpTable` |
| `CanBridge.cpp/.hpp` | Kvaser CAN 입출력, 센서/엔코더 변환, PWM 워치독 | `rx_loop`, `sensor_routine`, `tx_send` |
| `powerpack_config.yaml` | 런타임 파라미터와 보드/채널 보정값 | `PositionController`, `PressureRefGen`, `MPC_parameters` |
| `control.launch.py` | 실기 노드 구성과 launch override | `_setup` |

## 4. 축, 채널, 보드, PWM 매핑

기본 6축 구성에서 축 `i`는 두 압력 챔버와 한 엔코더로 구성된다.

| 물리량 | global id / board |
|---|---|
| 축 `i`의 양압 챔버 | `pos_gid=i`, pressure board `5+i` |
| 축 `i`의 음압 챔버 | `neg_gid=6+i`, pressure board `11+i` |
| 축 `i`의 각도 | encoder board `17+i` |
| 한 pressure board의 밸브 | `v1=micro`, `v2=atmosphere`, `v3=macro` |

코드 내부 밸브 순서는 항상 `{micro, macro, atm}`이지만 물리 보드의 PWM 슬롯은
`{v1=micro, v2=atm, v3=macro}`이다. 이 순서 차이는 `AcadosMpc::finish()`와 PWM
배치 코드를 읽을 때 특히 주의한다.

`ros2 launch ... axis:=1` 같은 단축 실행은 launch 파일에서 논리 `axis0`의
`pos_gid/neg_gid/actuator_idx`를 물리 축 1로 다시 매핑한다. 따라서 모니터 화면의
논리 축 번호와 실제 하드웨어 축 번호를 혼동하지 않는다.

## 5. 단위 경계

| 영역 | 압력 단위/기준 | 다른 주요 단위 |
|---|---|---|
| `Controller`, `AcadosMpc`, `Mppi` | kPa absolute | 밸브 명령 %, 시간 s |
| `PressureRefGen` | Pa gauge | 힘 N, 유량 kg/s, 부피 m³ |
| CAN 센서 raw | ADC count | YAML gain으로 kPa 변환 |
| 엔코더 토픽 | degree | 내부 미분은 deg/s, 삼각함수 직전 rad 변환 |
| 최종 PWM | 0..4095 | 내부 0..100%를 스케일링 |

```text
P_gauge[Pa] = (P_abs[kPa] - P_atm[kPa]) * 1000
P_abs[kPa]  = P_gauge[Pa] / 1000 + P_atm[kPa]
u_pwm       = round(clamp(u_percent, 0, 100) * 4095 / 100)
```

`PressureRefGen`에 절대압을 넘기거나, 그 결과인 게이지압을 그대로 MPC 목표로 쓰면
대기압 약 101.325 kPa만큼 오프셋이 생긴다. 경계의 변환 코드를 항상 같이 확인한다.

## 6. 위치 외부 루프의 의미

`run_optimized_pressure_ref()`에서 축별로 대략 다음 계산을 한다.

```text
e_theta = theta_ref - theta
tau_fb  = Kp*e_theta + Ki*integral(e_theta) - Kd*(omega - omega_ref)
tau_g   = m*g*L*sin(theta)
tau_cmd = tau_fb + tau_g + tau_friction
F_ref   = clamp(tau_cmd / reel_radius, F_min, F_max)
```

그 다음 `PressureRefGen`이 다음 힘 관계를 만족하도록 두 챔버 압력을 고른다.

```text
F_achieved = A_pos * P_pos_gauge - A_neg * P_neg_gauge
```

압력 쌍은 유일하지 않다. 같은 힘을 여러 P+/P- 조합으로 만들 수 있으므로 생성기는
힘 추종뿐 아니라 유량 사용량, 직전 해와의 부드러움, 탱크·이젝터 사용 비용까지 함께
최소화한다. 한 틱에 물리적으로 움직일 수 있는 범위는 `build_slew_box()`가 정한다.

## 7. 채널 MPPI의 의미

`AcadosMpc::solve()`의 개념적 순서는 다음과 같다.

1. 측정 압력, 레일 압력, 부피와 직전 밸브 상태로 롤아웃 초기 상태를 만든다.
2. 목표 압력 변화율에 필요한 유량을 역산해 `u_ref` 피드포워드를 만든다.
3. `K`개의 잡음 제어 시퀀스를 `NP` 스텝 동안 비선형 플랜트에 롤아웃한다.
4. 추종 오차, 명령 크기, 명령 변화, 반대 밸브 동시 개방 등의 비용을 계산한다.
5. 비용이 낮은 샘플의 잡음에 더 큰 지수 가중치를 주어 명목 `Δu`를 갱신한다.
6. 첫 스텝만 적용하고, 나머지 시퀀스는 한 칸 밀어 다음 틱 warm start로 쓴다.
7. 테이퍼, 크래킹 임계, 명령 LPF, 포화를 적용해 PWM으로 바꾼다.

롤아웃의 압력 동역학은 이상기체식에 기반한다.

```text
dP/dt = (R*T/V) * m_dot - (P/V) * dV/dt
```

밸브에는 압축성 오리피스 유동, 크래킹 임계, Bouc-Wen 히스테리시스, 2차 유량 동특성이
포함된다. 모델 파라미터가 실제 밸브와 다르면 MPPI 예측도 같이 틀린다는 점은 변하지 않는다.

## 8. 상태가 다음 틱으로 이어지는 곳

- 위치 적분기와 속도 추정: 정상상태 토크와 D항에 영향을 준다.
- `target_angle_slewed_`: 사용자 목표를 속도 제한한 값이다.
- `PressureRefGen::x_prev_`: 직전 P+/P- 최적해이며 smooth 비용과 SQP 초기값이다.
- `AcadosMpc`의 `last_u3_`, `last_applied3_`: 명령 변화 비용과 밸브 예측의 시작점이다.
- 밸브 `z`, `q`, `qd`: 히스테리시스와 2차 유량 동특성의 기억이다.
- `mppi::Solver::nom_`: 다음 틱으로 shift되는 MPPI 명목 보정 시퀀스다.
- 라인 PID 적분기와 macro gate 상태: 공유 레일 거동에 영향을 준다.
- 센서 LPF 상태: raw 압력과 제어 압력 사이에 지연을 만든다.

동일 목표인데 재실행마다 초기 과도가 다르면 위 상태의 초기화 시점, 센서 영점 보정 중
잔압, 실제 밸브/레일 초기 상태를 먼저 비교한다.

## 9. 파라미터를 읽는 법

실제 적용값은 다음 순서로 병합된다.

```text
powerpack_config.yaml
  < valve_params.yaml / pump_params.yaml / encoder_params.yaml (존재하면 덮어씀)
  < launch의 solver/axis/num_actuators/overrides 인자 (마지막 덮어씀)
```

- `period_ms`: ROS 제어 타이머의 공칭 주기. 이 기준 브랜치는 2 ms, 즉 500 Hz다.
- `PositionController`: 각도 slew, 위치 PID, 중력/마찰, 질량, 기하, 힘 제한.
- `PressureRefGen`: 힘을 압력으로 배분하는 비용과 공급/슬루 제약.
- `MPC_parameters`: 솔버 종류, 지평, 샘플 수, 피드포워드와 MPPI 비용.
- `channel_config.chN`: 채널별 부피 및 밸브 모델/적분 이득 override.
- `LinePID`, `MacroSwitch`: 공유 양압·음압 레일과 고유량 공급의 제어.
- `Safety`: 챔버/레일 과압과 비정상 센서 값에 대한 출력 차단 조건.
- `EncoderCalibration`: 각도 부호와 영점을 결정한다. 피드백 부호가 틀리면 튜닝으로
  안정화할 수 없다.

`axis:=...` 또는 `overrides:=...`를 썼다면 실행 명령과 런타임 파라미터를 로그에 같이 남긴다.

## 10. 제어 모드와 솔버 선택

`control_mode`는 바깥 루프를 고른다.

- `0`: 외부/TCP 압력 목표를 채널 MPC가 직접 추종.
- `1`: 휴리스틱 위치 제어. PID 출력을 bias 주변 P+/P- 차압으로 변환.
- `2`: 최적화 위치 제어. 목표 힘을 `PressureRefGen`이 압력 쌍으로 배분.

`MPC_parameters.solver`는 안쪽 압력 제어를 고른다.

- `qp`: 수치 선형화 + qpOASES의 이전 경로.
- `mppi`: 채널별 비선형 MPPI. 이 브랜치의 기본 이해 대상.
- `mppi_system`: 공유 레일까지 한꺼번에 예측하는 중앙집중 실험 경로.

둘은 독립 설정이다. `control_mode=2, solver=mppi`는 최적화 위치 외부 루프와 채널별
MPPI 압력 내부 루프를 함께 쓴다는 뜻이다.

## 11. 안전 게이트를 따라 읽기

- 필요한 pressure board raw가 한 번도 오지 않으면 제어를 시작하지 않는다.
- 시작 시 0.5초간 pressure sensor offset을 평균한다. 실제 잔압이 있으면 영점과 과압
  판단이 함께 틀어진다.
- 비정상 raw=0은 압력으로 환산하지 않고 직전 값을 유지한다.
- controller 명령이나 CAN RX가 끊기면 bridge watchdog이 안전 출력을 적용한다.
- 챔버 과압과 레일 이상은 Controller 후반부에서 PWM을 안전 상태로 바꾼다.

`0 PWM = 무조건 안전`은 아니다. 채널 밸브에는 닫힘이지만 라인 릴리프까지 0이면
펌프가 레일 압력을 높일 수 있어 `apply_safe_state()`가 별도 패턴을 쓴다.

## 12. 오버슈트와 진동을 로그에서 분리해서 보는 법

각도만 보지 말고 같은 시간축에서 최소한 아래를 겹쳐 본다.

1. 목표각 원본과 slew 목표, 실제 각도, 속도.
2. 목표 토크/힘과 실제 P+/P- 차압으로 계산한 힘.
3. `PressureRefGen`의 P+/P- 목표와 실제 챔버 압력.
4. micro/macro/atm 명령, crack threshold, 최종 PWM.
5. 양압/음압 레일 실제값과 목표값, macro gate.
6. 센서 raw와 LPF 값, timer dt/deadline 통계.

- 압력 목표가 각도를 지나친 뒤에도 큰 힘을 요구하면 외부 위치 루프, 적분 windup,
  중력/마찰 부호 또는 질량·릴 반경 모델을 의심한다.
- 압력 목표는 내려오는데 실제 압력이 계속 상승하면 내부 압력 루프, 밸브 지연,
  배기 능력, 센서 LPF 지연을 의심한다.
- 목표 압력이 흔들리면 생성기의 slew 경계, 공급압 리플, smooth 가중치와 warm start를 본다.
- 특정 축만 다르면 공통 게인보다 엔코더 부호/영점, 부피, 밸브 피팅, 누설과 배관을 본다.
- 모든 축이 같이 흔들리면 공유 레일과 line PID/macro gate의 결합을 먼저 본다.

이 기준 커밋은 “3축이 목표 위치에 도달한 적이 있는 코드”이지 오버슈트가 완전히 제거된
완성 제어기는 아니다. 성공 로그를 재현 기준으로 삼되 안정성의 증명으로 해석하지 않는다.

## 13. 수정 체크리스트

- absolute/gauge와 kPa/Pa 변환을 함수 경계에서 확인했는가?
- 논리 axis와 물리 `gid`, pressure board, encoder board 매핑을 확인했는가?
- 위치 루프, 압력 생성기, 채널 MPC 중 어느 계층을 바꾸는지 분리했는가?
- `period_ms`, MPPI `Ts`, 실제 CAN TX 주기가 다를 수 있음을 반영했는가?
- 적분기, warm start, 밸브 상태 추정의 리셋 조건을 정했는가?
- 최종 명령의 소유자가 하나인가? system MPPI와 line PID가 같은 PWM을 쓰면 안 된다.
- 실기 전 노드 중복, 센서 수신, 엔코더 부호, 워치독, 압력 제한을 저압에서 확인했는가?
