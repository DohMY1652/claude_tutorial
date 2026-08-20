# can_powerpack — 6축 공압 액추에이터 제어 시스템

ROS 2 (foxy) 패키지. 6개 공압 액추에이터의 위치를 제어하며, 각 축은 **양압 챔버와 음압 챔버를
동시에 써서 같은 방향으로 힘을 만든다.** 12개 챔버가 하나의 펌프를 나눠 쓰고, 급할 때 쓸 압축탱크와
이젝터가 공용으로 하나씩 있다.

> 이 문서는 시스템·제어기·코드 구조와 밸브 파라미터 피팅 도구의 사용법을 담는다.
> 물리적 배경과 설계 근거는 `압력레퍼런스_해설.pdf` (소스: `압력레퍼런스_해설.tex`) 를 볼 것.

---

## 1. 시스템 구조

### 힘 생성

```
F = P⁺·A⁺ − P⁻·A⁻          P⁻ 는 게이지 음수라 두 항 모두 양수 기여
τ = reel · F               reel = 25 mm,  A = π·25² = 1963.5 mm²
```

**이 시스템은 한 방향 힘만 낸다.** `P⁺ ≥ 0`, `P⁻ ≤ 0` 이므로 `τ` 는 절대 음수가 될 수 없고,
목표 토크는 항상 `≥ 0` 으로 클램프된다. 정격에서 `τ_max ≈ 7.76 N·m` 인데 중력이
`5 kg × 0.15 m = 7.36 N·m @90°` 라 여유가 5% 뿐이다 — 고각도에서 최적화가 중요한 이유다.

### 세 개의 물리적 병목

1. **유량은 압력차가 만든다.** `ṁ = A·Pu/√(RT)·Φ(Pd/Pu)`. 상류 압력에 비례하고 압력비 0.528
   아래는 초킹. 배기할 때 상류는 챔버 자신이라 **자기감쇠**한다 ("배기는 빠르다"는 틀렸다).
2. **하나의 펌프, 두 레일 = 시소.** 흡입량 ≡ 토출량(질량보존)이라 양쪽 다 최대는 불가능하다.
   → **능력경계**: 음압 −90 kPa 를 고집하면 양압은 335 kPa 에서 막히고, −80 으로 10 kPa 양보하면
   745 kPa 까지 열린다.
3. **압축탱크는 예비 배터리.** 213 mL @30 MPa → 레귤레이터 700 kPa. 컴프레서가 없어 회복되지 않고
   이젝터만 돌려도 약 1분이면 바닥난다. 상시 공급원이 아니라 **부스터**다.

핵심 통찰: **레일 압력이 곧 응답 속도다.** 채널 충진 유량은 레일 *압력*만 관계하고 펌프가 지금
돌고 있는지는 무관하다. 그리고 릴리프를 없애면 초기조건에 갇혀 제어 수단 자체가 사라진다 —
릴리프 셋포인트가 펌프 능력 배분 손잡이다.

### 보드 매핑

`channel_board_offset: 5`, `board_id = gid + 5`, PWM 평탄 인덱스 `= (board_id − 1)·3 + valve_idx`

| board | 역할 | v1 (idx 0) | v2 (idx 1) | v3 (idx 2) |
|---|---|---|---|---|
| 1 | 양압 레일 센서 | **릴리프 → 대기** (LinePID) | — | — |
| 2 | 음압 레일 센서 | **유입 ← 대기** (LinePID) | — | — |
| 3 | 압축탱크 센서 | 밸브 없음 | — | — |
| 4 | 이젝터 라인 센서 | **MacroSwitch** (on/off) | — | — |
| 5–10 | 양압 채널 gid 0–5 | micro (레일→챔버) | atm (챔버→대기) | macro (탱크→챔버) |
| 11–16 | 음압 채널 gid 6–11 | micro (챔버→음압레일) | atm (대기→챔버) | macro (챔버→이젝터) |
| 17–22 | 엔코더 6축 | — | — | — |

오리피스 지름 [mm]: fill 2.3 / vent 4.0 / boost 1.6 / suck 4.0 / admit 4.0 / eject 4.0.
**올리는 길은 둘(2.3+1.6), 내리는 길은 하나(4.0)** 인 비대칭이 성능을 좌우한다.

---

## 2. 제어기 구조 (4계층 캐스케이드)

```
[TCP 2293] 목표 각도 6개
     ↓ 500 Hz
① TorquePID (축별)        각도 오차 → 목표 토크 [N·m]
     τ = kp·e + ki∫e − kd·ω + m·g·L·sinθ + friction,  clamp ≥ 0
     ↓  F_ref = τ / reel
② PressureRefGen  50 Hz   목표 힘 → 12개 목표 압력 + 적응 레일 셋포인트
     ↓ (ZOH 10 tick)
③ AcadosMpc × 12  500 Hz  목표 압력 → 밸브 PWM 3개    ← ThreadPool 병렬
   LinePID × 2    500 Hz  레일 셋포인트 → 릴리프/유입 밸브
   MacroSwitch            음압 macro OR → 이젝터 구동
     ↓ board/pwm_cmd
[CanBridge] 또는 [VirtualPowerpack]
```

`control_mode`: `0` 압력 직접 / `1` 위치(휴리스틱) / **`2` 위치(최적화)** ← 현재 설정

### ② PressureRefGen — 이 프로젝트의 심장

시간 스케일 분리(특이섭동, ε ≈ 20 ms / 2 s = 0.01)로 2계층:

- **느린 계층 (레일 배분)** — 총 수요 크기로 능력경계 위에서 양/음 배분.
  `P⁻_sp = −30 + d̄·(−74.3 + 30)`, `P⁺_sp = min(cap_ppos(P⁻_sp), 30 + d̄·220)` [kPa gauge]
- **빠른 계층 (슬루 박스 + 소프트 최적화)** — 제약이 임의 마진이 아니라 **지금 이 순간 차압에서
  실제 흐르는 유량**이다:
  `ΔP⁺↑ = (ṁ_fill + ṁ_boost)·dt·nRT/V⁺ − n·P/V·V̇·dt`
  그래서 레일이 처지면 갈 거리가 자동으로 줄고, 챔버가 레일에 가까워지면 차압→0 이라
  **채널이 레일을 넘는 게 물리적으로 불가능**해진다.

목적함수: `J = 100·J_trk + 0.3·J_flow + 0.5·J_smooth + 15·J_tank + 25·J_eject`

- `J_trk` 는 **등식 제약이 아니라 소프트항**이다. 1000 N 을 요구해도 infeasible 로 멈추지 않고
  "한 스텝에 갈 수 있는 최대"를 돌려준다. 다축이 유량을 다투면 목표가 일시적으로 불가능해지는
  것이 정상이므로 필수.
- `J_tank`/`J_eject` 는 **레일 초과분에만** 벌점. 압력 상승 전체에 걸었던 초기 버전은 최적화가
  압력 올리기 자체를 회피해 90 N 목표에 1 N 만 내는 참사가 났었다.
- `J_fast`(빠른 쪽 우선)와 scarcity(잔량 반비례)는 **추종을 해쳐 제거**됐다. 슬루 박스가 이미
  물리 능력을 담고 있어 `J_trk` 하나로 자기균형 인수인계가 일어난다.

솔버: fmincon 대신 **박스 제약 SQP** — 중앙차분 수치 기울기 → Gauss-Newton 이차모형 →
qpOASES 박스 QP → 실제 목적함수로 backtracking line search, 최대 12 반복.

### ③ AcadosMpc — 채널당 1개, 12개 병렬

`n_x=1`(챔버 압력), `n_u=3`(micro/atm/macro), `NP=10`, Q=R=10.

1. **명령 테이퍼** — 오차가 `cmd_taper_kpa` 안이면 **크래킹 임계 위쪽 여유분**을 연속 감쇠
2. **피드포워드** — `P_dot = err·1000/target_time_constant` → 필요 유량 → 13-parameter
   밸브 역모델로 `u_pct` 역산 + 밸브별 `Ki·∫e`
3. **선형화** — 정적 유량의 수치 야코비안 → 스칼라 A, 1×3 B
4. **condensed MPC** — Δu 30변수, 박스만, qpOASES SQProblem. hot-start 실패 시 같은 틱에서
   cold-start 재시도
5. `u = u_ref + Δu0` → 크래킹 위쪽 테이퍼 → ×40.95 → PWM

### macro(탱크 부스트 / 이젝터) 개방 규칙

임의의 kPa 임계값 대신 물리에서 유도된 두 판정을 OR 로 결합한다 (반응성이 절약보다 우선):

- **생성기**: 축별 유량 부족률 `(요구 − 레일 능력)/요구 > macro_gate_frac(0.02)`.
  무차원이라 "레일이 수요의 몇 %를 못 대면 부른다"로 읽힌다. 절대 kg/s 는 챔버 부피·dt·압력
  스텝에 모두 비례해 사람이 판단할 수 없다.
- **MPC**: micro 밸브 명령 포화(`macro_micro_sat_pct = 100`). 레일 밸브를 완전히 열었는데도
  요구 유량에 못 미친다 = 레일만으로 부족. 100% 는 밸브 물리 한계라 튜닝 대상이 아니다.

---

## 3. 코드 구조

| 파일 | 역할 |
|---|---|
| `include/PneumaticFlow.hpp` | 오리피스 Φ, `valve_phys_kgps`, 물리 상수. **공용 기반** |
| `include/PistonPump.hpp` | 슬라이더-크랭크 1주기 평균 + 2D 능력 테이블 + `EjectorCurve` (ZL112A) |
| `include/src/PressureRefGen.*` | 압력 레퍼런스 생성기. 순수 계산, ROS 무의존 |
| `include/src/Controller.*` | **88 KB 단일 노드.** `AcadosMpc`·`QP`·`ThreadPool`·`RefTcpServer`·`Controller` |
| `include/src/VirtualPowerpack.*` | CanBridge 대체 가상 하드웨어 |
| `include/src/CanBridge.*` | 실제 Kvaser CANlib 브리지 (canlib 없으면 빌드 스킵) |
| `src/pressure_ref_test.cpp` | 생성기 단독 검증 (ROS 없이 4항목) |

실행파일: `pp_controller`, `can_bridge_node`, `virtual_powerpack`, `pressure_ref_test`,
`pneumatic_sim`(구세대). `-O3 -march=native`.

### 토픽 인터페이스

`VirtualPowerpack` 이 `CanBridge` 와 **비트 단위로 동일**하게 흉내낸다.

| 토픽 | 타입 | 길이 | 인덱스 → 의미 | 단위 |
|---|---|---|---|---|
| `board/sensors` | `UInt16MultiArray` | 16 | `[bid−1]` | **mV** (kPa 아님!) |
| `board/currents` | `Float64MultiArray` | 48 | `[(bid−1)·3+v]` | **mV** (`mA = mV/10`) |
| `board/analog` | `Float64MultiArray` | 9 | `[bid−17]` | deg (캘리브레이션 완료) |
| `board/pwm_cmd` | `UInt16MultiArray` | 75 발행 / **48 유효** | `[(bid−1)·3+v]` | 0–4095 |
| `controller/sensors_kpa` | `Float64MultiArray` | 25 | `[bid−1]` | **kPa abs** |
| `controller/mpc_refs_kpa` | `Float64MultiArray` | 12 | gid | kPa abs |
| `controller/pressure_ref_dbg` | `Float64MultiArray` | 12·N+6 | 아래 참조 | 50 Hz, mode 2 전용 |

`pressure_ref_dbg` 축당 12개: `[angle, angle_ref, tau_achieved, tau_ref, P⁺_ref, P⁻_ref,
ub_pos, lb_pos, lb_neg, ub_neg, starve_pos%, starve_neg%]`, 말미 6개:
`[rail⁺_sp, rail⁻_sp, tank, tank_low, m_boost, m_eject]`.

**raw → kPa 변환은 `pp_controller` 만 한다**: `kPa_abs = (mV − offset)·gain + atm_offset`.
캘리브레이션은 `config/powerpack_config.yaml` 의 `Sensor_calibration`.

### 빌드 · 실행

```bash
colcon build --packages-select can_powerpack --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 가상 하드웨어로 전체 시스템 (하드웨어 불필요)
ros2 launch can_powerpack virtual.launch.py control_mode:=2

# 실기
ros2 launch can_powerpack control.launch.py

# 생성기 단독 검증
./build/can_powerpack/pressure_ref_test

# 목표 각도 전송 (리틀 엔디언 double × num_actuators)
python3 src/can_powerpack/scripts/position_ref_client.py 127.0.0.1 2293 --once 45 45 45 45 45 45
```

---

## 4. 밸브 파라미터 피팅 도구

`channel_config.chN.*` 의 13-parameter 밸브 모델과 `PressureRefGen` 의 오리피스·유량계수를
실측으로 채우는 파이프라인. 레퍼런스는 `current_hysteresis_fitting_v02_26_04_01.m` 이다
(`pressure_reference_optimizer.m` 에는 피팅 코드가 없다 — 압력 레퍼런스 최적화기다).

| 스크립트 | 역할 |
|---|---|
| `valve_fit_model.py` | 모델 + Nelder-Mead(`fminsearch` 동등) + Savitzky-Golay 미분. ROS 무관 |
| `valve_fit_record.py` | 실험 구동 + 동기 기록 (rclpy) |
| `valve_fit_solve.py` | 이중 부피법 → 13-parameter 피팅 → 오리피스 계수 → yaml/report/플롯 |
| `valve_fit_selftest.py` | 하드웨어 없는 자기검증 (합성 → 복원) |

### 실험 구성

**한 번 실행 = (모드 1개) × (채널 목록).** 36개를 한 번에 하지 않는다 — 외부 압력 인가 부품과
고정부피 탱크를 채널마다 사람이 옮겨야 하므로 채널 전환마다 프롬프트에서 멈춘다.

| 모드 | 채널 | 외부 압력 인가 | 피팅되는 쌍 |
|---|---|---|---|
| `pos_micro` | 양압 gid 0–5 | board 1 (예 250 kPa) | v1(충전) + v2(배기) |
| `pos_macro` | 양압 gid 0–5 | board 3 (예 700 kPa) | v3(부스트) + v2(배기) |
| `neg_micro` | 음압 gid 6–11 | board 2 (예 30 kPa) | v1(흡입) + v2(유입) |
| `neg_macro` | 음압 gid 6–11 | board 4 (외부 진공) | v3(이젝트) + v2(유입) |

`v2` 는 micro·macro 두 모드에 모두 나오므로 **독립 2회 피팅 → 교차검증**이 된다.

### 사전 조건

- **`pp_controller` 를 띄우지 말 것.** 두 노드가 같은 `board/pwm_cmd` 에 발행하고 중재가 없다.
  `system_parameters.valve_operate: false` 도 해결책이 아니다 — 그 경우 **500 Hz 로 전부 0을
  발행**해서 스크립트 명령을 덮어쓴다. 스크립트가 시작 시 자동 검사한다.
- `can_bridge_node` 만 기동한다 (그런 launch 가 없어서 수동):
  ```bash
  ros2 run can_powerpack can_bridge_node --ros-args --namespace pack2 \
      --params-file src/can_powerpack/config/powerpack_config.yaml
  ```
- **액추에이터를 분리하고 고정부피 탱크를 붙인다.** 챔버 부피가 각도에 따라 변하면 절대 스케일이
  정해지지 않는다.
- 라인압은 외부 레귤레이터로 사람이 유지한다. 스크립트는 **감시만** 하고 제어하지 않는다.
- 이젝터는 쓰지 않는다. board 4 라인에 외부 진공을 직접 인가하고 MacroSwitch 는 닫아 둔다.

### 왜 이중 부피법이 필요한가

유량계가 없어 `Q` 를 챔버압 미분으로 합성해야 한다. 그러면 지배식에 `A_max/V` 곱만 나타나
**`A_max` 와 챔버 부피 `V` 가 분리되지 않는다.** `V` 는 각도에 따라 변하므로 대충 넣으면 다른
각도에서 틀린다. → 알려진 부피 `ΔV` 용기를 티로 달아 **같은 모드를 두 번** 기록하고,
공통 압력 구간의 통과시간 비 `r = Δt_B/Δt_A = (V+ΔV)/V` 에서 `V = ΔV/(r−1)` 를 얻는다.

한편 레퍼런스보다 나아진 점도 있다 — `board/currents` 의 **실측 전류**를 모델 입력으로 쓰므로
MATLAB 이 가정했던 `I = u·I_MAX` 보다 정확하다 (드라이버 전류루프가 피팅에서 빠진다).

### 사용법

```bash
cd src/can_powerpack

# 0) 하드웨어 없이 파이프라인 검증 — 실기 전에 반드시 통과시킬 것
python3 scripts/valve_fit_selftest.py --samples 100

# 1) 1회차: 고정부피 탱크만
python3 scripts/valve_fit_record.py --mode pos_micro --line-kpa 250

# 2) 2회차: 알려진 부피 100 mL 를 티로 추가 (이중 부피법)
python3 scripts/valve_fit_record.py --mode pos_micro --line-kpa 250 \
        --extra-volume-ml 100 --out results_fit/<1회차와_같은_디렉터리>

# 3) 피팅
python3 scripts/valve_fit_solve.py results_fit/<디렉터리> --extra-volume-ml 100

# 부피를 따로 실측했다면 이중 부피법 생략 가능
python3 scripts/valve_fit_solve.py results_fit/<디렉터리> --volume-ml 128.5
```

채널 전환 프롬프트: `Enter` 진행 / `s` 건너뛰기 / `r` 재실행 / `q` 종료.
단일 채널만: `--gids 0`. 처음에는 채널 하나로 검증한 뒤 확대할 것.

산출물: `valve_params.yaml`(머신 생성), `report.md`(R²·파라미터·식별성·오리피스 계수),
`fit_<모드>_ch<gid>_<밸브>.png`(예측 vs 실측 재생).

### 가상 하드웨어 리허설 (실기 전 권장)

```bash
# 파라미터 평탄화 (virtual.launch.py 가 하는 일을 수동으로)
python3 - <<'PY'
import yaml
def flat(p,v,o):
    if isinstance(v,dict):
        for k,x in v.items(): flat(f'{p}.{k}' if p else str(k), x, o)
    else: o[p]=v
def load(path,key):
    d=yaml.safe_load(open(path)) or {}; o={}
    flat('', (d.get(key) or {}).get('ros__parameters',{}), o); return o
p={}
p.update(load('src/can_powerpack/config/powerpack_config.yaml','/pack2/pp_controller'))
p.update(load('src/can_powerpack/config/powerpack_config.yaml','/pack2/can_bridge'))
p.update(load('src/can_powerpack/config/virtual_powerpack.yaml','/pack2/can_bridge'))
p['actuator_connected']=False       # 밸브 특성 시험 구성
yaml.safe_dump({'/pack2/can_bridge':{'ros__parameters':p}}, open('/tmp/virt_flat.yaml','w'),
               allow_unicode=True, sort_keys=False)
PY

# 가상 하드웨어만 (pp_controller 없이)
ros2 run can_powerpack virtual_powerpack --ros-args \
    -r __node:=can_bridge -r __ns:=/pack2 --params-file /tmp/virt_flat.yaml &

# 리허설 — 실제로 밸브를 구동해 폐루프 확인
python3 scripts/valve_fit_record.py --mode pos_micro --line-kpa 196.6 --gids 0 \
        --skip-line-check --no-zero --dry-run --publish-in-dry-run \
        --levels 0 55 65 80 100 --level-hold 1.5
```

`--dry-run` 만 주면 명령을 발행하지 않고 토픽·시퀀스·안전 경로만 확인한다.

### 안전 (pp_controller 가 없으므로 전부 스크립트 책임)

- 시작 시 `board/pwm_cmd` 의 **다른** 발행자를 검사해 pp_controller 충돌을 차단
- 0점 보정값이 yaml 과 `--zero-tol-kpa`(8 kPa) 이상 다르면 중단 — 압력이 남은 상태로 보정하면
  그 값이 오프셋으로 흡수돼 이후 모든 측정과 **과압 트립 기준이 함께 틀어진다**
- 스윕은 트립보다 안쪽(`trip_hi − 25 kPa`)에서 멈추고, **예측 정지**를 쓴다 — 밸브를 닫아도
  2차 동특성(wn≈40 rad/s) 때문에 유량이 수십 ms 꼬리를 남겨서, 700 kPa/s 로 차오를 때
  상한에서 닫아도 20 kPa 를 넘긴다. 멈출 때는 중립밸브를 즉시 열어 제동한다
- 과압/과진공 트립 → 해당 채널 v2 전개 + 대상밸브 폐쇄 후 중단
- 종료(정상·예외·Ctrl-C) 시 **전 채널 v2 개방 2 s 후 전부 0**.
  "전부 0" 은 양압 채널에서 압력이 갇히는 상태라 안전 상태가 아니다
- `CanBridge` 에는 워치독이 없다 — 마지막 명령이 250 Hz 로 영구히 나간다

### 알아야 할 모델 성질

```
A_eff = A_max · sigmoid(k_shape·(I + C_p·P − C_k))^alpha_shape
```

이 형태는 `(A_max, k_shape, C_k, alpha_shape)` 가 거의 자유롭게 상쇄되는 **평평한 다양체**를
갖는다. **완벽한 데이터로도 개별 파라미터가 정해지지 않는다** — 예측 유량 R² 0.97 인데
`alpha_shape` 가 3500 대 611 로 나오는 식이다. 레퍼런스 MATLAB 도 같은 문제를 안고 있었고
복원 검증을 하지 않았다 (그래서 코드에 `alpha_shape = 3884.2` 같은 값이 남아 있다).

→ `PARAM_BOUNDS` 로 최적화가 극단으로 달아나는 것을 막는다. 이것만으로 **크래킹 임계 오차가
12%p → 1%p** 로 줄었다. 판정도 개별 파라미터가 아니라 **부피 · 예측 유량 R² · 크래킹 임계**로 한다.

자기검증 결과 (`pos_micro` gid0, 합성 데이터):

| 지표 | 결과 | |
|---|---|---|
| 챔버 부피 | 128.50 → 132.27 mL (2.9%) | PASS |
| 예측 유량 R² | 0.967 (v1) / 0.938 (v2) | PASS |
| 크래킹 임계 | 60.6% → 57.9% / 60.0% | PASS |
| A_eff 곡선 RMS | 28% / 32% | 진단값 (과매개화로 0 불가) |

### 미완 — 결과를 실제로 쓰려면

현재 `ChannelConfig` 는 채널당 13-parameter 세트가 **하나**뿐이고 micro/atm/macro 에 같은 값을
쓴다 (`Controller.cpp:1250-1263`). 밸브별 피팅 결과를 로드하려면 로더를
`channel_config.chN.{micro,atm,macro}.*` 3세트로 확장해야 한다.
그때까지 `valve_params.yaml` 은 보관·리포트용이다.

---

## 4b. 펌프 파라미터 피팅 도구

`PumpGeom` 10개 필드가 전부 **예전 펌프 값**이다. 물리적으로도 이상하다 — 소기량이 924 LPM
인데 실측 토출이 ~1.1 g/s(≈0.9 L/s)라 체적효율 6%이고, `Cb_out`이 1.46 mm²뿐이라 유량이
전적으로 토출 체크밸브에 막혀 있다.

| 스크립트 | 역할 |
|---|---|
| `pump_fit_model.py` | `pump_piston_avg`/`PumpTable` Python 포팅 + 피팅. 단독 실행하면 포팅 검증 |
| `pump_fit_record.py` | 레일 실험 구동 + 기록 (rclpy). 인덱스 0·3 만 구동 |
| `pump_fit_solve.py` | 부피·누설 → 유량 맵 → 능력경계 → 피팅 → yaml/report/플롯 |
| `pump_fit_selftest.py` | 하드웨어 없는 자기검증 |

### 핵심 아이디어 — 라인 밸브 모델이 필요 없다

레일은 **대기 창구가 정확히 두 개뿐인 닫힌 회로**다 (board1 v1 = 양압→대기, board2 v1 =
대기→음압). board 3(탱크)·board 4(이젝터)는 레일에 영향을 주지 못하고, 채널 v1(micro)만
레일에 붙으므로 채널 PWM을 전부 0으로 두면 끊긴다. 따라서 **두 밸브를 닫으면** 밸브 유량이
0이 되고 부피만으로 펌프 유량이 나온다:

```
ṁ_pump = +V⁺/(R·T)·dP⁺/dt + leak⁺(P⁺)      (양압 레일)
       = −V⁻/(R·T)·dP⁻/dt + leak⁻(P⁻)      (음압 레일)
```

두 식이 질량보존으로 같아야 하므로 **매 측정점에서 교차검증**이 된다.

### 3단계

| 단계 | 펌프 | 내용 |
|---|---|---|
| `leak` | **OFF** | 양 밸브 폐쇄 감쇠 → 지수 시상수 τ. ΔV 회차의 τ 비가 `(V+ΔV)/V` 이므로 부피와 누설이 함께 풀린다 (펌프 무관) |
| `map` | ON | (relief, admit) 격자에서 정착 → 양 밸브 짧게 폐쇄 → 두 레일 dP/dt → `ṁ_pump` |
| `frontier` | ON | admit으로 P⁻를 목표에 잡고 relief를 닫아 P⁺를 스톨까지 램프 → **능력경계 직접 측정** |

`leak`은 3회 기록한다 — ①맨몸 ②ΔV를 양압 레일에 ③ΔV를 음압 레일에.

```bash
cd src/can_powerpack
python3 scripts/pump_fit_selftest.py                              # 실기 전 필수
python3 scripts/pump_fit_record.py --phase leak
python3 scripts/pump_fit_record.py --phase leak --extra-volume-ml 250 --extra-volume-rail pos
python3 scripts/pump_fit_record.py --phase leak --extra-volume-ml 250 --extra-volume-rail neg
python3 scripts/pump_fit_record.py --phase map
python3 scripts/pump_fit_record.py --phase frontier --ppos-ceiling 500
python3 scripts/pump_fit_solve.py results_pump/<디렉터리> --crank-m 0.02
```

### 안전 — 밸브 피팅과 정반대다

`valve_fit_record.py`는 안전 상태가 "v2 개방"이지만, 펌프 실험은 **양 밸브 전개**가 안전
상태다. 펌프가 도는 동안 relief(board1 v1)를 닫으면 양압 레일이 무한정 올라간다.
**"전 밸브 0"은 위험하다.** 종료·예외·Ctrl-C 모두 양 밸브를 열어 두 레일을 대기압으로
되돌린 뒤 0으로 간다. 양압 상한/음압 하한 예측 정지도 들어 있다.

> 참고: 밸브 피팅(`valve_fit_record.py`)은 **펌프를 분리하고 외부 공급원**을 쓰는 전제다.
> 그 스크립트는 board1 v1을 0(폐쇄)으로 고정하므로, 펌프를 돌린 상태로 쓰면 안 된다.

### 무엇을 믿을 수 있나 — 신뢰도 순서

자기검증으로 확인한 것 (합성 데이터):

| 산출물 | 정확도 | 신뢰도 |
|---|---|---|
| 레일 부피 · 누설 | **0.0 ~ 0.1%** | 높음 — 이중 부피법이 펌프와 무관 |
| 측정 유량 맵 (84점, P⁺ 2~443 / P⁻ −90~−6 kPa) | 중앙 오차 **0.7%** | 높음 |
| 측정 능력경계 (`ṁ_pump = leak⁺` 균형) | 중앙 불일치 **1.3%** | 높음 |
| 기하 피팅의 능력경계 | 회차마다 13% ~ 발산 | **낮음 — 쓰지 말 것** |

**5-파라미터 슬라이더-크랭크는 유량 데이터로 다중 모드다.** 측정 범위 안의 유량을 잘
맞추면서도 데드헤드(측정 범위 **밖** 외삽)가 크게 틀어진다 — 맵 RMS 19%로 맞추면서 압축비가
469까지 올라가 경계가 1200 kPa로 튀는 해가 나왔다. 그래서 `pump_params.yaml`은
**측정 테이블을 1차 산출물**로 싣는다:

- `PressureRefGen.pump_frontier_measured` — 컨트롤러가 쓰는 유일한 출력(`cap_ppos`)
- `Virtual.pump_map_measured` — 시뮬 `flow_out`용 측정 점들
- `pump: {...}` — 기하 피팅 산물, **참고용**

정확히 필요한 만큼만 판정하면 된다: 생성기의 `pos_sp_max_kpa`가 250 kPa gauge이므로
**능력경계가 250 위/아래인지만 갈라도** 컨트롤러 동작에는 충분하다.

### 알아둘 것

- **RPM은 유량 맵 전체에 비례한다.** 압력 리플 FFT로 추정을 시도하지만, CanBridge LPF
  (코너 ≈18 Hz)가 100 Hz를 크게 감쇠시키고 board1 분해능이 0.25 kPa/LSB로 거칠어
  **best-effort**다. 태코미터 실측이 가능하면 그쪽이 낫다.
- **크랭크 반경 `r`은 피팅하지 않는다** — 소기량과 곱으로만 나타나 따로 갈리지 않으므로
  실측값으로 고정하고 `--crank-m`으로 넘긴다.
- 피팅 좌표는 기하 8개가 아니라 `(V_swept, V_dead, A_out, A_in, l/r)`로 재매개화했다.
  유량은 소기량에, 데드헤드는 압축비(`V_dead`)에 걸려 물리적으로 분리돼 있다.

---

## 5. 실측이 필요한 파라미터

전체 인벤토리는 별도 문서로 정리했다 — 각 항목의 현재 값, 무엇을 정하는지, 어떻게 측정하는지.

**가장 먼저 할 세 가지:**

1. **밸브 크래킹 전류를 압력별로 측정** — 이미 ~50% 로 관측했으니 3~4개 상류압에서 재면
   `C_k`(스프링 프리로드)·`C_p`(압력 도움 계수)가 바로 고정된다. 13-parameter 중 가장 저렴하고
   영향이 크다.
2. **엔코더 board 20·21·22 캘리브레이션** — 6축인데 `EncoderCalibration.boards` 에 실측 2점이
   있는 건 17/18/19 뿐이다. 나머지 세 축은 전역 기본값으로 돌고 있고, 축별 실측값이
   860~2115 로 크게 달라 기본값으로는 맞을 수 없다.
3. **챔버 사구간 부피와 채널 고정 부피** — `Geometry.vol_offset_*_mm`, `tank_volume_*_ml`.
   슬루 한계가 부피에 반비례하므로 응답 속도를 직접 정한다.

**그 밖에 눈에 걸린 것:**

- `channel_config.chN` 에 **13-parameter 키가 하나도 없다** — 전부 `Controller.cpp:855-869` 의
  하드코딩 기본값으로 12채널이 같은 값을 쓴다. 채널별로 넣을 수 있는 구조인데 쓰이지 않는다.
- `C_z = 0` 이라 **Bouc-Wen 히스테리시스가 실질적으로 꺼져 있다.** `z` 를 매 틱 계산하지만
  `C_z·z` 항이 0 이라 유효면적에 반영되지 않는다.
- `reel_radius_mm` 이 **부피식의 mm/rad 환산과 토크 암을 겸한다.** 실기 구조에서 정말 같은 값인지
  확인이 필요하다 (시뮬 설정에는 `torque_arm_m` 으로 따로 있다).
- `link_length_m: 0.15` 는 중력 토크에 쓰이므로 **링크 길이가 아니라 무게중심 거리**여야 한다.
- **이젝터 실제 구동압이 특성표 범위를 넘는다** — 레귤레이터 700 kPa 인데 ZL112A 표는 400 까지고
  현재 400 에서 포화 처리된다.
- **`LPM_TO_KGPS` 두 개가 공존하고 10.7배 다르다** — 경험적(2.155e-4) vs 표준(2.007e-5).
  카탈로그값·물리 환산은 반드시 후자를 써야 한다.
- 음압 센서 gain 이 양압의 정확히 10배 작고 부호가 반대(−0.02525 vs 0.250)다. 센서 품번이
  다르면 정상이지만 딱 10배라 2점 검증 권장.

**죽은 파라미터** (넣어도 반영되지 않는다): `channel_config.chN.k0/k1/k2`,
`MPC_parameters.ejector_k`, `MPC_parameters.Ts`(폴백만), `leakage_u_pos/neg`(0),
`RefTcp.*`(비활성), `PositionController.axisN.mode1.*`(mode 2 미사용),
`du_min/du_max`(하드코딩, yaml 키 없음).

---

## 6. 개발 중 확인된 함정

시뮬레이터/제어기를 만지기 전에 알아두면 시간을 아낀다.

| 함정 | 실제 |
|---|---|
| `board/kpa_all` 토픽 | **존재하지 않는다.** 실제 이름은 `controller/sensors_kpa` |
| `board/sensors` 가 kPa | **mV 다.** `pp_controller` 만 변환한다 |
| `board/currents` 가 mA | **mV 다.** `mA = mV/10` |
| `board/pwm_cmd` 75개 유효 | **48개만** CanBridge 에 도달한다 (보드 16장) |
| `valve_operate: false` 면 스크립트가 밸브를 잡을 수 있다 | 아니다. **500 Hz 로 전부 0을 발행**해 덮어쓴다 |
| 라인 밸브 `0` = 안전 | **`0` = 닫힘 = 압력 무제한 상승.** 반전 블리드형이다 |
| 전 밸브 0 이 안전 상태 | 양압 채널에서 **압력이 갇힌다.** 안전 상태는 v2 개방 |
| `current_mode`/`control_type` 을 런타임에 바꿀 수 있다 | 파라미터 콜백이 없다. 노드 기동 시 넘겨야 한다 |
| `live_control.py`/`diagnostic_check.py` 의 캘리브레이션 테이블 | **stale.** `can_monitor.py` 것이 yaml 과 일치 |
| `log_topics.py`/`dashboard.py` 토픽명 | **죽은 이름**(`b0/b1/b2`). 템플릿으로 쓰지 말 것 |
| MPC 예측 모델이 정상 동작 | `A_scalar` 가 초킹 구간에서 **정확히 0** 이 되고, 연속시간 야코비안을 이산 전이행렬로 쓴다. Δu 가 사실상 0 이라 MPC 계층이 실질적으로 동작하지 않는다 |
| 시뮬 하네스가 결정론적 | **아니다.** 동일 빌드 반복 실행에서 오버슈트 +0.92~+2.27°, 정상상태 밸브 개방률 0~100% 로 흩어진다. 튜닝은 N회 평균으로 판단해야 한다 |
| 시뮬 밸브가 하류압 아래로 안 간다 | 2차 동특성이 **유량**에 걸려 있어 `Φ=0` 이 된 뒤에도 꼬리가 남아 언더슈트한다 |

---

## 7. 파일 안내

```
압력레퍼런스_해설.pdf / .tex          설계 근거·물리 배경 해설서
pressure_reference_optimizer.m       압력 레퍼런스 최적화기 (PressureRefGen 의 원본)
current_hysteresis_fitting_*.m       13-parameter 밸브 피팅 레퍼런스
src/can_powerpack/
  config/powerpack_config.yaml       메인 설정 (사람이 쓰는 파일)
  config/virtual_powerpack.yaml      가상 하드웨어 물리 파라미터
  launch/virtual.launch.py           가상 시스템 (하드웨어 불필요)
  launch/control.launch.py           실기
  scripts/valve_fit_*.py             밸브 파라미터 피팅 도구
  scripts/can_monitor.py             센서 모니터 (캘리브레이션 테이블이 yaml 과 일치)
  scripts/position_ref_client.py     목표 각도 TCP 클라이언트
```
