# 실기 피팅 런북

실제 하드웨어에서 밸브 36개와 펌프를 피팅하는 절차서다. 배경·이론은 `README.md`
(4·5·6·8절)에 있고, 여기는 **순서와 손이 할 일**만 적는다.

시뮬 리허설은 끝났다 — 자기검증 PASS, Phase L·M·F 완주, 결함 11건(밸브 4 + 펌프 7) 수정.
`abff9b5` 시점 기준.

---

## 0. 시작 전 (매번)

```bash
cd ~/claude_tutorial
colcon build --packages-select can_powerpack
source install/setup.zsh
cd src/can_powerpack
```

### `pp_controller` 를 절대 띄우지 않는다

두 노드가 같은 `board/pwm_cmd` 에 발행하고 **중재가 없다.** `system_parameters.valve_operate:
false` 도 해결책이 아니다 — 그 경우 pp_controller 가 500 Hz 로 전부 0 을 발행해 스크립트 명령을
덮어쓴다. 두 기록 스크립트가 시작 시 이걸 검사해서 감지되면 중단한다.

브리지만 띄운다:

```bash
ros2 run can_powerpack can_bridge_node \
     --ros-args --namespace pack2 --params-file config/powerpack_config.yaml
```

`run_powerpack.sh` 는 pp_controller 를 같이 띄우므로 **쓰지 않는다.**

### 사전 확인

```bash
ros2 topic hz /pack2/board/sensors        # 수신 있는지
ros2 topic hz /pack2/board/currents       # 실측 전류 — 모델 입력으로 쓴다
python3 scripts/can_monitor.py            # 압력이 정상 범위인지 눈으로
```

`board/sensors`·`board/currents` 는 **mV** 다 (kPa 아님). 전류는 `mA = mV/10`.

---

## 1. 파트 A — 밸브 36개

### 전제

- **펌프를 분리하고 외부 일정압 공급원을 연결한다.** 제어할 때는 제거한다.
- 라인압은 스크립트가 제어하지 않는다. 외부 레귤레이터로 유지하고 스크립트는 **감시만** 한다.
- 라인 밸브(board1 v1, board2 v1)는 반전 블리드형이라 스크립트가 0(폐쇄)으로 고정한다 →
  펌프를 돌린 상태로 쓰면 안 된다.
- MacroSwitch(idx 9)도 0 으로 둔다.

### 4 모드 — 한 번 실행 = 한 모드 = 6 채널

| `--mode` | 채널 | 외부 압력 인가 | 피팅되는 밸브 쌍 |
|---|---|---|---|
| `pos_micro` | gid 0–5 | board 1 (예 250 kPa abs) | v1(레일→챔버) + v2(챔버→대기) |
| `pos_macro` | gid 0–5 | board 3 (예 700 kPa abs) | v3(탱크→챔버) + v2 |
| `neg_micro` | gid 6–11 | board 2 (예 30 kPa abs) | v1(챔버→음압레일) + v2(대기→챔버) |
| `neg_macro` | gid 6–11 | board 4 (외부 진공) | v3(챔버→외부진공) + v2 |

**v2 는 네 모드에 모두 나온다** → 독립 4회 피팅이 되어 서로 교차검증된다. 불일치하면 리포트에
경고가 뜬다.

### 채널마다 손이 할 일

실행하면 채널마다 멈추고 이렇게 지시한다:

```
[1/6] gid 0 → board 5
  1. 외부 압력 인가 부품을 board 1 라인에 연결 (250 kPa abs)
  2. 고정부피 탱크를 board 5 채널로 옮길 것
  준비되면 Enter (s=건너뛰기, q=종료):
```

배관을 옮기고 Enter. 끝나면 `다음으로? (Enter=진행, r=이 채널 재실행, q=종료)` 가 뜬다.
결과가 이상하면 **`r` 로 그 채널만 다시** 돌릴 수 있다.

### 실행 — 이중 부피법이라 모드마다 2회차

```bash
# 1회차: 고정부피 탱크만
python3 scripts/valve_fit_record.py --mode pos_micro --line-kpa 250 --out results_fit/pos_micro

# 2회차: 알려진 부피(예 100 mL)를 티로 추가. **같은 --out 디렉터리에** 넣는다
python3 scripts/valve_fit_record.py --mode pos_micro --line-kpa 250 \
        --extra-volume-ml 100 --out results_fit/pos_micro

# 피팅
python3 scripts/valve_fit_solve.py results_fit/pos_micro --extra-volume-ml 100
```

> **같은 `--out` 에 넣어야 한다.** 솔버가 그 디렉터리의 `*.csv` 를 전부 읽어
> `extra_volume_ml` 값으로 1·2회차를 짝지운다. 따로 두면 부피를 못 푼다.

부피를 따로 실측했다면 2회차를 생략하고 `--volume-ml <mL>` 로 직접 준다.

네 모드를 같은 방식으로 반복한다. `--gids 0` 처럼 채널을 지정해 **한 채널만 먼저**
돌려보는 것을 권한다 (아래 4절).

### 왜 이중 부피법이 필요한가

유량계가 없다. `Q` 를 챔버압 미분으로 합성하면 지배식에 `A_max/V` 곱만 나타나 **`A_max` 와
챔버 부피 `V` 가 분리되지 않는다.** `V` 는 각도에 따라 변하므로 이걸 방치하면 다른 각도에서
모델이 틀린다. 알려진 ΔV 를 더한 2회차의 **통과시간 비**가 `(V+ΔV)/V` 이므로 절대 스케일이
고정된다.

### 주요 인자

| 인자 | 기본 | 뜻 |
|---|---|---|
| `--mode` | (필수) | 위 4종 |
| `--line-kpa` | (필수) | 인가한 라인압 [kPa abs]. 감시 기준값 |
| `--gids` | 모드 전체 | 채널 지정 (예 `--gids 0 1`) |
| `--extra-volume-ml` | 0 | 이중 부피법 2회차 태깅 |
| `--levels` | 17단 | 스윕 레벨 [%]. 크래킹 구간(45–65%)이 촘촘하다 |
| `--trip-hi-kpa` | 190 | 과압 트립 [kPa abs] |
| `--charge-level` | 75 | v2 피팅 전 충전 개도 [%] |
| `--stop-lead` | 0.10 | 예측 정지 리드 [s] |
| `--dry-run` | — | 밸브를 구동하지 않고 시퀀스만 확인 |

---

## 2. 파트 B — 펌프

### 전제 — 안전 상태가 밸브와 정반대다

밸브 실험은 "v2 개방"이 안전 상태지만, 펌프 실험은 **양 밸브 전개**가 안전 상태다.
펌프가 도는 동안 릴리프(board1 v1)를 닫으면 양압 레일이 무한정 올라간다.
**"전 밸브 0" 은 위험하다.** 종료·예외·Ctrl-C 모두 양 밸브를 열어 두 레일을 대기압으로
되돌린 뒤 0 으로 간다.

펌프 on/off 는 수동 스위치다. 스크립트가 프롬프트로 요청한다.

### 3 단계

| `--phase` | 펌프 | 내용 |
|---|---|---|
| `leak` | **OFF** | 양 밸브 폐쇄 감쇠 → 지수 시상수 τ. ΔV 회차의 τ 비가 `(V+ΔV)/V` → 부피와 누설이 함께 풀린다 (펌프 무관) |
| `map` | ON | (relief, admit) 격자에서 정착 → 양 밸브 짧게 폐쇄 → 두 레일 dP/dt → `ṁ_pump`. 두 레일이 독립 산출이라 **질량보존 교차검증**이 공짜로 된다 |
| `frontier` | ON | admit 으로 P⁻ 를 목표에 잡고 relief 를 닫아 P⁺ 를 스톨까지 램프 → **능력경계 직접 측정** |

### 실행 — 세 단계 결과를 같은 디렉터리에

```bash
D=results_pump/run1

# Phase L — 3회차. ①맨몸 ②ΔV→양압 ③ΔV→음압 (탱크가 하나뿐이라 옮겨 달아야 한다)
python3 scripts/pump_fit_record.py --phase leak --out $D
python3 scripts/pump_fit_record.py --phase leak --extra-volume-ml 250 --extra-volume-rail pos --out $D
python3 scripts/pump_fit_record.py --phase leak --extra-volume-ml 250 --extra-volume-rail neg --out $D

python3 scripts/pump_fit_record.py --phase map --out $D
python3 scripts/pump_fit_record.py --phase frontier --ppos-ceiling 500 --out $D

python3 scripts/pump_fit_solve.py $D --crank-m <실측 크랭크 반경 [m]>
```

`--crank-m` 은 **필수**다. 소기량과 곱으로만 나타나 피팅으로 갈리지 않으므로 실측값으로
고정한다. 대략(±10%)이어도 된다 — 소기량이 그만큼 반대로 흡수한다.

### 프롬프트 순서

```
0점 보정은 **펌프를 끈 상태**에서 해야 한다.
  펌프를 끄고 Enter (s=0점 보정 건너뛰기, q=종료):     ← 펌프 OFF 하고 Enter
── Phase L ──
  펌프를 켜고 Enter (q=종료):                          ← 펌프 ON
  이제 **펌프를 끄고** Enter (q=종료):                  ← 펌프 OFF (감쇠 기록 시작)
```

`s` 를 누르면 0점 보정을 건너뛰고 yaml offset 을 쓴다.

### 첫 회차에서 반드시 눈으로 볼 것 — Phase L 의 τ 와 R²

**이것만 리허설로 검증되지 않았다.** 시뮬 펌프를 끌 수 없어 레일이 감쇠하지 않기 때문이다.

τ 가 부피의 **절대 스케일**을 정하므로 품질 게이트를 넣어 뒀다:

```
pos 레일 (ΔV→none): τ=14.32 s  R²=0.9871  n=8009  ΔP=142.6 kPa     ← 이렇게 나와야 한다
pos 레일 (ΔV→none): 기각 — R²=0.31 < 0.90 — 1차 감쇠 모델이 안 맞는다
pos 레일 (ΔV→none): 기각 — 기록 구간 압력 변화 0.83 < 3.0 kPa — 감쇠가 없다
```

- **기각되면 사유가 나온다.** 감쇠가 없다 → 펌프가 정말 꺼졌는지, 밸브가 정말 닫혔는지 확인.
- τ 가 분 단위로 길면(실기가 시뮬보다 조밀한 경우) `--leak-seconds` 를 늘린다.
  게이트가 τ 를 기록 길이의 0.02–50 배로 요구하므로, **기록을 τ 의 0.1–5 배**로 잡는 게 좋다.
- 게이트를 낮추지 말 것. 통과하지 못한 τ 로 진행하면 부피·누설·맵·경계가 전부 함께 틀어진다.
- 정말 안 되면 부피를 실측해서 `--volume-pos-ml / --volume-neg-ml` 로 넘긴다. 그때는 누설이
  미측정이므로 솔버가 경고하고, yaml 에서 누설 키를 **빼고** 헤더에 목록으로 남긴다.

### Phase F 에서 볼 것

```
P⁺= 587.3 (+486.0 gauge)  P⁻=  21.3 kPa  → 스톨
P⁺= 604.6 (+503.3 gauge)  P⁻=  36.3 kPa  → 하한 경계(상한 도달)
```

`스톨` 이면 진짜 능력경계 점이다. `하한 경계(상한 도달)` 는 안전 상한에 먼저 닿은 것으로,
"경계가 최소 이 값 이상"이라는 정보다 (모델 피팅에서 한쪽 벌점만 받는다).

```
!! 상한 초과 25.3 kPa — 트립 여유 40 의 절반을 넘었다. --stop-lead 를 0.24 로 올리거나 ...
```

이 경고가 나오면 `--stop-lead` 를 올린다. 초과는 램프율에 비례하므로(≈0.05 s × 램프율)
실기 레일 부피가 작으면 커진다. 리허설에서는 190 kPa/s 에서 3–8 kPa 였다.

### 실용 판정 — 어디까지 정확해야 하나

컨트롤러가 펌프를 쓰는 곳은 **`cap_ppos` 하나**뿐이다(`PressureRefGen.cpp:79`).
생성기의 `pos_sp_max_kpa` 가 250 kPa gauge 이므로 **능력경계가 250 위/아래인지만 갈라도**
컨트롤러 동작에는 충분하다. 250 위면 `cap_ppos` 는 상시 느슨해 영향이 없다.

---

## 3. 결과를 컨트롤러에 반영하기

솔버가 만드는 파일을 `config/` 로 복사하고 다시 빌드한다.

```bash
cp results_fit/*/valve_params.yaml  config/valve_params.yaml
cp results_pump/*/pump_params.yaml  config/pump_params.yaml
cd ~/claude_tutorial && colcon build --packages-select can_powerpack
```

`control.launch.py` / `virtual.launch.py` 가 이 두 파일이 있으면 `powerpack_config.yaml`
**뒤에** 병합해서 덮어쓴다 (`control.launch.py:12-15`). 없으면 그냥 무시된다.

### 신뢰도 순서 — `pump_params.yaml`

1. `PressureRefGen.pump_frontier_measured` — 직접 측정. **컨트롤러가 쓰는 유일한 출력.**
2. `Virtual.pump_map_measured` — 직접 측정한 (P⁺,P⁻,ṁ). 시뮬 `flow_out` 용.
3. `pump: {...}` — 기하 피팅 산물. 소기량 × `Cb_in` 축퇴가 남아 개별 값은 배수로 틀릴 수 있다
   (자기검증: 능력경계 12–16% 오차). **참고용.**

### 밸브별 값 로딩 — 이제 된다

`channel_config.chN.{micro,atm,macro}.*` 3세트를 읽고, 없으면 평면 `chN.*` 로 폴백한다.
피드포워드 역모델·크래킹 임계·Bouc-Wen·MPPI 롤아웃·QP 선형화가 **전부 밸브별 값을 쓴다.**

반영 확인은 기동 로그 한 줄로 한다:
```
밸브별 13-parameter: 12/12 채널이 chN.{micro,atm,macro}.* 를 로드했다
```
`0/12` 면 병합이 안 된 것이다 (파일 위치·재빌드 확인). 피팅 전이면 `0/N` 이 정상이다.

`chamber_volume_ml` 도 함께 로드해 로그에 찍는다 — 지금은 진단용이고, 실제 부피는
액추에이터 각도에서 계산된다.

---

## 4. 권장 진행 순서 (위험 낮은 것부터)

1. `python3 scripts/valve_fit_selftest.py` / `pump_fit_selftest.py` — 코드가 안 바뀌었으면
   생략 가능. 스크립트를 손댔다면 **반드시** 통과시킨다.
2. **`--dry-run` 으로 한 번** — 토픽·인덱스·시퀀스 확인. 밸브를 구동하지 않는다.
3. **밸브 1채널 1모드** (`--mode pos_micro --gids 0`). 안전 트립을 의도적으로 유발해
   배기 밸브가 실제로 열리는지 먼저 확인할 것.
4. 그 CSV 로 `valve_fit_solve.py` 를 돌려 R² 와 크래킹 임계가 말이 되는지 본다.
   → **여기까지 통과하면** 6채널 → 4모드로 확대.
5. 펌프: Phase L(펌프 off) → M → F. Phase L 은 τ·R² 를 눈으로 확인하고 넘어간다.
6. 피팅 결과를 `VirtualPowerpack` 에 넣고 같은 실험을 재현해 실측 궤적과 겹치는지 본다.
   이게 피팅의 최종 수용 기준이다.
7. **제어기 실험은 4.5절**로 간다 — 피팅 → QP → 채널 MPPI → 중앙집중 MPPI,
   무액추에이터 1축→6축 → 액추에이터 1축→6축.

### 시뮬로 예행연습이 필요하면

```bash
# 파라미터 평탄화 (launch 가 하는 일을 수동으로) 후
ros2 run can_powerpack virtual_powerpack --ros-args \
     -r __node:=can_bridge -r __ns:=/pack2 --params-file /tmp/pump_virt.yaml
```

`virtual_powerpack` 은 `board/sensors`·`currents`·`pwm_cmd` 를 실기와 동일 형식으로 낸다.
`can_bridge` 로 리맵해야 스크립트가 찾는다.

---

## 4.5 실기 제어기 실험 — 순서와 스위치

**내일 진행할 순서다.** 피팅 → QP → 채널 MPPI → 중앙집중 MPPI, 각각 무액추에이터
1축→6축 → 액추에이터 1축→6축.

### 4.5.0 켜고 끄는 것 — 스위치 한 장 정리

전부 `ros2 launch` 인자로 줄 수 있다. **yaml 을 고치지 않아도 되고, 같은 빌드로 비교할 수
있다** (빌드를 바꿔 비교하면 원인 분리가 안 된다).

```bash
ros2 launch can_powerpack control.launch.py \
     solver:=qp|mppi|mppi_system \
     num_actuators:=1..6 \
     actuator_connected:=false|true \
     overrides:=MPC_parameters.sys_samples=128,MPC_parameters.mppi_estimator=true
```

| 인자 | 뜻 |
|---|---|
| `solver` | `qp` = 선형화 + 응축 qpOASES (예전 것) · `mppi` = 채널별 MPPI (**기본**) · `mppi_system` = 전체 시스템 중앙집중 |
| `num_actuators` | **축(=채널쌍) 수.** 1 이면 양압 gid 0 + 음압 gid 6 만 활성. 나머지 채널은 목표가 대기압이라 밸브가 닫힌 채 있다 |
| `actuator_connected` | `false` = 고정 탱크 부피(50 mL), 축 각도 동역학 없음 · `true` = 실제 부피/각도 |
| `overrides` | 그 밖의 파라미터 |

`control_mode` 는 **2 로 둔다**(yaml 기본). 무액추에이터에서도 mode 2 가 맞다 — 그때는
TorquePID 가 비활성이고 `τ_ref = m·g·L·sin(angle_ref)` 만 남아 **목표각이 곧 일정한 힘
지령**이 된다. 그러면 생성기가 그 힘을 양압/음압 챔버로 나눠 전 채널에 `P_ref` 를 주므로
압력 캐스케이드 전체를 축이 움직이지 않는 상태로 시험할 수 있다.

목표는 TCP 로 준다 (축 개수만큼):
```bash
python3 src/can_powerpack/scripts/position_ref_client.py 127.0.0.1 2293 --once 45
python3 src/can_powerpack/scripts/position_ref_client.py 127.0.0.1 2293 --once 45 45 45 45 45 45
```

### 4.5.1 0단계 — 피팅부터 (위 1·2절)

1절(밸브 4모드 × 채널)과 2절(펌프 3단계)을 그대로 돌린다. 그 다음 **결과를 반영**한다:

```bash
cp results_fit/*/valve_params.yaml  src/can_powerpack/config/valve_params.yaml
cp results_pump/*/pump_params.yaml  src/can_powerpack/config/pump_params.yaml
colcon build --packages-select can_powerpack --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**반영됐는지 기동 로그로 반드시 확인한다** — 이게 유일한 확인 수단이다:

```
밸브별 13-parameter: 12/12 채널이 chN.{micro,atm,macro}.* 를 로드했다
  ch0 피팅 챔버 부피 128.50 mL
```

`0/12` 가 나오면 병합이 안 된 것이다 (파일 위치·재빌드 확인). 피팅 전이라면 `0/N` 이
정상이고 하드코딩 기본값으로 돈다.

> 이번에 로더를 만들었다. 예전에는 채널당 13-parameter 세트가 하나뿐이어서 micro/atm/macro
> 에 같은 값을 썼고, **피팅 결과를 쓸 수 없었다.** 이제 피드포워드 역모델·크래킹 임계·
> Bouc-Wen·MPPI 롤아웃·QP 선형화가 전부 밸브별 값을 쓴다.

펌프 Phase L 이 구한 레일 부피·누설은 `pump_params.yaml` 이 **컨트롤러 쪽에도**
(`MPC_parameters.rail_volume_*` / `rail_leak_*`) 쓴다 — 중앙집중 MPPI 가 레일을 예측하는 데
필요하다.

### 4.5.2 1단계 — 무액추에이터, 1축 → 6축

목적: 압력 캐스케이드만 검증한다. 축이 움직이지 않으므로 부피가 고정이고 위험이 가장 낮다.

```bash
# 축 1개, 예전 QP 부터
ros2 launch can_powerpack control.launch.py \
     solver:=qp num_actuators:=1 actuator_connected:=false
python3 src/can_powerpack/scripts/position_ref_client.py 127.0.0.1 2293 --once 30
# 안정되면 45, 그다음 다시 20 등으로 몇 번 왕복
```

그 축 1개에서 **세 솔버를 차례로** 돌려 비교한다 (`solver:=` 만 바꾼다):
`qp` → `mppi` → `mppi_system`.

문제가 없으면 `num_actuators` 를 **1 → 2 → 3 → 6** 으로 올리며 각 단계에서 세 솔버를 반복한다.
축을 늘릴 때마다 레일 부하가 커지므로 **레일 압력을 반드시 함께 본다**(아래 4.5.4).

### 4.5.3 2단계 — 액추에이터 연결, 1축 → 6축

`actuator_connected:=true` 로 바꾸고 같은 순서를 반복한다. 이때부터 **축이 실제로 움직인다.**

- 각도 목표는 작게 시작한다 (예 10° → 20° → 45°).
- 엔코더 캘리브레이션이 **board 20·21·22 만 있고 나머지가 없다**(README 0절). 축을 6개까지
  늘리려면 그 전에 2점 캘리브레이션을 해야 한다. 안 하면 각도가 틀린 상태로 위치 제어를
  하게 된다 — 1단계(무액추에이터)는 각도를 힘 지령으로만 쓰므로 영향이 적지만
  2단계는 직접 영향을 받는다.

### 4.5.4 무엇을 보면서 돌리나

기동 직후 확인할 로그:

| 로그 | 정상 | 벗어나면 |
|---|---|---|
| `밸브별 13-parameter: N/M` | 피팅 후 `12/12` | `0/12` = 병합 실패 |
| `MPC 솔버 = ...` | 의도한 솔버 | 인자 오타 |
| `PWM 워치독: 200 ms` | 켜져 있어야 한다 | `0` 이면 안전장치 없음 |
| `펌프 테이블 완료 ... 능력경계` | 피팅값 반영 확인 | — |

돌면서 10 초마다 나오는 것:

| 로그 | 정상 | 벗어나면 |
|---|---|---|
| `틱 간격이 가정과 다르다` | **안 나와야 한다** | 나오면 실제 주기가 `period_ms` 와 다르다 — 컨트롤러는 dt 를 항상 `period_ms` 로 쓰므로 그만큼 모델 오차다. `period_ms` 를 실측에 맞춘다 |
| `MPPI: ... us 최대` | 틱 예산의 60% 이하 | 넘으면 `mppi_substeps`→1, 그다음 `mppi_samples` 를 내린다 |
| `중앙집중 MPPI 데드라인: N회 초과` | 0 | 나오면 `sys_samples` 를 줄이거나 `period_ms` 를 올린다 (250 Hz 도 무방) |
| `레일 예측오차` | < 2 kPa | 크면 레일 부피·누설 피팅값을 의심 |
| `관측기: 평균 \|잔차\|` | < 2 kPa | 크면 모델이 플랜트를 못 따라간다 |
| `[SAFETY] chN ... latched` | 안 나와야 한다 | 과압 래치. 목표를 낮추고 원인 확인 |
| `비유한 값이다` | 절대 안 나와야 한다 | 즉시 중단하고 보고할 것 |

### 4.5.5 안전 — 실기에서 바뀐 것

- **PWM 워치독 신설.** `board/pwm_cmd` 가 200 ms 이상 끊기면 브리지가 **채널 밸브를 닫고
  라인 밸브 2개를 전개**한다. 예전에는 `targets_` 가 영구 래치라 컨트롤러가 죽으면
  마지막 PWM 이 계속 나갔다. 그리고 **"전부 0" 은 안전 상태가 아니다** — board1 v1 이 0 이면
  릴리프가 닫혀 펌프가 양압 레일을 무한정 올린다.
- **비유한 값 차단.** 솔버가 NaN/Inf 를 내면 해당 밸브를 0 으로 떨어뜨리고 한 번 경고한다.
  예전에는 uint16 변환이 정의되지 않아 임의의 PWM 이 나갈 수 있었다.
- **중앙집중 솔버 데드라인.** 한 번의 solve 가 예산(기본 틱의 60%)을 넘기면 다음 틱들을
  건너뛰고 Δu=0(순수 피드포워드)로 간다. MPPI 는 워밍 스타트 기반이라 한 틱 쉬어도 이어서
  개선한다. 제어 루프가 멈추는 것보다 낫다.
- 기존 과압 보호(채널별 히스테리시스 래치)와 5초 밸브 잠금은 그대로다.
- **비상 정지**: `Ctrl-C` 로 컨트롤러를 끄면 워치독이 200 ms 뒤 안전 상태로 간다.
  즉시 멈추려면 펌프 전원을 끄는 것이 가장 확실하다.

### 4.5.6 솔버별 알아둘 것

| | 상태 | 실기에서 주의 |
|---|---|---|
| `qp` | 예전 기준선. 초킹 구간에서 `A_scalar=0` 이라 Δu≈0 — 사실상 피드포워드+적분항이다 | 가장 보수적이라 **먼저 돌릴 것**. 문제가 나면 배관·피팅·센서 쪽 문제다. 시뮬 IAE 분산이 넓다(12.8~20.1) — Δu 가 없어 하네스 타이밍에 좌우되기 때문이다. 실기에서도 반복 측정할 것 |
| `mppi` | **기본. 시뮬에서 가장 좋다** (QP 대비 IAE −62%, 정착 −54%) | 실시간 여유 큼(채널당 0.3 ms). 가장 먼저 시도할 개선 후보 |
| `mppi_system` | 구현·모델 검증됨(레일 예측 0.3 kPa)이나 시뮬 성능은 채널별보다 나쁘고 **타이밍 스파이크 미해결** | 기본 `sys_samples: 128` (256 보다 측정상 더 좋고 싸다). 데드라인이 루프를 지킨다 —
계측상 초과 49회 / 건너뜀 75틱(0.75%)이었다. 라인 밸브는 기본적으로 MPPI 가 잡지 않는다(`sys_control_lines: false`) — 잡게 하면 시뮬에서 레일이 불안정했다 |

실기 데이터가 시뮬과 다를 수 있으므로 세 개를 다 돌려 보는 것이 목적이다. 비교는
같은 축 수·같은 목표 궤적에서 하고, 하네스가 비결정론적이니 **한 번으로 판정하지 말 것.**

---

## 5. 코드 안내

8개 파일이고, `*_model.py` 는 ROS 무관 라이브러리, `*_record.py` 는 rclpy 노드,
`*_solve.py` 는 오프라인 피터, `*_selftest.py` 는 하드웨어 없는 검증이다.

```
        [실기]                    [CSV]                  [산출물]
valve_fit_record.py  ──▶  results_fit/*.csv  ──▶  valve_fit_solve.py  ──▶  valve_params.yaml
                                                        │                   report.md
                                                        └── valve_fit_model.py (모델·솔버)

pump_fit_record.py   ──▶  results_pump/*.csv ──▶  pump_fit_solve.py   ──▶  pump_params.yaml
                                                        │                   pump_report.md
                                                        └── pump_fit_model.py (펌프·솔버)
```

### 밸브

| 파일 | 역할 |
|---|---|
| `valve_fit_model.py` | 13-parameter 모델(`sigmoid_pow`·`phi`·`fold_euler`·Bouc-Wen), SG 미분 `dpdt`, `q_from_dpdt`, Nelder-Mead, 난수 탐색. **다른 셋이 여기서 import 한다** |
| `valve_fit_record.py` | 4 모드 시퀀스, 48개 shadow-array 발행, 채널별 수동 게이트, 과압/과진공 트립, 예측 정지·제동, 0점 보정 검증, 200 Hz CSV |
| `valve_fit_solve.py` | 통과시간법 부피 → 밸브별 13개 피팅 → 오리피스 계수(B절) → yaml·리포트·플롯 |
| `valve_fit_selftest.py` | 알려진 파라미터로 실험을 합성해 복원 확인. 판정은 **부피·유량 R²·크래킹 임계** |

`fold_euler` 는 MATLAB 원본의 20 Euler sub-step 을 2×2 선형 사상으로 접은 것이다
(수치적으로 동일, 8.5e-14 오차). 목적함수 1회가 ~1.5 s → 실용 속도가 됐다.

모델 입력 `I` 는 지령이 아니라 **실측 전류**(`board/currents`)다 — 드라이버 동특성이
피팅에서 빠진다. CSV 는 태그와 무관하게 `u_v1..u_v3`, `I_v1..I_v3` 를 매 행 기록한다
(reset/charge 구간에서 다른 밸브 전류가 모델에 섞이는 것을 막는다).

### 펌프

| 파일 | 역할 |
|---|---|
| `pump_fit_model.py` | `PumpGeom`, `pump_avg`(슬라이더-크랭크 1주기 평균, 동작점 벡터화), `frontier`/`cap_ppos`, `PumpMap`(17×17 이중선형), `mdot_from_rail`, `exp_decay_fit`, `volume_from_tau`, `fit_project`, `fit` |
| `pump_fit_record.py` | 인덱스 0·3 만 구동(나머지 46개는 0 발행), Phase L/M/F, 펌프 on/off 프롬프트, **양 밸브 전개 = 안전 상태**, 예측 정지·즉시 제동 |
| `pump_fit_solve.py` | τ → 이중 부피법 → 슬라이딩 창 맵 + 질량보존 잔차 → 능력경계 → 기하 피팅 → yaml·리포트·플롯. RPM 은 리플 FFT best-effort |
| `pump_fit_selftest.py` | 레일 ODE + 라인 밸브 + 누설 + 센서 양자화/LPF 로 3단계를 합성해 복원 확인 |

`pump_fit_model.py` 는 `valve_fit_model.py` 에서 `nelder_mead`·`random_search`·`phi`·`dpdt`
등을 import 해 재사용한다.

**재매개화**: 기하 8개가 아니라 `(V_swept, V_dead, A_out, A_in, l/r)` 로 푼다. 유량은 소기량에,
데드헤드는 압축비(`V_dead`)에 걸려 물리적으로 분리돼 있다. 크기 파라미터는 **로그 공간**에서
최적화한다 — 예전 펌프 값과 자릿수가 다를 수 있어 곱셈 노이즈로는 도달하지 못한다.

**핵심 아이디어 — 라인 밸브 모델이 필요 없다.** 레일은 대기 창구가 정확히 둘뿐인 닫힌 회로다
(board1 v1, board2 v1). 둘 다 닫으면 밸브 유량이 0 이 되고 부피만으로 펌프 유량이 나온다:

```
ṁ_pump = V⁺/(R·T)·dP⁺/dt + leak⁺(P⁺) = −V⁻/(R·T)·dP⁻/dt + leak⁻(P⁻)
```

두 식이 질량보존으로 같아야 하므로 **매 측정점에서 공짜 교차검증**이 된다. 밸브는 동작점을
옮기는 데만 쓰고, 측정은 "닫고 순간 기울기"로 한다.

### 알아 둘 것

- **`scipy` 가 없다** (numpy 1.17.4 / matplotlib 3.1.2 / Python 3.8.10). `fminsearch` 동등
  Nelder-Mead 를 직접 구현했다. 새 의존성 없음이 이 repo 관례다.
- **`LPM_TO_KGPS` 가 두 개 있고 10.7배 다르다** — 경험 상수 `2.155e-4` 와 표준
  `1.204/60/1000`. 리포트가 어긋나면 이 이중성을 먼저 의심할 것.
- `valve_char.py` / `valve_char2.py` 의 하드코딩 `VP` 리스트는 **구버전 피팅값**이다.
  C++/MATLAB 기본값과 다르므로 참조하지 말 것.
- `board/kpa_all` 은 **존재하지 않는다.** kPa 절대압은 `controller/sensors_kpa`(25개)다.
  단 그건 pp_controller 가 내므로 피팅 중에는 없다 — 스크립트가 직접 mV→kPa 환산한다.

---

## 6. 문제가 생기면

| 증상 | 원인·조치 |
|---|---|
| `pp_controller 가 떠 있다` 로 중단 | pp_controller 를 종료. `valve_operate: false` 로는 안 된다 |
| `0점 보정값이 yaml 과 너무 다르다` | 압력이 남아 있다. 대기압 복귀 후 재시작. 펌프 실험은 **펌프를 끄고** 해야 한다. 급하면 `--no-zero` |
| 펌프 Phase L 에서 `감쇠가 없다` | 펌프가 안 꺼졌거나 밸브가 안 닫혔다 |
| 펌프 Phase L 에서 `R² < 0.90` | 누설이 ΔP 에 비선형이거나 창이 짧다. `--leak-seconds` 를 늘려볼 것. 게이트를 낮추지 말 것 |
| Phase F 상한 초과 경고 | `--stop-lead` 를 올린다 |
| 밸브 트립이 잦다 | `--line-kpa` 를 낮추거나 `--charge-level` 을 낮춘다. 라인압 250 kPa 는 채널 정격 185 보다 높다 |
| 밸브 R² 가 낮다 | dP/dt 창(SG window 9)이 그 밸브의 과도(≈46 ms)에 비해 과한지 본다 |
| 기하 피팅이 이상하다 | 정상이다 — 참고용이다. 능력경계는 `pump_frontier_measured` 를 쓴다 |

### 판정 기준 — 개별 파라미터를 믿지 말 것

두 모델 다 과매개화돼 있다. 밸브 13-parameter 는 `(A_max, k_shape, C_k, alpha_shape)` 가
거의 자유롭게 상쇄되는 평평한 다양체를 갖고, 펌프는 소기량 × `Cb_in` 이 곱으로만 갈린다.
**소비 측이 실제로 쓰는 것으로 판정한다:**

| | 판정 기준 | 자기검증 결과 |
|---|---|---|
| 밸브 | 챔버 부피 · 예측 유량 R² · **크래킹 임계** | 2.9% / 0.94–0.97 / ±3%p |
| 펌프 | 레일 부피·누설 · 측정 맵 · **측정 능력경계** | 0.0–0.1% / 0.7% / 1.3% |

크래킹 임계는 상류압에 따라 변한다 (`C_p·Pin` 항):
Pin 110/155/250/351/700 kPa abs → **62.0 / 60.2 / 56.4 / 52.3 / 38.4 %**.
실측 ~50% 와 일치한다. 피팅 결과가 이 범위를 크게 벗어나면 의심할 것.
