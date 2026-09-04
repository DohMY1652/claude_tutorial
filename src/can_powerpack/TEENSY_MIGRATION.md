# 엔코더 CAN → Teensy 이관 + 전 제어 주기 200 Hz

20260903 작성, 20260904 에 2점 보정값 반영. 코드·보정 모두 실기 전원이 꺼진
상태에서 넣었다 — **실기 확인은 아직 한 번도 안 했다.**
아래 순서대로 하면 문제를 층별로 가를 수 있다.

---

## 아침에 하는 순서

### 0단계 — 빌드와 오프라인 테스트 (전원 없이도 된다)

```bash
cd ~/claude_tutorial
colcon build --packages-select can_powerpack --cmake-args -DCMAKE_BUILD_TYPE=Release
./build/can_powerpack/test_teensy_frame src/can_powerpack/test/data/teensy_5s.bin
```
`전부 통과` 가 나와야 한다. 실기 캡처 1000 프레임으로 파서를 돌리는 테스트다.

### 1단계 — 전원만 넣고 값이 들어오나

**아무것도 띄우지 말고:**
```bash
python3 src/can_powerpack/scripts/pp_check.py
```

봐야 할 것:
- **CAN 보드 1~16 이 전부 500 Hz** — 하나라도 빠지면 배선·전원·펌웨어
- **Teensy 200 Hz, 유실 0, CRC오류 0, status 0x0000**
- **축을 손으로 움직여 raw 가 따라 도는가** ← 배선 확인은 이것뿐이다
- **raw 가 7000~28400 범위인가.** 20260903 캡처(`test/data/teensy_5s.bin`)는
  raw 가 65~600 이었는데, 20260904 실측은 0° 에서 27220~28390 이다. 그때는 센서
  여자가 없었던 것으로 보인다. **raw 가 1000 아래면 센서 전원을 의심할 것.**
  (그 캡처는 프레임 형식 검증용이라 값이 달라도 테스트는 유효하다.)

화면 맨 아래 `[OK]/[주의]/[실패]` 판정 줄을 보면 된다.

### 2단계 — 2점 보정 (**20260904 에 값이 들어갔다.** 다시 잡을 때만)

현재 값 (0° 는 축별 실측, 90° 는 전 축 공통 0° − 20170):

| 채널 | 축 | raw @0° | raw @90° |
|---|---|---|---|
| ch0 | 1축 | 27220 | 7050 |
| ch1 | 2축 | 27330 | 7160 |
| ch2 | 3축 | 27500 | 7330 |
| ch3 | 4축 | 28060 | 7890 |
| ch4 | 5축 | 28850 | 8680 |
| ch5 | 6축 | 28775 | 8605 |

기울기는 6축 모두 −224.1 count/deg (90° 폭이 20170 count 로 공통).
풀스케일 32767 대비 0° 가 83~88 %,
90° 가 22~26 % 라 여유가 있다.
(20260904 2차 실측 — ch1·ch4·ch5 를 다시 쟀다.)

> **⚠ 죽은 센서는 각도만 봐서는 안 보인다.** 이 보정에서 raw=0 (전원 없음·단선)은
> **121~129°** 로 나오는데, 기계적 가동범위 120° 안이라 정상처럼 보인다.
> 구별되는 것은 **잡음**뿐이다 — 살아 있는 채널은 σ 1.4~2.4 LSB, 죽으면 ~0 이다.
> `pp_check.py` 가 σ<0.3 인 채널을 `[실패]` 로 잡는다. **그 줄을 반드시 볼 것.**

다시 잡으려면:

```bash
python3 src/can_powerpack/scripts/pp_check.py --calib
```
축을 0° 에 놓고 `0`, 90° 에 놓고 `9`, 그다음 `p` 를 누르면
`config/powerpack_config.yaml` 에 붙여 넣을 줄을 그대로 출력한다.
`/pack2/can_bridge` → `TeensyEncoder.channels` 에 넣고 다시 빌드.

> 보정이 **없는** 채널은 각도가 항상 0° 로 나가고 기동 시 ERROR 가 뜬다.
> raw 를 그대로 도 단위로 흘리면 "0° 에 있다"는 거짓말이 되어 위치 제어가
> 그걸 그대로 믿기 때문이다. 지금은 6채널 모두 들어가 있다.

넣은 뒤 다시 1단계를 돌려 **두 자세에서 각도가 실제와 맞는지 눈으로** 확인.

### 3단계 — 200 Hz 로 쏴도 통신이 버티나

```bash
python3 src/can_powerpack/scripts/pp_check.py --tx 200 --seconds 20
```
페이로드는 전부 0 이라 밸브는 안 움직인다.

- 보드 수신이 500 → **약 260 Hz** 로 떨어지는 게 예상값이다
  (지령 프레임 1개당 보드 프레임 ~9.5개가 사라진다는 실측에서 나온 수치).
  제어 200 Hz 보다 높으니 매 틱 새 값을 본다.
- **Teensy 는 200 Hz 를 그대로 유지해야 한다.** 여기서 같이 떨어지면
  CAN 문제가 아니라 USB/CPU 문제다.

### 4단계 — 제어기를 켜고

```bash
ros2 launch can_powerpack control.launch.py        # 창 1
python3 src/can_powerpack/scripts/pp_check.py      # 창 2
```
브리지가 떠 있으면 pp_check 가 **자동으로 ROS 모드**로 바뀐다 (시리얼·CAN 송신을
건드리지 않는다). 확인:
- `board/sensors` **200 Hz** = 제어 루프 주기
- `board/analog` **200 Hz**
- `board/pwm_cmd` **200 Hz**
- CAN 보드 수신이 260 Hz 근처에서 **고른가** (특정 보드만 낮으면 우선순위 기아)

브리지 로그의 진단 줄도 5 초마다 나온다:
```
CAN 수신 [1:262Hz 2:261Hz ...]
Teensy 엔코더: 200 Hz  유실 0  CRC오류 0  status 0x0000  raw[...]
```

### 5단계 — 예전 제어 재현

여기까지 통과하면 이전 실험을 그대로 돌린다.
**펌프 켜기 전에 macro 배관 상태를 확인할 것.**

---

## 무엇이 바뀌었나

### 엔코더가 CAN 에서 빠졌다

**왜.** CAN 실측에서 지령 프레임 1개당 보드 프레임 약 9.5개가 사라진다
(보드가 수신 처리로 ~1.2 ms 멈춘다). 비용은 바이트가 아니라 **프레임당**이라
페이로드를 줄여도 안 듣는다 (64+48B ≡ 8+8B, 랜덤값 ≡ 0 고정으로 확인).
그런데 엔코더 보드는 CAN ID 가 가장 높아(0x131~0x136) 우선순위 최하위였고,
지령 100 Hz 에서 **1 Hz 까지 굶었다.** 위치 제어가 눈을 잃는 것이라 가장 위험했다.

이제 CAN 에는 보드 1~16(밸브·압력)만 남고, 엔코더 6채널은 Teensy 가 USB 로
200 Hz 확정 공급한다.

### 코드

| 파일 | 내용 |
|---|---|
| `include/TeensyFrame.hpp` | 프레임 해독 (ROS·CAN 비의존 → 테스트 가능) |
| `src/CanBridge.cpp` | `teensy_loop()` 스레드. CAN `rx_thread_` 와 완전 독립 |
| `test/test_teensy_frame.cpp` | 실기 캡처로 파서 검증 (12 항목) |
| `test/data/teensy_5s.bin` | 실기 5 초 캡처 (1000 프레임) |
| `scripts/pp_check.py` | **통합 모니터** — CAN + Teensy 한 화면 + 판정 |

`board/analog` · `board/analog_raw` 토픽 이름·타입은 그대로라
**구독자는 전원 무수정**이다 (Controller, pp_logger, pp_monitor,
diagnostic_check, ctrl_eval, live_control, encoder_calib).

### 보정이 다르다

CAN 엔코더는 반전앰프 역산(`orig_mV = (4125−adc)/0.825`)이 필요했지만,
ADS1115 는 센서를 직접 읽으므로 **단순 2점 선형**이다:

```
angle_deg = (raw − raw_0deg) × 90 / (raw_90deg − raw_0deg)
```

역방향 장착(기울기 음수)도 그대로 동작한다 — 검증했다.

### 주기 200 Hz

| 파라미터 | 이전 | 지금 |
|---|---|---|
| `can_bridge.sensor_period_ms` | 10 | **5** ← 이게 제어 루프 주기다 |
| `pp_controller.period_ms` | 10 | **5** (위와 반드시 같아야 한다) |
| `can_tx_min_interval_ms` / `fallback_ms` | 10 | **5** |
| `vel_filter_alpha` (6축) | 0.15 | **0.073** (τ 65.7 ms 유지) |
| `sensor_filter_alpha` | 0.2 | **0.43** (τ 9.0 ms 복원) |

> **제어 틱은 타이머가 아니라 `board/sensors` 구독 콜백에서 돈다**
> (`Controller.cpp:1521` → `on_sensor`). 그래서 실제 주기를 정하는 것은
> 브리지의 `sensor_period_ms` 이고, `pp_controller.period_ms` 는 공칭 dt 로만
> 쓰인다. **둘이 어긋나면 dt 가 틀린 채로 돈다.**

#### 놓쳤다가 잡은 것

20260902 의 100 Hz 커밋에서 고정 alpha 필터를 전수 조사했다고 썼는데
**`sensor_filter_alpha` 를 빠뜨렸다.** `Controller.cpp:2802` 에서 제어 틱마다
압력에 걸리는 LPF 라 주기 의존이고, τ 가 **9.0 → 44.8 ms** 로 5 배 늘어 있었다.
압력 지연 5 배는 지금 쫓는 지연 문제에 직접 악영향이다. 이번에 0.43 으로 복원했다.

#### 주기와 무관함을 확인한 것 (손대지 말 것)

적분항 전부 `* dt` · `vel_raw = Δangle/dt` · `cmd_lpf_hz` 는 `exp(-2πf·dt)` ·
dt 실측+클램프 · `MPC_PHASES=1` · `obs_*` 는 `mppi_estimator:false` 라 미사용 ·
PressureRefGen 은 `gen_period_ms/period_ms` 데시메이션이라 20 ms 유지 ·
MPPI `Ts/NP/du_limit/ref_tau` 는 롤아웃 스텝 기준 ·
`ZERO_CALIB_MS`/`SLEW_SEED_TIMEOUT_MS` 는 시간 기준 (20260902 에 바꿈).

---

## 안전 장치

- **미보정 채널은 각도가 항상 0°** + 기동 ERROR. raw 를 도로 흘려 거짓말하지 않는다.
- **`pp_check.py` 는 수신이 없으면 각도 칸을 `(수신없음)` 으로 비운다.** raw 0 이
  121~127° 로 그럴싸하게 보이기 때문이다.
- **잡음이 없는 채널(σ<0.3)을 `[실패]` 로 잡는다** — 죽은 센서를 각도로는 못 가린다.
### 엔코더 두절 페일세이프 (3 단계)

두절 100 ms 를 감지하면 **감압 → 안전상태 → 종료** 로 내려간다.

| 단계 | 시점 | 하는 일 |
|---|---|---|
| 1 | 즉시 | `board/analog` 을 `teensy_failsafe_angle_deg`(기본 **180°**)로 고정 발행. 컨트롤러가 **자기 슬루 한계로 천천히** 감압한다 |
| 2 | +3000 ms | 하드 안전상태: 채널 밸브 전부 닫고 레일 배기 개방. 이후 컨트롤러 지령 무시 |
| 3 | +1500 ms | 노드 종료. 보드에 남는 마지막 지령은 안전상태. 런치 전체가 같이 내려간다 |

발행을 **끊지 않고 갈아끼우는** 이유: 끊으면 컨트롤러의 `encoder_angles_` 가
마지막 값에 얼어붙고, 컨트롤러는 그걸 현재 자세로 믿은 채 계속 민다.

**각도 값의 근거.** 요구 토크는
`tau_ref = max(0, kp·err + I + 마찰 + m·g·L·sin(angle))`,
`err = clamp(angle_ref − angle, ±5°)` 다. 두 힘이 동시에 걸린다:

- 각도를 올리면 err 이 음수가 되어 PID 가 압력을 뺀다 — 맞다. 다만 err 이
  밴드(±5°)에 잘려 **빼는 힘이 1.62 N·m 에서 포화**한다.
- 동시에 중력 FF 가 **실측각**을 쓰므로 `sin(angle)` 이 같이 커진다 (90° 최대 2.94).

각도별 `tau_ref` (목표 무관, 각도가 목표+5° 를 넘은 뒤):

| 가짜 각도 | 0° | 60° | 90° | 120° | 150° | 180° |
|---|---|---|---|---|---|---|
| tau_ref [N·m] | 1.62 | 0.93 | 1.32 | 0.93 | **0.00** | **0.00** |

- **0° 는 오히려 밀어 올린다** (err 이 양수가 되어 1.62 N·m) — 직관과 반대다.
- 기계적 최대 120° 는 0.93 N·m 가 남는다.
- **150° 이상이라야 정확히 0.** 기본값 180° 는 기계 범위 밖이라 로그에서
  "실측이 아니다" 가 바로 보이는 이점도 있다.

> ⚠ `actuator_connected:=false` 면 `tau_pid`·마찰이 0 이고 중력 FF 가 **목표각**을
> 쓰므로 **1 단계는 아무 효과가 없다.** 그때는 2 단계만이 실제 보호다.
- **두절 시 포트를 닫고 재연결한다.** 장치가 사라지면 `read()` 가 −1/EIO 가 아니라
  **0(EOF)** 을 돌려주는 경우가 있는데, 그때 그냥 기다리면 다시 꽂아도 영영
  복구되지 않는다. pty 로 재현해서 잡았다.
- **포트는 열렸는데 프레임이 한 번도 안 오면 2 초 뒤 재시도.** 펌웨어 정지,
  시작 명령 유실, 또는 **다른 프로세스가 같은 포트를 열었을 때**가 이 경우다.
- CAN 을 못 열면 죽는다 (`can_required:true`). 벤치 점검용으로만 `false`.
- 브리지가 내려가면 **런치 전체가 같이 내려간다** (`on_exit=Shutdown`).

### 시리얼은 한 번에 하나만

`/dev/ttyACM0` 을 두 프로세스가 열면 바이트가 무작위로 쪼개져 **양쪽 다 깨진다.**
`pp_check.py` 는 브리지가 떠 있으면 자동으로 ROS 모드로 바꿔 이 충돌을 피한다.
`teensy_monitor.py` 는 그 보호가 없으니 **브리지와 같이 띄우지 말 것.**
(20260904: `teensy_monitor.py` 도 이제 같은 yaml 2점 보정을 읽는다 — 예전에는
자체 학습 min/max × 120° 로 각도를 따로 만들어 브리지와 다른 값을 보여 줬다.)

---

## 알려진 문제

**`scripts/arduino/ads1115_6ch/ads1115_6ch.ino` 는 실기 펌웨어와 다르다.**
그 파일은 ASCII CSV 를 뱉는다고 돼 있지만 실기는 24 바이트 바이너리를 보낸다
(PGA 도 ±4.096V vs ±2.048V 로 어긋난다). 실측으로 확인한 것은 바이너리 쪽이다.
호스트 파서의 기준은 `scripts/teensy_monitor.py` 와 `include/TeensyFrame.hpp` 다.
**실기 소스를 확보해서 .ino 를 갱신할 것.**

### 밸브 슬롯 (20260904 실측으로 확정)

보드의 PWM 슬롯은 **v1=micro(미세 공급), v2=atm(대기 배기), v3=macro(대유량 공급)**.

모델 내부 배열 `pv[]` 는 `[micro, macro, atm]` 순이지만 `AcadosMpc::finish` 가
슬롯으로 내보낼 때 재배치한다 (`Controller.cpp`: `out3[1]=u0[2]`(atm),
`out3[2]=u0[1]`(macro)). `Controller.hpp:258` 의 "(0=micro, 1=macro, 2=atm)" 은
**모델 배열** 순서지 슬롯 순서가 아니다 — 여기서 헷갈리기 쉽다.

실측 (20260904_122116, board5 axis0):

| 구간 | v1 | v2 | v3 | 판정 |
|---|---|---|---|---|
| 충전 0→45 | **29.9 %** | 7.8 % | 0.2 % | v1 = 공급 |
| 배기 90→0 | 14.2 % | **24.3 %** | 0.0 % | v2 = 배기 |
| 90° 유지 | 16.8 % | 16.2 % | 0.1 % | v3 는 항상 쉼 |

> 20260903 에 이걸 반대로 알고 `can_control_auto.py` 의 `--no-macro` 가
> v2(배기)를 묶도록 바꿨다가 20260904 에 되돌렸다. 배기를 묶으면 보호가 아니라
> 그 반대다.

---

## 실기 없이 시험하는 법

pty 로 캡처를 200 Hz 재생해 Teensy 를 흉내 낼 수 있다. 이번 작업의 검증도
전부 이걸로 했다 (재연결 버그 2 개를 여기서 잡았다).

```python
# fake_teensy.py — pty 를 만들고 test/data/teensy_5s.bin 을 200 Hz 로 재생.
#   'r' 을 받아야 스트리밍을 시작하는 것까지 실기와 같게 흉내 낸다.
#   인자로 준 심링크가 pty 를 가리키므로, 죽였다 살리면 '케이블 뽑았다 꽂기'가 된다.
```
```bash
python3 fake_teensy.py /tmp/tlink &
ros2 run can_powerpack can_bridge_node --ros-args \
  --params-file <config> -r __ns:=/pack2 \
  -p teensy_port:=/tmp/tlink -p can_required:=false
```

> ROS 2 CLI 는 `TeensyEncoder.channels.0.raw_0deg` 처럼 **숫자 세그먼트가 든
> 파라미터명을 못 파싱한다.** YAML 로는 잘 들어간다 (기존
> `EncoderCalibration.boards."17"` 도 같은 구조다). CLI 로 보정을 덮어쓰려면
> 작은 YAML 오버레이 파일을 쓸 것.
