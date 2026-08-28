# 베스트 설정 스냅샷 — 2026-08-28

실기 `20260828_174653` (커밋 `acd49fa`) 에서 확인된, 지금까지 가장 좋은 값이다.
1축 압력 제어 실측 성능:

| | 오차 (마지막 3초) | p-p |
|---|---|---|
| 양압 | −0.35 ~ +0.96 kPa | 1.75 ~ 2.84 kPa |
| 음압 | −0.97 ~ +3.97 kPa | 0.47 ~ 15.44 kPa |

## 되돌리는 법

```bash
cp src/can_powerpack/config/best/valve_params.yaml     src/can_powerpack/config/
cp src/can_powerpack/config/best/powerpack_config.yaml src/can_powerpack/config/
colcon build --packages-select can_powerpack --cmake-args -DCMAKE_BUILD_TYPE=Release
```

git 태그로도 남겼다: `best-1axis-20260828`

## 이 값이 어떻게 나왔는지

밸브 파라미터는 전부 **실기 로그의 전류 기준**으로 적합했다 (지령 열은 스위칭
중에 믿을 수 없다 — HANDOFF S-10). 핵심 수치:

```
I_MAX 0.2505 A (실측, 전류[mA] = 2.505·지령[%], R=0.999)
pos/micro  A_max 0.032258  k 284.5405  C_k 0.13400850 @190   반개 44.4%
pos/atm    A_max 0.006000  k 413.1948  C_k 0.16236145 @141   반개 58.1%
pos/macro  A_max 0.015611  k 284.5405  C_k 0.18020850 @575   반개 44.4%
neg/micro  A_max 0.018500  k 284.5405  C_k 0.13251600 @80    반개 49.1%
neg/atm    A_max 0.015000  k 284.5405  C_k 0.11419700 @101.3 반개 40.7%
```

제어 쪽에서 결정적이었던 것 (자세한 내용은 HANDOFF S-7 ~ S-16):
- `finish()` 의 순서: **MPPI → 명령 LPF → 크래킹 하한 → 적분 트림**
  (모델·필터·하한을 이기려는 보정은 언제나 그것들 뒤에 와야 한다)
- 명령 LPF 는 **비대칭** — 열 때만 느리고 닫을 때는 즉시
- 크래킹 하한은 압력이 **원하는 방향으로** 안 움직일 때만 건다 (부호까지 본다)
- 레일 셋포인트와 챔버 레퍼런스를 서로의 능력으로 묶는다
  (`pos_headroom_kpa`, `chamber_neg_headroom_kpa`, `chamber_pos_headroom_kpa`)
