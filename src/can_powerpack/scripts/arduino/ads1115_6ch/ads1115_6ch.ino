/**
 * ⚠ 20260903: **이 파일은 실기에 올라간 펌웨어와 다르다.**
 *   여기 적힌 출력은 ASCII CSV 인데, 실기 Teensy 는 24 바이트 **바이너리**
 *   (SYNC 0xAA55 + seq + t_us + int16×6 + status + CRC16) 를 200 Hz 로 보낸다.
 *   PGA 도 다르다 (여기 ±4.096V/125µV, 호스트 쪽은 ±2.048V/62.5µV 로 가정).
 *   실측으로 확인한 것은 바이너리 쪽이다 (1000 프레임 CRC 전부 통과).
 *
 *   호스트 파서의 기준은 **scripts/teensy_monitor.py** 와
 *   **include/TeensyFrame.hpp** 다. 실기 소스를 확보하기 전까지 이 .ino 를
 *   신뢰하지 말 것. (캡처: test/data/teensy_5s.bin)
 *
 * ads1115_6ch.ino
 * ADS1115 × N (단일단 6 채널) → Teensy 4.0 I2C → USB Serial CDC → 랩탑
 *
 * ── 왜 3 칩인가 ────────────────────────────────────────────────────────────
 * ADS1115 는 앞단에 MUX 하나가 붙은 **단일 ADC** 다. 860 SPS 는 채널당이 아니라
 * 칩당 처리량이라, 한 칩이 몇 채널을 돌리느냐(=NSTEP)가 그대로 프레임 시간이 된다.
 *
 *   구성        NSTEP  순수 변환      I2C 포함 실측     200 Hz(5 ms) 여유
 *   2 칩 × 3ch    3    3 × 1163 =3.49ms   ~4.7 ms        6%   ← 빠듯하다
 *   3 칩 × 2ch    2    2 × 1163 =2.33ms   ~3.6 ms        28%  ← 권장
 *   6 칩 × 1ch    1    1 × 1163 =1.16ms   ~2.0 ms        60%  (버스 2 개 필요)
 *
 * 칩을 늘려도 변환은 **동시에** 돌기 때문에 프레임이 길어지지 않는다.
 * 늘어나는 건 I2C 트랜잭션뿐인데, 아래 파이프라이닝으로 대부분 숨긴다.
 *
 * ── 파이프라이닝 ───────────────────────────────────────────────────────────
 * step s 의 변환이 끝나면 s+1 변환을 **먼저** 걸고, 그게 도는 동안 s 의 결과를
 * 읽는다. ADS1115 의 변환 레지스터는 변환이 끝나는 순간에만 갱신되므로 변환 중에
 * 읽어도 직전 결과가 온전히 나온다. 이 성질로 읽기 시간을 변환 뒤에 숨긴다.
 *
 * 실측 프레임 시간은 stat 줄로 계속 뱉는다. FRAME_US 를 넘기 시작하면
 * USE_RDY_PIN 을 켜라 (선 1 가닥, 폴링 트랜잭션이 사라져 ~300 µs 벌어준다).
 *
 * ── 배선 (3 칩 기본 구성) ──────────────────────────────────────────────────
 *   ADS #1  ADDR → GND  → 0x48    A0,A1 = ch0, ch1
 *   ADS #2  ADDR → VDD  → 0x49    A0,A1 = ch2, ch3
 *   ADS #3  ADDR → SDA  → 0x4A    A0,A1 = ch4, ch5
 *   (한 버스에 4 개까지: ADDR → SCL 이면 0x4B)
 *
 *   공통    VDD → Teensy 3.3V     GND → GND
 *           SDA → Teensy 18 (Wire SDA)
 *           SCL → Teensy 19 (Wire SCL)
 *   풀업    SDA/SCL 각각 → 3.3V 로 2.2k~4.7k
 *           ★ 모듈마다 10k 내장이 흔하다. 3 개를 병렬로 달면 3.3k 가 되어 대개
 *             괜찮지만, 400 kHz 에서 파형이 무디면 모듈 풀업을 떼고 외부 2.2k
 *             하나만 남겨라.
 *   (선택) USE_RDY_PIN 사용 시  **마지막 칩**의 ALERT/RDY → Teensy 2 번 핀
 *
 *   ★ VDD 를 5V 로 올리지 말 것. ADS1115 의 I2C VIH = 0.7×VDD = 3.5V 인데
 *     Teensy 4.0 은 3.3V 로직이고 5V 톨러런트가 아니다. 양쪽 다 스펙 위반이다.
 *     0~5V 센서는 분압기로 3.3V 이하로 낮춰서 넣어라.
 *
 * ── 절대 입력 한계 ─────────────────────────────────────────────────────────
 *   GND-0.3V ~ VDD+0.3V.  PGA 를 ±6.144V 로 잡아도 VDD 위는 못 읽는다.
 *   단일단은 음수가 0 으로 잘리므로 유효 코드 범위가 0~32767 (15 bit) 이다.
 *
 * ── 출력 포맷 ──────────────────────────────────────────────────────────────
 *   데이터 : "<seq>,<t_us>,<err>,<r0>,<r1>,<r2>,<r3>,<r4>,<r5>\n"
 *            r* 는 raw 카운트(부호있는 16 bit). mV 환산은 랩탑에서 한다
 *            (can_monitor.py 와 같은 철학 — 선로에는 raw, 보정은 호스트에서).
 *   헤더   : "# ads1115 v2 fsr_mv=... dr_sps=... rate_hz=... nch=6 nchip=3 ..."
 *   통계   : "# stat tx_hz=... frame_us_avg=... frame_us_max=... i2c_err=... late=..."
 *
 *   err 비트: 0x01/0x02/0x04/0x08 = 해당 칩 I2C 실패 (비트 = 칩 인덱스)
 *             0x10 = 변환 대기 타임아웃,  0x20 = 프레임이 주기를 넘김
 *
 * ── 호스트 명령 (1 바이트) ─────────────────────────────────────────────────
 *   'i' → 헤더 재출력   'r' → seq/통계 리셋   's' → 스트리밍 토글
 *
 * ── 랩탑 수신 ──────────────────────────────────────────────────────────────
 *   scripts/teensy_monitor.py
 */

#include <Wire.h>

// ════════════════════════════════════════════════════════════════════════════
//  ★ 여기만 수정하면 됨
// ════════════════════════════════════════════════════════════════════════════

// ── I2C ─────────────────────────────────────────────────────────────────────
#define I2C_BUS         Wire        // Teensy 4.0: Wire(18/19), Wire1(17/16), Wire2(25/24)
#define I2C_CLOCK_HZ    400000L     // ADS1115 Fast Mode 상한. 1MHz 로 올리지 말 것 (스펙 밖)

// ── 칩 / 채널 매핑 ──────────────────────────────────────────────────────────
// 채널 번호 = 칩인덱스 × NSTEP + step.  NCHIP × NSTEP = 6 이어야 한다.
//
//   3 칩 구성 (권장) : NCHIP 3, NSTEP 2, AIN_MAP {{0,1},{0,1},{0,1}}
//   2 칩 구성        : NCHIP 2, NSTEP 3, AIN_MAP {{0,1,2},{0,1,2}}
#define NCHIP 3
#define NSTEP 2

static const uint8_t CHIP_ADDR[NCHIP] = { 0x48, 0x49, 0x4A };
static const uint8_t AIN_MAP[NCHIP][NSTEP] = {
  { 0, 1 },   // 0x48 → ch0, ch1
  { 0, 1 },   // 0x49 → ch2, ch3
  { 0, 1 },   // 0x4A → ch4, ch5
};

// ── 측정 설정 ───────────────────────────────────────────────────────────────
#define PGA_SEL         PGA_4_096V  // VDD=3.3V 면 ±4.096V 가 정답 (LSB 125µV)
#define DR_SEL          DR_860SPS   // NSTEP=2 면 DR_475SPS 도 들어갈 수 있다 (아래 NOTE)
#define FRAME_RATE_HZ   200

// ── 변환 완료 대기 방식 ─────────────────────────────────────────────────────
// false: 설정 레지스터의 OS 비트를 I2C 로 폴링 (배선 추가 없음, 기본)
// true : 마지막 칩의 ALERT/RDY 핀을 GPIO 로 감시 (선 1 가닥, ~300 µs/프레임 절약)
#define USE_RDY_PIN     false
#define RDY_PIN         2

// 폴링 전 미리 기다리는 시간 비율. 변환의 90% 를 그냥 자면 폴링이 1~2 회로 준다.
#define PREWAIT_RATIO   0.90f

// ── 출력 ────────────────────────────────────────────────────────────────────
#define BAUD_RATE       115200      // USB CDC 라 실제로는 무시된다. 호스트와 맞추기만 하면 됨
#define STAT_PERIOD_MS  1000        // stat 줄 주기

// ════════════════════════════════════════════════════════════════════════════

#define NCH (NCHIP * NSTEP)
static_assert(NCH == 6, "NCHIP × NSTEP 가 6 이어야 한다");
static_assert(NCHIP <= 4, "한 I2C 버스에 ADS1115 는 4 개까지 (주소 0x48~0x4B)");

// ── ADS1115 레지스터/비트 ───────────────────────────────────────────────────
#define REG_CONV        0x00
#define REG_CONFIG      0x01
#define REG_LO_THRESH   0x02
#define REG_HI_THRESH   0x03

#define CFG_OS_SINGLE   0x8000      // 쓰면 단발 변환 시작 / 읽으면 1=idle, 0=변환중
#define CFG_MODE_SINGLE 0x0100      // 단발 변환 모드
#define CFG_COMP_OFF    0x0003      // COMP_QUE=11 → 비교기 비활성 (ALERT 안 씀)
#define CFG_COMP_RDY    0x0000      // COMP_QUE=00 → 변환 1 회마다 ALERT/RDY 어서트

// PGA 선택지 (PGA_SEL 에 넣는다)
#define PGA_6_144V 0
#define PGA_4_096V 1
#define PGA_2_048V 2
#define PGA_1_024V 3
#define PGA_0_512V 4
#define PGA_0_256V 5
static const uint16_t PGA_BITS[6]   = { 0x0000, 0x0200, 0x0400, 0x0600, 0x0800, 0x0A00 };
static const uint16_t PGA_FSR_MV[6] = {   6144,   4096,   2048,   1024,    512,    256 };

// 데이터레이트 선택지 (DR_SEL 에 넣는다)
#define DR_8SPS   0
#define DR_16SPS  1
#define DR_32SPS  2
#define DR_64SPS  3
#define DR_128SPS 4
#define DR_250SPS 5
#define DR_475SPS 6
#define DR_860SPS 7
static const uint16_t DR_BITS[8] = { 0x0000, 0x0020, 0x0040, 0x0060,
                                     0x0080, 0x00A0, 0x00C0, 0x00E0 };
static const uint16_t DR_SPS[8]  = {      8,     16,     32,     64,
                                        128,    250,    475,    860 };

// ── err 비트 ────────────────────────────────────────────────────────────────
#define ERR_CHIP(i) ((uint8_t)(1u << (i)))   // 0x01, 0x02, 0x04, 0x08
#define ERR_TIMEOUT 0x10
#define ERR_LATE    0x20

// ── 파생 상수 ───────────────────────────────────────────────────────────────
static const uint32_t FRAME_US    = 1000000UL / FRAME_RATE_HZ;
static const uint32_t CONV_US     = 1000000UL / DR_SPS[DR_SEL];
// 내부 오실레이터 오차가 ±10% 라 최악은 변환이 1.11 배 길어진다. + 시동 여유.
static const uint32_t CONV_MAX_US = (uint32_t)(CONV_US * 1.25f) + 200;
static const uint32_t PREWAIT_US  = (uint32_t)(CONV_US * PREWAIT_RATIO);
static const uint8_t  LAST_CHIP   = NCHIP - 1;   // 마지막에 걸리므로 가장 늦게 끝난다

// ── 상태 ────────────────────────────────────────────────────────────────────
static uint32_t g_seq        = 0;
static bool     g_streaming  = true;
static uint32_t g_nextFrame  = 0;

static uint32_t g_statTxCnt  = 0;   // stat 창 안에서 보낸 프레임 수
static uint32_t g_statUsSum  = 0;   // 프레임 소요시간 합
static uint32_t g_statUsMax  = 0;   // 프레임 소요시간 최대
static uint32_t g_statErrCnt = 0;   // I2C 오류 프레임 수
static uint32_t g_statLate   = 0;   // 주기를 넘긴 프레임 수
static uint32_t g_statLastMs = 0;

// ════════════════════════════════════════════════════════════════════════════
//  ADS1115 저수준
// ════════════════════════════════════════════════════════════════════════════

static bool adsWriteReg(uint8_t addr, uint8_t reg, uint16_t val) {
  I2C_BUS.beginTransmission(addr);
  I2C_BUS.write(reg);
  I2C_BUS.write((uint8_t)(val >> 8));
  I2C_BUS.write((uint8_t)(val & 0xFF));
  return I2C_BUS.endTransmission() == 0;
}

static bool adsReadReg(uint8_t addr, uint8_t reg, uint16_t *out) {
  I2C_BUS.beginTransmission(addr);
  I2C_BUS.write(reg);
  if (I2C_BUS.endTransmission(false) != 0) return false;   // repeated start
  if (I2C_BUS.requestFrom(addr, (uint8_t)2) < 2)  return false;
  uint16_t v = (uint16_t)I2C_BUS.read() << 8;
  v |= (uint16_t)I2C_BUS.read();
  *out = v;
  return true;
}

/** 지정 AIN 에 대해 단발 변환을 시작시킨다. */
static bool adsStart(uint8_t addr, uint8_t ain) {
  uint16_t cfg = CFG_OS_SINGLE
               | ((uint16_t)(0x04 | (ain & 0x03)) << 12)   // MUX 100b..111b = AIN0..AIN3 단일단
               | PGA_BITS[PGA_SEL]
               | CFG_MODE_SINGLE
               | DR_BITS[DR_SEL]
               | (USE_RDY_PIN ? CFG_COMP_RDY : CFG_COMP_OFF);
  return adsWriteReg(addr, REG_CONFIG, cfg);
}

/** 변환 레지스터를 읽는다. 변환 중에 읽으면 **직전 변환 결과**가 나온다. */
static bool adsReadConv(uint8_t addr, int16_t *out) {
  uint16_t v;
  if (!adsReadReg(addr, REG_CONV, &v)) return false;
  *out = (int16_t)v;
  return true;
}

#if !USE_RDY_PIN
/** 변환이 끝났는가? (OS=1 이면 idle = 완료) */
static bool adsIdle(uint8_t addr, bool *io_ok) {
  uint16_t v;
  if (!adsReadReg(addr, REG_CONFIG, &v)) { *io_ok = false; return true; }
  return (v & CFG_OS_SINGLE) != 0;
}
#endif

/**
 * ALERT/RDY 를 "변환 완료 핀"으로 쓰려면 임계값 레지스터를 이렇게 박아야 한다.
 * (데이터시트: Hi_thresh MSB=1, Lo_thresh MSB=0 이면 비교기 대신 RDY 로 동작)
 */
#if USE_RDY_PIN
static void adsSetupRdy(uint8_t addr) {
  adsWriteReg(addr, REG_HI_THRESH, 0x8000);
  adsWriteReg(addr, REG_LO_THRESH, 0x0000);
}
#endif

// ════════════════════════════════════════════════════════════════════════════
//  프레임 샘플링
// ════════════════════════════════════════════════════════════════════════════

/** step 을 전 칩에 동시에 걸어 둔다. 먼저 건 칩이 먼저 끝난다. */
static void startAll(uint8_t step, uint8_t *err) {
  for (uint8_t c = 0; c < NCHIP; c++)
    if (!adsStart(CHIP_ADDR[c], AIN_MAP[c][step])) *err |= ERR_CHIP(c);
}

/**
 * 변환 완료를 기다린다.
 * 마지막 칩을 가장 늦게 걸었으므로 **그게 끝나면 나머지도 끝나 있다** —
 * 마지막 칩 하나만 보면 된다. (폴링 트랜잭션을 칩 수만큼 늘리지 않는 이유)
 */
static void waitConv(uint32_t startedUs, uint8_t *err) {
#if USE_RDY_PIN
  // RDY 는 변환 완료 때 짧게 LOW 로 떨어진다(오픈드레인, 풀업 필요).
  // I2C 트랜잭션이 전혀 없어 폴링 방식보다 프레임당 ~300 µs 빠르다.
  while (digitalReadFast(RDY_PIN) != LOW) {
    if (micros() - startedUs > CONV_MAX_US) { *err |= ERR_TIMEOUT; return; }
  }
#else
  // 변환 시간의 90% 는 그냥 기다린다 — 폴링 트랜잭션 수를 1~2 회로 줄인다.
  uint32_t elapsed = micros() - startedUs;
  if (elapsed < PREWAIT_US) delayMicroseconds(PREWAIT_US - elapsed);

  bool ok = true;
  while (!adsIdle(CHIP_ADDR[LAST_CHIP], &ok)) {
    if (!ok) break;
    if (micros() - startedUs > CONV_MAX_US) { *err |= ERR_TIMEOUT; return; }
  }
  if (!ok) {
    // 기준 칩이 죽었으면 폴링으로는 알 수 없으니 시간으로 때운다 — 나머지는 살린다.
    *err |= ERR_CHIP(LAST_CHIP);
    uint32_t e2 = micros() - startedUs;
    if (e2 < CONV_MAX_US) delayMicroseconds(CONV_MAX_US - e2);
  }
#endif
}

/**
 * 6 채널 한 프레임을 뜬다.
 *
 * 핵심은 **파이프라이닝**이다. step s 의 변환이 끝나면 s+1 변환을 즉시 걸고,
 * 그 변환이 도는 동안 s 의 결과를 읽는다. 변환 레지스터는 변환 종료 시점에만
 * 갱신되므로 이때 읽어도 s 의 값이 온전히 나온다. 이렇게 해야 읽기에 드는
 * I2C 시간이 변환 뒤에 숨어서, 칩을 늘려도 프레임이 거의 안 길어진다.
 */
static void sampleFrame(int16_t *raw, uint8_t *err) {
  *err = 0;

  // 기준 시각은 startAll **뒤**에 찍는다. 마지막 칩은 앞 칩들의 쓰기 트랜잭션
  // (칩당 ~110 µs) 만큼 늦게 변환을 시작하므로, 앞에서 찍으면 대기 시작점이
  // 그만큼 이르게 잡혀 폴링이 헛돈다 (3 칩 기준 step 당 4 회 → 1 회로 준다).
  // 타임아웃 기준으로도 이쪽이 맞다 — 마지막 칩의 변환이 실제 시작된 시각이다.
  startAll(0, err);
  uint32_t startedUs = micros();

  for (uint8_t s = 0; s < NSTEP; s++) {
    waitConv(startedUs, err);

    // 다음 변환을 **먼저** 걸어 두고
    if (s + 1 < NSTEP) {
      startAll(s + 1, err);
      startedUs = micros();
    }
    // 이번 결과는 그 변환이 도는 동안 읽는다
    for (uint8_t c = 0; c < NCHIP; c++)
      if (!adsReadConv(CHIP_ADDR[c], &raw[c * NSTEP + s])) *err |= ERR_CHIP(c);
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  출력
// ════════════════════════════════════════════════════════════════════════════

static void printHeader() {
  Serial.printf("# ads1115 v2 fsr_mv=%u dr_sps=%u rate_hz=%u nch=%u nchip=%u nstep=%u "
                "rdy=%d frame_budget_us=%lu conv_us=%lu\n",
                PGA_FSR_MV[PGA_SEL], DR_SPS[DR_SEL], FRAME_RATE_HZ,
                NCH, NCHIP, NSTEP, (int)USE_RDY_PIN,
                (unsigned long)FRAME_US, (unsigned long)CONV_US);
  // 채널 → (칩주소, AIN) 대응을 밝힌다. 배선 헷갈릴 때 이 줄만 보면 된다.
  Serial.print("# map");
  for (uint8_t c = 0; c < NCHIP; c++)
    for (uint8_t s = 0; s < NSTEP; s++)
      Serial.printf(" ch%u=0x%02X:A%u", c * NSTEP + s, CHIP_ADDR[c], AIN_MAP[c][s]);
  Serial.println();
  Serial.send_now();
}

/**
 * 칩 존재 확인 + 프레임 예산 계산.
 *
 * 부팅 때와 호스트의 'i' 명령에 모두 응답한다. 호스트(teensy_monitor.py)는 붙을 때
 * 입력 버퍼를 비우므로, 이걸 부팅 때만 뱉으면 **정작 사람이 볼 때는 이미 사라진
 * 뒤**다. 칩이 하나 빠진 걸 이 줄로 잡는데 그게 안 보이면 의미가 없다.
 */
static void printDiag() {
  // 칩이 실제로 붙어 있는지 확인 — 없는 채로 도는 게 제일 헷갈린다
  for (uint8_t c = 0; c < NCHIP; c++) {
    I2C_BUS.beginTransmission(CHIP_ADDR[c]);
    if (I2C_BUS.endTransmission() != 0)
      Serial.printf("# ERROR: 0x%02X 에 응답이 없다 (배선/ADDR 핀 확인)\n", CHIP_ADDR[c]);
  }

  // 변환 NSTEP 회 + 시작 트랜잭션(칩당 ~110 µs) + 마지막 읽기(칩당 ~140 µs).
  // 나머지 읽기는 파이프라이닝으로 변환 뒤에 숨으므로 예산에서 뺀다.
  uint32_t est = NSTEP * (CONV_US + 110UL * NCHIP) + 140UL * NCHIP;
  Serial.printf("# est frame_us~%lu / budget %lu us (%s)\n",
                (unsigned long)est, (unsigned long)FRAME_US,
                est < FRAME_US ? "OK" : "예산 초과 — 칩을 늘리거나 DR 을 올려라");

  if (DR_SEL == DR_860SPS)
    Serial.println("# NOTE: 860 SPS 는 잡음 최대 구간이다. 유효 분해능 ~13-14 bit 로 본다. "
                   "NSTEP=2 라면 DR_475SPS 로 낮춰 잡음을 줄일 여지가 있다 — "
                   "위 est 줄이 OK 인지 보고 정해라.");
  Serial.send_now();
}

static void printStat() {
  uint32_t now = millis();
  uint32_t dt  = now - g_statLastMs;
  if (dt < STAT_PERIOD_MS) return;

  float hz  = g_statTxCnt * 1000.0f / (float)dt;
  uint32_t avg = g_statTxCnt ? (g_statUsSum / g_statTxCnt) : 0;

  Serial.printf("# stat tx_hz=%.1f frame_us_avg=%lu frame_us_max=%lu "
                "i2c_err=%lu late=%lu\n",
                hz, (unsigned long)avg, (unsigned long)g_statUsMax,
                (unsigned long)g_statErrCnt, (unsigned long)g_statLate);
  Serial.send_now();

  g_statLastMs = now;
  g_statTxCnt = 0; g_statUsSum = 0; g_statUsMax = 0;
  g_statErrCnt = 0; g_statLate = 0;
}

static void handleCommand() {
  while (Serial.available()) {
    switch (Serial.read()) {
      // 진단은 I2C 프로브가 들어가서 그 프레임 하나가 늦어질 수 있다.
      // 호스트가 명시적으로 요청했을 때만 도므로 감수한다.
      case 'i': printHeader(); printDiag(); break;
      case 'r':
        g_seq = 0;
        g_statTxCnt = 0; g_statUsSum = 0; g_statUsMax = 0;
        g_statErrCnt = 0; g_statLate = 0;
        break;
      case 's': g_streaming = !g_streaming; break;
      default: break;
    }
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  Setup / Loop
// ════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(BAUD_RATE);          // USB CDC 라 baud 는 무시된다
  I2C_BUS.begin();
  I2C_BUS.setClock(I2C_CLOCK_HZ);

#if USE_RDY_PIN
  pinMode(RDY_PIN, INPUT_PULLUP);   // ALERT/RDY 는 오픈드레인이다
  // 폴링은 마지막 칩만 보지만 임계값은 **전 칩**에 박는다. adsStart 가 모든 칩에
  // COMP_QUE=00 을 쓰기 때문에, 임계값을 안 박은 칩의 ALERT 는 기본 임계값 기준의
  // 비교기 출력으로 뜬다. 지금은 배선이 없어 무해하지만 나중에 다른 칩 ALERT 를
  // 물렸을 때 조용히 오동작한다. 어느 칩 것을 물려도 RDY 로 동작하게 맞춰 둔다.
  for (uint8_t c = 0; c < NCHIP; c++) adsSetupRdy(CHIP_ADDR[c]);
#endif

  // 호스트가 붙을 때까지 잠깐 기다린다 (없어도 진행)
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) { /* wait */ }

  printHeader();
  printDiag();

  g_statLastMs = millis();
  g_nextFrame  = micros();
}

void loop() {
  handleCommand();

  // 주기 도달 대기. 밀리지 않도록 절대 시각으로 누적한다.
  uint32_t now = micros();
  if ((int32_t)(now - g_nextFrame) < 0) return;
  g_nextFrame += FRAME_US;

  // 한 프레임 이상 밀렸으면 따라잡기를 포기하고 위상을 다시 잡는다
  if ((int32_t)(micros() - g_nextFrame) > (int32_t)FRAME_US) {
    g_nextFrame = micros() + FRAME_US;
    g_statLate++;
  }

  int16_t raw[NCH];
  uint8_t err;
  uint32_t t0 = micros();
  sampleFrame(raw, &err);
  uint32_t dur = micros() - t0;

  if (dur > FRAME_US) { err |= ERR_LATE; g_statLate++; }

  g_seq++;
  g_statTxCnt++;
  g_statUsSum += dur;
  if (dur > g_statUsMax) g_statUsMax = dur;
  if (err & ~ERR_LATE) g_statErrCnt++;

  if (g_streaming) {
    // 한 줄을 버퍼에 만들어 한 번에 write 한다 (Serial.print 를 10 번 부르는 것보다 빠르다)
    char line[96];
    int n = snprintf(line, sizeof(line), "%lu,%lu,%u,%d,%d,%d,%d,%d,%d\n",
                     (unsigned long)g_seq, (unsigned long)t0, err,
                     raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
    Serial.write((const uint8_t *)line, n);
    Serial.send_now();   // 이게 없으면 버퍼가 차거나 flush 타임아웃까지 수 ms 늦는다
  }

  printStat();
}
