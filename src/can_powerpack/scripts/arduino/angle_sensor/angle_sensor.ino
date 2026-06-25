/**
 * angle_sensor.ino
 * SAE460API-12DN-XY (또는 AS5600 호환) I2C 각도 센서 → USB Serial
 *
 * ── 사용 방법 ──────────────────────────────────────────────────────────────
 * 1. SCAN_MODE true 로 업로드 → Serial Monitor(115200) 열어서 I2C 주소 확인
 * 2. 확인한 주소를 SENSOR_ADDR 에 입력, SCAN_MODE false 로 재업로드
 * 3. 센서 레지스터를 모를 경우 REG_DUMP_MODE true 로 레지스터 전체 덤프 가능
 *
 * ── 출력 포맷 (SCAN_MODE, REG_DUMP_MODE 둘 다 false 일 때) ──────────────
 *   "<raw_count>,<angle_deg>\n"   500 Hz
 *   예: "2048,180.00\n"
 *
 * ── PC 수신 ────────────────────────────────────────────────────────────────
 *   serial_angle_reader.py 로 수신 (포트, baud 일치 확인)
 *
 * ── 배선 ───────────────────────────────────────────────────────────────────
 *   센서 SDA → Arduino SDA  (Uno: A4, Mega: 20, Nano: A4)
 *   센서 SCL → Arduino SCL  (Uno: A5, Mega: 21, Nano: A5)
 *   센서 VCC → 3.3V or 5V   (센서 보드 스펙 확인)
 *   센서 GND → GND
 */

#include <Wire.h>

// ════════════════════════════════════════════════════════════════════════════
//  ★ 여기만 수정하면 됨
// ════════════════════════════════════════════════════════════════════════════

// 동작 모드 (하나만 true)
#define SCAN_MODE      false   // true: I2C 주소 스캔 (처음 연결 시)
#define REG_DUMP_MODE  false   // true: 0x00-0x7F 레지스터 전체 덤프

// 센서 설정 ── SAE460API 스펙 확인 후 수정
#define SENSOR_ADDR    0x36    // I2C 7-bit 주소 (SCAN_MODE로 확인)
#define REG_ANGLE_H    0x0C    // MSB 레지스터 주소
#define REG_ANGLE_L    0x0D    // LSB 레지스터 주소  (H와 같으면 2바이트 연속 읽기)
#define ANGLE_BITS     12      // 분해능 (12 → 4096 counts/rev)
#define ANGLE_MASK     0x0FFF  // MSB 마스크 (12-bit: 0x0FFF, 14-bit: 0x3FFF)
#define MSB_SHIFT      8       // MSB를 몇 비트 shift 하는지

// 출력 설정
#define BAUD_RATE          115200
#define SEND_INTERVAL_US   2000UL   // 2000 µs = 500 Hz

// ════════════════════════════════════════════════════════════════════════════

static const uint16_t COUNTS_PER_REV = (1u << ANGLE_BITS);  // 4096 or 16384
static uint32_t lastSendUs = 0;

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin();
  Wire.setClock(400000L);   // 400 kHz I2C Fast Mode

  if (SCAN_MODE) {
    scanI2C();
    return;
  }
  if (REG_DUMP_MODE) {
    dumpRegisters();
    return;
  }

  // 센서 존재 확인
  Wire.beginTransmission(SENSOR_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.print("# ERROR: no device at 0x");
    Serial.println(SENSOR_ADDR, HEX);
  }
}

// ── Main loop ────────────────────────────────────────────────────────────────
void loop() {
  if (SCAN_MODE || REG_DUMP_MODE) {
    delay(5000);
    if (SCAN_MODE)     scanI2C();
    if (REG_DUMP_MODE) dumpRegisters();
    return;
  }

  uint32_t now = micros();
  if (now - lastSendUs < SEND_INTERVAL_US) return;
  lastSendUs = now;

  uint16_t raw       = readRawAngle();
  float    angle_deg = (float)raw * (360.0f / COUNTS_PER_REV);

  Serial.print(raw);
  Serial.print(',');
  Serial.println(angle_deg, 2);
}

// ── Raw angle 읽기 ────────────────────────────────────────────────────────────
uint16_t readRawAngle() {
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(REG_ANGLE_H);
  Wire.endTransmission(false);          // repeated start (레지스터 포인터 유지)
  Wire.requestFrom((uint8_t)SENSOR_ADDR, (uint8_t)2);

  if (Wire.available() < 2) return 0;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();

  return ((uint16_t)(hi << MSB_SHIFT) | lo) & ANGLE_MASK;
}

// ── I2C 스캔 (SCAN_MODE) ────────────────────────────────────────────────────
void scanI2C() {
  Serial.println("# I2C scan ---");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("# FOUND: 0x");
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) Serial.println("# No device found.");
  Serial.println("# --- done");
}

// ── 레지스터 덤프 (REG_DUMP_MODE) ───────────────────────────────────────────
void dumpRegisters() {
  Serial.print("# Register dump for 0x");
  Serial.println(SENSOR_ADDR, HEX);
  for (uint8_t reg = 0x00; reg <= 0x7F; reg++) {
    Wire.beginTransmission(SENSOR_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)SENSOR_ADDR, (uint8_t)1);
    if (Wire.available()) {
      uint8_t val = Wire.read();
      Serial.print("# 0x");
      if (reg < 16) Serial.print('0');
      Serial.print(reg, HEX);
      Serial.print(" = 0x");
      if (val < 16) Serial.print('0');
      Serial.println(val, HEX);
    }
  }
  Serial.println("# --- done");
}
