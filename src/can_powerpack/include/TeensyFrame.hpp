#ifndef TEENSY_FRAME_HPP_
#define TEENSY_FRAME_HPP_

// Teensy 엔코더 스트림의 프레임 해독. **ROS·CAN 에 의존하지 않는다** —
// 그래야 test/test_teensy_frame.cpp 가 실기 없이 이 코드를 그대로 검증할 수 있다.
// (실기 캡처 test/data/teensy_5s.bin 로 돌린다. 파서를 손대면 그 테스트를 돌릴 것.)
//
// 포맷 (24 B, scripts/teensy_monitor.py 와 동일, 20260903 실기 검증):
//   [0..1]   SYNC 0xAA 0x55
//   [2..3]   seq      uint16
//   [4..7]   t_us     uint32
//   [8..19]  ch0~ch5  int16 × 6
//   [20..21] status   uint16   (비트 c = 칩 c I2C 오류)
//   [22..23] CRC16-CCITT (poly 0x1021, init 0xFFFF, non-reflected, 바이트 0..21)

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace teensy {

static constexpr int     FRAME_LEN = 24;
static constexpr int     NCH       = 6;
static constexpr uint8_t SYNC0     = 0xAA;
static constexpr uint8_t SYNC1     = 0x55;

struct Frame {
  uint16_t seq{0};
  uint32_t t_us{0};
  int16_t  ch[NCH]{};
  uint16_t status{0};
};

/// CRC16-CCITT. scripts/teensy_monitor.py 의 crc16() 과 **비트 단위로 같아야 한다.**
inline uint16_t crc16(const uint8_t* d, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; ++i) {
    c ^= (uint16_t)d[i] << 8;
    for (int b = 0; b < 8; ++b)
      c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}

/// 24 바이트가 유효한 프레임인지 보고, 맞으면 out 을 채운다.
/// 호출자는 p 가 최소 FRAME_LEN 바이트를 가리킴을 보장해야 한다.
inline bool decode(const uint8_t* p, Frame& out) {
  if (p[0] != SYNC0 || p[1] != SYNC1) return false;
  uint16_t want;
  std::memcpy(&want, p + 22, 2);
  if (crc16(p, 22) != want) return false;
  std::memcpy(&out.seq,    p + 2,  2);
  std::memcpy(&out.t_us,   p + 4,  4);
  std::memcpy(out.ch,      p + 8,  2 * NCH);
  std::memcpy(&out.status, p + 20, 2);
  return true;
}

/// 링버퍼에서 프레임을 꺼낸다.
///   buf/len : 아직 소비하지 않은 바이트
///   consumed: 이번 호출이 소비한 바이트 수 (호출자가 앞에서 잘라내면 된다)
///   crc_err : 동기는 맞았는데 CRC 가 틀린 횟수를 **더한다**
/// 반환: 프레임을 하나 꺼냈으면 true.
///
/// **CRC 가 틀리면 24 가 아니라 2 바이트만 버린다.** 0xAA55 가 데이터에 우연히
/// 나타났을 수 있어서, 24 를 통째로 버리면 바로 뒤의 진짜 프레임까지 삼킨다.
inline bool next(const uint8_t* buf, size_t len, Frame& out,
                 size_t& consumed, uint32_t& crc_err) {
  size_t i = 0;
  while (i + FRAME_LEN <= len) {
    if (buf[i] != SYNC0 || buf[i + 1] != SYNC1) { ++i; continue; }
    if (decode(buf + i, out)) {
      consumed = i + FRAME_LEN;
      return true;
    }
    ++crc_err;
    i += 2;
  }
  // 프레임을 못 찾았다. 부분 프레임이 남을 수 있으니 마지막 FRAME_LEN-1 바이트는 남긴다.
  consumed = (len >= (size_t)FRAME_LEN) ? (len - (FRAME_LEN - 1)) : 0;
  return false;
}

/// seq 불연속으로 유실 프레임 수를 센다 (16 비트 랩어라운드 처리).
inline uint16_t lost_between(uint16_t prev, uint16_t now) {
  return (uint16_t)(now - prev - 1);
}

}  // namespace teensy

#endif  // TEENSY_FRAME_HPP_
