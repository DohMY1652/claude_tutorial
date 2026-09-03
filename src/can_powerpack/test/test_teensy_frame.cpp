// Teensy 프레임 해독기(include/TeensyFrame.hpp) 검증.
//
// **실기 캡처로 돌린다.** test/data/teensy_5s.bin 은 20260903 에 실제 Teensy 에서
// 5 초 동안 받아 저장한 것이다 (1000 프레임, 200.0 Hz, CRC 전부 통과, 유실 0).
// 하드웨어 없이도 파서 회귀를 잡을 수 있는 유일한 수단이라 저장소에 넣어 두었다.
//
// 빌드·실행:  colcon build --packages-select can_powerpack
//             ./build/can_powerpack/test_teensy_frame src/can_powerpack/test/data/teensy_5s.bin

#include "TeensyFrame.hpp"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

static int g_fail = 0;

static void check(bool ok, const char* what) {
  std::printf("  [%s] %s\n", ok ? "OK  " : "실패", what);
  if (!ok) ++g_fail;
}

static void checkf(bool ok, const char* fmt, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  check(ok, buf);
}

int main(int argc, char** argv) {
  const std::string path = (argc > 1) ? argv[1] : "test/data/teensy_5s.bin";

  std::printf("\nTeensy 프레임 해독기 검증 — %s\n", path.c_str());
  std::printf("────────────────────────────────────────────────────────────\n");

  // ── 1. CRC 가 teensy_monitor.py 와 같은가 ────────────────────────────────
  // 파이썬으로 미리 구한 기준값. CRC 구현이 바뀌면 여기서 먼저 걸린다.
  {
    const uint8_t v[] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
    checkf(teensy::crc16(v, 9) == 0x29B1,
           "CRC16-CCITT('123456789') = 0x%04X (기준 0x29B1)", teensy::crc16(v, 9));
  }

  // ── 2. 실기 캡처 읽기 ────────────────────────────────────────────────────
  std::FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) {
    std::printf("  [실패] 캡처 파일을 못 연다: %s\n", path.c_str());
    std::printf("         (경로를 인자로 줄 것)\n\n");
    return 1;
  }
  std::fseek(f, 0, SEEK_END);
  const long sz = std::ftell(f);
  std::fseek(f, 0, SEEK_SET);
  std::vector<uint8_t> data((size_t)sz);
  if (std::fread(data.data(), 1, (size_t)sz, f) != (size_t)sz) {
    std::printf("  [실패] 캡처를 다 못 읽었다\n");
    std::fclose(f);
    return 1;
  }
  std::fclose(f);
  checkf(sz == 24000, "캡처 크기 %ld 바이트 (기대 24000 = 1000프레임 × 24 B)", sz);

  // ── 3. 통째로 넣었을 때 전부 해독되는가 ──────────────────────────────────
  size_t off = 0;
  uint32_t crc_err = 0, n = 0, lost = 0, status_or = 0;
  bool have_prev = false;
  uint16_t prev_seq = 0;
  int32_t mn[teensy::NCH], mx[teensy::NCH];
  for (int c = 0; c < teensy::NCH; ++c) { mn[c] = 1 << 30; mx[c] = -(1 << 30); }
  uint32_t prev_t = 0;
  long dt_min = 1 << 30, dt_max = 0;
  bool have_t = false;

  while (off < data.size()) {
    teensy::Frame fr;
    size_t consumed = 0;
    const bool got = teensy::next(data.data() + off, data.size() - off,
                                  fr, consumed, crc_err);
    off += consumed;
    if (!got) break;
    ++n;
    if (have_prev) lost += teensy::lost_between(prev_seq, fr.seq);
    prev_seq = fr.seq;
    have_prev = true;
    status_or |= fr.status;
    for (int c = 0; c < teensy::NCH; ++c) {
      if (fr.ch[c] < mn[c]) mn[c] = fr.ch[c];
      if (fr.ch[c] > mx[c]) mx[c] = fr.ch[c];
    }
    if (have_t) {
      const long d = (long)(fr.t_us - prev_t);
      if (d < dt_min) dt_min = d;
      if (d > dt_max) dt_max = d;
    }
    prev_t = fr.t_us;
    have_t = true;
  }

  checkf(n == 1000, "프레임 %u 개 해독 (기대 1000)", n);
  checkf(crc_err == 0, "CRC 오류 %u (기대 0)", crc_err);
  checkf(lost == 0, "seq 유실 %u (기대 0)", lost);
  checkf(status_or == 0, "status OR = 0x%04X (기대 0x0000 = I2C 오류 없음)", status_or);
  checkf(dt_min >= 4990 && dt_max <= 5010,
         "프레임 간격 %ld~%ld us (기대 5000±10 = 200 Hz)", dt_min, dt_max);
  std::printf("       채널 raw 범위:");
  for (int c = 0; c < teensy::NCH; ++c) std::printf(" ch%d %d~%d", c, mn[c], mx[c]);
  std::printf("\n");

  // ── 4. 바이트를 잘게 쪼개 넣어도 같은 결과인가 (스트리밍 재조립) ─────────
  // 실기에서는 read() 가 프레임 경계와 무관하게 잘라 준다. 여기서 안 깨져야 한다.
  {
    std::vector<uint8_t> buf;
    uint32_t n2 = 0, ce2 = 0;
    size_t src = 0;
    const size_t CH = 7;                 // 24 와 서로소인 크기로 일부러 어긋나게
    while (src < data.size() || !buf.empty()) {
      if (src < data.size()) {
        const size_t take = std::min(CH, data.size() - src);
        buf.insert(buf.end(), data.begin() + (long)src, data.begin() + (long)(src + take));
        src += take;
      }
      while (true) {
        teensy::Frame fr;
        size_t consumed = 0;
        const bool got = teensy::next(buf.data(), buf.size(), fr, consumed, ce2);
        if (consumed) buf.erase(buf.begin(), buf.begin() + (long)consumed);
        if (!got) break;
        ++n2;
      }
      if (src >= data.size() && buf.size() < teensy::FRAME_LEN) break;
    }
    checkf(n2 == n, "%zu 바이트씩 쪼개 넣어도 %u 프레임 (통째 %u)", CH, n2, n);
    checkf(ce2 == 0, "쪼개 넣었을 때 CRC 오류 %u (기대 0)", ce2);
  }

  // ── 5. 쓰레기가 앞에 붙어도 재동기하는가 ─────────────────────────────────
  {
    std::vector<uint8_t> junk = {0x00, 0xAA, 0xAA, 0x55, 0x01, 0xFF, 0xAA, 0x55};
    junk.insert(junk.end(), data.begin(), data.begin() + 24 * 10);
    uint32_t n3 = 0, ce3 = 0;
    size_t o = 0;
    while (o < junk.size()) {
      teensy::Frame fr;
      size_t consumed = 0;
      const bool got = teensy::next(junk.data() + o, junk.size() - o, fr, consumed, ce3);
      o += consumed;
      if (!got) break;
      ++n3;
    }
    checkf(n3 == 10, "앞에 쓰레기 8 바이트를 붙여도 10 프레임 회수 (%u)", n3);
    // 가짜 0xAA55 두 개가 CRC 에서 걸러졌어야 한다 — 그때 24 를 통째로 버렸다면
    // 진짜 프레임을 삼켜 n3 < 10 이 된다.
    checkf(ce3 >= 1, "가짜 SYNC 를 CRC 로 걸러냈다 (crc_err=%u)", ce3);
  }

  // ── 6. 한 비트만 뒤집으면 반드시 잡히는가 ────────────────────────────────
  {
    std::vector<uint8_t> bad(data.begin(), data.begin() + 24);
    bad[10] ^= 0x01;
    teensy::Frame fr;
    check(!teensy::decode(bad.data(), fr), "데이터 1비트 반전을 CRC 가 잡는다");
  }

  std::printf("────────────────────────────────────────────────────────────\n");
  if (g_fail == 0) std::printf("전부 통과\n\n");
  else             std::printf("**%d 항목 실패**\n\n", g_fail);
  return g_fail ? 1 : 0;
}
