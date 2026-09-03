"""실기 캡처(test/data/teensy_5s.bin)를 pty 로 200 Hz 재생한다 — Teensy 대역.

인자: 심링크 경로. 매 실행마다 pty 번호가 바뀌므로, 고정 경로인 심링크를
      브리지에 물려 두면 '케이블 뽑았다 꽂기'를 흉내 낼 수 있다.
"""
import os
import pty
import sys
import threading
import time

LINK = sys.argv[1] if len(sys.argv) > 1 else "/tmp/tlink"
CAP = "/home/risebrl/claude_tutorial/src/can_powerpack/test/data/teensy_5s.bin"

data = open(CAP, "rb").read()
m, s = pty.openpty()
name = os.ttyname(s)
try:
    os.remove(LINK)
except OSError:
    pass
os.symlink(name, LINK)
with open(LINK + ".port", "w") as f:
    f.write(name + "\n")
print(name, flush=True)


def feed():
    os.set_blocking(m, False)
    started = False
    while not started:                 # 'r' 을 받아야 시작 (실기와 같은 동작)
        try:
            if b"r" in os.read(m, 16):
                started = True
        except BlockingIOError:
            time.sleep(0.01)
        except OSError:
            return
    i = 0
    t = time.perf_counter()
    while True:
        if i + 24 > len(data):
            i = 0
        try:
            os.write(m, data[i:i + 24])
        except BlockingIOError:
            pass          # 읽는 쪽이 없어 버퍼가 참 — 프레임을 버리고 계속 간다
        except OSError:
            return
        i += 24
        t += 0.005
        d = t - time.perf_counter()
        if d > 0:
            time.sleep(d)
        else:
            t = time.perf_counter()


threading.Thread(target=feed, daemon=True).start()
while True:
    time.sleep(3600)
