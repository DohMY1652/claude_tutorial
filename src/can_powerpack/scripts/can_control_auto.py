#!/usr/bin/env python3
"""
can_control_auto.py — 원하는 주파수로 각 보드에 PWM 값을 자동 송신 + 수신율 측정

용도 두 가지:
  1. **버스 시험** — 송신율이 보드 수신율에 미치는 영향을 잰다. 20260902 에
     브리지 송신 100 Hz 에서 엔코더 보드(16~22)가 1 Hz 로 굶는 것을 찾았는데,
     그때까지의 시험은 모두 **값이 0 으로 고정**이었다. 보드가 PWM 레지스터를
     실제로 갱신할 때 비용이 다를 수 있어 `--pattern` 으로 갈라 본다.
  2. **밸브 구동 시험** — 실제로 값을 넣어 밸브·전류를 확인한다.

프레임 형식은 can_control.py 와 같다 (보드가 이미 받는 형식):
  0x100  보드 1~10   : uint16 v1,v2,v3 × 10 = 60 B, [60]=mode [61]=type [63]=heartbeat, 64 B
  0x101  보드 11~17  : uint16 × 7      = 42 B, [42]=mode [43]=type [47]=heartbeat, 48 B
  0x102  보드 18~22  : uint16 × 5      = 30 B, [30]=mode [31]=type [31+4]=heartbeat, 32 B

**전제 조건**
  · pp_controller / can_bridge_node 를 띄우지 말 것. 같은 0x100/0x101 에 발행해
    서로 덮어쓴다. 시작 시 검사한다.
  · can_monitor.py 는 같이 띄워도 된다 (수신 전용).

**안전**
  · **전 밸브(v1·v2·v3)에 값이 그대로 들어간다.** macro 도 포함이다 —
    macro 배관을 물리적으로 떼고 쓰는 것을 전제한다. 탱크가 충전된 채 macro 가
    연결돼 있으면 챔버가 과압된다 (20260829 에 액추에이터가 그렇게 부서졌다,
    HANDOFF S-34). 배관을 못 떼면 `--no-macro` 로 v3 를 0 에 묶을 것.
  · `--max` 로 값 상한을 둘 수 있다 (기본 4095).
  · 종료 시(Ctrl-C 포함) **전 밸브 0 을 세 번 보내고** 끝난다.
  · `busOff()` 는 절대 부르지 않는다 — 공유 채널을 내리면 어댑터가
    BUS_OFF+TX_PEND 로 잠겨 재연결해야 풀린다 (20260902 에 겪음).

사용 예
  # 버스 시험: 30 Hz, 값 0 고정 (밸브 안 움직임)
  python3 can_control_auto.py --hz 30 --pattern zero --seconds 12

  # 값이 매번 바뀌는 경우와 비교
  python3 can_control_auto.py --hz 30 --pattern random --seconds 12

  # 송신율 쓸어보기
  python3 can_control_auto.py --sweep 0.5,5,25,50,100 --pattern random

  # macro 배관을 못 뗐을 때 (v3 를 0 에 묶는다)
  python3 can_control_auto.py --hz 20 --pattern random --no-macro
"""
import argparse
import random
import struct
import subprocess
import sys
import threading
import time

from canlib import canlib, Frame

CHANNEL_NUM = 0
NOMINAL_BITRATE = canlib.canBITRATE_1M
DATA_BITRATE = 5000000
FD_TSEG1, FD_TSEG2, FD_SJW = 11, 4, 4
FDBRS = canlib.MessageFlag.FDF | canlib.MessageFlag.BRS

# (CAN ID, 첫 보드, 마지막 보드, 총 바이트, mode 오프셋)
GROUPS = [
    (0x100, 1, 10, 64, 60),
    (0x101, 11, 17, 48, 42),
    (0x102, 18, 22, 32, 30),
]
BOARD_ID_BASE = 0x120          # 보드 N 의 상태 프레임 = 0x120 + N
N_BOARDS = 22
PWM_MAX = 4095


def _pad(txt, w):
    """한글은 터미널에서 두 칸을 먹는다 — 그걸 감안해 오른쪽 정렬한다."""
    import unicodedata
    disp = sum(2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in txt)
    return ' ' * max(0, w - disp) + txt



def controller_running():
    """pp_controller / can_bridge_node 가 떠 있으면 이름을 돌려준다."""
    for name in ('pp_controller', 'can_bridge_node'):
        r = subprocess.run(['pgrep', '-af', name], capture_output=True, text=True)
        if r.returncode != 0:
            continue
        for ln in r.stdout.splitlines():
            pid, _, cmd = ln.partition(' ')
            if not cmd:
                continue
            import os
            if os.path.basename(cmd.split()[0]) == name:
                return name
    return None


def open_channel():
    ch = canlib.openChannel(channel=CHANNEL_NUM, flags=canlib.Open.CAN_FD)
    try:
        ch.setBusParams(NOMINAL_BITRATE)
        ch.setBusParamsFd(DATA_BITRATE, FD_TSEG1, FD_TSEG2, FD_SJW)
    except Exception:
        pass          # CanKing 등이 이미 설정해 둔 경우 — 무시한다
    ch.busOn()
    return ch


def make_targets(pattern, vmax, no_macro, tick):
    """보드 1~22 각각의 (v1, v2, v3) 를 만든다."""
    t = {}
    for b in range(1, N_BOARDS + 1):
        if pattern == 'zero':
            v = [0, 0, 0]
        elif pattern == 'random':
            v = [random.randint(0, vmax) for _ in range(3)]
        elif pattern == 'const':
            v = [vmax, vmax, vmax]
        elif pattern == 'ramp':
            # 보드마다 위상을 달리해 매 틱 값이 바뀌게 한다
            v = [((tick * 37 + b * 11 + k * 5) % (vmax + 1)) for k in range(3)]
        else:
            raise ValueError(pattern)
        if no_macro:
            v[2] = 0                                   # v3 = macro 를 묶는다
        t[b] = tuple(min(PWM_MAX, max(0, x)) for x in v)
    return t


def build_frames(targets, mode, ctype, heartbeat):
    frames = []
    for can_id, b0, b1, total, moff in GROUPS:
        p = bytearray(total)
        off = 0
        for b in range(b0, b1 + 1):
            v1, v2, v3 = targets[b]
            struct.pack_into('<HHH', p, off, v1, v2, v3)
            off += 6
        p[moff] = mode & 0xFF
        p[moff + 1] = ctype & 0xFF
        hb = moff + 5 if moff + 5 < total else total - 1
        p[hb] = heartbeat & 0xFF
        frames.append(Frame(id_=can_id, data=bytes(p), flags=FDBRS))
    return frames


def send_all_zero(ch, times=3):
    """정지용 — 전 밸브 0. 예외를 삼켜서라도 보낸다."""
    z = {b: (0, 0, 0) for b in range(1, N_BOARDS + 1)}
    for _ in range(times):
        for f in build_frames(z, 0, 0, 0):
            try:
                ch.write(f)
            except Exception:
                pass
        time.sleep(0.01)


def run_once(ch, hz, secs, pattern, vmax, no_macro, live=True):
    """hz 로 secs 초 송신하며 실제 송신 주파수와 보드별 수신율을 잰다."""
    stop = threading.Event()
    per = {}
    cyc_n = [0]          # 송신 사이클 수 (= 실제 지령 주파수)
    frm_n = [0]          # 송신 프레임 수 (사이클당 3 개)
    late = [0]           # 주기를 못 맞춘 사이클 수
    worst = [0.0]        # 최악 사이클 소요 시간 [ms]

    def rx():
        while not stop.is_set():
            try:
                m = ch.read(timeout=50)
                b = m.id - BOARD_ID_BASE
                if 1 <= b <= N_BOARDS:
                    per[b] = per.get(b, 0) + 1
            except (canlib.canNoMsg, canlib.canError):
                pass

    def tx():
        hb = 0
        tick = 0
        period = 1.0 / hz
        nxt = time.perf_counter()
        while not stop.is_set():
            t_a = time.perf_counter()
            hb = (hb + 1) & 0xFF
            tick += 1
            for f in build_frames(make_targets(pattern, vmax, no_macro, tick),
                                  0, 0, hb):
                try:
                    ch.write(f)
                    frm_n[0] += 1
                except Exception:
                    pass
            cyc_n[0] += 1
            el = (time.perf_counter() - t_a) * 1e3
            if el > worst[0]:
                worst[0] = el
            nxt += period
            d = nxt - time.perf_counter()
            if d > 0:
                time.sleep(d)
            else:
                late[0] += 1          # 이번 주기를 못 지켰다
                nxt = time.perf_counter()

    tr = threading.Thread(target=rx, daemon=True)
    tr.start()
    tt = threading.Thread(target=tx, daemon=True) if hz > 0 else None
    if tt:
        tt.start()

    time.sleep(1.0)                   # 안정화 — 통계에서 뺀다
    per.clear()
    cyc_n[0] = frm_n[0] = late[0] = 0
    worst[0] = 0.0

    t0 = time.perf_counter()
    p_c = p_f = 0
    p_rx = 0
    p_t = t0
    try:
        while True:
            now = time.perf_counter()
            if now - t0 >= secs:
                break
            time.sleep(min(1.0, max(0.02, secs - (now - t0))))
            now = time.perf_counter()
            if not live:
                continue
            w = now - p_t
            if w < 0.2:
                continue
            c, f_, r = cyc_n[0], frm_n[0], sum(per.values())
            print(f'    · 실송신 {(c - p_c) / w:6.1f} Hz  '
                  f'({(f_ - p_f) / w:5.0f} f/s)   '
                  f'수신 {(r - p_rx) / w:6.0f} f/s', flush=True)
            p_c, p_f, p_rx, p_t = c, f_, r, now
    finally:
        dt = time.perf_counter() - t0
        stop.set()
        time.sleep(0.3)

    b = [per.get(i, 0) / dt for i in range(1, N_BOARDS + 1)]
    return {
        'tx_hz': cyc_n[0] / dt,          # **실제로 나간 지령 주파수**
        'tx_fps': frm_n[0] / dt,
        'late': late[0],
        'worst_ms': worst[0],
        'rx_tot': sum(b),
        'g1': sum(b[0:10]) / 10,
        'g2': sum(b[10:15]) / 5,
        'g3': sum(b[15:22]) / 7,
        'per': b,
    }


def main():
    ap = argparse.ArgumentParser(
        description='각 보드에 PWM 값을 자동 송신하고, 실제 송신 주파수와 보드별 수신율을 잰다.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split('사용 예')[-1])
    ap.add_argument('--hz', type=float, default=20.0, help='목표 송신 주파수 [Hz] (기본 20)')
    ap.add_argument('--seconds', type=float, default=10.0, help='측정 시간 [s] (기본 10)')
    ap.add_argument('--pattern', choices=('zero', 'random', 'const', 'ramp'),
                    default='random', help='값 생성 방식 (기본 random)')
    ap.add_argument('--max', dest='vmax', type=int, default=PWM_MAX,
                    help=f'값 상한 0..{PWM_MAX} (기본 {PWM_MAX})')
    ap.add_argument('--no-macro', action='store_true',
                    help='v3(macro)를 0 에 묶는다. macro 배관을 못 뗐을 때 쓸 것.')
    ap.add_argument('--sweep', type=str, default=None,
                    help='주파수 목록을 쉼표로 (예: 0.5,5,25,50,100). --hz 를 무시한다.')
    ap.add_argument('--per-board', action='store_true',
                    help='끝에 보드 1~22 개별 수신율을 찍는다.')
    ap.add_argument('--quiet', action='store_true', help='1 초 주기 실시간 출력을 끈다.')
    ap.add_argument('--force', action='store_true',
                    help='pp_controller / can_bridge_node 가 떠 있어도 진행한다.')
    a = ap.parse_args()

    if not 0 <= a.vmax <= PWM_MAX:
        ap.error(f'--max 는 0..{PWM_MAX}')
    if a.hz <= 0:
        ap.error('--hz 는 0 보다 커야 한다')

    other = controller_running()
    if other and not a.force:
        print(f'[중단] {other} 가 돌고 있다. 같은 0x100/0x101 에 발행해 서로 덮어쓴다.\n'
              f'        먼저 내리거나 --force 로 무시할 것.')
        return 1

    print(f'[can_control_auto] 패턴 {a.pattern}   상한 {a.vmax}   '
          f'macro(v3) {"0 고정" if a.no_macro else "랜덤 포함"}')
    if a.pattern == 'zero':
        print('  (값이 전부 0 이라 밸브는 움직이지 않는다 — 순수 버스 시험)')
    elif not a.no_macro:
        print('  ** macro 밸브에도 값이 들어간다. 배관이 물려 있고 탱크가 충전돼 있으면 '
              '챔버가 과압된다. **')

    ch = open_channel()
    rows = []
    try:
        hz_list = ([float(x) for x in a.sweep.split(',') if x.strip()]
                   if a.sweep else [a.hz])
        for hz in hz_list:
            print(f'\n  ── 목표 {hz:g} Hz, {a.seconds:g} s ──')
            r = run_once(ch, hz, a.seconds, a.pattern, a.vmax, a.no_macro,
                         live=not a.quiet)
            r['want'] = hz
            rows.append(r)
            print(f'    → 실제 송신 {r["tx_hz"]:.1f} Hz '
                  f'(목표의 {100.0 * r["tx_hz"] / hz:.0f} %), '
                  f'{r["tx_fps"]:.0f} f/s, '
                  f'주기 미달 {r["late"]} 회, 최악 사이클 {r["worst_ms"]:.1f} ms')
            send_all_zero(ch, 1)

        print()
        print('  ' + ''.join(_pad(t, w) for t, w in (
            ('목표', 9), ('실제송신', 11), ('송신 f/s', 10), ('총수신', 9),
            ('보드1~10', 10), ('보드11~15', 11), ('보드16~22', 11))))
        print('  ' + '-' * 71)
        for r in rows:
            print(f'  {r["want"]:>7.1f}Hz{r["tx_hz"]:9.1f}Hz{r["tx_fps"]:10.0f}'
                  f'{r["rx_tot"]:9.0f}{r["g1"]:10.0f}{r["g2"]:11.0f}{r["g3"]:11.0f}')
        if a.per_board and rows:
            print('\n  보드별 수신율 [Hz] (마지막 구간)')
            pb = rows[-1]['per']
            for i in range(0, N_BOARDS, 11):
                print('   ' + ' '.join(f'{j + 1:>2}:{pb[j]:>4.0f}'
                                       for j in range(i, min(i + 11, N_BOARDS))))
        print()
        print('  * 송신이 없을 때 보드당 500 Hz 가 정상이다.')
        print('    보드 16~22 는 엔코더 — 여기가 굶으면 위치 제어가 눈을 잃는다.')
    except KeyboardInterrupt:
        print('\n[중단됨]')
    finally:
        print('[정지] 전 밸브 0 송신')
        send_all_zero(ch)
        # busOff() 는 부르지 않는다 — 공유 채널을 내리면 어댑터가 잠긴다.
        try:
            ch.close()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
