import time
import struct
import sys
import os
import threading
from canlib import canlib, Frame

CHANNEL_NUM = 0
NOMINAL_BITRATE = canlib.canBITRATE_1M
FD_BITRATE_MANUAL = 5000000
FD_TSEG1 = 11
FD_TSEG2 = 4
FD_SJW   = 4

# raw[3] (PA7) ADC -> mV -> 반전증폭 복원 mV
ADC_REF_MV = 3300.0
ADC_RES    = 4095.0
TO_MV = ADC_REF_MV / ADC_RES

def adc_to_orig_mv(raw_adc):
    adc_mv = raw_adc * TO_MV
    adc_mv = max(0.0, min(3300.0, adc_mv))
    return (4125.0 - adc_mv) / 0.825

board_data = {i: [0, 0, 0, 0, 0.0] for i in range(1, 23)}  # [raw0, raw1, raw2, raw3, orig_mv]
last_recv_time = {i: 0.0 for i in range(1, 23)}
running = True
rx_count = 0

def open_channel():
    try:
        ch = canlib.openChannel(channel=CHANNEL_NUM, flags=canlib.Open.CAN_FD)
        try:
            ch.setBusParams(NOMINAL_BITRATE)
            ch.setBusParamsFd(FD_BITRATE_MANUAL, FD_TSEG1, FD_TSEG2, FD_SJW)
        except Exception:
            pass
        try:
            ch.busOn()
        except Exception:
            pass
        return ch
    except Exception as e:
        print(f"[Fatal] 채널 열기 실패: {e}")
        sys.exit(1)

def rx_thread_func(ch):
    global rx_count
    while running:
        try:
            msg = ch.read(timeout=50)
            if 0x121 <= msg.id <= 0x136:
                board_id = msg.id - 0x120
                rx_count += 1

                if board_id >= 17:
                    # 엔코더 보드 (17~22): 최대 8바이트 파싱
                    d = msg.data + bytes(max(0, 8 - len(msg.data)))  # zero-pad to 8
                    raw = struct.unpack('<HHHH', d[:8])
                    orig_mv = adc_to_orig_mv(raw[3])
                    board_data[board_id] = [raw[0], raw[1], raw[2], raw[3], orig_mv]
                elif len(msg.data) >= 8:
                    # 압력/전류 보드 (1~16)
                    raw = struct.unpack('<HHHH', msg.data)
                    orig_mv = adc_to_orig_mv(raw[3])
                    board_data[board_id] = [raw[0], raw[1], raw[2], raw[3], orig_mv]

                last_recv_time[board_id] = time.time()
        except (canlib.canNoMsg, canlib.canError):
            continue

def print_dashboard():
    now = time.time()
    output = "\033[H"
    output += f"================ CAN Raw ADC Monitor ================\n"
    output += f" RX Count: {rx_count}\n"
    output += f" PA7 orig_mV = (4125 - raw[3]*{TO_MV:.4f}) / 0.825\n"
    output += f"------------------------------------------------------\n"
    output += f"| ID | raw[0] | raw[1] | raw[2] | raw[3] | orig_mV | State |\n"
    output += f"|----|--------|--------|--------|--------|---------|-------|\n"

    active_cnt = 0
    for bid in range(1, 23):
        d = board_data[bid]
        t_last = last_recv_time[bid]
        if now - t_last < 1.5 and t_last != 0:
            status = "OK"
            active_cnt += 1
        else:
            status = "Lost"

        output += (f"| {bid:02d} | {d[0]:6d} | {d[1]:6d} | {d[2]:6d} | {d[3]:6d} | {d[4]:7.1f} | {status:^5} |\n")

    output += f"======================================================\n"
    output += f" Active: {active_cnt} / 22   Ctrl+C to exit\n"

    sys.stdout.write(output)
    sys.stdout.flush()

def main():
    global running
    os.system('clear')
    print("Initializing Raw Monitor...")

    ch = open_channel()
    t_rx = threading.Thread(target=rx_thread_func, args=(ch,))
    t_rx.daemon = True
    t_rx.start()

    try:
        while True:
            print_dashboard()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        running = False
        t_rx.join(timeout=1.0)
        try: ch.close()
        except: pass
        print("Done.")

if __name__ == "__main__":
    main()
