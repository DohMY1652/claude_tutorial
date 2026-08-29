#!/usr/bin/env python3
"""
CAN 연결 자동 진단 (can_monitor.py + can_control.py 결합)

Phase 1 - Discovery : 모든 보드(1~22)로부터 오는 RX 프레임을 감시하여
                       실제로 버스에 붙어있는 보드가 몇 개/어느 ID인지 확인.
Phase 2 - Channel test : 제어 가능한 보드(1~22)에 대해 채널(CH1~CH3)을
                       하나씩 4095(PWM Full)로 구동하고, 그 보드의 전류
                       피드백(mA)이 임계값을 넘는지로 "채널 연결 여부" 판정.
Phase 3 - Summary : 보드 개수 / 보드별 연결 채널 수 / (17~22) 엔코더 각도를 표로 출력.

[!] 보드 18~22 제어(CMD_ID_GRP3 = 0x102)는 아직 실제 액추에이터가 없는
    상태에서 "나중에 추가될 액추에이터를 미리 테스트할 수 있게" 만든
    확장 프레임입니다. can_control.py / CanBridge.cpp 어느 쪽에도 이
    ID를 실제로 수신하는 펌웨어가 아직 없으므로 지금은 전송해도 아무
    반응이 없는 것이 정상입니다. 나중에 18~22번 보드에 밸브가 붙고
    펌웨어가 업데이트되면, 그 펌웨어가 실제로 사용하는 CAN ID/payload
    포맷과 이 값(0x102, 5보드 x 6바이트+모드/타입 2바이트=32바이트)이
    일치하는지 반드시 재확인하세요.

주의: 실제 밸브/액추에이터가 순간적으로 Full 구동됩니다. 각 채널은
      SETTLE_ON_S 동안만 켜고 즉시 0으로 되돌립니다.
"""
import time
import struct
import sys
import os
import threading
from canlib import canlib, Frame

# ================= 1. 공통 설정값 =================
CHANNEL_NUM = 0
NOMINAL_BITRATE = canlib.canBITRATE_1M
FD_BITRATE_MANUAL = 5000000
FD_TSEG1 = 11
FD_TSEG2 = 4
FD_SJW   = 4

CMD_ID_GRP1 = 0x100   # 보드 1~10
CMD_ID_GRP2 = 0x101   # 보드 11~17
CMD_ID_GRP3 = 0x102   # 보드 18~22  [!] 임시/미확정 ID - 위 docstring 참고
ESTOP_ID    = 0x000

# [전류 센서 물리량 변환] (can_monitor.py와 동일)
SHUNT_OHM  = 0.1
AMP_GAIN   = 100.0
ADC_REF_MV = 3300.0
ADC_RES    = 4095.0
TO_MV = ADC_REF_MV / ADC_RES

# [엔코더 캘리브레이션] (can_monitor.py와 동일, 정보 표시용)
ENCODER_OFFSET = 1740.0
ENCODER_GAIN   = 105.0 / (3127.0 - 1740.0)

# 보드별 2 점 실측 보정. **단일 출처는 powerpack_config.yaml 이다** —
# can_monitor.py 가 그 yaml 을 읽어 ENCODER_CALIB 을 만든다. 여기서 그걸 그대로
# 빌려 쓴다. 못 가져오면 위 일반 기본값으로 도는데, 실측 보드는 gain 이 **음수**라
# 각도가 반대로 읽힌다 — 그래서 무엇을 쓰는지 반드시 밝힌다.
try:
    import os as _os, sys as _sys
    _sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
    from can_monitor import ENCODER_CALIB, _CALIB_SOURCE as _ENC_SRC
except Exception as _e:
    ENCODER_CALIB, _ENC_SRC = {}, f'일반 기본값 (can_monitor 를 못 읽었다: {_e})'

# ================= 2. 테스트 파라미터 =================
NUM_BOARDS = 22
CONTROLLABLE_BOARDS = range(1, NUM_BOARDS + 1)   # 1~22 (18~22는 향후 액추에이터 대비)
ENCODER_BOARDS = range(17, NUM_BOARDS + 1)       # 17~22: 엔코더(각도) 겸용 보드

TEST_VALUE      = 4095    # 채널 테스트 시 인가할 제어값 (PWM full)
CURRENT_THRESH_MA = 200.0 # 연결 판정 전류 임계값

DISCOVERY_S   = 2.0   # Phase 1: 보드 존재 여부 확인 시간
SETTLE_ON_S   = 0.4   # 채널 ON 유지 후 대기 (전류 안정화)
SAMPLE_S      = 0.2   # ON 유지 중 전류 샘플링(peak 관측) 시간
SETTLE_OFF_S  = 0.2   # 채널 OFF 후 다음 테스트 전 대기

# ================= 3. 공유 상태 =================
board_currents  = {i: [0.0, 0.0, 0.0] for i in range(1, NUM_BOARDS + 1)}  # [I1,I2,I3] mA
board_angles    = {i: None for i in ENCODER_BOARDS}                       # deg (17~22)
last_recv_time  = {i: 0.0 for i in range(1, NUM_BOARDS + 1)}

targets       = [[0, 0, 0] for _ in range(NUM_BOARDS + 1)]  # index 1..22 사용
current_mode  = 0   # 0:Normal, 1:Debug (PWM은 Debug 필요)
control_type  = 0   # 0:PID, 1:PWM, 2:ID, 3:ZERO

running = True
lock = threading.Lock()


def calc_current_ma(mv_value):
    if AMP_GAIN == 0:
        return 0.0
    return mv_value / (SHUNT_OHM * AMP_GAIN)


def calc_original_voltage_mv(adc_mv):
    if adc_mv > 3300.0: adc_mv = 3300.0
    if adc_mv < 0.0: adc_mv = 0.0
    return (4125.0 - adc_mv) / 0.825


def calc_angle_deg(orig_mv, board_id):
    offset, gain = ENCODER_CALIB.get(board_id, (ENCODER_OFFSET, ENCODER_GAIN))
    return (orig_mv - offset) * gain


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


# ================= 4. RX 스레드 (can_monitor.py 로직 확장) =================
def rx_thread_func(ch):
    while running:
        try:
            msg = ch.read(timeout=50)
            if 0x121 <= msg.id <= 0x136:
                board_id = msg.id - 0x120
                last_recv_time[board_id] = time.time()

                if len(msg.data) >= 8:
                    raw = struct.unpack('<HHHH', msg.data[:8])
                    # raw[0..2] -> I1,I2,I3 (18~22는 아직 실제 센서가 없어 0에 가까운 값이
                    # 나오는 것이 정상이며, 나중에 액추에이터가 붙으면 그대로 유효해짐)
                    i1 = calc_current_ma(raw[0] * TO_MV)
                    i2 = calc_current_ma(raw[1] * TO_MV)
                    i3 = calc_current_ma(raw[2] * TO_MV)
                    with lock:
                        board_currents[board_id] = [i1, i2, i3]

                    if board_id in ENCODER_BOARDS:
                        orig_mv = calc_original_voltage_mv(raw[3] * TO_MV)
                        angle = calc_angle_deg(orig_mv, board_id)
                        with lock:
                            board_angles[board_id] = angle
        except (canlib.canNoMsg, canlib.canError):
            continue


# ================= 5. TX 스레드 (can_control.py 로직 확장) =================
def tx_thread_func(ch):
    global running
    while running:
        try:
            with lock:
                snap = [row[:] for row in targets]
                mode, ctype = current_mode, control_type

            payload_g1 = b''
            for bid in range(1, 11):
                v1, v2, v3 = snap[bid]
                payload_g1 += struct.pack('<HHH', v1, v2, v3)
            payload_g1 += struct.pack('<BB', mode, ctype)
            payload_g1 += b'\x00' * (64 - len(payload_g1))
            ch.write(Frame(id_=CMD_ID_GRP1, data=payload_g1,
                            flags=canlib.MessageFlag.FDF | canlib.MessageFlag.BRS))

            payload_g2 = b''
            for bid in range(11, 18):
                v1, v2, v3 = snap[bid]
                payload_g2 += struct.pack('<HHH', v1, v2, v3)
            payload_g2 += struct.pack('<BB', mode, ctype)
            payload_g2 += b'\x00' * (48 - len(payload_g2))
            ch.write(Frame(id_=CMD_ID_GRP2, data=payload_g2,
                            flags=canlib.MessageFlag.FDF | canlib.MessageFlag.BRS))

            # 보드 18~22 (향후 액추에이터 대비, 임시 CAN ID)
            payload_g3 = b''
            for bid in range(18, 23):
                v1, v2, v3 = snap[bid]
                payload_g3 += struct.pack('<HHH', v1, v2, v3)
            payload_g3 += struct.pack('<BB', mode, ctype)
            payload_g3 += b'\x00' * (32 - len(payload_g3))
            ch.write(Frame(id_=CMD_ID_GRP3, data=payload_g3,
                            flags=canlib.MessageFlag.FDF | canlib.MessageFlag.BRS))

            time.sleep(0.05)
        except Exception:
            time.sleep(0.2)


def set_target(board_id, ch_idx, value):
    with lock:
        row = [0, 0, 0]
        if ch_idx is not None:
            row[ch_idx] = value
        targets[board_id] = row


def set_mode(mode, ctype):
    global current_mode, control_type
    with lock:
        current_mode = mode
        control_type = ctype


def all_stop(ch):
    global current_mode, control_type
    with lock:
        for i in range(len(targets)):
            targets[i] = [0, 0, 0]
        current_mode = 0
        control_type = 0
    time.sleep(0.1)
    try:
        ch.write(Frame(id_=ESTOP_ID, data=b'',
                        flags=canlib.MessageFlag.FDF | canlib.MessageFlag.BRS))
    except Exception:
        pass


# ================= 6. Phase 1: Discovery =================
def run_discovery():
    print(f"[Phase 1] 보드 탐색 중... ({DISCOVERY_S:.0f}s)")
    t0 = time.time()
    while time.time() - t0 < DISCOVERY_S:
        remain = DISCOVERY_S - (time.time() - t0)
        sys.stdout.write(f"\r  탐색 중... {remain:4.1f}s 남음   ")
        sys.stdout.flush()
        time.sleep(0.1)
    print()

    now = time.time()
    present = {bid: (now - last_recv_time[bid] < 1.5 and last_recv_time[bid] != 0.0)
               for bid in range(1, NUM_BOARDS + 1)}
    return present


# ================= 7. Phase 2: Channel test =================
def run_channel_test(present):
    print(f"\n[Phase 2] 채널별 연결 테스트 시작 (값={TEST_VALUE}, 임계값={CURRENT_THRESH_MA:.0f}mA)")
    print(f"  * PWM/DEBUG 모드로 각 채널을 {SETTLE_ON_S+SAMPLE_S:.1f}s간 개별 구동합니다.")
    print(f"  * 보드 18~22는 아직 액추에이터가 없어 전류가 0mA 근처로 나오는 것이 정상입니다"
          f" (향후 액추에이터 추가 시 재실행하면 그대로 판정 가능).\n")

    set_mode(1, 1)  # DEBUG / PWM
    results = {}  # board_id -> [ (connected: bool, peak_ma: float), ... ] len 3

    try:
        for board_id in CONTROLLABLE_BOARDS:
            ch_results = []
            for ch_idx in range(3):
                label = f"Board {board_id:02d} / CH{ch_idx+1}"

                set_target(board_id, ch_idx, TEST_VALUE)
                time.sleep(SETTLE_ON_S)

                peak_ma = 0.0
                t_sample0 = time.time()
                while time.time() - t_sample0 < SAMPLE_S:
                    with lock:
                        cur = board_currents[board_id][ch_idx]
                    peak_ma = max(peak_ma, cur)
                    time.sleep(0.02)

                set_target(board_id, None, 0)
                time.sleep(SETTLE_OFF_S)

                connected = peak_ma >= CURRENT_THRESH_MA
                mark = "CONNECTED" if connected else "-"
                print(f"  {label:<22s}: {peak_ma:7.1f} mA  [{mark}]")
                ch_results.append((connected, peak_ma))

            results[board_id] = ch_results
    except KeyboardInterrupt:
        print("\n  [중단됨] 사용자에 의해 채널 테스트가 중단되었습니다.")

    return results


# ================= 8. Phase 3: Summary =================
def print_summary(present, results):
    print("\n" + "=" * 86)
    print(" CAN 연결 진단 결과 요약")
    print("=" * 86)

    board_count = sum(1 for v in present.values() if v)
    print(f" 감지된 보드 개수(RX 기준): {board_count} / {NUM_BOARDS}")
    detected_ids = [bid for bid, v in present.items() if v]
    print(f" 감지된 보드 ID: {detected_ids if detected_ids else '없음'}")

    print(f"\n [제어/전류 테스트 대상 보드 1~{NUM_BOARDS}]"
          f" (18~{NUM_BOARDS}는 액추에이터 추가 전이라 N/A로 보이는 것이 정상)")
    print(" --------------------------------------------------------------------------------")
    print(" | Board | RX  |     CH1     |     CH2     |     CH3     | 연결 채널 |  Angle(deg) |")
    print(" |-------|-----|-------------|-------------|-------------|-----------|-------------|")

    total_connected_channels = 0
    tested_boards = 0
    for board_id in CONTROLLABLE_BOARDS:
        rx_ok = "OK" if present.get(board_id) else "-"
        ch_results = results.get(board_id)

        if ch_results is None:
            row_cells = ["  (미실행)  "] * 3
            conn_cnt_str = "-"
        else:
            row_cells = []
            conn_cnt = 0
            for connected, peak in ch_results:
                if connected:
                    conn_cnt += 1
                row_cells.append(f"{peak:6.1f}mA {'OK' if connected else '  '}")
            conn_cnt_str = f"{conn_cnt}/3"
            total_connected_channels += conn_cnt
            tested_boards += 1

        if board_id in ENCODER_BOARDS and board_angles.get(board_id) is not None:
            angle_str = f"{board_angles[board_id]:7.2f}"
        else:
            angle_str = "-"

        print(f" | {board_id:5d} | {rx_ok:^3s} | {row_cells[0]:^11s} | "
              f"{row_cells[1]:^11s} | {row_cells[2]:^11s} |   {conn_cnt_str:^5s} | {angle_str:^11s} |")

    print(" --------------------------------------------------------------------------------")
    print(f" 전류 피드백 기반 연결 채널 총합: {total_connected_channels} "
          f"({tested_boards}개 보드 기준, 최대 {tested_boards*3})")


def main():
    global running
    os.system('cls' if os.name == 'nt' else 'clear')
    print("=" * 60)
    print("   CAN Connectivity Test (can_monitor + can_control)")
    print("=" * 60)

    ch = open_channel()

    t_rx = threading.Thread(target=rx_thread_func, args=(ch,))
    t_rx.daemon = True
    t_rx.start()

    t_tx = threading.Thread(target=tx_thread_func, args=(ch,))
    t_tx.daemon = True
    t_tx.start()

    results = {}
    present = {}
    try:
        present = run_discovery()
        results = run_channel_test(present)
    except KeyboardInterrupt:
        print("\n[중단됨] 사용자에 의해 테스트가 중단되었습니다.")
    finally:
        all_stop(ch)
        running = False
        t_rx.join(timeout=1.0)
        t_tx.join(timeout=1.0)
        try:
            ch.busOff()
        except Exception:
            pass
        try:
            ch.close()
        except Exception:
            pass

    print_summary(present, results)


if __name__ == "__main__":
    main()
