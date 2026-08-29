import time
import struct
import sys
import os
import threading
from canlib import canlib, Frame

# ================= 1. 설정값 및 상수 =================
CHANNEL_NUM = 0
NOMINAL_BITRATE = canlib.canBITRATE_1M

# CanKing과 병행 사용 시 이 값은 무시되지만, 단독 실행 시 사용됨
FD_BITRATE_MANUAL = 5000000
FD_TSEG1 = 11
FD_TSEG2 = 4
FD_SJW   = 4

# [전류 센서 물리량 변환]
SHUNT_OHM = 0.1
AMP_GAIN  = 100.0
ADC_REF_MV = 3300.0
ADC_RES    = 4095.0
TO_MV = ADC_REF_MV / ADC_RES

# [압력 센서 캘리브레이션 설정]
# 공식: Pressure = (Original_mV - Offset) * Gain + 101.325
# 딕셔너리 형식: { Board_ID : (Offset_mV, Gain) }
# 1~5V 센서가 진공 센서(전압 높을수록 압력 낮음)라면 Gain은 음수여야 합니다.
DEFAULT_OFFSET = 1000.0   # 기본 Offset (예: 1V 기준)
DEFAULT_GAIN   = -0.0253  # 기본 Gain (예: 4000mV 변할 때 약 -100kPa 변동)

PRESSURE_CALIB = {
    1:  (1112.0,  0.250),       # P_line_pos
    2:  (1020.0, -0.02525),     # P_line_neg
    3:  (1064.0,  0.250),       # P_line_macro
    4:  (1020.0, -0.02525),     # P_line_macro_neg
    5:  (1076.0,  0.250),       # gid 0  양압 ch0
    6:  (1077.0,  0.250),       # gid 1  양압 ch1
    7:  (1054.0,  0.250),       # gid 2  양압 ch2 — 2026-08-24 재영점 (기존 1089.0)
    8:  (1065.0,  0.250),       # gid 3  양압 ch3
    9:  (1072.0,  0.250),       # gid 4  양압 ch4
    10: (1070.0,  0.250),       # gid 5  양압 ch5
    11: (1070.0, -0.02525),     # gid 6  음압 ch6
    12: (1020.0, -0.02525),     # gid 7  음압 ch7
    13: (1012.0, -0.02525),     # gid 8  음압 ch8
    14: (1032.0, -0.02525),     # gid 9  음압 ch9
    15: (1010.0, -0.02525),     # gid 10 음압 ch10
    16: (1030.0, -0.02525),     # gid 11 음압 ch11
}

# [엔코더 캘리브레이션] orig_mV 기준 (반전증폭 역산 후)
# 공식: Angle = (orig_mV - Offset) * Gain
# orig_mV = (4125 - adc_mv) / 0.825  →  1740mV = 0도, 3127mV = 105도
ENCODER_OFFSET = 1740.0
ENCODER_GAIN   = 105.0 / (3127.0 - 1740.0)  # ≈ 0.07569 deg/mV
ENCODER_CALIB  = {i: (ENCODER_OFFSET, ENCODER_GAIN) for i in range(17, 23)}

# 보드별 raw ADC 실측값 (0도 / 90도) — 여기 raw 값만 입력하면
# calib_from_raw_2pt()가 offset/gain을 자동 계산해서 ENCODER_CALIB에 반영함
ENCODER_RAW_POINTS = {
    17: (2114, 3392),   # raw 2114 @ 0도, raw 3392 @ 90도 — 20260829 재장착 후 재측정
    18: (860, 2000),    # raw  860 @ 0도, raw 2000 @ 90도
    19: (2115, 3155),   # raw 2115 @ 0도, raw 3155 @ 90도
}

# 공유 데이터
board_data = {i: [0.0, 0.0, 0.0, 0.0] for i in range(1, 23)} # [I1, I2, I3, Pressure]
encoder_raw = {i: (0, 0, 0, 0) for i in range(17, 23)}       # 엔코더 보드 raw ADC (Raw1~Raw4)
last_recv_time = {i: 0.0 for i in range(1, 23)}
running = True

# 통계
rx_count = 0

# ================= 2. 계산 함수들 =================
def calc_current_ma(mv_value):
    """전류 계산 (기존 로직 유지)"""
    if AMP_GAIN == 0: return 0.0
    return mv_value / (SHUNT_OHM * AMP_GAIN)

def calc_original_voltage_mv(adc_mv):
    """
    반전 증폭기 로직 복원
    입력: ADC 전압 (3300mV ~ 0mV)
    출력: 원래 센서 전압 (1000mV ~ 5000mV)
    수식 유도: V_orig = (4125 - V_adc) / 0.825
    """
    # 노이즈로 인한 범위 초과 방지 (Clamping)
    if adc_mv > 3300.0: adc_mv = 3300.0
    if adc_mv < 0.0: adc_mv = 0.0

    return (4125.0 - adc_mv) / 0.825

def calib_from_raw_2pt(raw_0deg, raw_90deg):
    """raw ADC 값(0도/90도 실측)으로부터 (offset, gain)을 orig_mV 기준으로 자동 계산"""
    orig_mv_0deg  = calc_original_voltage_mv(raw_0deg  * TO_MV)
    orig_mv_90deg = calc_original_voltage_mv(raw_90deg * TO_MV)
    gain = 90.0 / (orig_mv_90deg - orig_mv_0deg)
    offset = orig_mv_0deg
    return offset, gain

ENCODER_CALIB.update({
    board_id: calib_from_raw_2pt(raw_0deg, raw_90deg)
    for board_id, (raw_0deg, raw_90deg) in ENCODER_RAW_POINTS.items()
})

def calc_pressure_kpa(orig_mv, board_id):
    offset, gain = PRESSURE_CALIB.get(board_id, (DEFAULT_OFFSET, DEFAULT_GAIN))
    return (orig_mv - offset) * gain + 101.325

def calc_angle_deg(raw_a, board_id):
    offset, gain = ENCODER_CALIB.get(board_id, (ENCODER_OFFSET, ENCODER_GAIN))
    return (raw_a - offset) * gain

# ================= 3. CAN 채널 초기화 =================
def open_channel():
    try:
        ch = canlib.openChannel(channel=CHANNEL_NUM, flags=canlib.Open.CAN_FD)

        # 설정 시도 (CanKing 켜져 있으면 에러 날 수 있음 -> 무시)
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

# ================= 4. 스레드: RX (수신 전담) =================
def rx_thread_func(ch):
    global rx_count

    while running:
        try:
            # 타임아웃 50ms
            msg = ch.read(timeout=50)

            if 0x121 <= msg.id <= 0x136:
                board_id = msg.id - 0x120
                rx_count += 1

                if board_id >= 17:
                    # 엔코더 보드 (17~22): raw[3] -> orig_mV 역산 후 각도 계산
                    if len(msg.data) >= 8:
                        raw = struct.unpack('<HHHH', msg.data[:8])
                        orig_mv = calc_original_voltage_mv(raw[3] * TO_MV)
                        angle = calc_angle_deg(orig_mv, board_id)
                        board_data[board_id] = [0.0, 0.0, 0.0, angle]
                        encoder_raw[board_id] = raw
                elif len(msg.data) >= 8:
                    # 압력/전류 보드 (1~16)
                    raw = struct.unpack('<HHHH', msg.data)
                    val_pa4 = calc_current_ma(raw[0] * TO_MV)
                    val_pa5 = calc_current_ma(raw[1] * TO_MV)
                    val_pa6 = calc_current_ma(raw[2] * TO_MV)
                    adc_mv_pa7 = raw[3] * TO_MV
                    orig_mv_pa7 = calc_original_voltage_mv(adc_mv_pa7)
                    val_pa7 = calc_pressure_kpa(orig_mv_pa7, board_id)
                    board_data[board_id] = [val_pa4, val_pa5, val_pa6, val_pa7]

                last_recv_time[board_id] = time.time()

        except (canlib.canNoMsg, canlib.canError):
            continue

# ================= 5. 메인: 대시보드 출력 =================
def print_dashboard():
    now = time.time()

    # 화면 갱신용 문자열 생성
    output = "\033[H"  # 커서 홈으로 이동
    output += f"========== Pressure & Current Monitor ==========\n"
    output += f" [System] RX Count: {rx_count}\n"
    output += f" [Formula] P = (Orig_mV - Offset) * Gain + 101.325\n"
    output += f"------------------------------------------------------------------\n"
    output += f"|  ID  |  I1 (mA)  |  I2 (mA)  |  I3 (mA)  | Pressure(kPa) | State |\n"
    output += f"|------|-----------|-----------|-----------|---------------|-------|\n"

    active_cnt = 0
    for bid in range(1, 17):
        vals = board_data[bid]
        t_last = last_recv_time[bid]
        if now - t_last < 1.5 and t_last != 0:
            status = "OK"
            active_cnt += 1
        else:
            status = "Lost"
        p_str = f"{vals[3]:8.3f}"
        output += f"|  {bid:02d}  | {vals[0]:8.1f}  | {vals[1]:8.1f}  | {vals[2]:8.1f}  |   {p_str}    | {status:^5} |\n"

    output += f"==================================================================\n"
    output += f"\n"
    output += f"|  ID  |  Raw1 |  Raw2 |  Raw3 |  Raw4 |    Angle (deg)   | State |\n"
    output += f"|------|-------|-------|-------|-------|------------------|-------|\n"

    for bid in range(17, 23):
        vals = board_data[bid]
        raw = encoder_raw[bid]
        t_last = last_recv_time[bid]
        if now - t_last < 1.5 and t_last != 0:
            status = "OK"
            active_cnt += 1
        else:
            status = "Lost"
        output += f"|  {bid:02d}  | {raw[0]:5d} | {raw[1]:5d} | {raw[2]:5d} | {raw[3]:5d} |     {vals[3]:8.2f} deg   | {status:^5} |\n"

    output += f"==================================================================\n"
    output += f" Active Boards: {active_cnt} / 22\n"
    output += f" * Press Ctrl+C to exit.\n"

    sys.stdout.write(output)
    sys.stdout.flush()

def main():
    global running
    os.system('cls' if os.name == 'nt' else 'clear')
    print("Initializing Monitor...")

    ch = open_channel()

    # RX 스레드 시작
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
