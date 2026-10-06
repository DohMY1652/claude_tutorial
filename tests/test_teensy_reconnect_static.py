"""복구 코드의 구조 회귀 검사. 실기/시뮬레이터/가상 직렬 포트를 실행하지 않는다."""
from pathlib import Path


SOURCE = Path(__file__).parents[1] / 'src/can_powerpack/src/CanBridge.cpp'


def test_new_serial_connection_gets_first_frame_window():
    source = SOURCE.read_text()
    loop = source.split('void CanBridge::teensy_loop()', 1)[1].split('void CanBridge::failsafe_tick()', 1)[0]
    assert 'bool received_since_open = false;' in loop
    assert 'received_since_open = true;' in loop
    assert 'if (teensy_fd_ >= 0 && !received_since_open)' in loop
    assert 'if (teensy_fd_ >= 0 && received_since_open)' in loop
    opened = loop.split('if (!teensy_open())', 1)[1].split('// ── 읽기', 1)[0]
    assert 'received_since_open = false;' in opened
    # 새 포트를 열었다는 이유만으로 엔코더 수신 시각/안전 상태를 정상으로 만들면 안 된다.
    assert 'teensy_last_ns_.store' not in opened
    assert 'teensy_stale_.store(false' not in opened
    assert 'failsafe_since_ns_.store' not in opened
    assert 'failsafe_latched_.store(false' not in loop


def test_existing_watchdog_and_failsafe_deadlines_preserved():
    source = SOURCE.read_text()
    assert 'if (age_ms > teensy_watchdog_ms_)' in source
    assert 'open_age > std::chrono::seconds(2)' in source
    assert 'ms >= teensy_failsafe_hold_ms_' in source
    assert 'ms >= (long long)teensy_failsafe_hold_ms_ + teensy_failsafe_vent_ms_' in source
