#!/usr/bin/env python3
"""sensor_zero.py — 전 채널을 대기압으로 보낸 뒤 압력 센서 영점을 다시 잡는다.

주기적으로 돌려 센서의 대기압 기준을 맞추는 정비용 스크립트다. 센서 영점이 밀리면
**과압 세이프티도 같이 밀린다** (190 kPa 트립이 실제 210 kPa 에서 걸리는 식) 이므로
드리프트를 방치하면 안 된다.

무엇을 하나
-----------
  1) 펌프가 꺼졌는지 확인받는다 (소프트웨어 제어가 없어 작업자가 끈다)
  2) 각 채널의 **대기 방출 밸브(v2)** 를 활짝 열어 챔버를 대기압으로 보낸다.
     동시에 board1 v1(양압 레일 → 대기)과 board2 v1(대기 → 음압 레일)을 열어
     레일도 대기압으로 만든다. 이 두 슬롯은 브리지의 워치독 안전 상태와 같다.
  3) 압력이 멈출 때까지 기다린 뒤 채널 밸브를 닫고, 유량이 없는 상태에서 샘플링한다
     (밸브가 열린 채로 재면 흐름 때문에 작은 차압이 남는다)
  4) 보드별 raw 평균 = 새 offset. 기존 값과의 차이를 kPa 로 환산해 보여준다
  5) --write 를 주면 config/powerpack_config.yaml 의
     ``Sensor_calibration.boards.N.offset`` 을 갱신한다 (원본은 백업)

기본은 **읽기 전용**이다. 값을 보고 판단한 뒤 --write 로 반영한다.

전제
----
  · can_bridge 가 돌고 있어야 한다 (board/sensors 를 받아 raw 를 읽는다)
  · pp_controller 는 **꺼져 있어야 한다** — board/pwm_cmd 에 발행자가 둘이면
    두 노드의 명령이 번갈아 나가고, LinePID 가 레일을 200/30 으로 되돌린다.
    (컨트롤러를 끄면 브리지 워치독이 레일을 대기압으로 여는 안전 상태로 간다.)
  · 보드 3/4 (탱크·이젝터 라인) 는 이 밸브들로 대기압을 만들 수 없다. 기본
    보드 목록에서 빠져 있고, 그 라인을 손으로 개방했다면 --boards 로 포함시킨다.

사용
----
    # 측정만 (드리프트 확인)
    python3 sensor_zero.py

    # 확인 후 yaml 반영
    python3 sensor_zero.py --write

    # 보드 3/4 까지 (탱크 라인도 대기압으로 개방한 경우)
    python3 sensor_zero.py --boards 1,2,3,4,5-16 --write
"""
from __future__ import annotations

import argparse
import datetime
import os
import re
import shutil
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray

NAMESPACE = "/pack2"
PWM_BOARDS = 16                  # 브리지가 PWM 을 내는 보드 수
PWM_TOTAL = 25 * 3               # board/pwm_cmd 배열 길이 (컨트롤러와 동일)
V_MICRO, V_ATM, V_MACRO = 0, 1, 2   # 보드 슬롯 v1/v2/v3
CHANNEL_BOARDS = range(5, 17)    # 압력 채널 = board 5..16
VENT_SLOT = 0                    # board1 v1 — 양압 레일 → 대기
ADMIT_SLOT = 3                   # board2 v1 — 대기 → 음압 레일
FULL = 4095
DEFAULT_BOARDS = "1,2,5-16"      # 3/4 (탱크·이젝터) 는 기본 제외

DEFAULT_YAML = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config", "powerpack_config.yaml")


def parse_boards(text: str) -> list[int]:
    """"1,2,5-16" → [1,2,5,...,16]. 압력 보드(1..16) 만 허용한다."""
    out: set[int] = set()
    for token in text.replace(" ", "").split(","):
        if not token:
            continue
        if "-" in token:
            a, _, b = token.partition("-")
            lo, hi = int(a), int(b)
            if lo > hi:
                raise ValueError(f"범위가 거꾸로다: {token}")
            out.update(range(lo, hi + 1))
        else:
            out.add(int(token))
    bad = [b for b in out if not 1 <= b <= PWM_BOARDS]
    if bad:
        raise ValueError(f"압력 보드는 1~{PWM_BOARDS} 다: {sorted(bad)}")
    return sorted(out)


class Zeroer(Node):
    def __init__(self) -> None:
        super().__init__("sensor_zero")
        self._raw: list[int] | None = None
        self._n_msg = 0
        self.create_subscription(UInt16MultiArray, f"{NAMESPACE}/board/sensors",
                                 self._on_sensors, 20)
        self._pub = self.create_publisher(UInt16MultiArray,
                                          f"{NAMESPACE}/board/pwm_cmd", 5)

    def _on_sensors(self, msg: UInt16MultiArray) -> None:
        self._raw = list(msg.data)
        self._n_msg += 1

    # ── 상태 ────────────────────────────────────────────────────────────────
    def raw(self, board: int) -> int | None:
        """board (1-based) 의 raw ADC. 0 은 '프레임을 못 받았다' 는 뜻이다."""
        if self._raw is None or len(self._raw) < board:
            return None
        return self._raw[board - 1]

    def rival_publishers(self) -> list[str]:
        """board/pwm_cmd 를 발행하는 **다른 노드** 이름 목록.

        개수만 세면(count_publishers) 죽은 노드의 그래프 엔트리가 잠깐 남아 있을 때
        멀쩡한 상황을 막아 버린다 (DDS 는 정리에 시간이 걸린다). 이름으로 보면
        누가 붙어 있는지 사용자가 바로 판단할 수 있다.
        """
        mine = self.get_name()
        out = []
        for info in self.get_publishers_info_by_topic(f"{NAMESPACE}/board/pwm_cmd"):
            if info.node_name != mine:
                ns = info.node_namespace.rstrip("/")
                out.append(f"{ns}/{info.node_name}")
        return out

    # ── 밸브 명령 ───────────────────────────────────────────────────────────
    def send_pwm(self, vent: bool, channels_open: bool) -> None:
        """레일 릴리프/유입은 항상 열어 두고, 채널 대기 방출만 켜고 끈다.

        '전부 0' 은 안전이 아니다 — board1 v1 이 닫히면 (펌프가 켜져 있을 때)
        양압 레일이 무한정 올라간다. 브리지의 안전 상태와 같은 규약을 지킨다.
        """
        data = [0] * PWM_TOTAL
        if vent:
            data[VENT_SLOT] = FULL
            data[ADMIT_SLOT] = FULL
        if channels_open:
            for bid in CHANNEL_BOARDS:
                data[(bid - 1) * 3 + V_ATM] = FULL
        msg = UInt16MultiArray()
        msg.data = data
        self._pub.publish(msg)


def spin_for(node: Zeroer, seconds: float, pwm: tuple[bool, bool] | None = None,
             hz: float = 20.0) -> None:
    """seconds 동안 스핀한다. pwm 을 주면 그 명령을 hz 로 계속 보낸다.

    브리지 PWM 워치독이 200 ms 이므로 반드시 주기적으로 재전송해야 한다.
    """
    end = time.monotonic() + seconds
    nxt = 0.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if pwm is not None and time.monotonic() >= nxt:
            node.send_pwm(*pwm)
            nxt = time.monotonic() + 1.0 / hz


def wait_settled(node: Zeroer, boards: list[int], tol_raw: float,
                 hold: float, timeout: float) -> bool:
    """모든 대상 보드의 raw 가 hold 초 동안 tol_raw 안에 머무르면 True."""
    hist: dict[int, list[int]] = {b: [] for b in boards}
    t0 = time.monotonic()
    stable_since: float | None = None
    while time.monotonic() - t0 < timeout:
        spin_for(node, 0.2, pwm=(True, True))
        ok = True
        for b in boards:
            v = node.raw(b)
            if v is None or v == 0:      # 0 = 그 보드 프레임이 아직 없다
                ok = False
                continue
            h = hist[b]
            h.append(v)
            del h[:-int(max(2, hold / 0.2))]
            if len(h) < 3 or (max(h) - min(h)) > tol_raw:
                ok = False
        if ok:
            if stable_since is None:
                stable_since = time.monotonic()
            elif time.monotonic() - stable_since >= hold:
                return True
        else:
            stable_since = None
    return False


def sample(node: Zeroer, boards: list[int], seconds: float) -> dict[int, list[int]]:
    """유량이 없는 상태에서 보드별 raw 를 모은다 (채널 밸브 닫음, 레일은 개방)."""
    acc: dict[int, list[int]] = {b: [] for b in boards}
    end = time.monotonic() + seconds
    nxt = 0.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if time.monotonic() >= nxt:
            node.send_pwm(vent=True, channels_open=False)
            nxt = time.monotonic() + 0.05
        for b in boards:
            v = node.raw(b)
            if v:
                acc[b].append(v)
    return acc


# ── yaml 읽기/쓰기 ─────────────────────────────────────────────────────────
# 주석과 서식을 보존해야 하므로 파서를 쓰지 않고 그 줄만 정규식으로 고친다.
_BOARD_LINE = re.compile(
    r'^(?P<head>\s*"(?P<bid>\d+)":\s*\{\s*offset:\s*)(?P<offset>[-\d.]+)(?P<tail>.*)$')


def read_calib(path: str) -> tuple[dict[int, float], dict[int, float], float]:
    """(offset, gain, atm_offset) 을 yaml 에서 읽는다."""
    import yaml
    with open(path, encoding="utf-8") as fp:
        doc = yaml.safe_load(fp)
    params = doc["/pack2/pp_controller"]["ros__parameters"]["Sensor_calibration"]
    offs = {int(k): float(v["offset"]) for k, v in params["boards"].items()}
    gains = {int(k): float(v["gain"]) for k, v in params["boards"].items()}
    return offs, gains, float(params.get("atm_offset", 101.325))


def write_offsets(path: str, new: dict[int, float]) -> str:
    """offset 만 제자리에서 갱신하고 백업 경로를 돌려준다."""
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = f"{path}.bak.{stamp}"
    shutil.copy2(path, backup)

    with open(path, encoding="utf-8") as fp:
        lines = fp.readlines()

    in_boards = False
    done: set[int] = set()
    for i, line in enumerate(lines):
        if re.match(r"^\s*boards:\s*$", line):
            in_boards = True
            continue
        if in_boards and re.match(r"^\s{0,6}\S", line) and '":' not in line:
            if not line.strip().startswith("#"):
                in_boards = False
        if not in_boards:
            continue
        m = _BOARD_LINE.match(line.rstrip("\n"))
        if not m:
            continue
        bid = int(m.group("bid"))
        if bid not in new:
            continue
        lines[i] = (f'{m.group("head")}{new[bid]:.1f}{m.group("tail")}'
                    f'  # 재영점 {stamp}\n')
        done.add(bid)

    missing = sorted(set(new) - done)
    if missing:
        raise RuntimeError(f"yaml 에서 보드 {missing} 줄을 못 찾았다 — 수동 확인 필요")

    with open(path, "w", encoding="utf-8") as fp:
        fp.writelines(lines)
    return backup


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--boards", default=DEFAULT_BOARDS,
                    help=f"영점을 다시 잡을 압력 보드 (기본: {DEFAULT_BOARDS}). "
                         "3/4 는 탱크·이젝터 라인이라 기본 제외")
    ap.add_argument("--vent", type=float, default=20.0,
                    help="대기압 배출 최대 대기 시간 [s] (기본: 20)")
    ap.add_argument("--settle", type=float, default=3.0,
                    help="밸브를 닫고 유량이 멎기를 기다리는 시간 [s] (기본: 3)")
    ap.add_argument("--sample", type=float, default=3.0,
                    help="영점 샘플링 시간 [s] (기본: 3)")
    ap.add_argument("--tol-raw", type=float, default=3.0,
                    help="정지 판정 raw 변동폭 (기본: 3 count)")
    ap.add_argument("--yaml", default=DEFAULT_YAML,
                    help="갱신할 설정 파일 (기본: 패키지 config/powerpack_config.yaml)")
    ap.add_argument("--write", action="store_true",
                    help="측정값을 yaml offset 에 반영한다 (기본은 측정만)")
    ap.add_argument("--yes", action="store_true",
                    help="펌프 정지 확인 문구를 생략한다")
    ap.add_argument("--force", action="store_true",
                    help="pp_controller 가 살아 있어도 진행 (권장하지 않음)")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    try:
        boards = parse_boards(args.boards)
    except ValueError as exc:
        print(f"[중단] --boards: {exc}", file=sys.stderr)
        return 2

    print("=== 센서 대기압 재영점 ===")
    print(f"대상 보드: {boards}")
    print("절차: 채널 대기 방출 개방 → 압력 정지 대기 → 밸브 닫고 샘플링 → offset 계산")
    print("\n펌프를 **끄고** 시작해야 한다 (소프트웨어 제어가 없다).")
    print("탱크·이젝터 라인이 가압된 상태라면 보드 3/4 는 대기압이 아니므로 제외한다.")
    if not args.yes:
        if input("펌프를 껐고 계통 압력을 확인했으면 YES 입력: ").strip() != "YES":
            print("취소했다.")
            return 2

    rclpy.init()
    node = Zeroer()
    rc = 1
    try:
        # 브리지 확인
        spin_for(node, 2.0)
        if node._n_msg == 0:
            print("[중단] board/sensors 가 오지 않는다 — can_bridge 가 떠 있는지 확인할 것.",
                  file=sys.stderr)
            return 1
        rivals = node.rival_publishers()
        if rivals and not args.force:
            print(f"[중단] board/pwm_cmd 를 이미 발행하는 노드가 있다: "
                  f"{', '.join(rivals)}", file=sys.stderr)
            print("       두 노드가 번갈아 명령하면 레일이 200/30 으로 되돌아가 대기압이 "
                  "되지 않는다. 그 노드를 내리고 다시 실행할 것.", file=sys.stderr)
            print("       내린 직후라면 DDS 그래프에서 사라지는 데 몇 초 걸린다 — "
                  "잠시 뒤 다시 실행하거나 --force 로 무시한다.", file=sys.stderr)
            return 1
        if rivals:
            print(f"[경고] --force: 다른 발행자가 있는 채로 진행한다 ({', '.join(rivals)})")

        offs, gains, atm = read_calib(args.yaml)

        # 1) 배출
        print(f"\n[배출] 채널 대기 방출 + 레일 릴리프/유입 개방, 최대 {args.vent:g} s")
        settled = wait_settled(node, boards, args.tol_raw, hold=1.0, timeout=args.vent)
        print("       " + ("압력 정지 확인." if settled else
                           "정지 판정 실패 — 그대로 진행한다 (누설/미개방 가능)."))

        # 2) 밸브 닫고 안정화 → 3) 샘플링
        print(f"[안정화] 채널 밸브 닫고 {args.settle:g} s 대기 (유량 없는 상태에서 잰다)")
        spin_for(node, args.settle, pwm=(True, False))
        print(f"[샘플링] {args.sample:g} s")
        acc = sample(node, boards, args.sample)

        # 4) 결과
        print(f"\n{'보드':>4} {'n':>5} {'raw 평균':>9} {'표준편차':>8} "
              f"{'기존 offset':>11} {'변화[raw]':>9} {'변화[kPa]':>10} {'현재 읽음[kPa]':>13}")
        new: dict[int, float] = {}
        warn: list[str] = []
        for b in boards:
            xs = acc[b]
            if len(xs) < 10:
                warn.append(f"board {b}: 샘플 {len(xs)}개 — 프레임이 오지 않는다 (건너뜀)")
                continue
            mean = statistics.fmean(xs)
            sd = statistics.pstdev(xs) if len(xs) > 1 else 0.0
            g = gains.get(b, 0.25)
            drift_raw = mean - offs.get(b, mean)
            reads = (mean - offs.get(b, mean)) * g + atm
            new[b] = round(mean, 1)
            print(f"{b:>4} {len(xs):>5} {mean:>9.1f} {sd:>8.2f} "
                  f"{offs.get(b, float('nan')):>11.1f} {drift_raw:>+9.1f} "
                  f"{drift_raw * g:>+10.2f} {reads:>13.2f}")
            if sd > args.tol_raw:
                warn.append(f"board {b}: 샘플 표준편차 {sd:.2f} count — 아직 압력이 움직인다")
        for w in warn:
            print(f"  [경고] {w}")

        if not new:
            print("[중단] 쓸 수 있는 측정이 없다.", file=sys.stderr)
            return 1

        # 5) 반영
        if args.write:
            backup = write_offsets(args.yaml, new)
            print(f"\n[반영] {args.yaml}")
            print(f"       백업: {backup}")
            print("       yaml 은 심볼릭 링크로 설치되므로 재빌드 없이 노드 재시작만 하면 된다.")
        else:
            print("\n[측정만] --write 를 주면 위 raw 평균을 offset 으로 반영한다.")
        rc = 0
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C.")
        rc = 130
    finally:
        # 브리지 안전 상태와 같은 명령으로 끝낸다 (레일 개방, 채널 폐쇄).
        # 여기서 멈추면 200 ms 뒤 워치독이 같은 상태를 다시 넣는다.
        try:
            for _ in range(5):
                node.send_pwm(vent=True, channels_open=False)
                time.sleep(0.05)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
