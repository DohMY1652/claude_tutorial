#!/usr/bin/env python3
"""6축(12채널) 무액추에이터 압력 스윕 레퍼런스 전송기.

이 스크립트가 TCP 클라이언트로 동작해 pp_controller의 ``RefTcpServer``에 접속한다.
기존 ``pressure_ref_client.py``는 양/음압 한 쌍만 보내지만, 이 스크립트는 12개
목표를 한 패킷으로 보내므로 여러 축을 동시에 시험할 수 있다. 파일명에 server가
남아 있는 것은 최초 버전의 흔적이며, 현재 네트워크 역할은 명확히 클라이언트다.

패킷 순서와 단위
------------------
  [axis1 P+, axis2 P+, ..., axis6 P+,
   axis1 P-, axis2 P-, ..., axis6 P-]  [kPa absolute]

전송 형식은 기존 RefTcpServer와 같은 little-endian double 12개(96바이트)다.
컨트롤러는 ``control_mode=0``과 ``RefTcpServer.all_channels=true``로 실행해야 한다.

기본 시험 순서
--------------
  1) axis1, axis2, ... axis6을 각각 단독 시험
  2) axis1+2, axis3+4, axis5+6 동시 시험
  3) axis1..6 전체 동시 시험

각 그룹에서 양압만 10/20/30/40/50 kPa 간격으로 대기압에서 180 kPa까지
올리고 각 스윕 뒤 대기압으로 복귀한다. 이어 음압만 같은 간격으로 대기압에서
20 kPa까지 내리고 각 스윕 뒤 대기압으로 복귀한다. 모든 목표는 --dwell초 동안
--send-hz 주기로 반복 전송된다.

Ctrl-C나 정상 종료 시에는 가능한 경우 12채널 모두 대기압을 여러 번 보낸다.
TCP가 끊기면 컨트롤러에는 마지막 목표가 남을 수 있으므로 제어기/펌프를 즉시
정지하고 실제 압력을 확인해야 한다.
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from dataclasses import dataclass
from typing import Sequence


NUM_AXES = 6
ATM_KPA = 101.325
POS_MAX_KPA = 180.0
NEG_MIN_KPA = 20.0
STEP_SIZES_KPA = (10.0, 20.0, 30.0, 40.0, 50.0)

# 사용자가 요청한 그룹 순서. 내부 인덱스는 0부터 시작하지만 표시는 1부터 한다.
AXIS_GROUPS = (
    (0,), (1,), (2,), (3,), (4,), (5,),
    (0, 1), (2, 3), (4, 5),
    (0, 1, 2, 3, 4, 5),
)


@dataclass(frozen=True)
class Stage:
    """5초 동안 유지할 한 목표와 사람이 읽을 설명."""

    group: tuple[int, ...]
    phase: str
    step_kpa: float | None
    target_kpa: float
    refs_kpa: tuple[float, ...]


def _axis_label(group: Sequence[int]) -> str:
    return "+".join(str(axis + 1) for axis in group) + "축"


def _ramp_values(start: float, stop: float, increment: float) -> list[float]:
    """start는 제외하고 stop은 반드시 포함하는 단조 목표열을 만든다.

    예: 101.325 -> 180, increment=30이면 [131.325, 161.325, 180.0].
    마지막 간격이 increment보다 작더라도 사용자가 지정한 끝값을 정확히 넣는다.
    """

    if increment <= 0.0:
        raise ValueError("increment는 0보다 커야 한다")
    direction = 1.0 if stop > start else -1.0
    values: list[float] = []
    value = start
    while True:
        candidate = value + direction * increment
        if (direction > 0.0 and candidate >= stop) or (
                direction < 0.0 and candidate <= stop):
            break
        values.append(candidate)
        value = candidate
    values.append(stop)
    return values


def _refs(group: Sequence[int], positive: float = ATM_KPA,
          negative: float = ATM_KPA) -> tuple[float, ...]:
    """선택 축만 바꾸고 나머지 12채널은 대기압인 배열을 만든다."""

    pos = [ATM_KPA] * NUM_AXES
    neg = [ATM_KPA] * NUM_AXES
    for axis in group:
        pos[axis] = positive
        neg[axis] = negative
    return tuple(pos + neg)


def build_stages() -> list[Stage]:
    """요청한 전체 단축 -> 축쌍 -> 6축 동시 시험 순서를 생성한다."""

    stages: list[Stage] = []
    for group in AXIS_GROUPS:
        # 그룹 전환 직후 모든 챔버가 대기압 목표인지 먼저 1스텝 확인한다.
        stages.append(Stage(group, "그룹 시작/대기", None, ATM_KPA, _refs(group)))

        # P+만 상승한다. P-와 선택되지 않은 모든 축은 계속 대기압이다.
        for step in STEP_SIZES_KPA:
            for target in _ramp_values(ATM_KPA, POS_MAX_KPA, step):
                stages.append(Stage(group, "양압 상승", step, target,
                                    _refs(group, positive=target)))
            stages.append(Stage(group, "양압 후 대기 복귀", step, ATM_KPA,
                                _refs(group)))

        # P-만 하강한다. P+와 선택되지 않은 모든 축은 계속 대기압이다.
        for step in STEP_SIZES_KPA:
            for target in _ramp_values(ATM_KPA, NEG_MIN_KPA, step):
                stages.append(Stage(group, "음압 하강", step, target,
                                    _refs(group, negative=target)))
            stages.append(Stage(group, "음압 후 대기 복귀", step, ATM_KPA,
                                _refs(group)))
    return stages


def encode_refs(refs_kpa: Sequence[float]) -> bytes:
    """kPa 목표 12개를 RefTcpServer의 little-endian double 패킷으로 만든다."""

    if len(refs_kpa) != 2 * NUM_AXES:
        raise ValueError(f"압력 목표는 {2 * NUM_AXES}개여야 한다")
    for pressure in refs_kpa:
        if not 0.0 <= pressure <= 1000.0:
            raise ValueError(f"비정상 압력 목표: {pressure} kPa absolute")
    return struct.pack(f"<{len(refs_kpa)}d", *refs_kpa)


def _fmt_refs(refs: Sequence[float]) -> str:
    pos = ", ".join(f"{v:7.3f}" for v in refs[:NUM_AXES])
    neg = ", ".join(f"{v:7.3f}" for v in refs[NUM_AXES:])
    return f"P+=[{pos}]  P-=[{neg}]"


def print_plan(stages: Sequence[Stage], dwell: float, verbose: bool) -> None:
    duration = len(stages) * dwell
    print("\n=== 압력 스윕 계획 ===")
    print(f"스테이지 {len(stages)}개 x {dwell:g}초 = "
          f"{duration / 60.0:.1f}분")
    print("채널 순서: [1P+,2P+,3P+,4P+,5P+,6P+,1P-,2P-,3P-,4P-,5P-,6P-]")
    print("그룹 순서:", " -> ".join(_axis_label(g) for g in AXIS_GROUPS))
    if verbose:
        for index, stage in enumerate(stages, 1):
            step = "-" if stage.step_kpa is None else f"{stage.step_kpa:g}"
            print(f"{index:03d} {_axis_label(stage.group):9s} "
                  f"{stage.phase:13s} step={step:>2s} "
                  f"target={stage.target_kpa:7.3f}  {_fmt_refs(stage.refs_kpa)}")


def _send_for_dwell(conn: socket.socket, payload: bytes,
                    dwell: float, send_hz: float) -> None:
    """한 목표를 dwell 동안 반복 전송해 순간적인 패킷 하나에 의존하지 않는다."""

    period = 1.0 / send_hz
    deadline = time.monotonic() + dwell
    while True:
        conn.sendall(payload)
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break
        time.sleep(min(period, remaining))


def _send_atmosphere(conn: socket.socket | None) -> None:
    """종료 전에 전 채널 대기압 목표를 반복 전송한다."""

    if conn is None:
        return
    refs = (ATM_KPA,) * (2 * NUM_AXES)
    payload = encode_refs(refs)
    try:
        for _ in range(10):
            conn.sendall(payload)
            time.sleep(0.05)
        print(f"\n[안전 복귀] 전 채널 대기압 목표 전송: {_fmt_refs(refs)}")
    except OSError as exc:
        print(f"\n[위험] 대기압 목표 전송 실패: {exc}", file=sys.stderr)
        print("       컨트롤러에는 마지막 목표가 남을 수 있다. 펌프/제어기를 즉시 정지할 것.",
              file=sys.stderr)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1",
                        help="pp_controller가 실행 중인 주소 (기본: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=2293,
                        help="RefTcpServer.port와 같은 포트 (기본: 2293)")
    parser.add_argument("--connect-timeout", type=float, default=30.0,
                        help="controller 접속을 재시도할 최대 시간 [s] (기본: 30)")
    parser.add_argument("--dwell", type=float, default=5.0,
                        help="각 스테이지 유지 시간 [s] (기본: 5)")
    parser.add_argument("--send-hz", type=float, default=10.0,
                        help="같은 목표 반복 전송률 [Hz] (기본: 10)")
    parser.add_argument("--dry-run", action="store_true",
                        help="TCP를 열지 않고 모든 스테이지와 12채널 값을 출력")
    parser.add_argument("--yes", action="store_true",
                        help="실기 시작 전 RUN 확인 문구를 생략")
    args = parser.parse_args()
    if args.dwell <= 0.0:
        parser.error("--dwell은 0보다 커야 한다")
    if args.send_hz <= 0.0:
        parser.error("--send-hz는 0보다 커야 한다")
    if args.connect_timeout <= 0.0:
        parser.error("--connect-timeout은 0보다 커야 한다")
    if not 1 <= args.port <= 65535:
        parser.error("--port는 1..65535여야 한다")
    return args


def main() -> int:
    args = parse_args()
    stages = build_stages()
    print_plan(stages, args.dwell, verbose=args.dry_run)
    if args.dry_run:
        return 0

    print("\n주의: 최대 양압 목표 180 kPa abs는 현재 190 kPa safety limit과 "
          "10 kPa 차이다.")
    print("TCP 단절 시 마지막 압력 목표가 컨트롤러에 남을 수 있다.")
    if not args.yes:
        answer = input("계획과 비상정지를 확인했으면 RUN을 입력: ").strip()
        if answer != "RUN":
            print("취소했다.")
            return 2

    conn: socket.socket | None = None
    try:
        deadline = time.monotonic() + args.connect_timeout
        while conn is None:
            try:
                conn = socket.create_connection((args.host, args.port), timeout=2.0)
            except OSError as exc:
                if time.monotonic() >= deadline:
                    raise TimeoutError(
                        f"{args.host}:{args.port}에 {args.connect_timeout:g}초 동안 "
                        f"접속하지 못했다: {exc}") from exc
                print(f"[접속 재시도] {args.host}:{args.port} — {exc}")
                time.sleep(1.0)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        print(f"[접속] pp_controller RefTcpServer: {args.host}:{args.port}")

        # 접속 직후에는 계획 시작 전에도 먼저 안전한 대기압 패킷을 보낸다.
        _send_atmosphere(conn)

        for index, stage in enumerate(stages, 1):
            payload = encode_refs(stage.refs_kpa)
            step = "-" if stage.step_kpa is None else f"{stage.step_kpa:g}"
            print(f"[{index:03d}/{len(stages):03d}] {_axis_label(stage.group):9s} "
                  f"{stage.phase:13s} step={step:>2s} "
                  f"target={stage.target_kpa:7.3f} kPa abs")
            print(f"             {_fmt_refs(stage.refs_kpa)}", flush=True)
            _send_for_dwell(conn, payload, args.dwell, args.send_hz)

        print("\n[완료] 전체 스윕을 마쳤다.")
        return 0
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C를 받았다.")
        return 130
    except (ConnectionError, OSError) as exc:
        print(f"\n[위험] TCP 통신 실패: {exc}", file=sys.stderr)
        print("       마지막 목표가 유지될 수 있다. 펌프/제어기를 즉시 정지할 것.",
              file=sys.stderr)
        return 1
    finally:
        _send_atmosphere(conn)
        if conn is not None:
            conn.close()


if __name__ == "__main__":
    raise SystemExit(main())
