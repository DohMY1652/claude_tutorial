#!/usr/bin/env python3
"""
position_ref_client.py — 위치 제어(RefTcpServer, control_mode=1) 테스트/데모 클라이언트

Controller의 RefTcpServer(기본 port 2293, 위치 모드)는 접속한 클라이언트로부터
[double axis0_deg, ...] (리틀 엔디언, 8*NUM_AXES 바이트, NUM_AXES=num_actuators)
메시지를 계속 받아 축별(0=board17, 1=board18, 2=board19) 목표 각도로 사용한다.

사용법:
  1) 대화형: 접속 후 "0 30 60" 처럼 축 개수만큼 각도를 입력하면 즉시 전송.
       python3 position_ref_client.py [host] [port]

  2) 1회 전송 후 종료 (스크립팅용):
       python3 position_ref_client.py [host] [port] --once <axis0_deg> <axis1_deg> <axis2_deg>
"""
import socket
import struct
import sys

NUM_AXES     = 1   # 1축만 구동 테스트 (3→1) — powerpack_config.yaml의 num_actuators와 일치시킬 것
DEFAULT_HOST = '127.0.0.1'   # RefTcpServer는 pp_controller가 뜬 머신에서 0.0.0.0으로 bind됨.
                              # pp_controller를 다른 머신에서 돌린다면 그 머신의 IP를 인자로 넘길 것.
DEFAULT_PORT = 2293


def send_angles(sock, angles):
    if len(angles) != NUM_AXES:
        raise ValueError(f"각도는 {NUM_AXES}개여야 합니다 (받은 값: {len(angles)}개)")
    sock.sendall(struct.pack(f'<{NUM_AXES}d', *angles))


def main():
    args = sys.argv[1:]
    host, port = DEFAULT_HOST, DEFAULT_PORT
    if args and not args[0].startswith('--'):
        host = args.pop(0)
    if args and not args[0].startswith('--'):
        port = int(args.pop(0))

    print(f"[position_ref_client] {host}:{port} 접속 중...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    print("[position_ref_client] 접속 완료.")

    try:
        if args and args[0] == '--once':
            angles = [float(x) for x in args[1:]]
            send_angles(sock, angles)
            print(f"[position_ref_client] 전송: {angles} deg → 종료")
            return

        print(f"[position_ref_client] 대화형 모드: axis0..axis{NUM_AXES-1} 목표 각도(deg)를 "
              f"공백으로 구분해 입력하세요 (예: 0 30 60). Ctrl-C로 종료.")
        while True:
            try:
                line = input("angles(deg) > ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not line:
                continue
            try:
                angles = [float(x) for x in line.split()]
                send_angles(sock, angles)
                print(f"  -> 전송: {angles} deg")
            except ValueError as e:
                print(f"  [ERR] {e} — 숫자 {NUM_AXES}개를 공백으로 구분해 입력하세요")
    finally:
        sock.close()
        print("\n[position_ref_client] 연결 종료.")


if __name__ == '__main__':
    main()
