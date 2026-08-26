#!/usr/bin/env python3
"""
pressure_ref_client.py — 직접 압력 제어(RefTcpServer, control_mode=0) 테스트 클라이언트

Controller의 RefTcpServer가 PRESSURE 모드일 때(control_mode != 1,2), 접속한 클라이언트로부터
[double pos_kpa, double neg_kpa] (리틀 엔디언, 16바이트, 절대압 kPa) 메시지를 받아
RefTcpServer.pos_gid / neg_gid 채널의 목표 압력(mpc_ref_kpa_)으로 즉시 반영한다.
PressureRefGen(토크→압력 변환)을 완전히 건너뛰고 MPC가 이 값을 바로 추종한다.

사용법:
  1) 1회 전송 후 종료:
       python3 pressure_ref_client.py [host] [port] --once <pos_kpa> <neg_kpa>

  2) 대화형: 접속 후 "150 60" 처럼 두 값을 입력하면 즉시 전송.
       python3 pressure_ref_client.py [host] [port]
"""
import socket
import struct
import sys

DEFAULT_HOST = '127.0.0.1'
DEFAULT_PORT = 2293   # RefTcpServer 포트. control_mode=0 이면 PRESSURE 모드로 동작한다.


def send_pressures(sock, pos_kpa, neg_kpa):
    sock.sendall(struct.pack('<2d', pos_kpa, neg_kpa))


def main():
    args = sys.argv[1:]
    host, port = DEFAULT_HOST, DEFAULT_PORT
    if args and not args[0].startswith('--'):
        host = args.pop(0)
    if args and not args[0].startswith('--'):
        port = int(args.pop(0))

    print(f"[pressure_ref_client] {host}:{port} 접속 중...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    print("[pressure_ref_client] 접속 완료.")

    try:
        if args and args[0] == '--once':
            pos_kpa, neg_kpa = float(args[1]), float(args[2])
            send_pressures(sock, pos_kpa, neg_kpa)
            print(f"[pressure_ref_client] 전송: pos={pos_kpa} kPa abs, neg={neg_kpa} kPa abs → 종료")
            return

        print("[pressure_ref_client] 대화형 모드: 'pos_kpa neg_kpa' (절대압) 입력. Ctrl-C로 종료.")
        while True:
            try:
                line = input("pressures(kPa abs) > ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not line:
                continue
            try:
                pos_kpa, neg_kpa = (float(x) for x in line.split())
                send_pressures(sock, pos_kpa, neg_kpa)
                print(f"  -> 전송: pos={pos_kpa} kPa, neg={neg_kpa} kPa")
            except ValueError as e:
                print(f"  [ERR] {e} — 'pos_kpa neg_kpa' 두 값을 공백으로 구분해 입력하세요")
    finally:
        sock.close()
        print("\n[pressure_ref_client] 연결 종료.")


if __name__ == '__main__':
    main()
