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
       python3 position_ref_client.py [host] [port] --once <axis0_deg> <axis1_deg> ...

축 개수는 실행 중인 컨트롤러의 num_actuators 를 조회해 자동으로 맞춘다.
조회가 안 되면 --axes N 으로 직접 지정한다. 개수가 틀리면 남는 값이 다음
명령으로 해석되어 각도가 곧바로 0 으로 되돌아간다.
"""
import socket
import struct
import sys

# 서버(RefTcpServer)는 **num_actuators × 8 바이트씩** 끊어 읽는다. 개수가 다르면
# 남는 바이트가 다음 메시지로 해석된다 — 2축인데 6개를 보내면 [30,30] 다음에
# [0,0], [0,0] 이 연달아 새 명령으로 들어가 각도가 즉시 0 으로 되돌아간다.
# 그래서 개수는 반드시 컨트롤러의 num_actuators 와 같아야 한다. 기본값은
# 실행 중인 컨트롤러에서 직접 물어본다.
NUM_AXES     = 6   # 자동 조회 실패 시 폴백
DEFAULT_HOST = '127.0.0.1'   # RefTcpServer는 pp_controller가 뜬 머신에서 0.0.0.0으로 bind됨.
                              # pp_controller를 다른 머신에서 돌린다면 그 머신의 IP를 인자로 넘길 것.
DEFAULT_PORT = 2293


def query_num_actuators(default=NUM_AXES):
    """실행 중인 pp_controller 에게 num_actuators 를 물어본다. 실패하면 default."""
    import subprocess
    try:
        out = subprocess.run(
            ['ros2', 'param', 'get', '/pack2/pp_controller', 'num_actuators'],
            capture_output=True, text=True, timeout=8).stdout
        for tok in out.replace(':', ' ').split():
            if tok.isdigit():
                n = int(tok)
                if 1 <= n <= 12:
                    return n
    except Exception:
        pass
    return default


def send_angles(sock, angles, n_axes=None):
    n = n_axes if n_axes else NUM_AXES
    if len(angles) == 1 and n > 1:
        angles = angles * n          # 값 하나면 전 축에 같은 각도
    if len(angles) != n:
        raise ValueError(f"각도는 {n}개여야 합니다 (받은 값: {len(angles)}개)")
    sock.sendall(struct.pack(f'<{n}d', *angles))


def main():
    args = sys.argv[1:]
    host, port = DEFAULT_HOST, DEFAULT_PORT
    if args and not args[0].startswith('--'):
        host = args.pop(0)
    if args and not args[0].startswith('--'):
        port = int(args.pop(0))

    n_axes = None
    if '--axes' in args:
        k = args.index('--axes')
        n_axes = int(args[k + 1]); del args[k:k + 2]
    if n_axes is None:
        n_axes = query_num_actuators()
        print(f"[position_ref_client] 컨트롤러 축 수 = {n_axes} "
              f"(--axes N 으로 직접 지정 가능)")
    globals()['NUM_AXES'] = n_axes

    print(f"[position_ref_client] {host}:{port} 접속 중...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    print("[position_ref_client] 접속 완료.")

    try:
        if args and args[0] == '--once':
            angles = [float(x) for x in args[1:]]
            send_angles(sock, angles, NUM_AXES)
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
                send_angles(sock, angles, NUM_AXES)
                print(f"  -> 전송: {angles} deg")
            except ValueError as e:
                print(f"  [ERR] {e} — 숫자 {NUM_AXES}개를 공백으로 구분해 입력하세요")
    finally:
        sock.close()
        print("\n[position_ref_client] 연결 종료.")


if __name__ == '__main__':
    main()
