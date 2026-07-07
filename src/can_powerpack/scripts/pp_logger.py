#!/usr/bin/env python3
"""
pp_logger.py — Powerpack 제어 결과 로거
  저장 위치: ~/result/YYYYMMDD_HHMMSS/
    - YYYYMMDD_HHMMSS.csv  (100 Hz 데이터)
    - YYYYMMDD_HHMMSS.png  (위치 추종 그래프)
"""
import os
import csv
import signal
import sys
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

NAMESPACE = '/pack2'
LOG_HZ    = 100.0


class PpLogger(Node):
    def __init__(self):
        super().__init__('pp_logger')
        self._lock = threading.Lock()

        # 데이터 버퍼
        self._pos_dbg  = [0.0] * 8
        self._sensors  = [101.325] * 25
        self._mpc_refs = [101.325] * 12
        self._encoders = [0.0] * 9

        ns = NAMESPACE
        self.create_subscription(Float64MultiArray, f'{ns}/controller/position_dbg',
                                 self._cb_pos_dbg, 10)
        self.create_subscription(Float64MultiArray, f'{ns}/controller/sensors_kpa',
                                 self._cb_sensors, 10)
        self.create_subscription(Float64MultiArray, f'{ns}/controller/mpc_refs_kpa',
                                 self._cb_refs, 10)
        self.create_subscription(Float64MultiArray, f'{ns}/board/analog',
                                 self._cb_analog, 10)

        # 폴더 및 파일 생성
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._ts = ts
        result_base = os.path.expanduser('~/result')
        self._run_dir = os.path.join(result_base, ts)
        os.makedirs(self._run_dir, exist_ok=True)

        csv_path = os.path.join(self._run_dir, f'{ts}.csv')
        self._png_path = os.path.join(self._run_dir, f'{ts}.png')

        self._file   = open(csv_path, 'w', newline='', buffering=1)
        self._writer = csv.writer(self._file)
        self._rows   = []  # PNG 생성용 메모리 버퍼

        header = [
            'time_sec',
            'angle_deg', 'target_deg', 'err_deg', 'vel_dps',
            'pid_kpa', 'ff_kpa', 'fric_kpa',
            'p_pos_ref_kpa', 'p_neg_ref_kpa',
            'p_pos_actual_kpa',
            'p_neg_actual_kpa',
            'p_line_pos_kpa',
            'p_line_neg_kpa',
            'p_macro_pos_kpa',
            'p_macro_neg_kpa',
        ] + [f'mpc_ref_gid{i}_kpa' for i in range(12)] \
          + [f'enc_bd{17 + i}_deg'  for i in range(6)]

        self._writer.writerow(header)
        self._start_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(f'[pp_logger] 저장 경로: {self._run_dir}')

        self.create_timer(1.0 / LOG_HZ, self._write_row)

    def _cb_pos_dbg(self, msg):
        with self._lock:
            n = min(len(msg.data), 8)
            self._pos_dbg[:n] = list(msg.data[:n])

    def _cb_sensors(self, msg):
        with self._lock:
            for i, v in enumerate(msg.data):
                if i < 25: self._sensors[i] = v

    def _cb_refs(self, msg):
        with self._lock:
            for i, v in enumerate(msg.data):
                if i < 12: self._mpc_refs[i] = v

    def _cb_analog(self, msg):
        with self._lock:
            for i, v in enumerate(msg.data):
                if i < 9: self._encoders[i] = v

    def _write_row(self):
        with self._lock:
            pos = list(self._pos_dbg)
            sen = list(self._sensors)
            ref = list(self._mpc_refs)
            enc = list(self._encoders)

        elapsed = (self.get_clock().now().nanoseconds - self._start_ns) / 1e9
        angle  = pos[0]
        target = pos[1]

        row = [
            f'{elapsed:.4f}',
            f'{angle:.4f}', f'{target:.4f}',
            f'{target - angle:.4f}', f'{pos[7]:.4f}',
            f'{pos[4]:.4f}', f'{pos[5]:.4f}', f'{pos[6]:.4f}',
            f'{pos[2]:.4f}', f'{pos[3]:.4f}',
            f'{sen[4]:.4f}',
            f'{sen[10]:.4f}',
            f'{sen[0]:.4f}', f'{sen[1]:.4f}',
            f'{sen[2]:.4f}', f'{sen[3]:.4f}',
        ] + [f'{v:.4f}' for v in ref[:12]] \
          + [f'{enc[i]:.4f}' for i in range(6)]

        self._writer.writerow(row)
        self._rows.append((elapsed, angle, target))

    def _save_png(self):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            import matplotlib.ticker as ticker

            times   = [r[0] for r in self._rows]
            angles  = [r[1] for r in self._rows]
            targets = [r[2] for r in self._rows]

            if not times:
                return

            mean_err = sum(abs(a - t) for a, t in zip(angles, targets)) / len(times)
            max_err  = max(abs(a - t) for a, t in zip(angles, targets))

            fig, ax = plt.subplots(figsize=(12, 5))
            fig.patch.set_facecolor('#111418')
            ax.set_facecolor('#161b22')

            ax.plot(times, targets, color='#4d94ff', linewidth=1.2,
                    linestyle='--', label='목표 각도')
            ax.plot(times, angles,  color='#2dd4a0', linewidth=1.5,
                    label='실제 각도')

            ax.set_xlabel('Time (s)', color='#64748b', fontsize=10)
            ax.set_ylabel('Angle (deg)', color='#64748b', fontsize=10)
            ax.set_title(
                f'{self._ts}    mean_err={mean_err:.2f}°  max_err={max_err:.2f}°',
                color='#e2e8f0', fontsize=11, pad=10
            )

            ax.tick_params(colors='#475569')
            for spine in ax.spines.values():
                spine.set_edgecolor('#2a3441')

            ax.grid(True, color='#1e2630', linewidth=0.8)
            ax.set_ylim(0, 100)
            ax.yaxis.set_major_locator(ticker.MultipleLocator(10))
            ax.xaxis.set_major_locator(ticker.MultipleLocator(10))

            legend = ax.legend(facecolor='#1e2630', edgecolor='#2a3441',
                               labelcolor='#e2e8f0', fontsize=9)

            plt.tight_layout()
            fig.savefig(self._png_path, dpi=150, facecolor=fig.get_facecolor())
            plt.close(fig)
            self.get_logger().info(f'[pp_logger] PNG 저장: {self._png_path}')
        except Exception as e:
            self.get_logger().error(f'[pp_logger] PNG 저장 실패: {e}')

    def destroy_node(self):
        self._file.flush()
        self._file.close()
        self._save_png()
        self.get_logger().info(f'[pp_logger] 완료: {self._run_dir}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PpLogger()
    _done = [False]

    def _cleanup():
        if _done[0]:
            return
        _done[0] = True
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # SIGTERM: 런치파일 종료 시 오는 신호 (finally 블록 미실행 → 명시적 처리)
    signal.signal(signal.SIGTERM, lambda s, f: (_cleanup(), sys.exit(0)))

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _cleanup()


if __name__ == '__main__':
    main()
