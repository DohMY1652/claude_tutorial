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
from std_msgs.msg import Float64MultiArray, UInt16MultiArray

SCHEMA_VERSION = 5           # 형식을 바꾸면 올린다. 과거 CSV 구분용(meta.json 에 기록)
#   5: refgen 말미 6 개(rail SP·탱크·부스트·이젝터) 열 추가
NAMESPACE  = '/pack2'
LOG_HZ     = 100.0
# powerpack_config.yaml 의 num_actuators 최대치(6)에 맞춘다. 실제로 6축보다 적게
# 돌려도 남는 축은 대기압으로 기록될 뿐이라 해가 없다. 3 으로 두면 6축 운전 시
# axis3~5 가 통째로 빠진다.
NUM_AXES   = 6               # PositionController.axis0~5 (board17~22)
POS_GIDS   = [0, 1, 2, 3, 4, 5]      # axis별 양압 채널 global_id
NEG_GIDS   = [6, 7, 8, 9, 10, 11]    # axis별 음압 채널 global_id
CHANNEL_BOARD_OFFSET = 5     # board_id = gid + CHANNEL_BOARD_OFFSET
PWM_BOARDS = 18              # board/pwm_cmd·board/currents 배열 크기 = PWM_BOARDS*3
VALVE_NAMES = ['v1micro', 'v2atm', 'v3macro']   # 보드 위 v1/v2/v3 순서
# 라인 밸브 pwm 인덱스 (Controller.cpp 기본값)
LINE_PWM_IDX = [('line_pos', 0), ('line_neg', 3), ('macro_sw', 9)]


def _pwm_base(gid):
    """gid 채널 보드의 board/pwm_cmd·currents 시작 인덱스."""
    return (gid + CHANNEL_BOARD_OFFSET - 1) * 3


def _use_cjk_font():
    """설치된 한글 글꼴을 하나 고른다. 없으면 조용히 기본값을 쓴다."""
    global _CJK
    if _CJK is not None:
        return _CJK
    _CJK = False
    try:
        import matplotlib
        from matplotlib import font_manager
        have = {f.name for f in font_manager.fontManager.ttflist}
        for name in ('NanumGothic', 'NanumBarunGothic', 'Noto Sans CJK KR',
                     'Noto Sans KR', 'Malgun Gothic', 'AppleGothic', 'UnDotum'):
            if name in have:
                matplotlib.rcParams['font.family'] = name
                matplotlib.rcParams['axes.unicode_minus'] = False
                _CJK = True
                return True
    except Exception:
        pass
    return False


_CJK = None


def _L(ko, en):
    """한글 글꼴이 있으면 한글, 없으면 영문 라벨."""
    return ko if _CJK else en


class PpLogger(Node):
    def __init__(self):
        super().__init__('pp_logger')
        self._lock = threading.Lock()

        # 데이터 버퍼
        self._pos_dbg  = [0.0] * (8 * NUM_AXES)
        self._sensors  = [101.325] * 25
        self._mpc_refs = [101.325] * 12
        self._encoders = [0.0] * 9
        self._pwm      = [0] * (PWM_BOARDS * 3)      # 지령 [0..4095]
        self._curr     = [0.0] * (PWM_BOARDS * 3)    # 실측 전류 [mV], mA = mV/10
        self._vols     = [0.0] * (2 * NUM_AXES)      # active_volumes_ml
        # controller/refgen_dbg: 축마다 12개
        #   [angle, angle_ref, tau_ref, tau_ach, P⁺ref, P⁻ref,
        #    ub_pos, lb_pos, lb_neg, ub_neg, starve_pos%, starve_neg%]
        # control_mode=2 에서는 position_dbg 가 발행되지 않아 각도/목표가 비어
        # 있었다. 슬루 경계와 유량부족률은 "왜 그 목표가 나왔는지" 를 설명한다.
        self._rg       = [0.0] * (12 * NUM_AXES)
        # 말미 공용 6 개: [rail_pos_sp, rail_neg_sp, tank, tank_low, boost g/s, eject g/s]
        self._rg_tail  = [0.0] * 6
        self._rg_seen  = False

        ns = NAMESPACE
        self.create_subscription(Float64MultiArray, f'{ns}/controller/position_dbg',
                                 self._cb_pos_dbg, 10)
        self.create_subscription(Float64MultiArray, f'{ns}/controller/sensors_kpa',
                                 self._cb_sensors, 10)
        self.create_subscription(Float64MultiArray, f'{ns}/controller/mpc_refs_kpa',
                                 self._cb_refs, 10)
        self.create_subscription(Float64MultiArray, f'{ns}/board/analog',
                                 self._cb_analog, 10)
        # 밸브가 실제로 무엇을 받고(pwm) 무엇이 흘렀는지(current) — 지령 0 인데
        # 압력이 오르는지 판별하려면 이 둘이 반드시 같이 있어야 한다.
        self.create_subscription(UInt16MultiArray, f'{ns}/board/pwm_cmd',
                                 self._cb_pwm, 10)
        self.create_subscription(Float64MultiArray, f'{ns}/board/currents',
                                 self._cb_curr, 10)
        self.create_subscription(Float64MultiArray, f'{ns}/controller/active_volumes_ml',
                                 self._cb_vol, 10)
        self.create_subscription(Float64MultiArray, f'{ns}/controller/pressure_ref_dbg',
                                 self._cb_refgen, 10)

        # 폴더 및 파일 생성
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._ts = ts
        result_base = os.path.expanduser('~/result')
        self._run_dir = os.path.join(result_base, ts)
        os.makedirs(self._run_dir, exist_ok=True)

        csv_path = os.path.join(self._run_dir, f'{ts}.csv')
        self._png_path = os.path.join(self._run_dir, f'{ts}.png')
        self._diag_png_path = os.path.join(self._run_dir, f'{ts}_valve.png')

        self._file   = open(csv_path, 'w', newline='', buffering=1)
        self._writer = csv.writer(self._file)
        self._rows   = []  # PNG 생성용 메모리 버퍼

        header = ['time_sec']
        for a in range(NUM_AXES):
            header += [
                f'angle_deg_axis{a}', f'target_deg_axis{a}', f'err_deg_axis{a}', f'vel_dps_axis{a}',
                f'pid_kpa_axis{a}', f'ff_kpa_axis{a}', f'fric_kpa_axis{a}',
                f'p_pos_ref_kpa_axis{a}', f'p_neg_ref_kpa_axis{a}',
                f'p_pos_actual_kpa_axis{a}', f'p_neg_actual_kpa_axis{a}',
            ]
        header += [
            'p_line_pos_kpa',
            'p_line_neg_kpa',
            'p_macro_pos_kpa',
            'p_macro_neg_kpa',
        ] + [f'mpc_ref_gid{i}_kpa' for i in range(12)] \
          + [f'enc_bd{17 + i}_deg'  for i in range(6)]

        # 밸브 지령 [%] 과 실측 전류 [mA] — 축별 양압/음압 채널 보드 3밸브씩
        for a in range(NUM_AXES):
            for sign, gid in (('pos', POS_GIDS[a]), ('neg', NEG_GIDS[a])):
                bd = gid + CHANNEL_BOARD_OFFSET
                for v in VALVE_NAMES:
                    header.append(f'pwm_pct_{sign}_bd{bd}_{v}_axis{a}')
                for v in VALVE_NAMES:
                    header.append(f'cur_mA_{sign}_bd{bd}_{v}_axis{a}')
        for name, idx in LINE_PWM_IDX:
            header.append(f'pwm_pct_{name}')
            header.append(f'cur_mA_{name}')
        for a in range(NUM_AXES):
            header.append(f'vol_pos_ml_axis{a}')
            header.append(f'vol_neg_ml_axis{a}')
        # 레퍼런스 생성기 내부 — 왜 그 목표가 나왔는지 설명하는 값들
        for a in range(NUM_AXES):
            header += [f'tau_ref_Nm_axis{a}', f'tau_ach_Nm_axis{a}',
                       f'ub_pos_kpa_axis{a}', f'lb_pos_kpa_axis{a}',
                       f'lb_neg_kpa_axis{a}', f'ub_neg_kpa_axis{a}',
                       f'starve_pos_pct_axis{a}', f'starve_neg_pct_axis{a}']
        # refgen_dbg 말미 공용 6 개. rail SP 가 없으면 "양압이 왜 느린가" 를
        # 로그만으로 못 가른다 — 셋포인트가 낮은 것인지(제어), 셋포인트는 높은데
        # 레일이 못 따라오는 것인지(공급) 구분이 안 된다.
        header += ['rail_pos_sp_kpa', 'rail_neg_sp_kpa', 'tank_kpa',
                   'tank_low', 'boost_gps', 'eject_gps']

        self._writer.writerow(header)
        self._header = header
        self._write_meta(csv_path, header)
        self._start_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(f'[pp_logger] 저장 경로: {self._run_dir}')

        self.create_timer(1.0 / LOG_HZ, self._write_row)

    def _write_meta(self, csv_path, header):
        """어떤 설정으로 돈 결과인지 남긴다 — 나중에 결과를 비교할 때 이게 없으면
        같은 형식인지조차 알 수 없다."""
        import json
        meta = {
            'schema_version': SCHEMA_VERSION,
            'timestamp': self._ts,
            'log_hz': LOG_HZ,
            'num_axes': NUM_AXES,
            'pos_gids': POS_GIDS,
            'neg_gids': NEG_GIDS,
            'valve_names': VALVE_NAMES,
            'csv': os.path.basename(csv_path),
            'columns': header,
        }
        try:
            import subprocess
            meta['git_commit'] = subprocess.run(
                ['git', 'rev-parse', '--short', 'HEAD'],
                cwd=os.path.dirname(os.path.abspath(__file__)),
                capture_output=True, text=True, timeout=3).stdout.strip() or None
        except Exception:
            meta['git_commit'] = None
        try:
            with open(os.path.join(self._run_dir, 'meta.json'), 'w') as f:
                json.dump(meta, f, indent=2, ensure_ascii=False)
        except Exception as e:
            self.get_logger().warn(f'[pp_logger] meta.json 실패: {e}')

    def _cb_pos_dbg(self, msg):
        with self._lock:
            n = min(len(msg.data), 8 * NUM_AXES)
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

    def _cb_pwm(self, msg):
        with self._lock:
            for i, v in enumerate(msg.data):
                if i < len(self._pwm): self._pwm[i] = int(v)

    def _cb_curr(self, msg):
        with self._lock:
            for i, v in enumerate(msg.data):
                if i < len(self._curr): self._curr[i] = float(v)

    def _cb_refgen(self, msg):
        # 메시지는 [축블록 12 개] × N + [공용 말미 6 개] 다. N 은 num_actuators 라
        # NUM_AXES(6)보다 작을 수 있으므로 말미는 **뒤에서** 읽어야 한다 —
        # 앞에서 세면 축 수가 다를 때 엉뚱한 값을 rail SP 로 적는다.
        d = list(msg.data)
        with self._lock:
            self._rg_seen = True
            for i, v in enumerate(d[:len(self._rg)]):
                self._rg[i] = float(v)
            if len(d) >= 6:
                self._rg_tail = [float(x) for x in d[-6:]]

    def _cb_vol(self, msg):
        with self._lock:
            for i, v in enumerate(msg.data):
                if i < len(self._vols): self._vols[i] = float(v)

    def _write_row(self):
        with self._lock:
            pos = list(self._pos_dbg)
            sen = list(self._sensors)
            ref = list(self._mpc_refs)
            enc = list(self._encoders)
            pwm = list(self._pwm)
            cur = list(self._curr)
            vol = list(self._vols)
            rg  = list(self._rg); rg_seen = self._rg_seen
            rgt = list(self._rg_tail)

        elapsed = (self.get_clock().now().nanoseconds - self._start_ns) / 1e9

        row = [f'{elapsed:.4f}']
        axis_angles = []
        for a in range(NUM_AXES):
            angle, target, p_pos, p_neg, p_pid, p_ff, p_fric, vel = pos[a*8:(a+1)*8]
            if rg_seen:                      # control_mode=2 는 position_dbg 를 안 낸다
                angle, target = rg[12*a], rg[12*a + 1]
                p_pos, p_neg  = rg[12*a + 4], rg[12*a + 5]
            pos_board_idx = POS_GIDS[a] + CHANNEL_BOARD_OFFSET - 1
            neg_board_idx = NEG_GIDS[a] + CHANNEL_BOARD_OFFSET - 1
            row += [
                f'{angle:.4f}', f'{target:.4f}',
                f'{target - angle:.4f}', f'{vel:.4f}',
                f'{p_pid:.4f}', f'{p_ff:.4f}', f'{p_fric:.4f}',
                f'{p_pos:.4f}', f'{p_neg:.4f}',
                f'{sen[pos_board_idx]:.4f}',
                f'{sen[neg_board_idx]:.4f}',
            ]
            axis_angles.append((angle, target))

        row += [
            f'{sen[0]:.4f}', f'{sen[1]:.4f}',
            f'{sen[2]:.4f}', f'{sen[3]:.4f}',
        ] + [f'{v:.4f}' for v in ref[:12]] \
          + [f'{enc[i]:.4f}' for i in range(6)]

        pwm_trace = []          # 진단 PNG 용: axis0 양압 3밸브
        for a in range(NUM_AXES):
            for sign, gid in (('pos', POS_GIDS[a]), ('neg', NEG_GIDS[a])):
                b = _pwm_base(gid)
                pcts = [pwm[b + k] / 40.95 for k in range(3)]
                row += [f'{x:.2f}' for x in pcts]
                row += [f'{cur[b + k] / 10.0:.2f}' for k in range(3)]
                if a == 0 and sign == 'pos':
                    pwm_trace = pcts
        for _name, idx in LINE_PWM_IDX:
            row.append(f'{pwm[idx] / 40.95:.2f}')
            row.append(f'{cur[idx] / 10.0:.2f}')
        for a in range(NUM_AXES):
            row.append(f'{vol[a]:.4f}')
            row.append(f'{vol[NUM_AXES + a]:.4f}')
        for a in range(NUM_AXES):
            b = 12 * a
            row += [f'{rg[b + 2]:.4f}', f'{rg[b + 3]:.4f}',
                    f'{rg[b + 6]:.4f}', f'{rg[b + 7]:.4f}',
                    f'{rg[b + 8]:.4f}', f'{rg[b + 9]:.4f}',
                    f'{rg[b + 10]:.2f}', f'{rg[b + 11]:.2f}']
        row += [f'{rgt[0]:.4f}', f'{rgt[1]:.4f}', f'{rgt[2]:.4f}',
                f'{rgt[3]:.0f}', f'{rgt[4]:.4f}', f'{rgt[5]:.4f}']

        self._writer.writerow(row)
        b0 = _pwm_base(POS_GIDS[0])
        self._rows.append((elapsed, axis_angles, ref[POS_GIDS[0]],
                           sen[POS_GIDS[0] + CHANNEL_BOARD_OFFSET - 1],
                           pwm_trace or [0.0, 0.0, 0.0],
                           [cur[b0 + k] / 10.0 for k in range(3)]))

    def _save_png(self):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            import matplotlib.ticker as ticker
            _use_cjk_font()

            times = [r[0] for r in self._rows]
            if not times:
                return

            # 축별 (실제, 목표) 각도 시퀀스
            per_axis = [[r[1][a] for r in self._rows] for a in range(NUM_AXES)]

            # 다크 서페이스용 categorical 팔레트 (blue/orange/aqua, all-pairs 검증됨)
            axis_colors = ['#3987e5', '#d95926', '#199e70']

            fig, ax = plt.subplots(figsize=(12, 5))
            fig.patch.set_facecolor('#111418')
            ax.set_facecolor('#161b22')

            err_parts = []
            for a in range(NUM_AXES):
                angles  = [ang for ang, _ in per_axis[a]]
                targets = [tgt for _, tgt in per_axis[a]]
                color = axis_colors[a % len(axis_colors)]

                ax.plot(times, targets, color=color, linewidth=1.1, linestyle='--',
                        alpha=0.7, label=f'axis{a} ' + _L('목표', 'target'))
                ax.plot(times, angles, color=color, linewidth=1.6,
                        label=f'axis{a} ' + _L('실제', 'actual'))

                mean_err = sum(abs(x - y) for x, y in zip(angles, targets)) / len(times)
                max_err  = max(abs(x - y) for x, y in zip(angles, targets))
                err_parts.append(f'axis{a}: mean={mean_err:.2f}° max={max_err:.2f}°')

            ax.set_xlabel('Time (s)', color='#64748b', fontsize=10)
            ax.set_ylabel('Angle (deg)', color='#64748b', fontsize=10)
            ax.set_title(
                f'{self._ts}    ' + '  |  '.join(err_parts),
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
                               labelcolor='#e2e8f0', fontsize=9, ncol=NUM_AXES)

            plt.tight_layout()
            fig.savefig(self._png_path, dpi=150, facecolor=fig.get_facecolor())
            plt.close(fig)
            self.get_logger().info(f'[pp_logger] PNG 저장: {self._png_path}')
        except Exception as e:
            self.get_logger().error(f'[pp_logger] PNG 저장 실패: {e}')

    def _save_diag_png(self):
        """axis0 양압 채널: 레퍼런스/실측 압력 · 밸브 지령 · 실측 전류를 겹쳐 본다.

        지령이 0 인데 압력이 오르면 밸브가 새는 것이고, 지령이 0 이 아니면
        컨트롤러가 여는 것이다. 두 경우를 한 장에서 구분하려고 만든 그림이다.
        """
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            _use_cjk_font()

            if not self._rows or len(self._rows[0]) < 6:
                return
            t    = [r[0] for r in self._rows]
            pref = [r[2] for r in self._rows]
            pact = [r[3] for r in self._rows]
            pwm  = [r[4] for r in self._rows]
            cur  = [r[5] for r in self._rows]

            colors = ['#3987e5', '#d95926', '#199e70']
            fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
            fig.patch.set_facecolor('#111418')

            axes[0].plot(t, pref, color='#94a3b8', ls='--', lw=1.2, label=_L('레퍼런스', 'reference'))
            axes[0].plot(t, pact, color='#3987e5', lw=1.5, label=_L('실측', 'measured'))
            axes[0].set_ylabel(_L('압력 [kPa]', 'pressure [kPa]'), color='#64748b', fontsize=10)

            for k, name in enumerate(VALVE_NAMES):
                axes[1].plot(t, [p[k] for p in pwm], color=colors[k], lw=1.3, label=name)
                axes[2].plot(t, [c[k] for c in cur], color=colors[k], lw=1.3, label=name)
            axes[1].set_ylabel(_L('밸브 지령 [%]', 'valve cmd [%]'), color='#64748b', fontsize=10)
            axes[2].set_ylabel(_L('실측 전류 [mA]', 'current [mA]'), color='#64748b', fontsize=10)
            axes[2].set_xlabel('Time (s)', color='#64748b', fontsize=10)

            errs = [abs(a - b) for a, b in zip(pact, pref)]
            mx   = [max(p[k] for p in pwm) for k in range(3)]
            axes[0].set_title(
                f'{self._ts}  axis0 ' + _L('양압', 'pos')
                + f' (board{POS_GIDS[0] + CHANNEL_BOARD_OFFSET})   '
                + _L('추종오차 평균 ', 'err mean ')
                + f'{sum(errs)/len(errs):.2f} ' + _L('최대 ', 'max ')
                + f'{max(errs):.2f} kPa   |   '
                + '  '.join(f'{n} ' + _L('최대 ', 'max ') + f'{v:.1f}%'
                            for n, v in zip(VALVE_NAMES, mx)),
                color='#e2e8f0', fontsize=10, pad=10)

            for ax in axes:
                ax.set_facecolor('#161b22')
                ax.tick_params(colors='#475569')
                ax.grid(True, color='#1e2630', linewidth=0.8)
                for sp in ax.spines.values():
                    sp.set_edgecolor('#2a3441')
                ax.legend(facecolor='#1e2630', edgecolor='#2a3441',
                          labelcolor='#e2e8f0', fontsize=9, ncol=3)

            plt.tight_layout()
            fig.savefig(self._diag_png_path, dpi=150, facecolor=fig.get_facecolor())
            plt.close(fig)
            self.get_logger().info(f'[pp_logger] 밸브 진단 PNG: {self._diag_png_path}')
        except Exception as e:
            self.get_logger().error(f'[pp_logger] 밸브 진단 PNG 실패: {e}')

    def destroy_node(self):
        self._file.flush()
        self._file.close()
        self._save_png()
        self._save_diag_png()
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
    except Exception:
        # 종료 과정에서 나온 예외라면 무시한다.
        #
        # 런치가 SIGINT 를 보내면 rclpy 의 신호 처리기가 먼저 컨텍스트를 닫는다.
        # 그 순간 spin() 은 아직 wait set 을 만드는 중이라 무효해진 컨텍스트로
        # RCLError("the given context is not valid") 를 던진다 (경로에 따라
        # ExternalShutdownException 이 오기도 한다). 둘 다 정상 종료 과정이고,
        # 우리 쪽 _cleanup 은 아직 안 돌았을 수 있으므로 rclpy.ok() 로 판정한다.
        # 잡지 않으면
        # 종료코드 1 이 되어 런치가 "process has died" 로 보고한다 — CSV·PNG 는
        # 멀쩡히 저장되는데도 실패처럼 보인다.
        # rclpy 가 이미 컨텍스트를 닫았으면(=정상 종료 과정이면) 무시한다.
        if rclpy.ok() and not _done[0]:
            raise
    finally:
        _cleanup()


if __name__ == '__main__':
    main()
