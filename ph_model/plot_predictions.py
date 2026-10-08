"""Plot saved offline predictions only; no fitting, ROS, or hardware access."""
import argparse
import csv
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.backends.backend_pdf import PdfPages


DEFAULT = Path('reports/residual_actuator4_20261007/nominal_refit_v3/all_runs.csv')
SERIES = [('angle_actual_deg', '실측', '#20252b', 1.7, '-'),
          ('angle_nominal_refit_deg', '공칭 모델', '#e38714', 1.3, '--'),
          ('angle_residual_selected_deg', '전체 모델 (공칭 + residual)', '#1479bf', 1.3, '-')]


def rmse(predicted, measured):
    return float(np.sqrt(np.mean((predicted - measured)**2)))


def block_slices(data):
    """Never draw a line across a run boundary or model-state reset."""
    n = len(data['run_id'])
    if not n:
        return []
    starts = np.r_[0, np.flatnonzero(
        (data['run_id'][1:] != data['run_id'][:-1]) |
        (data['block_id'][1:] != data['block_id'][:-1]) |
        (data['block_start'][1:] == 1)) + 1]
    return [slice(int(a), int(b)) for a, b in zip(starts, np.r_[starts[1:], n])]


def read_csv(path):
    with path.open(newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError('Empty predictions CSV')
    data = {k: np.array([r[k] for r in rows], dtype=str if k in ('run_id', 'role') else float)
            for k in rows[0]}
    for key, values in data.items():
        if key not in ('run_id', 'role') and not np.isfinite(values).all():
            raise ValueError(f'Nonfinite column: {key}')
    return data


def traces(ax, data, time, error=False):
    for i, s in enumerate(block_slices(data)):
        for key, label, color, width, style in SERIES[int(error):]:
            y = data[key][s]
            if error:
                y = y - data['angle_actual_deg'][s]
            ax.plot(time[s], y, color=color, lw=width, ls=style,
                    label=label if i == 0 else None)
    if error:
        ax.axhline(0, color='#777777', lw=.6)
    ax.grid(alpha=.2)
    ax.set_ylabel('예측 − 실측 [°]' if error else '각도 [°]')


def title(data, name):
    actual = data['angle_actual_deg']
    return (f'{name}  |  RMSE: 공칭 {rmse(data[SERIES[1][0]], actual):.2f}° / '
            f'전체 {rmse(data[SERIES[2][0]], actual):.2f}°')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--csv', type=Path, default=DEFAULT)
    ap.add_argument('--output', type=Path)
    args = ap.parse_args()
    out = args.output or args.csv.parent / 'graphs'
    out.mkdir(parents=True, exist_ok=True)
    font = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font.exists():
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams.update({'axes.unicode_minus': False, 'font.size': 10})
    data = read_csv(args.csv)
    runs = list(dict.fromkeys(data['run_id']))
    groups = [{k: v[data['run_id'] == name] for k, v in data.items()} for name in runs]
    labels = []
    seed = 0
    for name, group in zip(runs, groups):
        profile = name.rsplit('_', 1)[1]
        if profile == 'S6':
            profile += f' seed {seed}'
            seed += 1
        role = {'train': '학습', 'development': '개발',
                'historical_diagnostic_prefix': '기존 진단 (비블라인드)'}.get(group['role'][0], group['role'][0])
        labels.append(f'{profile} · {role}')
    with PdfPages(out / 'comparison_all_pages.pdf') as pdf:
        def save(fig, name):
            fig.savefig(out / (name + '.png'), dpi=180, facecolor='white')
            pdf.savefig(fig)
            plt.close(fig)

        fig, axes = plt.subplots(2, 1, figsize=(22, 8), sharex=True,
                                 gridspec_kw={'height_ratios': [2, 1]})
        traces(axes[0], data, data['time_s'])
        traces(axes[1], data, data['time_s'], error=True)
        axes[0].legend(loc='upper left', ncol=3)
        axes[0].set_title(title(data, '전체 데이터 (학습·개발·진단 혼합) — 최신 모델'), pad=35)
        for group, label in zip(groups, labels):
            start, end = group['time_s'][[0, -1]]
            for ax in axes:
                ax.axvline(start, color='#777777', lw=.6, alpha=.45)
            axes[0].text((start + end)/2, 1.02, label.replace(' · ', '\n'),
                         transform=axes[0].get_xaxis_transform(), ha='center', fontsize=7)
        axes[1].set_xlabel('그래프용 연속 시간 [s] — 실행/유효 블록별 초기 상태 재설정, 경계 연결 없음')
        fig.tight_layout()
        save(fig, '01_all_runs_overlay')

        fig, axes = plt.subplots(int(np.ceil(len(runs)/3)), 3, figsize=(19, 13), squeeze=False)
        for ax, group, label in zip(axes.flat, groups, labels):
            traces(ax, group, group['t_mono_s'])
            ax.set_title(title(group, label), fontsize=9)
            ax.set_xlabel('실행 시간 [s]')
        for ax in axes.flat[len(groups):]:
            ax.set_visible(False)
        handles, names = axes[0, 0].get_legend_handles_labels()
        fig.legend(handles, names, loc='upper center', ncol=3)
        fig.suptitle('프로파일별 비교 — 검정: 실측 / 주황: 공칭 / 파랑: 전체 모델', y=.965)
        fig.tight_layout(rect=(0, 0, 1, .95))
        save(fig, '02_profiles_overview')

        for index, (name, group, label) in enumerate(zip(runs, groups, labels), start=1):
            fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True,
                                     gridspec_kw={'height_ratios': [2.5, 1.2, 1]})
            time = group['t_mono_s']
            traces(axes[0], group, time)
            traces(axes[1], group, time, error=True)
            axes[0].legend(loc='best', ncol=3)
            axes[0].set_title(title(group, label) + '\n' + name)
            for i, s in enumerate(block_slices(group)):
                for key, text, color in [('p_pos_kpa_abs', '측정 양압 챔버', '#bc4e68'),
                                         ('p_neg_kpa_abs', '측정 음압 챔버', '#509b60')]:
                    axes[2].plot(time[s], group[key][s], color=color, lw=1,
                                 label=text if i == 0 else None)
                if i:
                    for ax in axes:
                        ax.axvline(time[s.start], color='#888888', ls=':', lw=.8)
            axes[2].set_ylabel('압력 [kPa abs]')
            axes[2].set_xlabel('원래 실행 시간 [s] — 점선: 유효 블록 시작/상태 재설정')
            axes[2].legend(loc='best', ncol=2)
            axes[2].grid(alpha=.2)
            fig.tight_layout()
            save(fig, f'{index+2:02d}_{name}')
    print(f'Saved {len(runs)+2} PNG plots and comparison_all_pages.pdf to {out}')


if __name__ == '__main__':
    main()
