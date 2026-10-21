#!/usr/bin/env python3
"""Create four figures without modifying the copied original log files."""
import csv
import hashlib
import json
import os
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
ANALYSIS = ROOT / 'test_runs/steering-angle-analysis-20261022/out/gps'
os.environ.setdefault('MPLCONFIGDIR', '/tmp/steering-test-figure-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.collections import LineCollection
import numpy as np

FONT = Path('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
if FONT.exists():
    font_manager.fontManager.addfont(str(FONT))
    plt.rcParams['font.family'] = font_manager.FontProperties(fname=str(FONT)).get_name()
plt.rcParams.update({'axes.unicode_minus': False, 'font.size': 11, 'axes.titlesize': 13,
                     'axes.labelsize': 11, 'savefig.facecolor': 'white'})
HASHES = {}


def read(path):
    data = path.read_bytes()
    HASHES[str(path.relative_to(ROOT))] = hashlib.sha256(data).hexdigest()
    return data.decode()


def values(rows, key):
    return np.array([float(row[key]) for row in rows])


def waypoint_figure(base, summary, rows, label):
    t = values(rows, 'elapsed_s')
    xy = np.column_stack((values(rows, 'east_m'), values(rows, 'north_m')))
    selected = np.array([r['selected'] == 'True' for r in rows])
    fig, ax = plt.subplots(figsize=(11, 10), constrained_layout=True)
    segments = np.stack((xy[:-1], xy[1:]), axis=1)
    collection = LineCollection(segments, cmap='viridis', norm=plt.Normalize(t[0], t[-1]), linewidth=1.2, alpha=.85)
    collection.set_array(t[:-1])
    ax.add_collection(collection)
    points = ax.scatter(*xy.T, c=t, cmap='viridis', s=7, alpha=.75, linewidths=0)
    ax.scatter(*xy[selected].T, s=12, facecolors='none', edgecolors='#1476bd', linewidths=.6,
               label='GPS로 선정한 원선회 waypoint')
    center = np.array(summary['center_enu_m'])
    angles = np.linspace(0, 2*np.pi, 500)
    ring = center + summary['antenna_radius_m'] * np.column_stack((np.cos(angles), np.sin(angles)))
    ax.plot(*ring.T, '--', color='#d34b3d', lw=1.8, label='GPS 위치에 적합한 원')
    ax.scatter(*center, marker='+', s=90, color='#d34b3d', label='적합 원 중심')
    ax.scatter(*xy[0], marker='*', s=150, color='black', zorder=5, label='기록 시작')
    ax.scatter(*xy[-1], marker='X', s=70, color='#c22f58', zorder=5, label='기록 종료')
    indices = np.flatnonzero(selected)
    for fraction in [.2, .65]:
        k = int(len(indices)*fraction)
        i, j = indices[k], indices[min(k+14, len(indices)-1)]
        ax.annotate('', xy=xy[j], xytext=xy[i], arrowprops={'arrowstyle': '-|>', 'lw': 2.2, 'color': '#1476bd'})
    sign_label = '시계 방향' if summary['signed_sweep_deg'] < 0 else '반시계 방향'
    info = (f"GPS 안테나 반경: {summary['antenna_radius_m']:.2f} m\n"
            f"후륜축 중심 반경 추정: {summary['rear_radius_assumed_m']:.2f} m\n"
            f"GPS 실효 조향각 크기: {summary['effective_angle_abs_deg']:.2f}°\n"
            f"선회 방향: {sign_label}")
    ax.text(.98, .98, info, transform=ax.transAxes, ha='right', va='top', fontsize=11,
            bbox={'facecolor': 'white', 'edgecolor': '#d5d9df', 'alpha': .95, 'boxstyle': 'round,pad=.6'})
    ax.set(aspect='equal', xlabel='동쪽 변위 [m]', ylabel='북쪽 변위 [m]')
    ax.autoscale_view()
    ax.margins(.12)
    ax.grid(alpha=.25)
    ax.legend(loc='upper center', bbox_to_anchor=(.5, -.08), ncol=3, fontsize=9, framealpha=.95)
    fig.colorbar(points, ax=ax, shrink=.8, label='기록 시작 후 경과시간 [초]')
    fig.suptitle(f'{label} GPS waypoint 재현\n원시 GNSS 안테나 위치 · 첫 fix를 ENU 원점으로 사용', fontsize=17)
    fig.supxlabel('실효각 가정: 축간거리 1.328 m, 안테나 전방 오프셋 0.559 m, 평면 무슬립 자전거 모델', fontsize=10)
    target = base / 'gps_waypoints.png'
    fig.savefig(target, dpi=190)
    plt.close(fig)
    return target


def comparison_figure(base, summary, label):
    metadata = json.loads(read(base/'metadata.json'))
    files = {v['topic']: base/k for k, v in metadata['files'].items()}
    steering = list(csv.DictReader(read(files['/steering_feedback']).splitlines()))
    odom = list(csv.DictReader(read(files['/gnss/odom']).splitlines()))
    origin = int(odom[0]['received_monotonic_ns'])
    st = np.array([int(r['received_monotonic_ns'])-origin for r in steering])/1e9
    end = (int(odom[-1]['received_monotonic_ns'])-origin)/1e9
    angle = values(steering, 'steer_angle')
    valid = np.array([r['steer_angle_valid'] == 'True' for r in steering])
    lo, hi = summary['start_elapsed_s'], summary['end_elapsed_s']
    mask = valid & (st >= lo) & (st <= hi)
    gps_angle = np.sign(summary['signed_sweep_deg'])*summary['effective_angle_abs_deg']
    median = float(np.median(angle[mask]))
    fig, axes = plt.subplots(2, 1, figsize=(13, 8), constrained_layout=True, height_ratios=[1.3, 1])
    for ax in axes:
        ax.plot(st, np.where(valid, angle, np.nan), color='#2275b7', lw=1.2,
                label='조향 피드백 표시각 (미보정·참고값)')
        ax.plot([lo, hi], [gps_angle, gps_angle], color='#d44a3b', lw=2.5,
                label=f'GPS 원 반경으로 역산한 실효각 ({gps_angle:+.2f}°)')
        ax.axvspan(lo, hi, color='#e7b248', alpha=.14)
        ax.set_ylabel('각도 [°]')
        ax.grid(alpha=.25)
    axes[0].set_xlim(0, end)
    if st[0] > 1:
        axes[0].axvspan(0, st[0], color='gray', alpha=.13, label='조향 피드백 미수신 구간')
    axes[0].set_title('전체 기록 — GPS 역산값은 선정된 원선회 구간에만 표시')
    axes[0].legend(loc='best', fontsize=9)
    axes[0].set_xlabel('기록 시작 후 경과시간 [초]')
    axes[1].set_xlim(lo, hi)
    low, high = min(float(min(angle[mask])), gps_angle), max(float(max(angle[mask])), gps_angle)
    axes[1].set_ylim(low-1, high+1)
    axes[1].set_title('GPS로 선정한 안정 원선회 구간 확대')
    axes[1].set_xlabel('기록 시작 후 경과시간 [초]')
    note = (f"조향 표시각 중앙값 {median:+.2f}°  |  GPS 실효각 {gps_angle:+.2f}°\n"
            f"표시값의 차이 {median-gps_angle:+.2f}° — 각도 정의가 달라 추종 오차로 해석하지 않음")
    axes[1].text(.5, .5, note, transform=axes[1].transAxes, ha='center', va='center', fontsize=11,
                 bbox={'facecolor': 'white', 'edgecolor': '#d5d9df', 'alpha': .95, 'boxstyle': 'round,pad=.6'})
    fig.suptitle(f'{label} 조향 표시각–GPS 실효 조향각 비교', fontsize=17)
    fig.supxlabel('GPS 각은 진행방향(yaw)이 아닌 원선회 등가 조향각의 구간 대표값입니다.\n'
                  'GPS 계산·구간 선정은 조향 표시값과 독립적이며, 구간 밖으로 역산값을 연장하지 않았습니다.', fontsize=10)
    target = base/'steering_vs_gps_angle.png'
    fig.savefig(target, dpi=190)
    plt.close(fig)
    return target, median, float(gps_angle)


def main():
    summary_path = ANALYSIS/'summary.json'
    summaries = json.loads(read(summary_path))
    points = list(csv.DictReader(read(ANALYSIS/'gps_points.csv').splitlines()))
    outputs = []
    for summary in summaries:
        base = HERE/summary['run']
        label = '우회전' if summary['label'] == 'right' else '좌회전'
        rows = [row for row in points if row['label'] == summary['label']]
        waypoint = waypoint_figure(base, summary, rows, label)
        comparison, median, gps_angle = comparison_figure(base, summary, label)
        outputs.append({'run': summary['run'], 'label': label, 'gps_circle': summary,
                        'displayed_feedback_median_deg': median, 'gps_signed_effective_angle_deg': gps_angle,
                        'files': [{'path': str(path.relative_to(HERE)), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
                                  for path in [waypoint, comparison]]})
    original_manifest = json.loads((HERE/'copy_manifest.json').read_text())
    count = 0
    for record in original_manifest['records']:
        for f in record['files']:
            path = HERE/record['destination']/f['path']
            assert hashlib.sha256(path.read_bytes()).hexdigest() == f['sha256'], str(path)
            count += 1
    manifest = {'inputs': HASHES, 'figures': outputs, 'original_files_unchanged': count,
                'gps_line_definition': 'Signed constant circle-fit equivalent angle, shown only over the GPS-selected interval; not instantaneous GPS heading or wheel-angle ground truth.',
                'reproduce': 'python3 main/src/control_manager/log_significant/steering_test/make_figures.py'}
    (HERE/'figure_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'figures': [f['path'] for o in outputs for f in o['files']],
                      'original_files_unchanged': count}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
