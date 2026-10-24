#!/usr/bin/env python3
"""Compare the two requested motor trials; results are written under doc/graphs."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

DOC = Path(__file__).resolve().parent
DEFAULTS = ['20261024T130249-c41e8d31-trial1', '20261024T130533-45eb40df-KP16160RETRY-trial1']


def read_trial(name):
    path = Path(name)
    if not path.is_dir():
        path = DOC/'graphs'/name
    summary = json.loads((path/'summary.json').read_text())
    events = json.loads((path/'events.json').read_text())
    with (path/'actual.csv').open() as file:
        actual = list(csv.DictReader(file))
    with (path/'ref.csv').open() as file:
        reference = list(csv.DictReader(file))
    if any(r['valid'].lower() != 'true' for r in actual):
        raise ValueError('Invalid feedback samples present; inspect gaps before comparing')
    trial = dict(path=path, summary=summary, events=events,
                 t=np.array([float(r['time_s']) for r in actual]),
                 v=np.array([float(r['feedback_rpm']) for r in actual]),
                 tick=np.array([int(r['motor_mcu_ms']) for r in actual]),
                 rt=np.array([float(r['time_s']) for r in reference]),
                 rv=np.array([float(r['command_rpm']) for r in reference]))
    assert np.all(np.diff(trial['t']) > 0)
    assert np.all(np.diff(trial['rt']) >= 0)
    assert max(np.diff(trial['t'])) < summary['settings']['motor_feedback_timeout_s']
    return trial


def segments(trial, lo, hi, cuts=()):
    t, rt = trial['t'], trial['rt']
    grid = np.unique(np.r_[lo, hi, list(cuts), t[(t > lo) & (t < hi)], rt[(rt > lo) & (rt < hi)]])
    assert t[0] <= lo and t[-1] >= hi and rt[0] <= lo and rt[-1] >= hi
    actual = trial['v'][np.searchsorted(t, grid[:-1], side='right')-1]
    reference = trial['rv'][np.searchsorted(rt, grid[:-1], side='right')-1]
    return grid[:-1], np.diff(grid), actual, reference


def metrics(trial, lo, hi, exclusions=()):
    # Both streams are held between observations. Integrate on their union of
    # timestamp boundaries so repeated feedback and 100-Hz refs are not equally
    # weighted as independent samples. All times use the PC ROS clock.
    cuts = [point for interval in exclusions for point in interval if lo < point < hi]
    starts, dt, actual, reference = segments(trial, lo, hi, cuts)
    keep = np.ones(len(dt), dtype=bool)
    for a, b in exclusions:
        keep &= ~((starts >= a) & (starts < b))
    error = actual[keep]-reference[keep]
    weights = dt[keep]
    duration = float(sum(weights))
    return dict(start_s=lo, end_s=hi, integrated_duration_s=duration,
                rmse_rpm=float(math.sqrt(np.sum(error**2*weights)/duration)),
                mae_rpm=float(np.sum(abs(error)*weights)/duration),
                bias_rpm=float(np.sum(error*weights)/duration),
                max_positive_error_rpm=float(max(error)), min_signed_error_rpm=float(min(error)),
                iae_rpm_s=float(np.sum(abs(error)*weights)),
                actual_mean_rpm=float(np.sum(actual[keep]*weights)/duration),
                actual_min_rpm=float(min(actual[keep])), actual_max_rpm=float(max(actual[keep])))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trials', nargs='*', help='Two extracted trial folders, defaults to the requested pair')
    args = parser.parse_args()
    names = args.trials or DEFAULTS
    if len(names) != 2:
        parser.error('Specify exactly two trials')
    trials = [read_trial(name) for name in names]
    assert trials[0]['summary']['settings'] == trials[1]['summary']['settings'], 'Profiles/settings differ'
    settings = trials[0]['summary']['settings']
    origin = lambda trial: trial['summary']['arm_ros_ns']
    brake_times = [min((e.get('command_ns') or e['ros_ns'])-origin(trial)
                       for e in trial['events'] if e['event'] == 'brake')/1e9 for trial in trials]
    rise, hold, fall = (settings[k] for k in ['startup_time_s', 'hold_time_s', 'transition_time_s'])
    boundary = [0., rise, rise+hold, rise+hold+fall, rise+2*hold+fall]
    assert all(t >= boundary[-1] for t in brake_times), 'One trial braked before nominal completion'
    assert boundary[-1] == 12., 'This comparison uses a 12 s common observation interval'
    phase_names = ['initial_ramp', 'high_hold', 'falling_ramp', 'low_hold']
    phases = list(zip(phase_names, boundary[:-1], boundary[1:]))
    factor = math.pi*settings['wheel_diameter_m']*3.6/60
    targets = [settings['start_speed_kph']/factor, settings['end_speed_kph']/factor]
    out = DOC/'graphs/compare-c41e8d31-vs-45eb40df'
    out.mkdir(parents=True, exist_ok=True)
    zero_intervals = []
    report = []
    for trial in trials:
        row = dict(trial=trial['path'].name, overall=metrics(trial, 0., 12.), phases={}, steady={})
        for name, lo, hi in phases:
            row['phases'][name] = metrics(trial, lo, hi)
        for label, lo, hi, target in [('high', 5., 6., targets[0]), ('low', 11., 12., targets[1])]:
            values = metrics(trial, lo, hi)
            fresh = np.r_[True, np.diff(trial['tick']) != 0]
            measured = trial['v'][(trial['t'] >= lo) & (trial['t'] < hi) & fresh]
            values.update(target_rpm=target,
                          signed_error_pct=100*values['bias_rpm']/target,
                          fresh_sample_std_rpm=float(np.std(measured)), fresh_sample_count=len(measured),
                          tail_mean_within_5pct=abs(values['bias_rpm']) <= .05*target)
            row['steady'][label] = values
        after_ramp = (trial['t'] >= rise) & (trial['t'] < 12.) & (trial['v'] == 0)
        indexes = np.where(after_ramp)[0]
        row['zero_valued_intervals'] = []
        for group in np.split(indexes, np.where(np.diff(indexes) != 1)[0]+1):
            if not len(group):
                continue
            a, b = float(trial['t'][group[0]]), float(trial['t'][group[-1]+1])
            zero_intervals.append((a, b))
            row['zero_valued_intervals'].append([a, b])
        reached = trial['t'][(trial['t'] >= 0) & (trial['t'] < rise+hold) & (trial['v'] >= .9*targets[0])]
        row['first_90pct_high_target_s'] = float(reached[0]) if len(reached) else None
        report.append(row)

    # Recalculate BOTH trials over the same intervals if an isolated zero-valued
    # observation is excluded. This is sensitivity analysis, not data correction.
    for row, trial in zip(report, trials):
        row['sensitivity_excluding_zero_intervals_from_both'] = metrics(trial, 0., 12., zero_intervals)
        row['falling_sensitivity_excluding_zero_intervals_from_both'] = metrics(trial, 6., 9., zero_intervals)
    improvement = {key: 100*(1-report[1]['overall'][key]/report[0]['overall'][key])
                   for key in ['rmse_rpm', 'mae_rpm', 'iae_rpm_s']}
    artifact = dict(trials=report, b_improvement_pct=improvement, common_window_s=[0., 12.],
                    settings=settings, zero_intervals_excluded_only_in_sensitivity=zero_intervals,
                    method='PC time since arm; exact time-weighted zero-order holds for recorded reference and feedback; final brake excluded',
                    limitation='Two ramp trials, one run each. Actual controller Kp/Ki, load and battery conditions are not recorded in these settings. Not a step-response identification.',
                    inputs_sha256={str(trial['path']/name):hashlib.sha256((trial['path']/name).read_bytes()).hexdigest()
                                   for trial in trials for name in ['actual.csv', 'ref.csv', 'summary.json', 'events.json']})
    (out/'metrics.json').write_text(json.dumps(artifact, ensure_ascii=False, indent=2))
    with (out/'metrics.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['trial', 'interval', 'rmse_rpm', 'mae_rpm', 'bias_rpm', 'actual_mean_rpm'])
        for row in report:
            for label, result in [('overall_0_to_12s', row['overall']), *row['phases'].items(), *row['steady'].items()]:
                writer.writerow([row['trial'], label, *[result[k] for k in ['rmse_rpm', 'mae_rpm', 'bias_rpm', 'actual_mean_rpm']]])

    installed = {font.name for font in font_manager.fontManager.ttflist}
    for name in ['Noto Sans CJK KR', 'Noto Sans CJK JP', 'NanumGothic']:
        if name in installed:
            plt.rcParams['font.family'] = name
            break
    plt.rcParams.update({'axes.unicode_minus': False, 'font.size': 11, 'figure.dpi': 160})
    labels = ['A: 13:02:49 (c41e8d31)', 'B: KP16160RETRY (45eb40df)']
    colors = ['#2764ac', '#e67e22']
    fig, axes = plt.subplots(3, 1, figsize=(13, 10.5), layout='constrained',
                              gridspec_kw={'height_ratios': [1.25, 1, .8]})
    for trial, color, label in zip(trials, colors, labels):
        mask = (trial['t'] >= 0) & (trial['t'] <= 12.)
        axes[0].plot(trial['t'][mask], trial['v'][mask], color=color, lw=1.7, label=label)
        starts, _, actual, reference = segments(trial, 0., 12.)
        axes[1].step(starts, actual-reference, where='post', color=color, lw=1.2, label=label)
    for i, trial in enumerate(trials):
        mask = (trial['rt'] >= 0) & (trial['rt'] <= 12.)
        axes[0].step(trial['rt'][mask], trial['rv'][mask], where='post', color='black' if i == 0 else '#888',
                     lw=1.4 if i == 0 else 1., ls='-' if i == 0 else '--', alpha=.8,
                     label='ref A / ref B' if i == 0 else '_nolegend_')
    for ax in axes[:2]:
        for b in boundary[1:-1]: ax.axvline(b, color='#999', ls=':', lw=.8)
        ax.grid(alpha=.2)
        ax.set_xlim(0, 12.)
        ax.axhline(0, color='#777', lw=.6)
        ax.legend(fontsize=9, loc='best')
    axes[0].set_title('모터 추종성능 비교 | 동일 0→10→5 km/h 환산 명령', loc='left', weight='bold', fontsize=16)
    axes[0].set_ylabel('RPM')
    axes[1].set_ylabel('actual − ref (RPM)')
    axes[1].set_xlabel('R 이후 시간 (s) — 최종 제동 구간 제외')
    if zero_intervals:
        a, b = zero_intervals[0]
        axes[0].annotate(f'A: {a:.2f}s에서 0 RPM 기록\n약 {(b-a)*1000:.0f} ms', xy=(a, 0), xytext=(a+.5, 27),
                         arrowprops={'arrowstyle': '->', 'color': '#555'}, fontsize=9)
    x = np.arange(2)
    for i, row in enumerate(report):
        values = [-row['steady'][key]['bias_rpm'] for key in ['high', 'low']]
        bars = axes[2].bar(x+(i-.5)*.32, values, .32, color=colors[i], label=labels[i])
        axes[2].bar_label(bars, fmt='%.2f', padding=3, fontsize=10)
    axes[2].set_xticks(x, ['10 km/h 명령 유지 마지막 1초 (5~6s)', '5 km/h 명령 유지 마지막 1초 (11~12s)'])
    axes[2].set_ylabel('목표 미달 RPM\n작을수록 좋음')
    axes[2].set_ylim(0, 18)
    axes[2].grid(axis='y', alpha=.2)
    axes[2].legend(fontsize=9)
    fig.get_layout_engine().set(rect=(0, .05, 1, .94))
    fig.text(.06, .015, f"공통 0~12s RMSE: A {report[0]['overall']['rmse_rpm']:.2f} / B {report[1]['overall']['rmse_rpm']:.2f} RPM"
             f" | B 오차 {improvement['rmse_rpm']:.1f}% 감소.\n"
             '두 시험 모두 3초 램프 입력. GNSS 실측 속도가 아니며, 실제 게인·부하 동일 여부는 이 로그만으로 확인할 수 없습니다.',
             fontsize=9, color='#555')
    for ext in ['png', 'pdf']: fig.savefig(out/f'tracking_comparison.{ext}')
    plt.close(fig)

    a, b = report
    text = f'''# 모터 추종성능 비교

A: `{a['trial']}`  
B: `{b['trial']}`

두 기록의 시험 설정은 같다. 0→10 km/h 환산 3초 상승, 최소 3초 유지, 10→5로 3초 감소, 최소 3초 유지다.
STEP 시험이 아닌 램프 시험이다. 최종 제동을 제외한 공통 0~12초를 비교했다.
시각은 PC 명령 송신과 피드백 수신 기준이며, 오차는 actual−ref다.

| 지표 | A | B |
| --- | ---: | ---: |
| 전체 RMSE (RPM) | {a['overall']['rmse_rpm']:.2f} | {b['overall']['rmse_rpm']:.2f} |
| 전체 MAE (RPM) | {a['overall']['mae_rpm']:.2f} | {b['overall']['mae_rpm']:.2f} |
| 감속 구간 최대 양의 오차 (RPM) | {a['phases']['falling_ramp']['max_positive_error_rpm']:.2f} | {b['phases']['falling_ramp']['max_positive_error_rpm']:.2f} |
| 10 명령 마지막 1초 평균 RPM (목표 {targets[0]:.2f}) | {a['steady']['high']['actual_mean_rpm']:.2f} | {b['steady']['high']['actual_mean_rpm']:.2f} |
| 위 구간 목표 대비 오차 (%) | {a['steady']['high']['signed_error_pct']:.2f} | {b['steady']['high']['signed_error_pct']:.2f} |
| 5 명령 마지막 1초 평균 RPM (목표 {targets[1]:.2f}) | {a['steady']['low']['actual_mean_rpm']:.2f} | {b['steady']['low']['actual_mean_rpm']:.2f} |
| 위 구간 목표 대비 오차 (%) | {a['steady']['low']['signed_error_pct']:.2f} | {b['steady']['low']['signed_error_pct']:.2f} |
| 10 명령 유지 마지막 1초 새 표본 표준편차 (RPM) | {a['steady']['high']['fresh_sample_std_rpm']:.2f} | {b['steady']['high']['fresh_sample_std_rpm']:.2f} |
| 5 명령 유지 마지막 1초 새 표본 표준편차 (RPM) | {a['steady']['low']['fresh_sample_std_rpm']:.2f} | {b['steady']['low']['fresh_sample_std_rpm']:.2f} |

이 두 회차에서는 B의 RMSE가 {improvement['rmse_rpm']:.1f}%, MAE가 {improvement['mae_rpm']:.1f}% 작다.
두 유지 구간 모두 B가 목표에 더 가깝다. 다만 두 시험 모두 유지 끝에서 목표보다 낮으며,
마지막 1초 평균이 목표 ±5%에 들어오지 않는다. 소프트웨어 정착은 목표 오차를 요구하지 않아 정상 완료와 양립한다.
유지 끝 10 명령의 작은 변동폭은 A가 조금 작고, 5 명령은 비슷하다. 주된 차이는 목표 미달이다.

A의 약 7.69~7.73초에 `valid=True`인 0 RPM 값이 새 MCU tick 한 개로 기록됐다.
급격한 79→0→86 RPM 기록의 원인을 이 로그만으로 확정하지 않는다. 본 비교에는 그대로 포함했다.
민감도 확인으로 두 시험에서 동일한 이 시간 구간을 제외하면 RMSE는
A {a['sensitivity_excluding_zero_intervals_from_both']['rmse_rpm']:.2f}, B {b['sensitivity_excluding_zero_intervals_from_both']['rmse_rpm']:.2f} RPM으로 B가 여전히 작다.

다만 감속 과도응답은 별도 주의가 필요하다. 6~9초의 최대 양의 추종 오차는
A {a['phases']['falling_ramp']['max_positive_error_rpm']:.2f}, B {b['phases']['falling_ramp']['max_positive_error_rpm']:.2f} RPM이다.
위 0 RPM 관측 구간을 양쪽에서 동일하게 제외한 감속 구간 RMSE는
A {a['falling_sensitivity_excluding_zero_intervals_from_both']['rmse_rpm']:.2f}, B {b['falling_sensitivity_excluding_zero_intervals_from_both']['rmse_rpm']:.2f} RPM이다.
따라서 B의 전체·유지 오차 개선을 감속 중 순간 튐까지 개선됐다고 해석하지 않는다.

각 조건 1회 비교이며 실제 Kp/Ki·하중·배터리 조건은 이 설정 JSON에 없다.
`KP16160RETRY`는 기록 이름이며 실제 게인 수치를 검증한 것은 아니다.
GNSS 실측 속도나 통제된 게인 변경의 인과 효과로 해석하지 않는다.

![추종성능 비교](tracking_comparison.png)

실행: `python compare_motor_trials.py` (doc 디렉터리). 수치·입력 해시는 `metrics.json`, 구간별 표는 `metrics.csv`에 저장했다.
'''
    (out/'comparison.md').write_text(text)
    print(json.dumps(dict(improvement_pct=improvement, results=[dict(trial=r['trial'], overall=r['overall'], steady=r['steady'],
                      sensitivity=r['sensitivity_excluding_zero_intervals_from_both']) for r in report]), ensure_ascii=False, indent=2))
    print('Saved:', out)


if __name__ == '__main__':
    main()
