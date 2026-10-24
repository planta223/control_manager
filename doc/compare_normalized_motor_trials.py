#!/usr/bin/env python3
"""Normalize the three requested reference trials and save the comparison in doc/graphs."""
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

from compare_motor_trials import read_trial, metrics, segments

DOC = Path(__file__).resolve().parent
DEFAULTS = ['20261024T143456-40056501-15초10키로-trial1',
            '20261024T143612-bff60b23-20키로5초-trial1',
            '20261024T144141-feaa7e81-30키로5초-trial1']


def sustained_below(trial, start, threshold, hold=.2):
    """Return first observed onset held below a threshold for at least hold seconds."""
    onset = None
    for stamp, rpm in zip(trial['t'], trial['v']):
        if stamp < start:
            continue
        if abs(rpm) > threshold:
            onset = None
        elif onset is None:
            onset = stamp
        elif stamp-onset >= hold:
            return float(onset-start)
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trials', nargs='*', help='Three extracted trial folders; defaults to the requested recent accepted runs')
    args = parser.parse_args()
    names = args.trials or DEFAULTS
    if len(names) != 3:
        parser.error('Exactly three trials are required')
    trials = [read_trial(name) for name in names]
    settings = [t['summary']['settings'] for t in trials]
    for key in ['startup_time_s', 'transition_time_s', 'hold_time_s', 'end_speed_kph',
                'wheel_diameter_m', 'motor_feedback_sign', 'brake_on_complete', 'wait_for_settle']:
        assert all(s[key] == settings[0][key] for s in settings), f'Different profile/measurement setting: {key}'
    assert settings[0]['startup_time_s'] == 5. and settings[0]['hold_time_s'] == 0.
    assert settings[0]['transition_time_s'] == 0. and settings[0]['end_speed_kph'] == 0.
    assert settings[0]['wait_for_settle'] is False and settings[0]['brake_on_complete'] is False
    out = DOC/'graphs/normalized-40056501-bff60b23-feaa7e81'
    out.mkdir(parents=True, exist_ok=True)
    summaries = []
    for i, trial in enumerate(trials):
        s = trial['summary']['settings']
        factor = math.pi*s['wheel_diameter_m']*3.6/60
        target = s['start_speed_kph']/factor
        zero = next(e for e in trial['events'] if e['event'] == 'motor_zero')
        zero_ns = zero['command_ns']
        zero_s = (zero_ns-trial['summary']['arm_ros_ns'])/1e9
        actual_before_zero = float(zero['feedback_rpm'])
        assert actual_before_zero > 0
        normal_result = next(e for e in trial['events'] if e['event'] == 'result'
                             and e.get('reason') == 'speed_transition_complete')
        assert normal_result['stop_action'] == 'motor_zero'
        # Reject any brake command before the normally measured motor-zero stop.
        assert not any(e['event'] == 'brake' and e['ros_ns'] < normal_result['ros_ns'] for e in trial['events'])
        assert abs(max(trial['rv'])/target-1) < .01
        accel = metrics(trial, 0., 5.)
        ramp = (trial['t'] >= 0) & (trial['t'] < 5.)
        row = dict(label=chr(65+i), trial=trial['path'].name, configured_target_kph=s['start_speed_kph'],
                   target_rpm=target, max_transmitted_ref_rpm=float(max(trial['rv'])),
                   ramp_rmse_pct=100*accel['rmse_rpm']/target,
                   ramp_mae_pct=100*accel['mae_rpm']/target,
                   ramp_bias_pct=100*accel['bias_rpm']/target,
                   actual_at_zero_rpm=actual_before_zero,
                   reached_target_pct=100*actual_before_zero/target,
                   ramp_peak_pct=100*float(max(trial['v'][ramp]))/target,
                   zero_command_s=zero_s,
                   below_10pct_of_pre_zero_rpm_s=sustained_below(trial, zero_s, actual_before_zero*.1),
                   measured_stop_confirmation_s=(normal_result['confirmed_at_ns']-zero_ns)/1e9,
                   zero_before_stop_s=(normal_result['stopped_since_ns']-zero_ns)/1e9,
                   zero_values_during_ramp_after_half_second=[],
                   reference_source=trial['summary']['reference_source'], stop_action=normal_result['stop_action'])
        z = np.where((trial['t'] >= .5) & (trial['t'] < 5.) & (trial['v'] == 0))[0]
        for group in np.split(z, np.where(np.diff(z) != 1)[0]+1):
            if len(group):
                row['zero_values_during_ramp_after_half_second'].append(
                    [float(trial['t'][group[0]]), float(min(5., trial['t'][group[-1]+1]))])
        summaries.append(row)
        trial['normalized'] = row
        with (out/f'{row["label"]}-normalized.csv').open('w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['time_since_R_s', 'time_since_motor_zero_s', 'actual_rpm',
                             'actual_pct_of_target', 'ref_rpm', 'ref_pct_of_target', 'error_pct_of_target',
                             'actual_pct_of_pre_zero_rpm_after_motor_zero'])
            idx = np.clip(np.searchsorted(trial['rt'], trial['t'], side='right')-1, 0, len(trial['rv'])-1)
            for t, v, r in zip(trial['t'], trial['v'], trial['rv'][idx]):
                writer.writerow([t, t-zero_s, v, 100*v/target, r, 100*r/target, 100*(v-r)/target,
                                 100*v/actual_before_zero if t >= zero_s else ''])
    artifact = dict(trials=summaries, target_normalization='100 * recorded RPM / configured first target RPM',
                    error_normalization='100 * (actual RPM - recorded transmitted reference RPM) / configured first target RPM',
                    coast_normalization='100 * actual RPM / measured feedback RPM just before motor-zero command',
                    ramp_metrics_window_s=[0., 5.], weighting='exact time-weighted zero-order holds; not an unweighted mean of repeated telemetry',
                    coast_10pct_hold_s=.2,
                    selected_records='Three latest named accepted extracted RPM recordings; rejected 143807 recording excluded',
                    warnings=['C is named 30키로5초 but its configured and transmitted target is 20 km/h equivalent',
                              'Zero-valued and abrupt telemetry samples are preserved; physical oscillation and telemetry errors are not distinguished',
                              'One trial each. Gain, load, road and battery conditions are not verified. No GNSS ground speed.',
                              'There is no constant-speed hold; the endpoint value is not a steady-state measurement'],
                    inputs_sha256={str(t['path']/f):hashlib.sha256((t['path']/f).read_bytes()).hexdigest()
                                   for t in trials for f in ['summary.json', 'actual.csv', 'ref.csv', 'events.json']})
    (out/'metrics.json').write_text(json.dumps(artifact, ensure_ascii=False, indent=2))
    with (out/'metrics.csv').open('w', newline='') as file:
        fields=[k for k in summaries[0] if k != 'zero_values_during_ramp_after_half_second']
        writer=csv.DictWriter(file, fields);writer.writeheader()
        writer.writerows({k:r[k] for k in fields} for r in summaries)

    installed = {font.name for font in font_manager.fontManager.ttflist}
    for font in ['Noto Sans CJK KR', 'Noto Sans CJK JP', 'NanumGothic']:
        if font in installed:
            plt.rcParams['font.family'] = font
            break
    plt.rcParams.update({'font.size': 10, 'axes.unicode_minus': False, 'figure.dpi': 160})
    fig, axes = plt.subplots(2, 2, figsize=(14, 10), layout='constrained')
    colors = ['#2764ac', '#e67e22', '#278354']
    labels = ['A: 10 명령 (40056501)', 'B: 20 명령 (bff60b23)', 'C: 20 명령 (파일명 30, feaa7e81)']
    for trial, color, label in zip(trials, colors, labels):
        row=trial['normalized']; target=row['target_rpm']
        mask=trial['t'] >= 0
        axes[0, 0].plot(trial['t'][mask], 100*trial['v'][mask]/target, color=color, lw=1.2, label=label)
        mask=(trial['t'] >= 0) & (trial['t'] <= 5.)
        axes[0, 1].plot(trial['t'][mask], 100*trial['v'][mask]/target, color=color, lw=1.2, label=label)
        t, _, v, r=segments(trial,0.,5.)
        axes[1, 0].step(t,100*(v-r)/target,where='post',color=color,lw=1.05,label=label)
        mask=trial['t'] >= row['zero_command_s']
        axes[1, 1].plot(trial['t'][mask]-row['zero_command_s'],
                        100*trial['v'][mask]/row['actual_at_zero_rpm'],color=color,lw=1.5,label=label)
    end=max(t['t'][-1] for t in trials)
    axes[0, 0].plot([0,5,5,end],[0,100,0,0],'k--',lw=1.4,label='명목 ref: 5초 상승 후 0')
    axes[0, 1].plot([0,5],[0,100],'k--',lw=1.4,label='ref')
    axes[0, 0].set(title='전체 응답 — 각 목표 명령 = 100%', xlabel='R 이후 시간 (s)', ylabel='목표 대비 RPM (%)', xlim=(0,end))
    axes[0, 1].set(title='상승 구간 확대 — 원본 값 유지', xlabel='R 이후 시간 (s)', ylabel='목표 대비 RPM (%)', xlim=(0,5))
    axes[1, 0].set(title='상승 중 정규화 추종 오차', xlabel='R 이후 시간 (s)', ylabel='(actual − ref) / 목표 RPM (%)', xlim=(0,5))
    axes[1, 1].set(title='모터 0 이후 — 직전 실제 RPM = 100%', xlabel='모터 0 명령 이후 시간 (s)', ylabel='0 명령 직전 RPM 대비 (%)', xlim=(0,12.5))
    axes[1, 1].axhline(10,color='#777',ls=':',lw=1)
    for ax in axes.flat:
        ax.grid(alpha=.2)
        ax.axhline(0,color='#777',lw=.6)
        ax.legend(loc='best',fontsize=8)
    fig.suptitle('최근 3회 정규화 비교 | 실제 명령: 10 / 20 / 20 km/h 환산',fontsize=16,weight='bold')
    fig.get_layout_engine().set(rect=(0,.06,1,.94))
    fig.text(.05,.015,'목표 RPM으로 정규화했으며 실제 피드백의 최대값으로 맞추지 않았습니다. 초반 0값·급격한 튐을 제거하지 않았습니다.\n'
             '세 시험 모두 5초 램프 → 0 RPM STEP, 브레이크 OFF입니다. C의 기록 이름은 30이지만 실제 명령은 20입니다.',fontsize=10,color='#555')
    for ext in ['png','pdf']:fig.savefig(out/f'normalized_comparison.{ext}')
    plt.close(fig)
    lines=['# 최근 3개 모터 로그 정규화 비교','',
           '각 첫 목표 RPM을 100%로 하고 R 시각을 맞췄다. 실제 최대값으로 정규화하지 않아 목표 미달과 튐이 그대로 드러난다.',
           '모터 0 이후 곡선은 별도로 0 명령 직전 측정 RPM을 100%로 맞췄다.',
           '**C는 파일명에 30이 들어 있지만 설정과 송신 명령은 20 km/h 환산이다.**',
           '셋 다 5초 상승 램프, 끝 목표 0, 전환시간 0, 유지시간 0, 정상 완료 브레이크 OFF다.', '',
           '| 기록 | 실제 목표 (RPM) | 상승 정규화 RMSE | 0 명령 직전 도달률 | 상승 중 최대 기록값/목표 | 0 후 10% 이하¹ | 정지 확인² |',
           '| --- | ---: | ---: | ---: | ---: | ---: | ---: |']
    for row in summaries:
        lines.append(f"| {row['label']} ({row['configured_target_kph']:g} 명령) | {row['target_rpm']:.2f} | {row['ramp_rmse_pct']:.2f}% | {row['reached_target_pct']:.1f}% | {row['ramp_peak_pct']:.1f}% | {row['below_10pct_of_pre_zero_rpm_s']:.2f}s | {row['measured_stop_confirmation_s']:.2f}s |")
    lines+=['','¹ 0 명령 직전 측정값의 10% 이하로 0.2초 연속 유지한 최초 구간 시작. PC 수신시각 기준이다.',
            '² RPM 절댓값 ≤1이 0.5초 지속되어 확인된 시각. 모두 모터 0 명령 이후 시간이며 기계식 제동은 제외했다.', '',
            '상승 중 오차는 C와 B가 A보다 작지만, 끝점 도달률은 A > B > C다. 따라서 한 지표로 전체 우열을 정하지 않는다.',
            'B에는 목표 대비 약 160%의 큰 순간 기록이 있으며, 세 기록 모두 초반 0값·급격한 상하 변화가 반복된다.',
            '원본을 그대로 포함한 지표다. 실제 제어 진동인지 피드백 측정·전송 문제인지 이 로그만으로 확정할 수 없다.',
            '유지 구간이 없어 끝점 값을 정상상태 오차로 부르지 않는다. 부하·노면·배터리·게인 동일 여부도 별도 확인이 필요하다.', '',
            '## 사용한 기록','']
    lines += [f"- {row['label']}: `{row['trial']}`" for row in summaries]
    lines += ['', '![정규화 비교](normalized_comparison.png)', '',
              '실행: doc에서 `python compare_normalized_motor_trials.py`. PNG/PDF, 정규화 CSV, 지표와 입력 해시는 같은 폴더에 저장한다.']
    (out/'comparison.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(summaries,ensure_ascii=False,indent=2))
    print('Saved:',out)


if __name__=='__main__':
    main()
