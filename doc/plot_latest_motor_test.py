#!/usr/bin/env python3
"""Plot the latest closed RPM trial; all outputs stay beside this file in doc/graphs.

Usage: python plot_latest_motor_test.py
Optional: --run RECORDING_DIRECTORY or --log-root DIRECTORY (repeatable).
Reads rosbag2 SQLite/CDR directly; no ROS environment or running node is required.
"""
import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import sqlite3
import struct
import sys
from zoneinfo import ZoneInfo


DOC = Path(__file__).resolve().parent
MAIN = DOC.parents[2]


class Cdr:
    def __init__(self, data):
        if data[:2] not in (b'\x00\x00', b'\x00\x01'):
            raise ValueError('Unsupported CDR representation')
        self.data, self.pos = data, 4
        self.endian = '<' if data[1] == 1 else '>'

    def number(self, fmt):
        size = struct.calcsize(fmt)
        self.pos += (-(self.pos-4)) % size
        value, = struct.unpack_from(self.endian+fmt, self.data, self.pos)
        self.pos += size
        return value

    def string(self):
        size = self.number('I')
        if size < 1 or self.pos+size > len(self.data):
            raise ValueError('Invalid CDR string length')
        value = self.data[self.pos:self.pos+size-1].decode('utf-8')
        self.pos += size
        return value


def messages(run, names):
    bags = sorted((run/'bag').glob('*.db3'))
    if not bags:
        raise ValueError('bag/*.db3 없음')
    rows = []
    for bag in bags:
        with sqlite3.connect(bag.resolve().as_uri()+'?mode=ro', uri=True) as db:
            topics = dict(db.execute('select id,name from topics'))
            ids = [i for i, name in topics.items() if name in names]
            if ids:
                sql = 'select timestamp,topic_id,data from messages where topic_id in ('
                sql += ','.join('?' for _ in ids)+') order by timestamp,id'
                rows.extend((stamp, topics[topic], data) for stamp, topic, data in db.execute(sql, ids))
    return sorted(rows, key=lambda row: row[0])


def events_in(run):
    return [json.loads(Cdr(data).string()) for _, _, data in messages(run, {'/vehicle/brake_test'})]


def load_candidate(run):
    meta = json.loads((run/'metadata.json').read_text())
    if meta.get('complete') is not True:
        raise ValueError('기록 종료/저장 완료 전')
    events = events_in(run)
    arms = [e for e in events if e.get('event') == 'armed'
            and e.get('settings', {}).get('feedback_source') == 'motor_rpm']
    if not arms:
        raise ValueError('시작된 motor_rpm 시험 없음')
    arm = max(arms, key=lambda event: event['ros_ns'])
    selected = [e for e in events if e.get('session') == arm.get('session')
                and e.get('trial') == arm['trial']]
    return run, meta, arm, selected


def select_run(args):
    if args.run:
        return load_candidate(args.run.expanduser().resolve())
    roots = args.log_root or [MAIN/'log/pi_load_20261025/runs', MAIN/'log/vehicle_recorder']
    candidates = {}
    for root in roots:
        root = root.expanduser()
        if root.exists():
            for path in root.rglob('metadata.json'):
                if path.parent.parent.name == 'bag':
                    continue
                try:
                    meta = json.loads(path.read_text())
                    created = datetime.fromisoformat(meta['created_at']).timestamp()
                    candidates[path.parent.resolve()] = created
                except (ValueError, KeyError, OSError):
                    continue
    ordered = sorted(candidates, key=lambda p: candidates[p], reverse=True)
    reasons = []
    for run in ordered:
        try:
            selected = load_candidate(run)
            for reason in reasons[:5]:
                print('새 기록 제외:', reason)
            return selected
        except (ValueError, KeyError, OSError, sqlite3.Error, struct.error) as exc:
            reasons.append(f'{run.name}: {exc}')
    raise ValueError('분석 가능한 종료된 motor_rpm 기록이 없습니다. 검색 경로: '
                     + ', '.join(str(p) for p in roots)
                     + ('; '+ '; '.join(reasons[:5]) if reasons else ''))


def command_event(data):
    cdr = Cdr(data)
    stamp = cdr.number('i')*10**9+cdr.number('I')
    cdr.string()  # header.frame_id
    topic = cdr.string()
    stage = cdr.number('B')
    cdr.number('f')  # steering_raw
    speed = cdr.number('f')
    brake = bool(cdr.number('B'))
    return stamp, topic, stage, speed, brake


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, help='특정 기록 폴더 지정; 생략하면 최근 기록 자동 선택')
    parser.add_argument('--log-root', type=Path, action='append', help='자동 검색 루트 변경; 여러 번 지정 가능')
    args = parser.parse_args()
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib import font_manager
        import numpy as np
    except ImportError:
        raise ValueError('그래프 패키지가 없습니다. 같은 Python으로 설치: python -m pip install matplotlib numpy')

    run, meta, arm, events = select_run(args)
    settings = arm['settings']
    origin = arm.get('command_ns') or arm['ros_ns']
    sign = settings.get('motor_feedback_sign', -1.)
    factor = math.pi*settings.get('wheel_diameter_m', .47)*3.6/60
    scale = settings.get('motor_full_scale_rpm', 891.387)
    results = [e for e in events if e.get('event') == 'result']
    # A later Q can produce a second, explicit-brake result. Prefer the normal
    # profile's measured-stop result for a completed reference experiment.
    selected_result = next((e for e in results if e.get('reason') == 'speed_transition_complete'),
                           results[-1] if results else None)
    end = (selected_result['ros_ns'] if selected_result else max(e['ros_ns'] for e in events)) + 800_000_000
    start = origin-600_000_000
    motors, commands = [], []
    for stamp, name, data in messages(run, {'/motor_feedback', '/vehicle/command_events'}):
        if name == '/motor_feedback' and start <= stamp <= end:
            cdr = Cdr(data)
            motors.append((stamp, cdr.number('i'), cdr.number('I'), bool(cdr.number('B'))))
        elif name == '/vehicle/command_events':
            ns, topic, stage, raw, brake = command_event(data)
            if topic == arm.get('command_topic', '/cmd') and start <= ns <= end:
                commands.append((ns, stage, raw*scale, brake))
    if not motors or not any(row[3] for row in motors):
        raise ValueError(f'유효 모터 피드백 없음: {run}')
    if not math.isfinite(factor) or factor <= 0 or sign not in (-1., 1.):
        raise ValueError('기록의 바퀴 지름 또는 피드백 부호 설정이 유효하지 않습니다')

    # Prefer PC transmission events, then command-publication events, then JSON status.
    stage = 2 if any(c[1] == 2 for c in commands) else 1
    refs = [(stamp, rpm, brake) for stamp, source, rpm, brake in commands if source == stage]
    reference_source = 'PC TRANSMITTED' if stage == 2 else 'COMMAND publication'
    if not refs:
        refs = [(e['ros_ns'], e['command_rpm'], e.get('state') in ('BRAKING', 'COMPLETE'))
                for e in events if e.get('command_rpm') is not None]
        reference_source = '시험 JSON (약 5 Hz 상태값 포함)'
        print('주의: command_events가 없어 JSON 명령을 사용합니다. 빠른 응답의 명령 시각 정밀도가 낮습니다.')
    refs.sort()
    if not refs:
        raise ValueError('명령 RPM 기록 없음')
    # Use the last held value at the left boundary; otherwise the pre-arm command was zero.
    if refs[0][0] > start:
        refs.insert(0, (start, 0., True))
    refs.append((end, refs[-1][1], refs[-1][2]))
    t = np.array([(stamp-origin)/1e9 for stamp, _, _, _ in motors])
    raw = np.array([rpm for _, rpm, _, _ in motors], dtype=float)
    valid = np.array([valid for _, _, _, valid in motors])
    actual = np.where(valid, raw*sign, np.nan)
    rt = np.array([(stamp-origin)/1e9 for stamp, _, _ in refs])
    rv = np.array([rpm for _, rpm, _ in refs])
    idx = np.clip(np.searchsorted(rt, t, side='right')-1, 0, len(rv)-1)
    error = actual-rv[idx]
    out = DOC/'graphs'/f'{run.name}-trial{arm["trial"]}'
    out.mkdir(parents=True, exist_ok=True)

    installed = {font.name for font in font_manager.fontManager.ttflist}
    for font in ['Noto Sans CJK KR', 'Noto Sans CJK JP', 'NanumGothic', 'DejaVu Sans']:
        if font in installed:
            plt.rcParams['font.family'] = font
            break
    plt.rcParams.update({'axes.unicode_minus': False, 'font.size': 10, 'figure.dpi': 160})
    fig, axes = plt.subplots(3, 1, figsize=(12.5, 10), sharex=True, layout='constrained')
    for ax, unit_scale, ylabel in [(axes[0], factor, 'RPM-equivalent speed (km/h)'),
                                   (axes[1], 1., 'RPM')]:
        ax.step(rt, rv*unit_scale, where='post', color='#2166ac', lw=2, label='ref')
        ax.plot(t, actual*unit_scale, color='#e87d15', lw=1.4, label=f'actual = raw × {sign:+g}')
        if sign == -1.:
            ax.plot(t, np.where(valid, raw, np.nan)*unit_scale, color='#777', lw=1,
                    ls=':', alpha=.75, label='raw feedback (before sign)')
        ax.set_ylabel(ylabel)
        ax.legend(loc='best')
    axes[2].plot(t, error, color='#b13250', lw=1.2, label='feedback − ref')
    axes[2].set_ylabel('Error (RPM)')
    axes[2].legend(loc='best')
    labels = {'speed_transition_start': 'transition start', 'speed_transition_end': 'transition end',
              'brake': 'brake', 'motor_zero': 'motor 0 / brake OFF'}
    for e in events:
        if e.get('event') in labels:
            time = ((e.get('command_ns') if e['event'] in ('brake', 'motor_zero') else None) or e['ros_ns'])-origin
            x = time/1e9
            for ax in axes:
                ax.axvline(x, color='#7e4860', ls='--', lw=.9)
            axes[0].text(x+.03, .04, labels[e['event']], rotation=90,
                         transform=axes[0].get_xaxis_transform(), fontsize=8, va='bottom')
    # Highlight only observed true status samples, without inferring a higher-rate settle history.
    statuses = [e for e in events if e.get('event') == 'status']
    settled = [e for e in statuses if e.get('settling') is True]
    if settled:
        axes[1].scatter([(e['ros_ns']-origin)/1e9 for e in settled],
                        [e['feedback_rpm'] for e in settled], s=22, color='#27904e', zorder=4,
                        label='recorded settling=True')
        axes[1].legend(loc='best')
    for ax in axes:
        ax.grid(alpha=.2)
        ax.axhline(0, color='#777', lw=.6)
        ax.axvline(0, color='#999', lw=.8, ls=':')
        ax.set_xlim((start-origin)/1e9, (end-origin)/1e9)
    local = datetime.fromtimestamp(origin/1e9, ZoneInfo('Asia/Seoul'))
    profile = (f"{settings.get('start_speed_kph', '?')} → {settings.get('end_speed_kph', '?')} km/h equivalent"
               f" | startup {settings.get('startup_time_s', '?')}s | transition {settings.get('transition_time_s', '?')}s"
               f" | hold {settings.get('hold_time_s', '?')}s")
    axes[0].set_title(f'{local:%Y-%m-%d %H:%M:%S} KST | Motor RPM trial {arm["trial"]}\n{profile}',
                      loc='left', fontsize=13, linespacing=1.6)
    axes[2].set_xlabel('Time since arm command (s), PC ROS clock')
    reason = selected_result.get('reason') if selected_result else next((e.get('reason') for e in reversed(events)
                                                            if e.get('reason')), 'unknown')
    fig.get_layout_engine().set(rect=(0, .055, 1, .94))
    fig.text(.06, .014,
             f'Not GNSS ground speed. ref source: {reference_source}. Result: {reason}.\n'
             f'Settling markers are recorded status values; target tracking error is not a settle criterion.',
             fontsize=9, color='#555')
    for extension in ['png', 'pdf']:
        fig.savefig(out/f'ref_actual.{extension}')
    plt.close(fig)

    with (out/'actual.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['time_s', 'received_ros_ns', 'motor_mcu_ms', 'valid', 'raw_rpm',
                         'feedback_rpm', 'equivalent_kph', 'held_reference_rpm', 'error_rpm'])
        for i, (stamp, rpm, tick, ok) in enumerate(motors):
            writer.writerow([t[i], stamp, tick, ok, rpm, actual[i], actual[i]*factor, rv[idx[i]], error[i]])
    with (out/'ref.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['time_s', 'ros_ns', 'command_rpm', 'equivalent_kph', 'brake'])
        writer.writerows(((stamp-origin)/1e9, stamp, rpm, rpm*factor, brake) for stamp, rpm, brake in refs)
    summary = dict(source_run=str(run), recording_created_at=meta.get('created_at'),
                   review_status=meta.get('review_status'), recording_complete=meta['complete'],
                   trial=arm['trial'], session=arm.get('session'), arm_ros_ns=origin, reference_source=reference_source,
                   reason=reason, transition_completed=any(e.get('transition_completed') is True for e in events),
                   stop_action=selected_result.get('stop_action') if selected_result else None,
                   result_present=bool(results), settings=settings, recorded_settled_statuses=len(settled),
                   max_recorded_window_samples=max((e.get('window_sample_count', 0) for e in events), default=0),
                   valid_motor_samples=int(valid.sum()), invalid_motor_samples=int((~valid).sum()),
                   selection='latest created_at among closed recordings containing an armed RPM trial; last RPM arm in that recording',
                   timing='PC ROS command header and motor reception; no MCU/ROS clock subtraction',
                   error_definition='feedback_rpm - most recent recorded command_rpm',
                   speed_basis='wheel-diameter equivalent, not GNSS ground speed')
    (out/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    (out/'events.json').write_text(json.dumps(events, ensure_ascii=False, indent=2))
    print('기록:', run)
    print(f'회차: {arm["trial"]} | 종료 사유: {reason} | feedback sign: {sign:+g}')
    print(f'기록된 settling=True 상태: {len(settled)}개')
    print('그래프:', out/'ref_actual.png')
    print('PDF:', out/'ref_actual.pdf')
    print('CSV/설정/선택 근거:', out)


if __name__ == '__main__':
    try:
        main()
    except (ValueError, KeyError, OSError, sqlite3.Error, struct.error) as exc:
        print(f'그래프 생성 실패: {exc}', file=sys.stderr)
        sys.exit(1)
