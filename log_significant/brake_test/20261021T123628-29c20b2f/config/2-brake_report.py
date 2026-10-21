"""고규범 신규 로직 (2026-10-21): offline recorder journal -> brake trial CSV."""
import argparse
import csv
import json
from pathlib import Path
import sys


def rows(paths):
    trials = {}
    for path in paths:
        with Path(path).open() as stream:
            for line in stream:
                try:
                    item = json.loads(line)
                    if item.get('topic') != '/vehicle/brake_test':
                        continue
                    event = json.loads(item['data']['data'])
                    if event['event'] not in ('armed', 'brake', 'result'):
                        continue
                    key = (event['session'], event['trial'])
                    row = trials.setdefault(key, dict(session=key[0], trial=key[1], completed=False))
                    row.update({k: v for k, v in event.items() if k not in ('schema', 'event', 'ros_ns', 'settings')})
                    row['completed'] |= event['event'] == 'result'
                except (ValueError, KeyError):
                    continue
    for row in trials.values():
        row['target_reached'] = row.get('reason') == 'target_reached'
        start, end = row.get('command_ns', 0), row.get('stopped_since_ns', 0)
        row['command_to_stop_s'] = (end-start)/1e9 if start and end and end >= start else None
        yield row


def main():
    parser = argparse.ArgumentParser(description='vehicle_recorder messages.jsonl 제동 시험 CSV 요약')
    parser.add_argument('journals', nargs='+')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    data = list(rows(args.journals))
    if not data:
        parser.error('제동 시험 이벤트가 없습니다. 기록 토픽과 기록 시작 시점을 확인하십시오.')
    columns = list(dict.fromkeys(k for row in data for k in row))
    with Path(args.output).open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(data)
    print(f'{len(data)}개 시험 결과 저장: {args.output}')
    for filename in args.journals:
        metadata = Path(filename).parent/'metadata.json'
        if not metadata.exists() or not json.loads(metadata.read_text()).get('complete'):
            print(f'기록 완전성 미확인: {filename}', file=sys.stderr)


if __name__ == '__main__':
    main()
