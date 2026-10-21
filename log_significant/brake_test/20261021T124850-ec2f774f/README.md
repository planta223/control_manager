# 20km/h 주행 및 제동 시험

2026-10-21 사용자 지정: 모터 출력 상한을 0.5에서 1.0으로 변경한 뒤 수행한 20km/h 시험.
앞선 목표속도 미달 시험(`20261021T123941-0815ffe0`)과 구분한다.

## 기록 구간 및 저장 검증

- 폴더명은 UTC 기준: `20261021T124850-ec2f774f`.
- 메시지 수신 구간: 2026-10-21 21:48:50.661 ~ 21:50:06.096 KST.
- `metadata.json`: `complete=true`, `written=16828`, `dropped=0`, `pending=0`, `errors=[]`.
- `messages.jsonl` 16,828건 모두 JSON 파싱 성공; metadata 저장 건수와 일치.
- 토픽별 CSV 5개 모두 JSONL의 해당 토픽 건수와 일치.
- 설정·코드 사본은 `config/`에 보존되어 있으며 snapshot 오류는 없다.
- 기록기 저장 완료와 제동시험 최종 결과 완료는 별개다. 아래 `result` 누락 사항을 확인한다.

| 토픽 | 기록 건수 | 평균 수신 주파수 |
|---|---:|---:|
| `/gnss/odom` | 1,509 | 약 20.00Hz |
| `/steering_feedback` | 3,777 | 약 50.07Hz |
| `/vehicle/stop_status` | 3,005 | 약 40.11Hz |
| `/vehicle/command_events` | 8,333 | 약 200.09Hz (명령 발행·전송 관측 합계) |
| `/vehicle/brake_test` | 204 | 약 4.90Hz (상태·이벤트 합계) |

## 주행 관측

- 세션: `af015418-c0e1-4518-bade-b73de9042625`, trial 1.
- 목표속도: 20km/h.
- 시험 시작: 21:49:29.615 KST.
- 목표 도달 제동 명령: 21:49:41.856 KST (시작 후 약 12.24초).
- 제동 사유: `target_reached`.
- 제동을 유발한 GNSS 관측 속도: 약 20.213km/h.
- 전체 기록의 GNSS 최고 속도: 약 21.185km/h.
- 실제 상태 기록의 `max_throttle=1.0`, 출력 최고값 1.0 확인.
- 마지막 상태의 모터 출력은 0, 상태는 `BRAKING`, 사유는 `target_reached`.

## 정지 관측 및 결과 누락

- 마지막 `/vehicle/stop_status`: `stopped=true`, `speed_valid=true`, `command_active=true`, `command_fresh=true`.
- 정지 구간 시작: 21:49:44.366 KST, 정지 확인: 21:49:44.906 KST.
- 정지 관측기의 `distance_valid=true`, 송신 시점부터 정지 위치까지의 직선거리 약 6.692m.
  이는 정지 관측기의 참고값이며 실제 브레이크 작동 시점부터의 물리 제동거리를 뜻하지 않는다.
- `/vehicle/brake_test`에는 `armed`와 `brake` 이벤트만 있고 최종 `result` 이벤트가 없다.
- 제동 이벤트의 `command_ns=1792586981855844494`와 정지 관측기의
  `command_stamp=1792586981865495240`이 약 9.651ms 다르다.
  현재 시험 노드는 두 시각이 정확히 일치해야 결과를 발행하도록 구현되어 있다.
- 따라서 `brake_report`에서 이 시험은 `target_reached=true`이지만 `completed=false`로 처리된다.
  주행·자동 제동·정지 관측 데이터는 보존됐으나, 완료된 제동거리 결과로 확정하지 않는다.

기존 JSONL·CSV·metadata를 읽어서 확인했으며 원본 기록은 변경하지 않았다.
