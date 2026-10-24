# 터미널 운용 메뉴

기존 main 컨테이너 안에서 ROS 환경과 빌드 결과를 source한 뒤 실행한다.

```bash
ros2 run control_manager control_console
```

첫 화면에서 1=test, 2=brake_inspection, 3=preliminary_lap, 4=final_lap을 선택한다.
Enter 없이 숫자 키로 선택한다. 모드 선택을 생략하려면 다음을 사용한다.

```bash
ros2 run control_manager control_console --mode preliminary_lap
```

호스트에서 접속할 때 이미지 ID와 컨테이너 ID도 함께 전달할 수 있다.

```bash
docker exec -it \
  -e CONTROL_IMAGE_ID="$(docker inspect --format '{{.Image}}' main)" \
  -e CONTROL_CONTAINER_ID="$(docker inspect --format '{{.Id}}' main)" \
  main bash
```

이미지 ID를 환경에서 제공하지 않은 직접 실행은 metadata의 docker_image_id가 null이다.

## 조작

- 0 또는 c: 모터 명령 0·브레이크 OFF를 유지한다. 경로 추종 조향과 진행 위치 갱신은 계속한다.
- 1 또는 r: 출발/재개. 현재 시각을 포함한 PauseRequest(pause=false)를 발행한다.
- 2 또는 p: 일시정지. PauseRequest(pause=true)를 발행한다.
- 3 또는 q, Ctrl+C, 입력 EOF: 일시정지 후 종료 절차를 시작한다.
- 모드 변경은 현재 운용을 종료한 후 다른 모드로 다시 실행한다.

모터 0에서 1/R을 누르면 구동을 재개하고, 2/P를 누르면 기존 모터 0·브레이크 ON으로 전환한다.
3/Q·Ctrl+C·EOF 종료도 기존 제동 및 정지 확인 절차를 사용한다. 센서 무효·명령 만료·경로 이탈·종점
보호는 모터 0 요청보다 우선한다. 모터 0은 PC의 RPM 명령이 0이라는 뜻이며 모터 보드의 enable을
끄는 명령은 아니다. 실제 감속이나 회생제동 동작은 모터 제어기에서 확인해야 한다.
PauseRequest.motor_zero와 ControlStatus.motor_zero 필드를 추가했으므로 control_manager를 재빌드하고
관련 제어 노드를 새로 실행한다. 기존 PauseRequest(pause=true/false)는 motor_zero=false가 기본값이다.

시작 시 사용한 모드 설정의 사본에 start_paused=true를 적용한다. 원본 YAML은 수정하지 않는다.
AUTO 운용만 지원하며 외부 센서와 trans 송신기는 자동 실행하지 않는다.
`--steering-zero-deg`에는 조향 정렬 시험에서 직진한 `--steering-deg` 값을 같은 부호로 지정한다.
기본값은 0°이며 양수는 좌회전이다. +1.2°에서 직진했다면 `--steering-zero-deg 1.2`로 실행한다.
Vehicle Interface가 자율주행의 논리 조향각에 이 값을 더한 뒤 raw 명령으로 변환한다.
논리 0°는 보드 +1.2°, 논리 +3°는 보드 +4.2°가 된다. 별도 trans 보정은 적용하지 않는다.
출발 전 정지·초기 직진·제동 중 유지 각도에도 같은 변환을 적용하며, 수동 정렬/제동시험에는 적용하지 않는다.
차량 조향 한계와 trans full scale 안쪽의 유한값만 허용한다. 보정 후 송신 범위를 넘지 않도록
실행 사본의 차량 조향 한계를 줄여 제어·변속 곡률 검증에 함께 반영한다. 원본 vehicle.yaml은 유지한다.
보정값과 적용 조향 한계는 실행별 operation.yaml/metadata.json에 남고, actuator_raw는 보정된 값이다.
보정 지원이 없는 이전 Vehicle Interface에서 0이 아닌 값을 사용하면 출발 전에 오류로 종료한다.
정지 후 저장 heading으로 재개할 때의 한계는 `--stopped-heading-max-distance-m`과
`--stopped-heading-max-reacquire-time-s`로 실행별 지정할 수 있다. 첫 값은 정지 위치에서의 변위(m),
두 번째 값은 실측 이동이 시작된 후의 대기시간(s)이며 정지 대기시간은 포함하지 않는다.
거리는 유한한 0 이상의 값이며 0이면 거리 제한만 해제한다. 시간은 유한한 양수만 허용하며,
거리 제한을 해제해도 계속 적용된다. 생략하면 YAML의 기존 값을 유지하며 현재 control.yaml은 1m·3초다.
값은 실행별 설정 사본과 metadata에 남고 원본 YAML은 바꾸지 않는다. 적용에는 콘솔을 새로 실행해야 한다.
화면은 ControlStatus 응답을 기준으로 갱신한다. 재개 수락은 실제 출발을 뜻하지 않는다.
진행 조건이 충족되지 않으면 제어 코드는 정지 명령을 유지한다.
test의 기본 지도는 MAP_test.json이다. 지도 없는 모드는 기존 GPS 경로 의존성을 그대로 가진다.

목표속도를 바꿔 시험하려면 `--cruise-speed-kph`를 지정한다. 원본 mission/mode YAML 대신
해당 실행의 operation.yaml 사본에 적용하며 실제 Motion Planning 설정값을 화면과 metadata에 남긴다.
생략하면 기존 YAML의 목표속도를 사용한다. 현재 공통 mission.yaml에는 20km/h가 있으므로,
속도가 미정인 시험은 이 값을 임의로 채택하지 말고 정한 속도를 명시한다.

```bash
# TEST_SPEED_KPH는 이번 시험에서 정한 양수 km/h 값. 미지정 시 실행하지 않음.
: "${TEST_SPEED_KPH:?시험 목표속도를 먼저 지정하세요}"
ros2 run control_manager control_console --mode test --cruise-speed-kph "$TEST_SPEED_KPH"
```

변속 옵션을 생략하면 이 값은 고정 목표속도다. `--variable-speed`를 사용하면 속도 상한이다.

`--max-accel-kphps`는 AUTO 속도 명령의 증가율을 (km/h)/s로 제한한다. 출발·재개와
주행 중 목표속도 증가에 모두 적용한다. 감소는 `--max-decel-kphps`로 제한할 수 있으며,
생략하면 기존처럼 즉시 감소한다. 정지·제동·무효 입력은 두 제한을 우회해 즉시 적용한다.
0에서 출발하고 매 제어 주기에 `min(목표속도, 이전 명령 + 상한 × dt)`로 증가한다.
정지/무효/입력 만료 시 이력을 초기화하며, timer 지연을 한 번에 따라잡지 않도록 dt는 최대 0.1초로 계산한다.
CLI 값은 내부 ROS 파라미터 max_accel_mps2에 /3.6하여 적용한다. 2.0 (km/h)/s는 약 0.5556m/s²이며,
2km/h 명령에서 1초 뒤 4km/h 명령이 되는 증가율이다. km/s²와는 다른 단위다.
이는 명령 제한이며 실제 가속도는 GNSS로 검증해야 한다. 수동 조작에는 적용하지 않는다.
생략하면 vehicle.yaml의 vehicle.limits 값을 사용한다. 현재 가속도 기본값은 10.0(km/h)/s,
감속도는 0.0(미정)이다. 양수로 설정한 차량 공통 상한보다 큰 실행 옵션은 거부한다. CLI에는 양수만 지정한다.

```bash
# 5km/h·2(km/h)/s는 사용법 예시이며 시험 확정값이 아니다.
ros2 run control_manager control_console --mode test \
  --cruise-speed-kph 5.0 --max-accel-kphps 2.0
```

일시정지로 시작하며 출발 후 지정한 상한으로 속도 명령을 올린다. 초기 heading 확보
직진 구간에도 같은 제한이 적용된다. 상한과 목표속도는 화면과 metadata에 기록한다.

## 지도 곡률 기반 자동 변속

```bash
# 아래 숫자는 사용법 예시이며 실측 차량 성능/확정 시험값이 아니다.
ros2 run control_manager control_console --mode test --variable-speed \
  --cruise-speed-kph 20 --max-accel-kphps 5 --max-decel-kphps 5 \
  --planned-decel-kphps 3 \
  --max-lateral-accel-mps2 1.5
```

최고속도 옵션 생략 시 mission.yaml의 공통 운용속도를 사용한다.
가속도·감속도·횡가속도 옵션 생략 시 vehicle.yaml의 vehicle.limits를 사용한다.
횡가속도 기본값은 max_lateral_accel_g=0.20이며 내부에서는 9.80665를 곱해 1.96133m/s²로 변환한다.
변속은 적용할 세 가속도 값이 모두 양수여야 한다. 현재 감속도가 미정(0)이므로
--max-decel-kphps를 지정하거나 vehicle.yaml에 양수 감속도를 설정해야 한다.
종방향 가속/감속 CLI 단위는 (km/h)/s, 횡가속도 CLI 단위는 m/s²다.
`--planned-decel-kphps`는 곡선·종점 감속거리 계산에 사용할 감속도 가정값이며,
`--max-decel-kphps`의 명령 감소율과 별도로 지정할 수 있다. 양수이며 적용 명령 감소율 이하여야 한다.
작을수록 낮은 미래 속도를 더 먼 앞쪽 경로에 전파해 일찍 감속하도록 계획한다.
생략하면 기존 동작을 유지한다. `--max-decel-kphps`가 지정됐으면 그 값을 계획에도 사용하고,
그마저 생략하면 기존 YAML 계획 설정 또는 차량 감속도를 사용한다. `--variable-speed`와 함께 사용한다.
설정 예시의 3(km/h)/s는 실측 성능이 아니며, 실제 일반 감속 성능보다 과대하게 가정하지 않는다.
변속 활성 여부와 모든 적용값을 실행별 설정 사본 및 metadata에 남긴다.
화면에는 속도 상한, 현재 경로의 계획 목표속도, 제한 적용 후 속도 명령, GNSS 실측 속도를 구분한다.

Motion Planning이 후륜축 기준 정적 경로에서 전후 1m 간격의 세 점으로 곡률을 계산하고,
`v_curve = sqrt(max_lateral_accel / abs(curvature))`를 최고속도 이하로 제한한다.
미래 곡선과 종점의 낮은 속도를 `v_before² <= v_after² + 2*b*distance`로 앞쪽 경로에 전파한다.
mission.yaml의 `speed_profile.response_time_s=0.5`는 미측정 지연 가정으로,
미래의 더 낮은 속도까지 감속하기 전 반응거리를 추가 확보하는 데 사용한다.
이 속도 계획 거리는 조향 PP의 2~6m lookahead와 독립이다. PP는 계속 GNSS 실측 속도를 사용한다.
`--max-accel-kphps`는 Motion Planning의 거리 기반 전방 가속 제약과 시간별 목표속도 증가율에도 적용한다.
`--max-decel-kphps`는 계획 목표속도 감소율과 최종 명령 감소율에 함께 적용하며,
`--planned-decel-kphps`는 기존처럼 곡선·종점 전 감속거리 가정값이다.
출발/재개 및 입력 오류 복구 때 GNSS 실측 속도로 계획 기준을 초기화하고, 이후에는 이전 계획에서
가감속 제한을 지키며 진행한다. 센서 응답이 늦다고 매번 상승을 처음부터 시작하지 않는다.
일시정지/비활성 상태에서는 목표속도 0으로 유지하고 계획 기준을 초기화한다.
GNSS 속도가 경로의 속도 상한을 넘으면 `MEASURED_OVERSPEED`를 표시하고 추가 가속을 억제한다.
현재 속도에서 즉시 낮은 상한에 도달할 수 없는 경우 `PLANNED_DECELERATION_LIMIT`를 표시하며
일반 감속 제한으로 낮춘다. 이미 발생한 과속의 해소나 물리 제한 준수는 보장하지 않는다.
정지·입력 오류는 즉시 우선하며, GNSS 속도 PID와 저크 제한은 포함하지 않는다.
설정된 최대 조향각으로 불가능한 경로 곡률은 저속으로 대체하지 않고 정지 처리한다.

진행 위치는 GlobalPose를 사용하며 예선 모드에서는 Motion Control과 같은
route_progress 프로필을 실행 사본에 복사해 교차 구간의 다른 가지로 점프하지 않도록 한다.
경로 내용/점 수를 바꾼 경우 노드를 재시작하고 프로필도 다시 확인한다.
GNSS 위치/속도 또는 경로가 무효·만료되면 정지하고, 활성화된 상위 미션/장애물 정지도 즉시 우선 적용한다.

감속도와 횡가속도 상한, 지연은 실측 보장값이 아니다. 현재 종방향 출력은 실제 RPM 환산이며
GNSS 속도오차를 되먹임하는 PI/PID는 포함하지 않는다. 일반 감속은 목표 RPM을 줄이는 방식이다.
실제 감속과 곡선 속도 준수, 물리 종점 정지 위치는 GNSS 실차 시험으로 확인해야 한다.

## 종료 확인

종료 요청 이후 시각의 최신 ControlStatus.paused=true와 유효한 selected_cmd의
속도 0 / brake=true를 확인한다. dry_run=true이면 이 정지 명령 확인 후 launch를 종료한다.
<!-- 실제 출력 모드에서는 추가로 유효하고 최신인 MotionState 속도가 기본 0.1 m/s 이하로
0.5초 유지되는지 확인한다. 확인할 수 없으면 메뉴와 제어 노드를 유지하고 종료 대기를 표시한다. -->
실제 출력 모드에서는 공용 vehicle_recorder의 `/control/stop_status`를 사용한다.
서로 다른 유효 GNSS 샘플의 `|v| <= 0.1 m/s`가 0.5초 지속되어야 정지를 확정하고,
시작/확정 샘플 시각을 각각 보존한다. 센서가 무효·끊김이면 확인하지 않는다.
종료 요청 후 새 샘플이 있어야 하며, 이미 연속 정지 중이었다면 기존 구간 시작 시각을 유지한다.
확인할 수 없으면 메뉴와 제어 노드를 유지하고 종료 대기를 표시한다.
`--stop-speed-mps`, `--stop-hold-s`는 이 소프트웨어 종료 확인 기준이며 물리 제동 성능 설정이 아니다.
정지 명령 수락과 실측 차량 정지는 구분해서 표시한다.

명령 요청은 짧은 간격으로 최신 timestamp를 넣어 재시도하지만, 수락 이전에 화면 상태를
임의로 변경하지 않는다. 같은 ROS 도메인에서 이미 실행 중인 control_manager와 중복 시작하지 않는다.

## 로그 보존

기본 경로는 `/workspace/log/control_console/<실행시각>-<모드>-<구분자>/`이다.
main의 bind mount에 의해 호스트의 `main/log/control_console/`에 보존된다.

- launch.log: launch와 하위 노드의 stdout/stderr 전체. 주기 로그·경고·오류 포함.
- ros/: 해당 launch의 ROS 로그 디렉터리.
- ros_console/: 터미널 메뉴 노드 자체의 ROS 로그 디렉터리.
- events.jsonl: 조작 요청·응답·상태 변경·종료 확인 기록.
- metadata.json: 실행 명령, Git commit, 이미지/컨테이너 ID, PID, 출력 모드, 종료 결과.
- config/: 실제 사용한 YAML 사본과 지도(사용 시).
- source-changes.patch, control_console.py: 미커밋 변경과 메뉴 코드 사본.
- console-error.log: 메뉴 자체의 예외가 발생했을 때의 상세 기록.
- 주행 시계열은 별도 `vehicle_recorder`의 기록 폴더에 저장한다. 정지 관측 노드는 파일을 쓰지 않는다.

ROS 로그 레벨은 낮추지 않는다. 화면 출력만 분리하며 로그를 /dev/null로 버리지 않는다.
launch 출력은 파일로 직접 연결하여 메뉴가 로그를 읽거나 화면을 갱신하는 속도와 분리한다.
로그 경로는 `--log-root`, 지도는 `--map-file`, 설정은 `--operation-file`, `--pipeline-file`,
`--mission-file`, `--control-file`, `--vehicle-file`로 지정할 수 있다.
