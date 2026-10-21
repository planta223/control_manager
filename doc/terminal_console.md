# 터미널 운용 메뉴

호스트의 프로젝트 루트에서 기존 main 컨테이너를 사용한다.

```bash
bash main/IF_YOU_WANT_TO_RUN/control_console.sh
```

첫 화면에서 1=test, 2=brake_inspection, 3=preliminary_lap, 4=final_lap을 선택한다.
Enter 없이 숫자 키로 선택한다. 모드 선택을 생략하려면 다음을 사용한다.

```bash
bash main/IF_YOU_WANT_TO_RUN/control_console.sh --mode preliminary_lap
```

main 컨테이너 내부에서는 `ros2 run control_manager control_console`로 직접 실행한다.
호스트 진입 스크립트는 Docker 이미지 ID와 컨테이너 ID도 기록하도록 전달한다.
이미지 ID를 환경에서 제공하지 않은 직접 실행은 metadata의 docker_image_id가 null이다.

## 조작

- 1 또는 r: 출발/재개. 현재 시각을 포함한 PauseRequest(pause=false)를 발행한다.
- 2 또는 p: 일시정지. PauseRequest(pause=true)를 발행한다.
- 3 또는 q, Ctrl+C, 입력 EOF: 일시정지 후 종료 절차를 시작한다.
- 모드 변경은 현재 운용을 종료한 후 다른 모드로 다시 실행한다.

시작 시 사용한 모드 설정의 사본에 start_paused=true를 적용한다. 원본 YAML은 수정하지 않는다.
AUTO 운용만 지원하며 외부 센서와 trans 송신기는 자동 실행하지 않는다.
화면은 ControlStatus 응답을 기준으로 갱신한다. 재개 수락은 실제 출발을 뜻하지 않는다.
진행 조건이 충족되지 않으면 제어 코드는 정지 명령을 유지한다.
지도 없는 test/brake_inspection은 기존 GPS 경로 의존성을 그대로 가진다.

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
