# control_manager

자율주행과 수동주행의 명령 선택·모드 전환을 개발하기 위한 ROS 2 패키지입니다.
현재는 **idle 노드와 빌드·설정·launch 환경만** 제공합니다.
자율주행, 수동주행, 모드 전환, 센서 구독, 제어 명령 발행, UDP 송신은 아직 구현하지 않았습니다.

## 위치와 기존 패키지와의 통일성

- 호스트 경로: `main/src/control_manager/`
- 기존 main 컨테이너 경로: `/workspace/src/control_manager/`
- 기존 `docker-compose.yml`의 `./main:/workspace` 마운트를 사용합니다. 별도 Docker 설정은 추가하지 않았습니다.
- `gnss_tools`와 동일하게 ROS 2 Humble, C++17, `ament_cmake`, `.cxx`/`.hxx`, XML launch를 사용합니다.
- 패키지 이름은 `control_manager`, 실행 파일은 `control_manager_node`, 기본 노드 이름은 `control_manager`입니다.
- 의존성은 현재 사용하는 `rclcpp`, `rcl_interfaces`와 launch 관련 패키지만 선언했습니다.

## 폴더 구조

```text
control_manager/
├── README.md
├── .gitignore
├── LICENSE
├── package.xml
├── CMakeLists.txt
├── config/
│   └── control_manager.yaml
├── include/control_manager/
│   └── control_manager_node.hxx
├── launch/
│   └── control_manager.xml
└── src/
    ├── control_manager_node.cxx
    └── main.cxx
```

| 파일 | 역할 |
|---|---|
| `package.xml` | 패키지 정보와 ROS 의존성 |
| `CMakeLists.txt` | C++17 실행 파일 빌드 및 설정·launch 설치 |
| `include/control_manager/control_manager_node.hxx` | ROS 노드 선언 |
| `src/control_manager_node.cxx` | 초기 parameter 검사와 idle 시작 로그 |
| `src/main.cxx` | ROS 초기화·spin·종료와 시작 오류 처리 |
| `config/control_manager.yaml` | 초기 `mode`, `dry_run` 설정 |
| `launch/control_manager.xml` | 이 패키지의 노드만 시작하는 launch |

## 현재 동작

| Parameter | 기본값 | 현재 계약 |
|---|---|---|
| `mode` | `idle` | `idle`만 허용 |
| `dry_run` | `true` | `true`만 허용 |

두 parameter는 시작 이후 변경할 수 없습니다.
`mode=autonomous`, `mode=manual` 또는 `dry_run=false`로 시작하면 오류 메시지와 함께 종료합니다.
이 값들을 바꾸는 것으로 주행 기능이 활성화되지는 않습니다.
실행 가능한 기반 노드일 뿐, 주행 제어가 구현된 상태가 아닙니다.

## 개발할 책임과 기존 코드의 경계

아래는 후속 개발 방향이며 현재 구현된 기능 목록이 아닙니다.

| 책임 | 후속 개발 내용 | 참고할 기존 코드 |
|---|---|---|
| 자율주행 | 경로·미션 구간에 따른 목표 조향/속도 결정 | `../gnss_tools/src/pure_pursuit.cxx`, `../gnss_tools/src/guidance_node.cxx` |
| 수동주행 | 수동 입력 수신, 유효성·최신성 검사 | `../manual_control/manual_control/` |
| 모드 관리 | idle/manual/autonomous 전환과 명령 선택 | `../gnss_tools/include/guidance_cxx/state_machine.hxx` |
| 주행 허용 판단 | 상태 추정 품질·입력 timeout·정지 요청 검사 | `../gnss_tools/src/guidance_node.cxx`의 `evaluateFollowGate()` |
| 최종 명령 전달 | 단일 송신 주체와 명령 단위·범위 확정 | `../trans/trans/`, `../gnss_tools/include/guidance_cxx/message.hxx` |

기존 경로는 상대 위치 참고용입니다. 이 패키지가 해당 구현을 복사하거나 실행하거나 런타임 의존하는 것은 아닙니다.
기존 `guidance_node`와 `trans`에는 각각 UDP 송신 경로가 있습니다.
실제 제어 연결 시에는 최종 액추에이터 명령을 보내는 주체 하나를 정하고 모드별로 명령을 선택해야 합니다.
그 통합 작업과 기존 패키지 변경은 이번 기본 환경에 포함하지 않았습니다.

## 인터페이스와 차량 parameter

새 제어 topic과 메시지 contract는 아직 정의하지 않았습니다.
다음 기존 인터페이스는 설계 참고 대상입니다.

- 상태 추정 입력: `/localization/odom` (`nav_msgs/msg/Odometry`). 현재 guidance는 `map`/`base_link` frame을 요구합니다.
- 수동 입력: 기존 `manual_control`의 `cmd` (`std_msgs/msg/Float32MultiArray`), `[steering, speed, brake]` 순서. 새 제어 명령 형식은 별도로 결정해야 합니다.
- 차선 입력: 기존 Lane AI는 overlay 이미지를 발행합니다. lane prediction을 제어에 전달하는 데이터 인터페이스는 추가 설계 대상입니다.
- 지도 입력: dashboard final-map JSON과 guidance waypoint CSV는 형식이 다릅니다. 경로와 미션 section 로더는 아직 없습니다.

기존 설정의 wheelbase `1.328 m`/`1.04 m`, GPS 전방 offset `0.559 m`/`0 m`,
guidance raw 속도 `350`과 저장된 모터 펌웨어 입력 범위 `-100~100`은 아직 통일되지 않았습니다.
이 패키지에는 해당 값을 임의로 선택해 복사하지 않았습니다.
물리 속도(m/s), raw 명령, motor RPM은 후속 인터페이스에서 구분해야 합니다.

## 빌드와 실행 방법

아래 명령은 참고용이며 패키지 생성 작업에서는 빌드하거나 실행하지 않았습니다.
ROS 2 Humble과 기존 main 이미지의 개발 도구가 준비된 셸을 기준으로 합니다.
컨테이너에서는 `/workspace`, 호스트에서는 이 저장소의 `main/`에서 시작합니다.

기존 `main/build`, `main/install`, `main/log`와 분리하려면 빌드 산출물을 새 패키지 안에 둡니다.

```bash
source /opt/ros/humble/setup.bash

colcon --log-base src/control_manager/log build \
  --base-paths src/control_manager \
  --packages-select control_manager \
  --build-base src/control_manager/build \
  --install-base src/control_manager/install \
  --symlink-install

source src/control_manager/install/setup.bash
ros2 launch control_manager control_manager.xml
```

launch는 GNSS, 카메라, 수동 입력 노드, 기존 guidance 또는 trans를 함께 시작하지 않습니다.
종료는 `Ctrl+C`로 합니다.

## 검증 범위

초기 구성은 파일 내용과 package/launch/config의 이름 연결을 정적으로 확인하는 범위입니다.
컴파일·ROS launch·노드 실행·하드웨어 연결 검증은 수행하지 않았습니다.
빌드 성공이나 자율·수동 주행 기능의 완성을 의미하지 않습니다.
