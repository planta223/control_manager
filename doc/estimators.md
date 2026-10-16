# Estimator 통합 결과 (0.5.0)

검증 수치는 통합 당시 기록입니다. 현재 후반 command path는 [command_path.md](command_path.md)를 따릅니다.

## 1. 최종 명칭과 책임

executable / ROS node / YAML section의 기준 이름은 **global_state_estimator**, **lane_state_estimator**입니다.
C++ class만 GlobalStateEstimatorNode / LanePoseEstimatorNode 표기를 사용합니다.
기존 두 state 생성 노드를 하나로 통합했으며 GlobalPose와 MotionState 메시지는 합치지 않았습니다.

- GlobalPose: global/map-relative 위치·방향·uncertainty·validity·source Header.
  `position_valid`는 위치 단독 검증, `initial_heading_unavailable`은 GNSS 최초 heading 미확보를 구분합니다.
  `valid`는 위치와 yaw 모두 유효할 때만 true입니다. 위치 표준편차 상한은 현재 3 m입니다.
- MotionState: vehicle dynamic state, speed/yaw-rate 및 각각 validity·source Header.
- LanePose: lane-relative lateral error(m)·heading error(rad)·confidence·source Header.

```text
GNSS / IMU / actual VehicleFeedback        External Lane AI
                  \                         /
                        sensor_interface
                      /                 \
        internal GNSS/IMU/feedback    lane_observation
                    |                       |
          global_state_estimator     lane_state_estimator
              /          \                 |
         GlobalPose    MotionState       LanePose
              |             |              |
              +-------------|----+---------+
                            |    |
GlobalPath -----------------|--> path_planning
                            |         |
                            |    ReferencePath
                            |         |
                            +--> motion_planning <-- ObjectState / MissionLimit
                            |         |
                            |     MotionPlan
                            +--> motion_control <-- GlobalPose (existing map tracking)
                                      |
                             control_selector → vehicle_interface (protection / dry_run)
```

Map/Path는 planning reference입니다. mode request, ManualCommand, E-STOP은 command/event입니다.
이들은 ingress/estimator로 보내지 않습니다. 실제 외부 센서/feedback은 ingress를 통과합니다.

## 2–4. 모드와 required inputs

```yaml
/control/global_state_estimator:
  ros__parameters:
    pose_mode: GNSS
    motion_mode: GNSS
```

| 출력 | mode | 필요한 내부 입력 | 현재 구현 |
|---|---|---|---|
| GlobalPose | GNSS | sensors/gnss_odom | 기존 GNSS pose 검증/출력 |
| GlobalPose | GNSS_IMU | sensors/gnss_odom + sensors/imu | GNSS_IMU_NOT_IMPLEMENTED |
| GlobalPose | NONE | 없음 | POSE_DISABLED |
| MotionState | GNSS | sensors/gnss_odom | GNSS speed; yaw_rate_valid=false |
| MotionState | GNSS_IMU | sensors/gnss_odom + sensors/imu | GNSS_IMU_NOT_IMPLEMENTED |
| MotionState | WHEEL | vehicle_feedback | WHEEL_FEEDBACK_CONTRACT_NOT_IMPLEMENTED |
| MotionState | NONE | 없음 | MOTION_DISABLED |

NONE은 정상 disabled configuration이며 오류 로그를 내지 않습니다. disabled 출력은 Header frame만 설정하고
stamp=0, validity=false, 명확한 reason으로 발행합니다. 미구현 출력도 유효한 관측을 주장하지 않습니다.
잘못된 mode 문자열은 startup error입니다. 각 mode는 read-only parameter이며 변경에는 재시작이 필요합니다.
Guidance GPS/LANE/HYBRID는 source mode와 독립입니다.
Path Planning의 guidance_mode는 operation.yaml에서 시작 시 선택하며 변경 후 재시작합니다.
GPS는 GlobalPose+GlobalPath, LANE은 LanePose, HYBRID는 세 입력만 구독합니다.
별도 Guidance 모드 요청/상태 토픽은 없습니다. ReferencePath만 발행하며 사용할 경로가 없으면 빈 경로입니다.

## 5. 독립 validity와 기존 검증 보존

각각 estimatePose / estimateMotion에서 자기 mode만 처리합니다. 공통 validity gate가 없습니다.
구독은 선택한 두 모드의 required input 합집합입니다. 예를 들어 NONE/WHEEL에서는 GNSS와 IMU를 구독하지 않습니다.

GNSS pose의 quaternion norm, frame/child_frame, source/receipt freshness, position covariance 각각/합,
yaw covariance, finite position 검사를 유지합니다.
GNSS부에서 antenna offset x=0.559 m / y=0.0 m를 보정한 후륜축 중심 base_link odom을 사용합니다.
수동 기준점 확인 스위치는 없으며, estimator에서 lever arm을 중복 보정하지 않습니다.
GNSS motion의 twist.linear.x 추출 및 speed covariance/frame/source/receipt freshness/finite 검사도 보존했습니다.
pose covariance/quaternion 오류는 speed를 무효화하지 않고 speed variance/NaN 오류는 pose를 무효화하지 않습니다.
source stamp는 heartbeat로 갱신하지 않습니다. 서로 다른 source 사이 시간 동기화/보간은 새로 구현하지 않았습니다.
GNSS stamp의 sec<0 또는 nanosec>=1e9는 시간 변환 전에 거부합니다.
해당 출력은 INVALID_GNSS_TIMESTAMP, validity=false, stamp=0이며 정상 입력이 오면 복구합니다.

## 6. No-GNSS LANE configuration

```yaml
/control/global_state_estimator:
  ros__parameters:
    pose_mode: NONE
    motion_mode: WHEEL
```

이 조합은 정상 기동합니다. GlobalPose는 POSE_DISABLED이고 GNSS 입력을 요구하지 않습니다.
MotionState는 내부 VehicleFeedback만의 확장 지점이며 현재 WHEEL 자체가 미구현이므로 invalid입니다.
GNSS absence가 실패 이유가 아니며 유효 feedback fixture를 넣어도 가짜 yaw-rate/speed를 만들지 않습니다.
미래 bicycle model의 wheelbase는 vehicle.yaml에서 읽고 실제 측정 단위/시간·보정 계약 뒤에만 사용해야 합니다.

## 7. 제거된 기존 이름 (이행 기록에만 사용)

다음은 현재 architecture의 노드가 아니라 **제거 대상 목록**입니다. 호환 alias는 없습니다.

| 제거 항목 | 이전 이름 | 현재 대체 |
|---|---|---|
| executable | gnss_state_estimator_node | global_state_estimator |
| executable | motion_state_estimator_node | global_state_estimator |
| YAML node section | /control/gnss_state_estimator | /control/global_state_estimator |
| YAML node section | /control/motion_state_estimator | /control/global_state_estimator |
| source | src/gnss_state_estimator_node.cxx | src/global_state_estimator.cxx |
| source | src/motion_state_estimator_node.cxx | src/global_state_estimator.cxx |
| mode parameter | estimation_mode | pose_mode + motion_mode |

기존 build/install의 package-owned generated tree를 지우고 새로 빌드해 이전 executable이 남지 않도록 했습니다.
외부 launch/설정은 수정하지 않았으므로 기존 이름을 사용하는 외부 호출자는 담당 팀에서 이행해야 합니다.

## 8–9. Lane estimator / topic ownership

| Topic | Publisher | Consumer |
|---|---|---|
| /control/sensors/gnss_odom | sensor_interface | global_state_estimator (선택 모드에 따라) |
| /control/sensors/imu | sensor_interface (optional) | global_state_estimator (GNSS_IMU 선택 시) |
| /control/vehicle_feedback | sensor_interface (현재 unavailable) | global_state_estimator (WHEEL) |
| /control/global_pose | global_state_estimator | planning / tracking |
| /control/motion_state | global_state_estimator | motion planning / tracking |
| /control/lane_observation | sensor_interface | lane_state_estimator |
| /control/lane_pose | lane_state_estimator | path_planning |

Lane observation/pose는 동일한 4정보 LanePose wire type을 재사용합니다. 별도의 observation message는 만들지 않습니다.
Ingress는 외부 타입 경계·기초 검증, lane_state_estimator는 내부 관측의 finite/source freshness/confidence [0,1]
검사와 state 전달을 맡습니다. low-but-valid confidence는 planner가 lane_min_confidence로 결정합니다.
잘못된 관측은 drop하고 재발행하지 않습니다. 마지막 정상 관측의 만료는 planner source/receipt timeout으로 처리합니다.
두 단계 모두 원래 lateral error/heading error/confidence/frame/stamp를 보존합니다. geometry/temporal filter는 구현하지 않습니다.
외부 source와 observation/final-state가 겹치는 remap loop도 거부합니다.

## 10. Build / test

- ROS 2 Humble isolated clean configure/build 성공. 새 executable 2개를 포함해 총 11개, message 11개.
- GTest 34개 / CTest 4개 모두 통과. colcon 합산 38 tests / 0 errors / 0 failures / 0 skipped.
- 신규 mode/Lane smoke: 52개 확인 항목 통과. 전체 3×4 mode 조합, 선택 입력 구독 격리,
  NONE의 정상 disabled 동작, GNSS 없음 + WHEEL reason, Lane 검증/만료/loop 차단을 포함합니다.
- 기존 ingress 45개, state validation 78개, 전체 pipeline 68개, Tracking 59개 확인 항목 통과.
- 원래 GNSS pose/speed 검증 함수 몸체를 이전 source와 문자열 비교로 보존 확인했습니다.
  Covariance/quaternion/source time/frame 오류가 두 output을 독립적으로 무효화하는 기존 tests도 통과했습니다.
- 당시 GlobalPose/MotionState/VehicleFeedback wire fields 변경 없음.
  현재 GlobalPose에는 위의 최초 heading 확보용 필드 두 개를 추가했습니다.
- Lane 계약은 LanePose로 통일: lateral_error_m, heading_error_rad, confidence, header.stamp.
  외부 adapter와 내부 observation에도 같은 타입을 사용하고 최종 topic은 /control/lane_pose입니다.
  종오차 필드는 제거했습니다. 기존 외부 publisher에는 타입/topic 이행이 필요합니다.
- planner/PID/PP/motion planning/control/safety source와 안전 기본값을 보존했습니다.
- 모든 산출물·로그는 control_manager 내부. 실제 hardware/UDP actuator 시험 없음.

로그: log/estimators_modes_smoke_result.txt, estimators_ingress_smoke_result.txt,
estimators_state_smoke_result.txt, estimators_pipeline_smoke_result.txt,
estimators_tracking_smoke_result.txt 및 build/control_manager/test_results/.

## 11. 의도적 NOT_IMPLEMENTED / 변경하지 않은 영역

- pose GNSS_IMU: 검증된 fusion/IMU extrinsics/covariance/time alignment 없음.
- motion GNSS_IMU: GNSS speed + IMU gyro 의도만 준비. gyro-to-body 회전/validity/time 조건 미확정.
- motion WHEEL: 실제 feedback 계약과 추정 알고리즘 미확정.
- Lane/Hybrid path generation 및 local tracking-frame adapter는 계속 미구현.
- PID/PP 계산식·gain, motion planning/control, safety, map loader, actuator UDP는 변경하지 않았습니다.
- 기존 dry_run=true, PID disabled/zero gains, 미확정 safety limits 기본값은 유지합니다.

기존 ingress 구현·외부 파일 변경 제한도 유지합니다. 이전 작업의 외부 symlink 상태를 추측해 복구하지 않습니다.

## Estimator 통합 단계의 변경 파일 (이전 작업)

Estimator 통합 당시 source snapshot과 비교한 목록이며, Lane 파일 표기만 현재 계약에 맞게 갱신했습니다. 앞선 ingress 작업 변경은 보존한 상태에서 확장했습니다.
삭제된 구 source 두 개도 포함합니다. 해당 구 이름은 제거 이력일 뿐 신규 architecture 이름이 아닙니다.

```text
CMakeLists.txt
README.md
config/control_manager.yaml
doc/estimators.md
doc/guidance_modes.md
doc/sensor_ingress.md
doc/state_split.md
include/control_manager/core/types.hxx
include/control_manager/nodes.hxx
include/control_manager/ros_support.hxx
launch/control_manager.xml
msg/LanePose.msg
package.xml
src/global_state_estimator.cxx
src/gnss_state_estimator_node.cxx
src/lane_state_estimator.cxx
src/motion_state_estimator_node.cxx
src/sensor_interface_node.cxx
test/estimator_modes_smoke_test.py
test/ingress_smoke_test.py
test/smoke_test.py
test/state_split_smoke_test.py
```

## 12. Estimator 통합 단계 WRITE SCOPE 검증 (이전 작업)

작업 전후 외부 39,697개 항목의 mode/size/mtime_ns/SHA256/symlink target과 각 외부 Git 상태·diff를 비교했습니다.
EXTERNAL_CHANGES [] / EXTERNAL_GIT_CHANGES []이며 전후 fingerprint가 동일합니다.

- baseline: log/guidance_audit/estimators_before.json
- final: log/guidance_audit/estimators_after.json
- fingerprint: 3af5ad01c37c119c4cdf780fc6c66670f0d1fa62fae4a83547db84f931c02020
- 이번 시작 시점의 package source snapshot: log/estimators_before_sources.json
- 설치 검증: canonical executable/ROS name/YAML 일치, 노드/message 각각 11개,
  이전 executable과 접미사 alias 없음, config/document 경로 정상, dry_run/PID 기본값 유지.
- git diff --check 통과. 기존 외부 calibration 변경 및 과거 main/log/latest* 상태는 보존했습니다.

Modified files outside control_manager: 0
