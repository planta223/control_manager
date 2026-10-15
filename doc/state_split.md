# GlobalPose / MotionState 분리 보고서 (0.3.0)

## 변경 범위와 문제

VehicleState에 global pose와 motion validity가 묶여 있었고, motion planning/safety도 동일한
localization gate를 사용했습니다. LANE mode의 No-GNSS 입력 구조를 표현하도록 두 estimator,
두 메시지와 독립된 freshness 검증으로 이행했습니다. 외부 센서/GUI/펌웨어/launch는 수정하지 않습니다.

전체 node/topic ASCII, subscribe/publish 표, mode별 required input 및 breaking change는
[README](../README.md#현재-architecture-030-상태-분리)에 있습니다.

## 추가 / 제거

- GlobalPose.msg: map x/y/yaw, position x/y 및 yaw marginal variance, valid/reason/source Header.
- MotionState.msg: body speed/yaw rate, 각각 validity, reason/source Header.
- state_estimator_node → gnss_state_estimator_node rename, motion_state_estimator_node 추가.
- VehicleState.msg, 기존 executable/source 및 vehicle_state topic 제거. 호환 bridge 없음.
- LaneState는 lateral_error_m / longitudinal_error_m / confidence / header.stamp 계약.
  heading_error_rad/valid는 제거. Lane perception은 외부 팀 책임, sensor relay를 거치지 않습니다.
- Guidance GPS/LANE/HYBRID enum 및 config 명칭 변경, 숫자 0/1/2 유지.
- TrackingState는 기존 알고리즘 호출 인자만 제공합니다. ROS/shared estimator state가 아닙니다.

## 현재 구현과 미구현 경계

| 기능 | 실제 동작 |
|---|---|
| GNSS GlobalPose | 원래 quaternion/frame/covariance/finite/reference/freshness 검증 보존 |
| GNSS_IMU GlobalPose | 입력 자리/설정만, invalid GNSS_IMU_NOT_IMPLEMENTED |
| GNSS MotionState | 기존 twist.linear.x와 speed variance/frame/time 검증, yaw-rate invalid |
| WHEEL MotionState | 내부 입력 자리/vehicle.yaml wheelbase만, invalid WHEEL_FEEDBACK_CONTRACT_NOT_IMPLEMENTED |
| GPS Guidance | GlobalPose/GlobalPath 검증 후 pass-through; Lane 무시 |
| LANE Guidance | Lane만 검증; global 결손은 오류 아님; LANE_NOT_IMPLEMENTED |
| HYBRID Guidance | GlobalPose/GlobalPath/Lane 검증 후 HYBRID_NOT_IMPLEMENTED |
| Motion Planning | GlobalPose 구독 없음; MotionState/Path/Object/MissionLimit 사용 |
| Motion Control | GlobalPose/MotionState를 따로 검증한 뒤 기존 PID/PP 호출 |
| Safety | MotionState 항상 필요, fresh LANE status일 때만 GlobalPose 비필수 |

부족한 Lane 데이터로 path/heading을 만들지 않았습니다. PID gain/부호, PP 수식, map loader,
actuator/UDP 및 vehicle geometry 값은 바꾸지 않았습니다. 잘못된 mode 문자열은 시작 오류,
잘못된 runtime enum은 invalid 상태이며 silent fallback은 없습니다.

## Fail-closed / blocker

- 0/future/stale observation, receipt timeout, frame/finite/covariance/quaternion 불량은 해당 state invalid.
- Guidance 필수 입력 결손/미구현은 empty ReferencePath → invalid MotionPlan → AUTO stop.
- MotionState invalid/stale는 LANE에서도 safety stop. GPS/HYBRID는 GlobalPose도 필요.
- stale/missing/unknown GuidanceStatus는 보수적으로 GlobalPose를 요구; MANUAL에도 적용.
- E-STOP / command timeout / unknown objects / unverified limits / command NaN/range stop 유지.
- **Architecture blocker:** 기존 PID/PP는 map pose가 필요. base_link local path는
  LOCAL_REFERENCE_PATH_NOT_SUPPORTED로 거부. 가짜 zero pose/TF를 사용하지 않음.
- 시간 보간/동기화는 미구현: pose와 motion은 각각 freshness gate만 적용.
- 외부 계약 blocker: actual wheel speed/steering feedback 단위·calibration·topic/type/time,
  Lane 오차 부호·기준점·frame. 아직 임의 외부 ROS contract를 정의하지 않음.

다음 순서: 외부 입력 계약 합의 → local tracking-frame adapter → 검증된 Lane/Hybrid 경로 생성 →
fusion/실차 tuning/actuator 계약 검증. 지금은 No-GNSS LANE 주행 완성이 아니라 dependency 분리 단계입니다.

## 검증 방법

모든 build/install/log/temp/ROS log는 이 패키지 내부입니다. 새 generated tree에서 빌드해
구 VehicleState message와 state_estimator executable이 남지 않는지 검사합니다.
GTest/CTest와 localhost-only DDS domain 179의 세 smoke suite를 사용합니다. 실제 센서/차량 송신 없음.
테스트에서만 사용하는 positive gain/limits, lane/path/feedback fixture는 실차 추천값이 아닙니다.

## 검증 결과

- ROS 2 Humble isolated configure/build 성공. 최종 compile error/warning 없음.
- GTest **29개 통과** (core 9, guidance 6, tracking 14), CTest **3/3 통과**.
  colcon test-result 합산: **32 tests, 0 errors, 0 failures, 0 skipped**.
- 상태 분리 smoke **78개**, 기존 pipeline smoke **68개**, 확장 Tracking smoke **59개** 통과.
- 독립 pose/motion invalidation, covariance/quaternion/frame/zero·future·stale/receipt timeout,
  unverified reference, unsupported estimators, Lane freshness/confidence/finite, Hybrid dependencies,
  LANE no-global safety, stale guidance policy, local-path controller blocker를 검증했습니다.
- 기존 Tracking 수식은 HEAD 대비 VehicleState→TrackingState 타입명 변경만 있는지 비교했습니다.
  PID 기본 enabled=false, gains=0, max_steering_rad=0과 dry_run=true를 자동 확인했습니다.
- XML 11 node / 설치 executable 11개 / custom message 10개 / config·문서 경로 정합성 확인.
  build/install에 구 vehicle_state generated 파일과 state_estimator_node executable 없음.
- 로그: log/state_split_smoke_result.txt, log/state_split_legacy_smoke_result.txt,
  log/state_split_tracking_smoke_result.txt, build/control_manager/test_results/.
- 실제 센서·차량 연결이나 actuator UDP 송신은 시험하지 않았습니다.

## 변경 파일 전체 목록

모두 이 패키지 기준 상대경로입니다. 소스 rename은 Git에서 삭제/신규로 표시될 수 있습니다.
아래에는 삭제된 구 message/source도 포함합니다. generated build/install/log는 Git ignore 대상입니다.

```text
CMakeLists.txt
README.md
config/control_manager.yaml
doc/guidance_modes.md
doc/state_split.md
include/control_manager/core/guidance.hxx
include/control_manager/core/lateral_controller.hxx
include/control_manager/core/tracking.hxx
include/control_manager/core/types.hxx
include/control_manager/nodes.hxx
include/control_manager/ros_support.hxx
launch/control_manager.xml
msg/GlobalPose.msg
msg/GuidanceStatus.msg
msg/LaneState.msg
msg/MotionState.msg
msg/VehicleState.msg
package.xml
src/core.cxx
src/gnss_state_estimator_node.cxx
src/guidance.cxx
src/motion_control_node.cxx
src/motion_planning_node.cxx
src/motion_state_estimator_node.cxx
src/path_geometry.cxx
src/path_planning_node.cxx
src/pid_lateral_controller.cxx
src/pure_pursuit.cxx
src/safety_limit_node.cxx
src/state_estimator_node.cxx
src/tracking.cxx
test/smoke_test.py
test/state_split_smoke_test.py
test/test_control_core.cxx
test/test_guidance.cxx
test/test_tracking.cxx
test/tracking_smoke_test.py
```

## 외부 WRITE SCOPE 최종 확인

작업 전/후 control_manager 밖 39,697개 항목의 mode/size/mtime_ns 및 파일 SHA256·symlink target,
각 외부 Git 저장소 status/diff를 비교했습니다. 추가/삭제/변경 모두 0입니다.
기존 calibration 수정사항과 이전 main/log/latest* symlink 상태는 그대로 보존했습니다.

- baseline: log/guidance_audit/state_split_before.json
- final: log/guidance_audit/state_split_after.json
- EXTERNAL_CHANGES [] / EXTERNAL_GIT_CHANGES []
- 외부 snapshot SHA256 (전후 동일):
  3af5ad01c37c119c4cdf780fc6c66670f0d1fa62fae4a83547db84f931c02020
- package git diff --check 통과. tracked/untracked 변경은 패키지 내부 37개 경로뿐입니다.

Modified files outside control_manager: 0

이는 이번 작업의 변경 비교 결과이며 이전 작업의 unresolved symlink 기록을 소급해 해결했다는 뜻은 아닙니다.
