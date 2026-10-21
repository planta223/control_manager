# Sensor ingress 경계 / 이전 검증 기록

현재 estimator 통합 사항은 [estimators.md](estimators.md)가 기준입니다.
후반 command path의 현재 책임과 연결은 [command_path.md](command_path.md)를 참조합니다.

## 2026-10-21 인식 결과 입력 변경

`/control/vehicle_feedback`도 제거했다. 실제 값 변환 없이 invalid heartbeat만 발행하던
내부 확장용 토픽이므로 GNSS 기반 네 모드에는 필요하지 않다. 메시지/활성화 설정과
WHEEL의 해당 구독을 제거했다. `/motor_feedback`, `/steering_feedback`의 원본 relay는 유지하며,
WHEEL은 입력 구독 없이 기존 NOT_IMPLEMENTED/invalid 상태를 명시한다.

현재 pipeline과 4개 modes YAML에서는 원시 RGB/LiDAR 입력/relay를 제거했다.
IMU·depth·lane은 미구현 용도를 주석으로 표시하고 비활성으로 유지한다.
기존 raw relay/관측 전달이 존재해도 IMU 융합·depth 판단·LANE/HYBRID 주행은 미구현이다.

| 외부 토픽 | 내부 relay | 활성화 설정 | ROS 타입 설정 |
|---|---|---|---|
| /sensors/detectCone | /control/sensors/cone | cone_enabled | perception_types.cone (미정, 빈 값) |
| /sensors/detectObstacle | /control/sensors/obstacle | obstacle_enabled | perception_types.obstacle (미정, 빈 값) |
| /sensors/detectTraffic | /control/sensors/traffic | traffic_enabled | perception_types.traffic = control_manager/msg/TrafficLight |

Sensor Interface가 활성화된 결과만 같은 타입으로 relay한다. 원본 timestamp·valid·필드는
변경하지 않는다. 입력·출력 모두 SensorDataQoS로 reliable/best-effort 발행자를 수신한다.
타입은 외부 발행자와 정확히 일치하고 해당 메시지 패키지가 실행 환경에 설치되어야 한다.
활성화했는데 타입이 비어 있거나 잘못됐으면 시작 오류로 알린다.
false이면 결과 subscription과 relay publisher를 모두 만들지 않는다.

test: 세 입력 모두 false. brake_inspection: cone 필수 예정이나 타입 확정 전까지 false.
preliminary_lap/final_lap: traffic만 true. obstacle은 선택 구현이므로 false이다.
cone/obstacle 타입 확정 후 pipeline.yaml의 perception_types에 `패키지/msg/타입`을 넣고,
필요한 modes YAML의 해당 enabled를 true로 바꾼다. 임의 필드를 추가하지 않았다.

TrafficLight.msg는 modes/README.md의 traffic_id/status/timestamp/valid/color_confidence 계약을 따른다.
같은 필드여도 다른 패키지 이름의 메시지는 별개 타입이므로, 팀원이 외부 패키지를 사용하면
perception_types.traffic도 그 타입으로 변경해야 한다.

이번 변경은 결과 수신 경로까지다. 결과를 ObjectState로 검증·변환하고 목표 신호 선택,
현시 계산, 콘 제동 지점 판단을 수행하는 adapter/mission은 미구현이다.
따라서 4개 모드의 object_state_enabled는 false로 유지한다. 원시 RGB/depth는 Object Manager가
더 이상 구독하지 않는다. 아래 raw RGB/LiDAR 및 VehicleFeedback 흐름·과거 시험 수치는 이전 구현 이력이다.

## 1. 기존 역할

sensor_interface는 GNSS/IMU/RGB/LiDAR/Depth raw relay였습니다. LanePose는 외부에서 planner로
직접 입력했고 WHEEL에는 실제 feedback topic 경로가 없었습니다.

## 2. 변경된 역할

`sensor_interface_node` is the single ingress for real-time external observations and vehicle feedback used by control_manager.

센서/엔코더 raw 입력은 같은 타입·값으로 relay하고 Lane 입력은 timestamp/finite를 검증합니다.
GNSS/IMU fusion, 엔코더 단위·부호 보정, Lane estimation, yaw-rate 계산, planning,
tracking, actuator 송신은 이 계층에서 하지 않습니다.

## 3. 최종 데이터 흐름

```text
External GNSS / IMU / Lane AI / encoder_motor Motor-Steering feedback
                            |
                    sensor_interface
           +----------------+----------------+
           |                                 |
  sensors/gnss_odom,imu +              lane_observation
      vehicle_feedback                       |
           |                         lane_state_estimator
   global_state_estimator                    |
      /             \                    LanePose
 GlobalPose       MotionState                 |
      |                                 path_planning <-- GlobalPath
      +--------------------------------------+
```


## 4–5. External input / internal topic

| External input (topics.*) | Internal publish | Type / condition |
|---|---|---|
| gnss: /gnss/odom | /control/sensors/gnss_odom | nav_msgs/Odometry |
| imu: /imu/data | /control/sensors/imu | sensor_msgs/Imu; imu_enabled |
| rgb: /camera/image_raw | /control/sensors/rgb | sensor_msgs/Image |
| lidar: /lidar/points | /control/sensors/lidar | sensor_msgs/PointCloud2 |
| depth: /camera_dep/depth/image_raw | /control/sensors/depth | sensor_msgs/Image; depth_enabled |
| external_lane: empty default | /control/lane_observation | lane_enabled=true and configured LanePose topic required |
| external_motor_feedback: /motor_feedback | /control/sensors/motor_feedback | encoder_motor/msg/MotorFeedback; motor_feedback_enabled |
| external_steering_feedback: /steering_feedback | /control/sensors/steering_feedback | encoder_motor/msg/SteeringFeedback; steering_feedback_enabled |

센서/엔코더 raw relay는 입력·출력 모두 SensorDataQoS를 유지합니다. Lane 입력도 SensorDataQoS로
reliable/best-effort source를 받을 수 있고 내부 Lane/기존 VehicleFeedback 출력은 reliable depth 10입니다.
활성 raw 입력/출력이 빈 이름이거나 source와 output이 remap 후 같으면
시작 오류로 loop를 차단합니다. 외부 Lane topic default를 실제 팀 계약처럼 임의로 지정하지 않았습니다.

## 6. LanePose 흐름과 validation

External Lane → sensor_interface → lane_observation → lane_state_estimator → LanePose → path_planning.
LanePose는 lane 기준 차량 상대 pose이며 필드는 lateral_error_m / heading_error_rad / confidence / header.stamp입니다.
header.frame_id와 heading_error_rad의 원본 rad 값·부호도 그대로 보존하며 curvature/polyline/geometry는 추가하지 않습니다.
외부 타입이 LanePose와 다르면 ingress 변환 callback의 후속 구현이 필요합니다.

정상 관측은 timestamp·숫자를 그대로 relay합니다. NaN/Inf, confidence 범위 초과,
zero/future/stale/negative/noncanonical timestamp는 drop + throttled warning입니다.
마지막 정상 관측을 다시 publish하지 않으므로 planner의 기존 source/receipt timeout이 만료시킵니다.
따라서 malformed 입력은 즉시 invalid 토픽으로 바꾸는 것이 아니라 기존 관측이 만료될 때까지
최대 기존 freshness window 동안 유지될 수 있습니다. 낮은 유한 confidence는 전달하고 planner가 판단합니다.
GPS/LANE/HYBRID 알고리즘 및 planner source는 수정하지 않았습니다.

## 7. GNSS/IMU 흐름

External GNSS/IMU → ingress raw relay → sensors/gnss_odom 또는 sensors/imu → global_state_estimator.
global_state_estimator의 motion_mode=GNSS도 같은 내부 GNSS topic을 사용합니다. 기존 추정 수식/quality gate를 유지합니다.

## 8. Motor/steering 흐름

```text
encoder_motor /motor_feedback
  → Sensor Interface → /control/sensors/motor_feedback (MotorFeedback 그대로)
encoder_motor /steering_feedback
  → Sensor Interface → /control/sensors/steering_feedback (SteeringFeedback 그대로)
```

`MotorFeedback`는 motor_rpm(int32), motor_rpm_mcu_ms(uint32), motor_rpm_valid(bool),
`SteeringFeedback`는 steer_angle(float32, degree), steer_angle_mcu_ms(uint32),
steer_angle_valid(bool)이다. 메시지 타입은 encoder_motor 패키지에서 생성한다.
위 외부/내부 토픽은 pipeline.yaml과 standalone 기본값이 같다. operation.yaml의
motor_feedback_enabled/steering_feedback_enabled는 각각 구독과 relay 출력을 함께 켠다.
기본 operation은 둘 다 true이며 standalone에서 설정 파일 없이 실행하면 false이다.

callback에서 원본 메시지를 그대로 전달한다. 값, 부호, 단위, MCU tick과 valid를 변경하지
않으며 valid=false도 전달한다. timeout 처리나 캐시 재발행/20Hz 제한 timer를 추가하지 않는다.
encoder_motor가 만든 유효성은 센서 건강/보정 검증을 뜻하지 않는다. 두 보드의 MCU tick은
서로 및 ROS 시각과 독립적이며, 0/순환/역행도 그대로 전달한다. 향후 consumer가 실제 입력의
유효성/수신 경과시간/시각 진행을 판단해야 한다. Header를 요구하는 current()는 직접 적용 불가하다.

RPM→m/s, degree→rad, 부호·영점·구동비 보정, 두 입력의 동기화와 상태 추정은 후속 작업이다.
이번 raw relay는 기존 정규화된 VehicleFeedback을 생성하지 않는다.
vehicle_feedback_enabled=true일 때만 종전의 stamp=0 / validity=false unavailable heartbeat를
발행하며 기본값은 false이다. WHEEL 추정은 계속 NOT_IMPLEMENTED/invalid이다.
Global State Estimator의 기본 GNSS 모드는 유지한다.

## 9. 제외 입력과 이유

- Global Map/Path: reference/planning data → planner.
- Vehicle Control request: operator command → control_selector.
- Guidance/Tracking mode: operation.yaml의 시작 설정이며 변경 후 재시작.
- ManualCommand: command path → control_selector.

## 10. Custom message

VehicleFeedback.msg 추가: Header, speed_mps, steering_angle_rad, speed_valid, steering_valid.
내부 body speed와 실제 좌측 양의 road-wheel 각도이며 외부 firmware wire type을 확정한 것이 아닙니다.
VehicleCommand는 목표 명령이고 MotionState에는 실제 steering이 없으므로 이 메시지를 분리했습니다.
valid field는 공통 acquisition time을 의미합니다. 서로 다른 시각의 speed/steering을 임의로
한 timestamp로 합치지 않습니다. 시간 정합 계약 전에는 해당 field를 invalid로 유지해야 합니다.
helper는 source freshness/frame/finite/기존 validity를 확인하며 invalid를 valid로 승격하지 않습니다.
검증되지 않은 raw 변환, 운용 한계나 bicycle yaw-rate 계산은 넣지 않았습니다.
Lane 계약은 LanePose로 변경했습니다. 종오차 필드 대신 heading_error_rad를 사용하며 최종 topic은 /control/lane_pose입니다.
외부 adapter와 내부 observation도 LanePose 타입을 사용하므로 이전 외부 publisher는 별도로 이행해야 합니다.

## 11. 변경 파일 이력

이전 ingress 단계 파일 목록은 Git 이력/패키지 로그에 보존합니다. 최신 변경 목록은 estimators.md를 참고합니다.
아래 build/test 수치는 ingress 단계에서 측정한 이력입니다.

## 12. Build

ROS 2 Humble package-only configure/build 성공, compile error/warning 없음.
노드 11개 유지, custom message 11개(새 VehicleFeedback 포함), 문서/설정 install 경로 확인.
모든 build/install/log/tmp/ROS output은 control_manager 내부로 지정했습니다.

## 13. Test

- GTest 34개 통과: 기존 core 9 + guidance 6 + tracking 14 + 신규 ingress 5.
- CTest 4/4 통과. colcon 합산 38 tests / 0 errors / 0 failures / 0 skipped.
- 신규 ingress smoke 44개 확인 항목 통과.
- 기존 pipeline smoke 68개, 상태 분리 smoke 78개, Tracking smoke 59개 모두 통과.
- GNSS/IMU/RGB/LiDAR/Depth 정확한 payload/frame/stamp/covariance relay, Lane 네 값/stamp 보존,
  malformed/time/range drop와 회복, confidence 정책 분리, topic loop 거부, graph ownership을 확인했습니다.
- WHEEL 내부 feedback 연결과 invalid output, 외부 미설정 기본값, Map/operator/safety 경계 제외 확인.
- planner/GNSS estimator/PID/PP/motion planning/control/safety 소스가 HEAD와 동일함을 비교 확인했습니다.
- 테스트 fixture만 사용했으며 외부 하드웨어/액추에이터 UDP 시험은 하지 않았습니다.

로그: log/ingress_smoke_result.txt, log/ingress_pipeline_smoke_result.txt,
log/ingress_state_split_smoke_result.txt, log/ingress_tracking_smoke_result.txt,
build/control_manager/test_results/. 실행 환경/명령은 README를 따릅니다.

## 14–16. 미구현 / required external contract / breaking change

- 실제 motor/wheel 및 steering topic/type/단위/부호/calibration/source time·정합 계약 필요.
- 실제 Lane wire type/topic/오차 기준 합의 필요. 현재 adapter type은 opt-in LanePose일 뿐입니다.
- WHEEL estimator, GNSS_IMU fusion, Lane/Hybrid path, actual actuator UDP는 계속 미구현입니다.
- Lane 외부 publisher는 내부 /control/lane_pose 직접 발행을 중단하고 별도 합의 topic으로 이행해야 합니다.
  해당 이름을 topics.external_lane에 설정해야 relay가 켜집니다. 외부 코드는 변경하지 않았습니다.
- GPS/LANE/HYBRID·Tracking·Control 의미는 그대로입니다. 현재 estimator 명칭/설정은 estimators.md를 따릅니다.
- ROS topic 소유 정책이지 DDS writer 인증을 추가한 것은 아닙니다. 내부 observation/feedback의 publisher는 ingress, 최종 LanePose publisher는 lane_state_estimator가 맡습니다.

## 17. 외부 변경 검증

작업 전/후 외부 39,697개 항목의 mode/size/mtime_ns, 파일 SHA256, symlink target 및
각 외부 Git 저장소 status/diff를 비교했습니다. 결과는 EXTERNAL_CHANGES [] / EXTERNAL_GIT_CHANGES []입니다.
외부 기존 calibration 수정사항과 이전 main/log/latest* 링크 상태는 그대로 보존했습니다.

- baseline: log/guidance_audit/ingress_before.json
- final: log/guidance_audit/ingress_after.json
- 전후 동일 외부 fingerprint: 3af5ad01c37c119c4cdf780fc6c66670f0d1fa62fae4a83547db84f931c02020
- git diff --check 통과. tracked/untracked 변경은 control_manager 내부 18개 경로입니다.

Modified files outside control_manager: 0

이번 작업 기준이며 과거 unresolved symlink 기록을 소급해 복구했다는 의미는 아닙니다.
