# control_manager

ROS 2 Humble / C++17 / ament_cmake, `.cxx` / `.hxx`, XML launch 기반 상위제어 skeleton입니다.
**실차 주행 준비 완료를 뜻하지 않습니다.** 기본 IDLE, PID disabled/zero gains, 미확정 운용 한계는 0,
`dry_run=true`를 유지합니다. UDP backend가 없으며 `dry_run=false`는 시작 오류입니다.

## 현재 architecture (독립 상태 출력 + 두 estimator)

**노드는 global_state_estimator로 통합하고 GlobalPose/MotionState 메시지는 분리 유지합니다.**
Lane 상대 상태는 별도 lane_state_estimator가 담당합니다. 두 estimator의 executable/ROS node/YAML 이름은
각각 `global_state_estimator`, `lane_state_estimator`이며 `_node` 접미사를 붙이지 않습니다.

```text
External GNSS / IMU / motor·steering feedback / Lane AI / RGB / LiDAR / Depth
                                  |
                          sensor_interface
             +--------------------+---------------------+
             |                    |                     |
  sensors/gnss_odom, imu    lane_observation       sensors/rgb,lidar,depth
  + vehicle_feedback              |                     |
             |           lane_state_estimator     object_manager
   global_state_estimator         |                     |
        /          \           LanePose             ObjectState
   GlobalPose    MotionState      |                     |
        |             |           |                     |
        +-------------|-----------+                     |
                      |      path_planning <---- GlobalPath <---- global_path_provider
                      |                                MAP_*.json + GNSS datum (read-only)
                      |        GPS/LANE/HYBRID            |
                      |             |                    |
                      |        ReferencePath             |
                      +------> motion_planning <---------+ + MissionLimit
                      |             |
                      |         MotionPlan
                      +------> motion_control <--- GlobalPose* (existing PID/PP)
                                    |
                               AutoCommand + ManualCommand
                                    |
                      control_selector [IDLE/MANUAL/AUTO]
                                    |
                      selected_cmd → vehicle_interface (protection / dry_run)

* Global/map tracking dependency remains; local Lane tracking adapter is NOT_IMPLEMENTED.
```

GlobalPose는 map 기준 위치/방향, LanePose는 lane 기준 차량의 상대 pose, MotionState는 차량 운동입니다.
Guidance와 각 state source mode는 별개입니다. Map/command/E-STOP은 ingress나 estimator를 거치지 않습니다.

## Node별 subscribe / publish

표의 제어 topic은 `/control/` 접두사입니다. 원본 topic/config default는 `ros_support.hxx`와 YAML에 모았습니다.
기존 외부 node는 launch하지 않습니다. launch는 아래 **11개 executable**만 시작합니다.

| Executable | Subscribe | Publish / 책임 |
|---|---|---|
| sensor_interface_node | 활성화한 외부 센서/Lane 입력 | 대응 sensors/*·lane_observation, optional vehicle_feedback; 단일 ingress |
| global_state_estimator | 선택한 모드가 요구하는 sensors/gnss_odom, sensors/imu, vehicle_feedback | global_pose + motion_state; 독립 validity |
| lane_state_estimator | lane_enabled=true일 때 lane_observation | 같은 조건으로 lane_pose; finite/confidence/freshness 검사·원본 보존 |
| global_path_provider | 기존 최종 지도 JSON 파일 + GNSS GetParameters 읽기 전용 조회 | global_path; 실제 GNSS 원점/투영식으로 변환한 후륜축 경로 |
| path_planning_node | GPS: global_pose + global_path; LANE: lane_pose; HYBRID: 세 입력 | reference_path만 발행; Guidance owner |
| object_manager_node | object_state_enabled=true일 때 활성 RGB/LiDAR/Depth만 | 같은 조건으로 object_state; detection 없음, UNKNOWN/invalid |
| mission_based_limit_node | mission 활성 시 optional object_state; 구간 정책 활성 시 global_pose + mission_map | mission_limit; 구간 속도 상한·마진·정지·불확실성 판단 |
| motion_planning_node | reference_path, motion_state; 활성화한 object_state/mission_limit만 | motion_plan; 활성 제한·장애물 검사, 비활성은 reason에 표시 |
| motion_control_node | global_pose, motion_state, motion_plan, status | auto_cmd만 발행; PID/PP, speed pass-through |
| control_selector_node | auto_cmd, manual_cmd, mode_request; vehicle_interface_status는 FSM 관찰용 | selected_cmd, status; arbitration/FSM |
| vehicle_interface_node | selected_cmd, estop | vehicle_interface/status; 최종 명령 보호와 dry-run 진단, actuator 송신 없음 |

## Sensor ingress 경계

**`sensor_interface_node` is the single ingress for real-time external observations and vehicle feedback used by control_manager.**

Raw sensor relay는 기존 동작을 유지합니다. 외부 Lane 결과는 ingress에서만 받고 `/control/lane_observation`으로 전달합니다.
`lane_state_estimator`가 이 관측을 검증해 `/control/lane_pose`를 발행하고 planner는 최종 state만 구독합니다. 실제 hardware callback/단위·부호 변환도 앞으로 이 node에 둡니다.
GNSS localization, fusion, Lane filtering/path generation, yaw-rate/motion estimation은 수행하지 않습니다.

| 외부 입력 / 설정 | 내부 출력 | 상태 / QoS |
|---|---|---|
| topics.gnss = /gnss/odom | /control/sensors/gnss_odom | Odometry, gnss_enabled=true일 때, SensorDataQoS 그대로 |
| topics.imu = /imu/data | /control/sensors/imu | Imu, imu_enabled=true일 때만 |
| topics.rgb = /camera/image_raw | /control/sensors/rgb | Image, rgb_enabled=true일 때만 |
| topics.lidar = /lidar/points | /control/sensors/lidar | PointCloud2, lidar_enabled=true일 때만 |
| topics.depth = /camera_dep/depth/image_raw | /control/sensors/depth | Image, depth_enabled=true일 때만 |
| topics.external_lane = "" (미합의) | /control/lane_observation | lane_enabled=true + 비어 있지 않은 토픽 필요; 입력 SensorDataQoS, 출력 reliable depth 10 |
| topics.external_motor_feedback = "" | /control/vehicle_feedback의 speed 필드 | adapter 미구현, motor_feedback_enabled=true이면 시작 오류 |
| topics.external_steering_feedback = "" | /control/vehicle_feedback의 steering 필드 | adapter 미구현, steering_feedback_enabled=true이면 시작 오류 |

Lane 외부 topic을 추측해 기본 연결하지 않았습니다. 현재 adapter는 **control_manager/msg/LanePose wire type**을
명시적으로 선택하는 방식이며 타 팀의 실제 message type이 확정되었다는 뜻이 아닙니다.
`lane_enabled=true`와 함께 `topics.external_lane`을 합의된 별도 topic으로 지정합니다.
외부 타입이 다르면 ingress에 변환 callback이 필요합니다.
source/observation/final-state가 remap 후 겹치면 시작 오류로 relay loop를 차단합니다.
기존처럼 외부 publisher가 내부 `/control/lane_pose`에 직접 publish하는 연결은 이행 대상입니다.
이는 node/topic 소유 정책이며 DDS 접근 인증을 추가한 것은 아닙니다.

Lane의 finite lateral/heading/confidence, confidence [0,1], source timestamp를 ingress에서 검사합니다.
0/미래/만료/음수/비정규 timestamp 및 NaN/Inf/range 불량은 **drop + throttled warning**입니다.
정상 sample의 숫자·frame·stamp는 그대로 전달합니다. 마지막 정상 sample을 재발행하거나 timestamp를 갱신하지 않습니다.
불량 입력이 계속되면 planner는 마지막 정상 관측을 기존 state_timeout_s 내에 만료시킵니다.
즉 즉시 invalid 메시지를 만드는 방식이 아니며, 현재 기본 만료 상한은 관측 시각으로부터 0.5초입니다.
낮지만 정상 범위인 confidence는 전달하고 실제 최소 confidence 정책은 planner가 계속 결정합니다.

`VehicleFeedback`는 내부 실제 관측 계약입니다. **VehicleCommand는 목표 명령**, MotionState는 추정 결과이므로
actual steering을 표현할 메시지로 재사용하지 않았습니다. 외부 계약을 합의하면 sensor_interface의
`publishFeedback` 경계로 정규화된 측정값을 전달하도록 연결합니다. 현재 unavailable heartbeat timer는
그때 실제 adapter lifecycle/timeout 처리로 교체해야 합니다.

외부 실제 모터/구동륜 엔코더와 조향 엔코더는 각각 `topics.external_motor_feedback`,
`topics.external_steering_feedback`으로 Sensor Interface에 들어오는 구조로 예약했습니다.
외부 publisher가 내부 `/control/vehicle_feedback`으로 직접 연결되는 구조가 아닙니다.
외부 메시지 타입, encoder count/RPM 등의 단위, 기어비·영점·부호·road-wheel 변환,
측정 timestamp와 두 입력의 시간 정합을 합의한 뒤 이 node에서 내부 m/s·rad로 정규화합니다.
실제 외부 구독·변환은 없으며 encoder enable=true를 거부합니다. 비활성 입력의 토픽 이름은
미리 설정할 수 있고, 토픽 이름만으로 구독이 활성화되지는 않습니다.
`vehicle_feedback_enabled=true`일 때만 stamp=0, 두 validity=false인 unavailable 상태를 발행하고,
WHEEL 운동 추정도 계속 NOT_IMPLEMENTED입니다. GNSS 모드 동작은 바뀌지 않습니다.

**Ingress 제외:** Global Map/Global Path는 reference/planning data로 planner에 직접 연결합니다.
Vehicle Control mode_request는 control_selector, ManualCommand도 control_selector,
E-STOP은 vehicle_interface로 직접 전달합니다. Guidance/Tracking은 operation.yaml의 시작 설정입니다.
Guidance는 operation.yaml의 guidance_mode로 시작 시 고정하며 별도 요청 토픽은 없습니다.
어느 것도 ingress나 estimator를 경유하지 않습니다.

## Message / 시간 계약

- **GlobalPose** (`/control/global_pose`): Header, `x_m, y_m, yaw_rad`,
  `position_variance_x_m2, position_variance_y_m2, yaw_variance_rad2`, `valid`, `reason`.
  map 기준 rear-axle pose 계약. uncertainty는 GNSS marginal variance이며 full fusion covariance가 아닙니다.
- **MotionState** (`/control/motion_state`): Header, `speed_mps, yaw_rate_radps`,
  `speed_valid, yaw_rate_valid`, `reason`. body-frame longitudinal speed와 motion validity를 분리합니다.
  GNSS speed는 기존 `Odometry.twist.twist.linear.x`를 그대로 사용합니다. yaw-rate는 항상 invalid입니다.
- **VehicleFeedback** (`/control/vehicle_feedback`): Header, `speed_mps`, `steering_angle_rad`,
  `speed_valid`, `steering_valid`. body 전진 speed와 좌측 양의 실제 road-wheel 각도입니다.
  두 유효 필드는 동일 acquisition time을 뜻하며 서로 다른 시각의 측정을 새 stamp로 묶지 않습니다.
  외부 adapter는 timestamp 정합/단위/부호/보정을 합의해야 합니다. 현재 변환·합성은 미구현입니다.
  내부 검증은 source freshness/frame/finite와 원래 validity를 확인하고 invalid를 valid로 올리지 않습니다.
  검증되지 않은 physical max speed/steering limit은 가정하지 않습니다.
- **LanePose** (`/control/lane_pose`): lane_state_estimator 발행. Header, `lateral_error_m`, `heading_error_rad`, `confidence`.
  **외부 Lane AI가 제공하는 네 정보는 횡오차(m)·heading 오차(rad)·confidence·측정 timestamp(header.stamp)**입니다.
  Lane 기준 차량의 상대 pose입니다. heading 오차는 외부 입력을 그대로 사용하며 곡률/polyline/geometry 필드는 추가하지 않습니다.
  오차 부호·기준점·frame의 구체적 의미는 외부 팀과 합의가 필요합니다. 현재는 경로 변환에 사용하지 않습니다.
  finite 값, confidence 범위 [0,1]와 `lane_min_confidence`, source/receipt freshness로 내부 가용성을 판단합니다.
- **ObjectState**: traffic light/sign/obstacle 상태. `obstacle_valid=false`는 장애물 없음이 아닙니다.
  현재 perception은 미구현이며 invalid 관측만 발행합니다. LiDAR가 obstacle primary, Depth는 optional 보조입니다.
- **ReferencePath / GlobalPath**: 표준 `nav_msgs/Path`. global_path는 파일 제공자가 좌표계를 맞춘 정적 map snapshot;
  reliable transient-local depth 1이며 시간 만료 대신 datum/frame/finite/비퇴화 경로를 검사합니다.
  reference_path는 계획 생성 시각을 사용하며 invalid이면 빈 경로입니다.
- **MotionPlan**: Header, Path, target_speed_mps, stop_request, valid, reason.
  motion planning은 map 또는 base_frame Path를 전달할 수 있고 Path frame을 plan에 보존합니다.
  비기하학적 MissionLimit의 기존 map Header 계약은 유지합니다.
- **VehicleCommand**: 좌측 양수 `target_steer_rad`, 전진 `target_speed_mps`, bool brake, valid, reason.
  내부 물리 단위이며 motor raw/normalized joystick/실제 feedback과 다릅니다.
- **VehicleInterfaceStatus**: Header, dry_run, protected_command. 최종 보호 결과의 진단 전용이며
  실제 feedback이나 actuator 입력이 아닙니다. protected_command는 VehicleCommand 타입을 재사용합니다.
- **ControlStatus**: 차량 제어 모드/FSM. Motion Control의 PID 이력 관리에 사용합니다.

GlobalPose와 MotionState는 독립 C++ core type도 제공합니다. ROS VehicleState/msg/topic은 제거했습니다.
`TrackingState`는 기존 PID/PP 호출 직전에만 만드는 **알고리즘 인자**입니다. estimator output이나
공유 state cache/ROS topic이 아닙니다. pose와 motion freshness를 먼저 각각 검사합니다.

센서 source stamp는 반복 발행으로 갱신하지 않습니다. 수신 monotonic age와 ROS source age를 함께 검사하며
0/미래/만료 stamp를 거부합니다. 입력이 없거나 unsupported estimator이면 stamp=0, validity=false입니다.
서로 다른 시각의 pose/motion은 각각 timeout으로 제한하지만 시간 동기화/보간은 구현하지 않았습니다.
선택 명령도 원본 stamp를 보존합니다. sim clock/호스트 간 clock 동기화는 운용 측의 전제입니다.

## 서로 독립적인 모드

| 축 | 값 | 소유 node |
|---|---|---|
| Vehicle Control | IDLE / MANUAL / AUTO | control_selector |
| Guidance | GPS / LANE / HYBRID | path_planning |
| GlobalPose source (pose_mode) | GNSS / GNSS_IMU / NONE | global_state_estimator |
| Motion source (motion_mode) | GNSS / GNSS_IMU / WHEEL / NONE | global_state_estimator |
| Tracking | PID / PURE_PURSUIT | motion_control |

### Estimator 동작 / 독립 모드

```yaml
/control/global_state_estimator:
  ros__parameters:
    pose_mode: GNSS
    motion_mode: GNSS
```

| 출력 / mode | 필요한 내부 입력 | 현재 동작 |
|---|---|---|
| GlobalPose / GNSS | sensors/gnss_odom | 기존 GNSS pose 검증 후 출력 |
| GlobalPose / GNSS_IMU | sensors/gnss_odom + sensors/imu | GNSS_IMU_NOT_IMPLEMENTED, invalid |
| GlobalPose / NONE | 없음 | POSE_DISABLED, invalid, stamp=0; 정상 configuration |
| MotionState / GNSS | sensors/gnss_odom | speed 검증, yaw_rate_valid=false |
| MotionState / GNSS_IMU | sensors/gnss_odom + sensors/imu | GNSS_IMU_NOT_IMPLEMENTED, 두 motion validity=false |
| MotionState / WHEEL | vehicle_feedback만 | WHEEL_FEEDBACK_CONTRACT_NOT_IMPLEMENTED, invalid |
| MotionState / NONE | 없음 | MOTION_DISABLED, invalid, stamp=0; 정상 configuration |

`pose_mode`와 `motion_mode`는 독립 enum/설정/계산 함수입니다. 구독은 두 모드에 필요한 입력의 합집합이고,
각 출력은 자기 입력만 검사합니다. Pose invalid가 유효 speed를 무효화하거나 그 반대가 되는 공통 gate는 없습니다.
`pose_mode=NONE, motion_mode=GNSS`에서도 speed는 정상 생성됩니다. `NONE` 출력의 0 값은 관측값이 아니며
invalid/disabled와 stamp=0으로 명시합니다. 메시지·topic은 서로 합치지 않았습니다.

GNSS pose의 position/yaw, quaternion norm, parent map/child base_link, source/receipt freshness,
finite XY, position variance 각각 및 합, yaw variance 검사를 유지합니다.
GNSS odom의 base_link는 후륜축 중심 계약이며, GNSS부가 antenna offset x=0.559 m / y=0.0 m를 보정합니다.
수동 기준점 확인 스위치는 사용하지 않으며, 여기서 lever arm을 중복 보정하지 않습니다.
GNSS motion은 기존 twist.linear.x와 speed variance/frame/freshness/finite를 검사합니다.
GNSS stamp의 sec<0 또는 nanosec>=1e9이면 시간 변환 전에 INVALID_GNSS_TIMESTAMP / invalid / stamp=0을 발행합니다.
IMU/VehicleFeedback이 없어도 GNSS 모드의 두 출력은 유효할 수 있습니다.

GNSS_IMU motion의 의도는 GNSS speed + IMU gyro yaw rate입니다. 현재 IMU-to-body rotation,
validity/time alignment가 검증되지 않아 임의 결합하지 않고 명시적으로 미구현 처리합니다.
WHEEL은 내부 feedback만 구독하고 GNSS를 요구하거나 fallback하지 않습니다. 향후 검증된 입력과
vehicle.yaml의 wheelbase로 v/L*tan(delta)를 사용할 수 있지만 현재 계산하지 않습니다.
따라서 No-GNSS LANE 목표 설정 `pose_mode=NONE, motion_mode=WHEEL`은 정상 기동하되,
pose는 disabled이고 motion은 WHEEL 미구현으로 invalid입니다. GNSS absence가 그 실패 이유는 아닙니다.

`lane_state_estimator`는 내부 `lane_observation`(LanePose wire type 재사용)을 받아 state를 발행합니다.
횡오차와 heading 오차의 finite 여부, confidence [0,1], source freshness를 검사하고 Header와 세 숫자를 그대로 전달합니다.
confidence가 낮지만 정상 범위이면 유지하며 planner의 lane_min_confidence가 사용 여부를 결정합니다.
잘못된 관측은 drop, 입력 중단 시 반복 발행하지 않으므로 downstream timeout이 작동합니다.
Lane filtering/curvature/polyline/geometry 추정은 하지 않습니다. heading 오차의 부호·단위 변환 없이 입력 rad 값을 보존합니다.
`lane_enabled=false`이면 observation 구독과 lane_pose 발행을 모두 생성하지 않습니다.

### 기존 최종 지도 연결

기본 파일은 `main/decision/script/map_generator/test/review/maps/MAP_2026-10-15_195100_test.json`입니다.
`final map`은 이 파일의 `format: gps-final-map`을 뜻하며, 별도 파일 이름이 아닙니다.
같은 날짜의 `11단계_*.json`은 편집 프로젝트/검증 이력이며 입력 대상이 아닙니다.
`gps-map.json`은 배경 지도입니다. `best_map`이라는 파일/기본 선택 설정은 발견되지 않았습니다.
최신 최종 지도를 기본 후보로 사용하며 여러 지도 사이의 실차 품질 우열을 판정한 것은 아닙니다.

`pipeline.yaml`의 `/control/global_path_provider.map_file`에 현재 workspace의 절대 경로를 지정합니다.
다른 workspace에서는 이 경로를 바꾸십시오. 파일은 읽기만 하며 원본을 수정하지 않습니다.
파일 제공자는 `gps-final-map` v1, WGS84/ENU/m, `rear_axle_center`, 열린 경로,
증가하는 순번·거리 및 유한한 위경도를 검사합니다. 현재 경로는 5,066점, 약 506.45 m입니다.
편집 이력의 안테나 offset도 x=0.559 m, y=0 m이며 이미 후륜축 기준으로 생성되어 있습니다.
따라서 추가 offset을 적용하거나 경로를 재피팅하지 않습니다. 차선/과제 구간은 이번 GPS 경로 입력에서 사용하지 않습니다.

저장된 e/n은 dashboard의 WGS84 ECEF 기반 ENU이고, GNSS odom은 자체 LocalCartesian 근사식입니다.
제공자는 `/gnss_node/get_parameters`에서 `datum_lat`, `datum_lon`, `map_frame`, `base_frame`만 조회하고,
파일의 lat/lon을 현재 `gnss_cxx::LocalCartesian`과 같은 WGS84 수식으로 XY 재투영합니다.
XY 계산에 고도가 들어가지 않으므로 `datum_alt`나 미확정 지도 고도를 추가하지 않습니다. 출력 z=0은 2D 경로입니다.
GNSS 자동 원점은 첫 유효 GGA 이후 GNSS가 자신의 datum 파라미터에 기록한 값을 사용합니다.
기존 GNSS node는 별도 실행해야 하며, 구버전 GNSS binary가 자동 원점을 파라미터에 공개하지 않으면
datum이 NaN으로 남아 경로를 발행하지 않습니다. `gnss_node` 이름은 pipeline에서 변경할 수 있습니다.

원점 확인 전에는 경로를 발행하지 않습니다. 확인 후 reliable/transient-local/depth 1로 1회 발행하고,
1초 간격으로 원점을 조회하여 원점 변경 시 재투영합니다. 서비스 부재/응답 timeout/NaN/frame 불일치나
GNSS 문서의 5 km 투영 범위 밖 지도는 기존 경로를 빈 경로로 철회합니다. 조회 기반 감지는 즉시가 아니며,
GNSS 측정 중단은 기존 GlobalPose freshness 검사가 별도로 처리합니다.
Path Planning의 `map_datum_verified=true` 기본값은 이 제공자의 정합 검사를 전제로 합니다.

`operation.yaml`의 공통 `guidance_mode`를 두 노드가 공유합니다. LANE이거나 `map_file: ""`이면
파일 제공자는 조회/timer/publisher를 만들지 않습니다. 별도 ROS 지도 제공자를 연결할 때는 map_file을 비우고,
같은 topic/QoS 및 GNSS 원점·투영·후륜축 계약을 확인하십시오. 파일 제공자와 외부 제공자를 동시에 사용하지 않습니다.
Path Planning의 GPS 입력은 기존대로 GlobalPose + GlobalPath 두 개, 출력은 ReferencePath 하나입니다.

검증: `test/test_global_path_map.py`는 파일 형식·실제 지도·좌표 변환을 검사하고,
`test/global_path_smoke_test.py`는 실제 지도 파일 + 가짜 읽기 전용 GNSS 서비스로 latched 발행,
Path Planning 전달, 원점 변경, 좌표계/원점 불량 및 서비스 중단 시 경로 철회와 복구를 확인합니다.
ROS fixture는 기존 smoke와 동일한 localhost 전용 domain 179 환경에서 실행합니다.

### Guidance required input

| Guidance | GlobalPose | GlobalPath | LanePose | 현재 실제 결과 |
|---|---|---|---|---|
| GPS | 필수 | 필수 | 무시 | 검증 후 global Path pass-through |
| LANE | 불필요 | 불필요 | 필수 | Lane 검사 후 LANE_NOT_IMPLEMENTED, empty Path |
| HYBRID | 필수 | 필수 | 필수 | 세 입력 검사 후 HYBRID_NOT_IMPLEMENTED, empty Path |

GPS/HYBRID는 GlobalPose → GlobalPath 순으로 결손을 보고합니다. LANE은 이 두 조건을 검사하지 않습니다.
LANE/HYBRID 시작 설정은 lane_enabled=true를 요구합니다. 모드에 필요한 입력만 구독하며,
GPS에서는 lane_enabled=true여도 LanePose를 구독하지 않습니다. LANE은 GlobalPose/GlobalPath를 구독하지 않습니다.
활성 Lane 오류: `LANE_POSE_UNAVAILABLE`, `LANE_VALUES_INVALID`, `LANE_CONFIDENCE_LOW`.
Lane이 정상이어도 현재 네 정보만으로 검증된 ReferencePath를 만들 수 없어 미구현 상태로 닫습니다.
GPS로 silent fallback하거나 fake lane correction을 만들지 않습니다. HYBRID는 GPS+Lane이며 IMU fusion과 다릅니다.
`guidance_mode`는 GPS / LANE / HYBRID 세 값만 허용하는 read-only 시작 설정이며 변경 후 재시작합니다.
NONE이나 잘못된 문자열은 시작 오류입니다. Path Planning은 ReferencePath만 기본 20 Hz 생성 시각으로 발행합니다.
입력 불량이나 미구현 모드이면 빈 경로를 발행합니다. 별도 GuidanceStatus 토픽은 사용하지 않습니다.
기존 msg/GuidanceStatus.msg 파일은 이력으로 보존하며 메시지 생성·설치 목록에서는 제외합니다.

### Downstream / safety / architecture blocker

Motion planning은 **GlobalPose를 구독하지 않습니다**. ReferencePath + 유효 MotionState와 활성화된 MissionLimit/ObjectState로
계획합니다. mission_based_limit는 정상 주행 속도/신호 정책이고, vehicle_interface는 선택 명령 이후 actuator-facing 보호를 담당합니다.
Obstacle detected이면 motion planning은 회피 대신 정지합니다. speed PID는 없습니다.

### GNSS-only Mission 구간 정책

운용속도와 Mission 상수는 `config/mission.yaml`에 둡니다. 공통 `cruise_speed_kph`가
등속 목표이며 수정 가능합니다. Motion Planning/Mission이 m/s로 변환합니다.
Mission 비활성 상태에서도 Motion Planning은 이 등속 목표를 사용합니다.
기존 `cruise_speed_mps`를 명시적으로 override하면 해당 m/s 값이 우선하며,
기존 `mission_max_speed_mps` override는 기본 Mission 상한만 따로 제한할 때 사용할 수 있습니다.
일반 운용 설정에서는 이 중복 단위 key를 추가하지 않고 `cruise_speed_kph` 하나를 수정합니다.

`mission.yaml` 공통 영역의 두 활성화 key를 사용합니다.

| mission_limit_enabled | mission_sections_enabled | 동작 |
|---|---|---|
| false | false/true | Mission node는 입력·출력·timer 없음, Motion Planning은 공통 등속 목표 사용 |
| true | false | Mission은 기본 속도 상한 발행, 위치·지도 구간 구독 없음 |
| true | true | 실제 위치와 지도 task_sections로 구간별 제한 계산 |

기본값은 둘 다 false입니다. 구간 정책만 켜기 위해 운용 모드나 센서 선택을 바꾸지 않습니다.
MissionLimit의 소비자는 기존 Motion Planning 하나이며 Ld나 sensor/Guidance mode를 바꾸지 않습니다.
Pure Pursuit Ld는 실제 MotionState 속도로 계산하며 PID는 Ld를 사용하지 않습니다.

구간 정책 활성 시 파일 제공자가 `/control/mission_map`에 `MissionMap`을 reliable/transient-local/depth 1로 발행합니다.
이 메시지는 GNSS에 맞춘 전체 경로, 원본 s_m 배열, task_sections 이름·시작·끝을 한 snapshot에 담습니다.
기존 global_path/ReferencePath 계약은 그대로입니다. GNSS datum/frame 조회 실패 시 두 지도 snapshot을 철회합니다.
Mission 구간 정책 비활성 시 추가 map publisher가 없습니다.

`section_ids`의 정책별 `map_section`을 지도 구간 이름에 연결합니다. 현재 `straight_1 → 직진1`을 준비했으며,
지도 구간 경계인 약 18.47~56.51 m는 설정에 중복 기입하지 않습니다.
`max_speed_kph` 생략 시 공통 운용속도, `margin_before_m`/`margin_after_m`는 각각 감속 적용 구간의 앞/뒤 확장 거리입니다.
예를 들어 원래 [100,120] m 구간에 앞 10 m / 뒤 5 m 마진이면 [90,125] m에서 제한 속도를 적용합니다.
`stop_required=true`는 원래 구간에 적용하며 감속 마진만으로 조기 정지를 요구하지 않습니다.
정지 지점으로의 가감속 프로파일·정차 유지 시간·자동 재출발 조건은 이번 구현에 없습니다.
stop_required 구간에서 정지하면 별도 정책/설정 변경 없이 스스로 그 구간을 빠져나오지 못합니다.

GlobalPose를 선분에 투영하고 원본 누적거리를 보간합니다. GNSS XY 분산의 trace와
`position_sigma_multiplier`로 위치 여유를 계산합니다. 이것은 full covariance 기반 확률 보장이 아닌 보수적인 정책입니다.
경계에 걸치면 가능한 구간 중 최소 속도 / 정지 요구 OR를 적용합니다.
경로 이탈, 서로 떨어진 경로 후보, invalid/stale/잘못된 stamp의 pose,
지도 철회/불량 및 설정한 이름이 지도에 없으면 invalid/stop입니다.
겹치는 정책은 최소 속도 / 정지 OR이며, 명시적으로 정책을 연결하지 않은 지도 구간은 기본 상한을 사용합니다.
불확실성 상수는 실차 보정 완료값이 아닌 software policy/TUNE입니다.
속도 제한은 단계적으로 바뀌며 미리 넓힌 적용 구간이 자동 가감속 프로파일을 생성하는 것은 아닙니다.

검증은 `test/test_mission.cxx`, `test/mission_smoke_test.py`, `test/global_path_smoke_test.py`에서 수행합니다.

**현재 PID/PP는 map pose와 map 경로가 필요합니다.** local/base_link ReferencePath가 오면
`LOCAL_REFERENCE_PATH_NOT_SUPPORTED`로 명시적으로 차단합니다. 0 pose/identity TF로 우회하지 않습니다.
LANE planner의 No-GNSS dependency 분리는 완료됐지만 No-GNSS end-to-end 주행은 아직 구현되지 않았습니다.
향후 local reference frame/시간 기준과 controller tracking-frame adapter가 합의되어야 합니다.

상태 validity/freshness 검사는 해당 planning/control node에서 담당합니다. 후단 selector/interface는
GlobalPose, MotionState, ObjectState를 구독하지 않습니다. AUTO는 invalid/stale planning 입력이
invalid MotionPlan → invalid AutoCommand → selector stop으로 이어집니다. MANUAL에는 localization/obstacle gate를 적용하지 않습니다.
장애물 검출/관측 만료에 따른 AUTO 정지는 기존 motion_planning에서 처리하며 추가 판단 알고리즘은 없습니다.

control_selector는 mode별 source 선택, source frame/validity/timeout 검사, IDLE 정지를 담당합니다.
vehicle_interface는 E-STOP, 선택 명령 수신/source timeout, non-finite/음수 속도/제동·속도 모순,
미검증 운용 한계와 범위 초과를 검사합니다. 범위 초과는 기존처럼 clamp 대신 stop입니다.
E-STOP true는 explicit false까지 유지됩니다. vehicle_interface가 자체 timer로 command loss를 처리하므로 selector 중단도 정지시킵니다.
최종 보호 결과는 내부 값이며 actuator backend는 미구현입니다. `/control/vehicle_interface/status`는
`VehicleInterfaceStatus` 진단 전용으로 dry_run과 protected_command를 표시합니다. 실제 feedback이 아닙니다.
control_selector는 이 진단을 FSM에만 사용하고 source 선택에는 사용하지 않습니다. 기존 final-protection 기반
FAULT/READY/DRIVING 관찰 및 PID reset 계약을 유지합니다. 진단 중단은 FSM FAULT이며 actuator 입력 경로가 되지 않습니다.

물리 E-STOP/재시작 latch/인증/하드웨어 watchdog은 미구현입니다. 안전 범위 및 PID gain은 모두 기존 값 그대로입니다.

AUTO는 auto_cmd만, MANUAL은 manual_cmd만 선택합니다. invalid ControlMode 요청은 IDLE입니다.
FSM INIT/READY/DRIVING/STOPPED/FAULT는 그대로이며 DRIVING은 명령 단계이지 actual motion feedback이 아닙니다.
RGB는 traffic light/sign/Lane 입력용이며 실제 recognition은 없습니다. IMU/Depth는 config opt-in이고 runtime toggle은 없습니다.
Pure Pursuit는 GNSS 패키지 책임이 아니라 Motion Control 내부 lateral controller입니다.

## Command path 단순화

현재 변경 및 상세 책임표는 [Command path 보고서](doc/command_path.md)를 참조합니다.
패키지 이름은 control_manager이며 arbitration node는 control_selector입니다.
SelectedCommand는 기존 VehicleCommand 의미와 `/control/selected_cmd` 토픽을 유지합니다.
최종 command 중계 토픽 없이 vehicle_interface 안에서 보호 후 dry-run을 수행합니다.
검증 결과: Humble 격리 빌드 성공, GTest 36개 + CTest 4개 통과,
6개 smoke suite의 350개 확인 항목 통과. Planning/Control/PID/PP 알고리즘 소스는 동일합니다.

## Breaking changes / 다음 구현 경계

- 두 state 생성 노드를 `global_state_estimator`로 통합했습니다. pose_mode/motion_mode를 별도로 설정합니다.
  구 개별 executable/YAML/estimation_mode alias는 제공하지 않습니다. 제거 목록은 [Estimator 보고서](doc/estimators.md)에 있습니다.
- `lane_state_estimator`를 추가했습니다. ingress 출력은 lane_observation, 최종 lane_pose의 publisher는 lane estimator입니다.
- GlobalPose/MotionState topic과 기존 필드는 유지합니다. GlobalPose에는 최초 heading 확보를 위한
  `position_valid`, `initial_heading_unavailable`을 추가했습니다. 메시지 사용 패키지 재빌드와 전체 노드 재시작이 필요합니다.
- Lane 계약은 `LanePose`로 변경했습니다. 외부 adapter와 `/control/lane_observation`은 `control_manager/msg/LanePose`를 사용하며,
  최종 출력은 `/control/lane_pose`입니다. 종오차 필드는 제거하고 `heading_error_rad`를 사용합니다.
  이전 Lane 타입/최종 topic의 호환 alias는 없습니다. 외부 publisher는 새 타입에 맞게 별도로 이행해야 합니다.
  Lane 입력 미가용 상태의 reason은 `LANE_POSE_UNAVAILABLE`입니다.
- 외부 Lane publisher는 별도 합의 topic + topics.external_lane 설정을 사용합니다. 외부 코드는 수정하지 않았습니다.
- 외부 launch가 이전 estimator executable을 실행한다면 담당 팀에서 이행해야 합니다.
- 다음 우선순위: 실제 feedback/IMU/Lane 기준 계약 → local tracking-frame adapter → 검증된 Lane/Hybrid 경로 및 source 알고리즘.

## Tracking Controller (2026-10-15)

현재 구조에서 `motion_control_node`만 Tracking 선택과 ROS I/O를 소유합니다.
Guidance(GPS/LANE/HYBRID), Vehicle Control(IDLE/MANUAL/AUTO), Localization은 별개입니다.
`GlobalPose.x_m/y_m/yaw_rad`, `MotionState.speed_mps` 및 각각의 stamp/validity와
`MotionPlan.reference_path`의 `poses[].pose.position.x/y`, frame/stamp를 그대로 사용합니다.
Path의 quaternion heading은 사용하지 않고 가장 가까운 유효 선분의 tangent를 계산합니다.
`MotionPlan.valid/stop_request/target_speed_mps`와 AutoCommand의 VehicleCommand 계약은 유지합니다.

| Tracking 시작 설정 | 구현 / 실제 동작 |
|---|---|
| PID (0) | GNSS 경로 lateral PID + heading P. 기본 tuning은 비활성 placeholder |
| PURE_PURSUIT (1, 테스트 선택) | 후륜축 기준 최근접 선분 투영 + 속도 적응형 lookahead |
| 나머지 문자열 / NONE | 시작 오류; fallback 없음 |

현재 PURE_PURSUIT도 속도에 따라 lookahead가 변하는 Adaptive Pure Pursuit에 해당합니다.
Pure Pursuit 계열은 PURE_PURSUIT 옵션 하나로 선택합니다.

Motion Control은 `auto_cmd` 하나만 발행합니다. 조향·목표 속도·제동·유효성·계산 사유는
VehicleCommand에 포함됩니다. PLAN_STOP은 유효한 speed=0/brake=true 정지 명령이고,
목표 도달도 유효한 GOAL_REACHED 정지 명령입니다. 입력 불량은 invalid/정지입니다.
TrackingStatus 토픽과 메시지 생성은 제외했으며 msg/TrackingStatus.msg는 과거 정의로 보관합니다.
tracking_mode는 read-only 시작 설정이며 변경 후 노드를 재시작합니다.
명령은 기존 rate_hz(20 Hz)로 발행합니다. 잘못된 tracking_mode 문자열은 시작 오류입니다.

### PID 수식 / 부호 / 시간

진행 방향 단위 tangent를 t=(tx,ty), 차량에서 최근접 경로 projection까지의 벡터를 d=(dx,dy)라 하면:

```text
e_y   = tx * dy - ty * dx                       [m]
e_psi = atan2(sin(path_yaw-yaw), cos(...))        [rad]
I     = clamp(I_previous + e_y*dt, -I_max, I_max) [m*s]
D     = (e_y - e_y_previous) / dt                [m/s]
steer = clamp(kp_lateral*e_y + ki_lateral*I
            + kd_lateral*D + kp_heading*e_psi,
              -max_steering_rad, +max_steering_rad)
```

경로 진행 방향의 **왼쪽에 차량이 있으면 e_y<0 → 우조향**, 오른쪽이면 e_y>0 → 좌조향입니다.
전륜 road-wheel steering은 기존처럼 rad, 좌회전이 양수입니다. 예: 동쪽 직선에서 차량 y=+1 m이면
오차 -1 m입니다. 차량 heading이 경로보다 CCW로 틀어졌으면 heading correction은 음수입니다.
PID는 조향만 반환하며 속도는 MotionPlan 목표를 차량 상한으로 제한합니다. brake는 node/plan/safety가 결정합니다.
PID 내부 max_steering_rad는 vehicle.yaml 조향 범위의 최대 절댓값에서 구하며 별도 설정하지 않습니다.
Motion Control은 PID / Pure Pursuit 계산 후 차량의 실제 최소·최대 조향 범위로 다시 제한합니다.

`dt`는 node timer 사이의 steady-clock 실제 경과 시간입니다. 첫 sample/reset 직후에는 I=D=0이며
그 다음부터 누적합니다. integral clamp가 최소 anti-windup입니다. NaN/Inf, 계산 overflow, dt<=0 또는
max_dt_s 초과는 invalid와 reset입니다. forward-only baseline으로 끝점 뒤나 반대 방향 경로는 거부하고,
끝점 도달은 기존 goal_tolerance_m를 사용합니다. loop/교차점 progress 판정은 구현하지 않았습니다.

다음 경우 PID 이력을 reset합니다: 노드 재시작, 입력 invalid/stale, stop plan, goal,
경로 점 좌표 변경, 긴 dt. 같은 경로의 stamp 갱신만으로는 reset하지 않습니다.
`/control/status`를 추가 구독하여 IDLE/MANUAL/FAULT/INIT 및 stale/missing status에서는
PID 이력을 지우고 **P + heading P preview만** 계산합니다. 이때 reason=PID_PREVIEW_STATE_RESET입니다.
AUTO가 READY/DRIVING/STOPPED이고 status도 신선할 때만 I/D 이력을 유지합니다.
이 구독은 ControlMode를 바꾸거나 safety 권한을 이전하지 않습니다. FAULT의 원인인 final command를
기다리며 모든 AUTO 출력을 막는 순환 대기 없이 기존 안전 조건이 회복될 수 있도록 preview는 허용합니다.
preview AutoCommand도 실제로 선택할지는 control_selector/vehicle_interface가 결정합니다.

GlobalPose/MotionState/MotionPlan은 source stamp와 수신 timeout을 검사하고, 주행 계산에서는 nested
reference_path.header.stamp도 검사합니다. 잘못되거나 만료된 경로로 유효 steering을 만들지 않습니다.
시각 변환 전에 sec >= 0, nanosec < 1,000,000,000 형식을 확인합니다. 잘못된 형식을 정규화하거나
수신 시각으로 대체하지 않으며, 필수 입력 오류는 invalid 정지 후 정상 입력으로 복구합니다.
ControlStatus도 같은 형식을 확인하며 불량 시 기존 정책대로 PID 이력을 지우고 P/heading P preview를 사용합니다.
ReferencePath를 주기적으로 생성하는 기존 planner 계약은 유지되며 upstream Global Path는 정적 snapshot입니다.

### GPS 초기 heading 확보 / 경로 이탈 검사

GNSS 위치 표준편차 상한은 `operation.yaml`의 `max_position_std_m=3.0`이며
`sqrt(cov[0]+cov[7])` 기준입니다. yaw 품질 기준은 기존 값을 사용합니다.
GlobalPose의 `position_valid`는 위치·좌표계·시각 검증 결과, `initial_heading_unavailable`은
GNSS가 한 번도 관측 heading을 제공하지 않았고 현재 yaw를 미관측 covariance로 표시한 상태입니다.
`valid`는 계속 위치와 yaw 모두 유효하다는 뜻이며, 미확보 yaw를 유효한 0으로 만들지 않습니다.
Path Planning은 최초 heading 미확보 시에도 위치와 전역 경로가 유효하면 ReferencePath를 제공합니다.

`operation.yaml`의 `heading_bootstrap_enabled=true`일 때, Motion Control은 AUTO에서만
최초 heading 확보용 직진을 허용합니다. 유효 위치·속도·MotionPlan·경로와 정지 요구 없음,
경로 이탈 검사 통과가 모두 필요합니다. 조향은 0, 목표 속도는 MotionPlan을 차량 상한으로 제한합니다.
`control.yaml`의 `heading_bootstrap_timeout_s=3.0`은 최초 AUTO 직진 시작부터 steady-clock으로 셉니다.
IDLE에서 기다리는 시간은 포함하지 않으며, 시작 후 입력 오류나 모드 전환으로 시간을 다시 주지 않습니다.
시간 초과는 정지하고, heading을 얻으면 정상 PID / Pure Pursuit로 전환합니다.
heading을 한 번 얻은 뒤 잃었거나 yaw 품질이 나쁜 경우에는 직진 유예를 주지 않습니다.
PID를 선택했으면 PID 활성화와 유효 게인 설정도 필요합니다.

`control.yaml`의 `max_cross_track_m=3.0`은 현재 위치에서 경로 선분까지의 최소 거리입니다.
3 m 초과 시 PID / Pure Pursuit와 초기 직진 모두 `PATH_DEVIATION_EXCEEDED`로 정지합니다.
미션 활성화와 독립적이며 MANUAL 명령에는 적용하지 않습니다.
GNSS는 최초 heading 전에는 레버암 회전 보정을 할 수 없어 초기 odom 위치가 안테나 위치입니다.
초기 직진의 거리 검사는 그 입력 위치를 사용하며, heading 확보 후 GNSS의 후륜축 보정 위치를 사용합니다.
진행 위치 기억과 종점 정지 유지 기능은 아직 추가하지 않았습니다.

`test/heading_bootstrap_smoke_test.py`는 외부 GNSS/하드웨어 없이 실제 Sensor Interface부터
최종 보호까지 연결하여 초기 직진·시간 초과·heading 전환/소실·위치 품질·경로 이탈 경계를 검증합니다.

### Tracking parameters

모드와 PID 활성화는 `config/operation.yaml`, 계산 파라미터는 `config/control.yaml`의
`/control/motion_control`에 있습니다. 휠베이스는 `config/vehicle.yaml`을 사용합니다.

| Parameter | 기본값 | 의미 / 단위 |
|---|---:|---|
| tracking_mode | PURE_PURSUIT | 테스트 시작 설정, config 변경 후 재시작 |
| pid.enabled | false | tuning 후 명시적으로 활성화 |
| pid.kp_lateral | 0.0 | rad/m |
| pid.ki_lateral | 0.0 | rad/(m*s) |
| pid.kd_lateral | 0.0 | rad*s/m |
| pid.kp_heading | 0.0 | rad/rad |
| pid.integral_limit_m_s | 0.0 | 적분 상한, m*s |
| pid.max_dt_s | 0.5 | software timeout, s |
| lookahead_min_m | 2.0 | 속도 적응형 lookahead 최소값, m |
| lookahead_max_m | 6.0 | 속도 적응형 lookahead 최대값, m |
| lookahead_gain_s | 1.0 | Ld = clamp(min + gain * 실제 속도, min, max), s |
| goal_tolerance_m | 0.3 | PID / Pure Pursuit 공통 종점 도착 허용 거리, m |
| max_cross_track_m | 3.0 | 경로 이탈 정지 기준, m |
| heading_bootstrap_timeout_s | 3.0 | 최초 heading 확보용 AUTO 직진 제한 시간, s |

PID가 disabled 또는 모든 gain=0이면 PID_NOT_CONFIGURED로 닫습니다.
양의 test fixture gain은 실차 추천값이 아닙니다. 기존 최종 safety의 operating_limits_verified와
steering/speed 범위 검사는 별도로 유지합니다. 실측 운용 한계나 actuator calibration을 추정하지 않았습니다.
기존 `lookahead_min_m/max_m/gain_s`, `goal_tolerance_m`, measured wheelbase 설정은 유지합니다.

## Configuration / 측정값

차량 수치는 `main/config/vehicle.jsonc`을 기준으로 이 패키지의 `vehicle.yaml`에 유지합니다.
현재 설정은 다음 다섯 파일로 나눕니다. Mission 상수와 구간별 정책은 mission.yaml에서 관리합니다.

| 파일 | 책임 |
|---|---|
| `config/pipeline.yaml` | 토픽 연결, map/base frame, map 입력 파일 경로 |
| `config/operation.yaml` | 시작 모드, Mission 외 활성화 설정, 주기·입력 timeout·품질 기준·최종 보호 허용 설정 |
| `config/control.yaml` | PID 게인·적분 상한·dt 기준, Pure Pursuit lookahead, 공통 도착 허용 거리 |
| `config/mission.yaml` | Mission 노드·구간 정책 활성화, 공통 운용속도, 구간별 제한·마진·정지·불확실성 상수 |
| `config/vehicle.yaml` | 차량 치수와 센서 장착 수치, 조향각·속도 명령 범위 |

launch는 각 노드에 vehicle → pipeline → mission → operation 순서로 설정을 로드합니다.
Motion Control에는 mission 다음에 control.yaml을 추가로 로드합니다.
node-specific parameter key는 `/control/<node name>`입니다.
기존 통합 설정은 `config/legacy/control_manager.yaml`의 legacy snapshot으로 보존하며,
기본 launch와 현재 smoke fixture는 사용하지 않습니다. 앞으로의 설정 변경은 분리한 파일에 반영합니다.
기존 `params_file:=...` 인자는 `operation_file`의 호환 alias로 유지합니다.
두 인자를 모두 지정하면 명시적인 `operation_file`이 사용됩니다.

```bash
ros2 launch control_manager control_manager.xml \
  pipeline_file:=/absolute/path/pipeline.yaml \
  mission_file:=/absolute/path/mission.yaml \
  control_file:=/absolute/path/control.yaml \
  operation_file:=/absolute/path/operation.yaml

ros2 run control_manager global_state_estimator --ros-args -r __ns:=/control \
  --params-file /absolute/path/vehicle.yaml \
  --params-file /absolute/path/pipeline.yaml \
  --params-file /absolute/path/mission.yaml \
  --params-file /absolute/path/operation.yaml
```

Motion Control을 직접 실행할 때는 `--params-file /absolute/path/control.yaml`도 지정합니다.
제어 계산값을 바꾼 뒤에는 Motion Control을 재시작합니다.

Vehicle Control mode는 시작값이며 실행 중 전환은 기존 mode_request topic을 사용합니다.
Guidance/Tracking, Pose/Motion source와 센서 활성화 등 read-only parameter 변경은 재시작이 필요합니다.
활성화 key는 각 파일의 공통 영역에서 한 번 정의하고 consumer가 함께 사용합니다.
Mission 활성화는 mission.yaml, 다른 센서/Object 활성화는 operation.yaml에서 설정합니다.
기본 operation은 GNSS만 활성입니다. RGB/LiDAR/IMU/Depth/Lane/encoder와 보조 feedback/Object/Mission은 비활성입니다.
원시 센서·Lane relay는 입력과 대응 출력을 함께 끄며, vehicle_feedback은 별도 발행 스위치입니다.
Lane/Object/Mission 비활성 node는 관측 구독·출력·timer를 만들지 않습니다.
필수 입력 비활성화와 선택 모드가 모순되면 시작 오류이며, GNSS_IMU/WHEEL/encoder 미구현은 그대로입니다.
외부 Lane 연결은 lane_enabled=true와 `pipeline.yaml`의 `topics.external_lane`을 함께 지정합니다.
Motion Planning은 object_state_enabled/mission_limit_enabled가 true인 입력만 구독하고
기존 validity/timeout/stop 정책을 적용합니다. false인 입력은 가짜 valid 관측 대신 해당 의존성을 제외하고
MotionPlan.reason에 OBJECT_DISABLED/MISSION_DISABLED를 표시합니다. Mission 비활성 시 공통 운용속도를 사용합니다.
기본 운용속도는 mission.yaml의 cruise_speed_kph를 따릅니다. PID disabled, IDLE, dry_run 및 최종 actuator 보호는 유지합니다.
YAML은 연결 설정을 제공하며 실제 노드 기동은 launch, 구독·발행 생성은 C++가 담당합니다.

`test/operation_smoke_test.py`는 구독·발행 graph, 필수 입력/모드 모순, Object/Mission 네 조합의
실제 planning 결과·활성 입력 만료/정지, Guidance 세 시작 모드의 구독 및 실행 중 변경 거부를 검증합니다.
기존 smoke와 같은 ROS_DOMAIN_ID=179 / localhost / dds_local.xml 환경에서 실행합니다.

| 항목 | 값 (m) | 출처 |
|---|---:|---|
| length / width / height | 2.222 / 1.280 / 1.3005 | 사용자 실측 |
| rear axle / front axle from rear | 0.225 / 1.553 | 사용자 실측 |
| wheelbase / track width | 1.328 / 1.060 | 사용자 실측 |
| LiDAR rear-axle x / z | 0.559 / 1.224 | 실측 x=0.784−0.225 |
| Camera rear-axle x / z | 0.636 / 1.140 | 실측 x=0.861−0.225 |
| LiDAR / Camera y | 0.0 | **중심선 가정, 미실측** |

현재 계산에 쓰는 geometry는 wheelbase뿐입니다. 나머지는 향후 footprint/extrinsic의 문서화된
config이며 TF나 perception 보정에 사용하지 않습니다.

차량 명령 범위는 `vehicle.limits` 한 곳에 둡니다. 사용자 지정 범위는 조향 -30~+30 deg,
속도 -20~+20 km/h이며 내부에서는 rad / m/s로 변환합니다. 음수 속도는 후진 명령입니다.
Motion Control은 AUTO 조향각과 목표 속도를 차량 범위로 제한하고, 공통 후처리에서 제한 시 reason에 `_LIMITED`를 붙입니다.
PID / Pure Pursuit 자동 추종은 전진 전용이므로 음수 MotionPlan 목표는 계속 거부합니다.
Vehicle Interface는 AUTO / MANUAL 공통으로 양·음수 범위를 검사하며, 범위 밖 명령은 clamp하지 않고 정지합니다.
`operating_limits_verified=true`는 지정한 차량 범위 사용을 허용하며 false이면 최종 명령을 차단합니다.
차량 범위가 없거나 유한하지 않거나 0을 포함하지 않는 범위이면 노드는 시작 오류입니다.

**미확정**: wheel-angle/actuator calibration,
GNSS/IMU xyz/RPY, camera/LiDAR rotation, motor raw↔m/s, brake physical model.
lookahead, timeout, covariance quality gate는 소프트웨어 초기 tuning이며 실측 사실이 아닙니다.

## Build / launch / test

컨테이너의 `/workspace`는 호스트 `main/`입니다. 아래 명령은 `main/build`, `main/install`,
`main/log`를 사용하지 않습니다. 빌드·런타임 로그와 임시 파일도 package 내부에 둡니다.

```bash
cd /workspace
source /opt/ros/humble/setup.bash
mkdir -p src/control_manager/log/{tmp,ros,colcon_home}
export TMPDIR=/workspace/src/control_manager/log/tmp
export ROS_LOG_DIR=/workspace/src/control_manager/log/ros
export ROS_HOME=/workspace/src/control_manager/log/ros
export COLCON_HOME=/workspace/src/control_manager/log/colcon_home
export PYTHONDONTWRITEBYTECODE=1
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base src/control_manager/log build \
  --base-paths src/control_manager --packages-select control_manager \
  --build-base src/control_manager/build --install-base src/control_manager/install \
  --symlink-install --cmake-args -DBUILD_TESTING=ON
source src/control_manager/install/setup.bash
colcon --log-base src/control_manager/log test \
  --base-paths src/control_manager --packages-select control_manager \
  --build-base src/control_manager/build --install-base src/control_manager/install
colcon --log-base src/control_manager/log test-result --test-result-base src/control_manager/build --verbose
ros2 launch control_manager control_manager.xml
```

하드웨어 없는 검증은 기존 실행 중인 ROS 시스템과 분리된 ROS_DOMAIN_ID / localhost 환경에서 수행합니다.
`test/smoke_test.py`는 10-node launch의 기본 invalid stop, 생성된 msg, topic 연결,
GNSS 입력/stale, 계획/Pure Pursuit 연결, 모드별 선택, E-STOP, 범위/timeout을 검증합니다.
테스트 fixture의 센서/명령/limits는 **테스트 전용**이며 runtime perception이 가짜 관측을 생성하지 않습니다.
실행 절차 및 결과는 아래 검증 기록을 참고합니다.

## 기존 구조와 충돌 / 팀 협의 필요

- 기존 `guidance_node`와 `trans`에 별도 UDP 경로가 있고 trans의 TEST_MODE 기본값도 false입니다.
  신규 vehicle_interface를 미래 단일 actuator owner로 전환하기 전에 기존 sender 실행 정책을 팀과 합의해야 합니다.
  새 launch는 기존 sender를 시작하지도 중지하지도 않습니다.
- gnss_tools 내부 정책은 guidance의 UDP 소유와 custom msg 금지를 규정합니다.
  이 신규 package는 요청한 정책대로 custom msg와 별도 actuator ownership 경계를 설계했습니다.
  외부 AGENTS/코드/설정은 변경하지 않았습니다.
- MANUAL은 명령 유효성/timeout과 actuator 보호를 적용하며 localization/object 의미 검사는 하지 않습니다.
  해당 자동주행 조건은 planning/control에서만 검사합니다.
- 기존 wheelbase 기본 1.04 m 등은 이 패키지에 복사하지 않고 사용자 실측 1.328 m를 사용합니다.
  GNSS lever arm 수치와 LiDAR x=0.559 m를 혼동해서 적용하지 않습니다.
- manual_control의 `cmd` Float32MultiArray, keyboard_control의 steering_cmd/speed_cmd는
  단위 및 제동 계약이 다릅니다. 확인된 calibration을 갖춘 adapter가 필요합니다.
- guidance raw speed와 trans/펌웨어 raw 범위는 m/s 계약이 아닙니다. raw conversion은 미구현입니다.
- dashboard final-map JSON은 내부 파일 제공자로 연결했습니다. legacy guidance CSV 입력은 지원하지 않으며,
  차선/과제 구간의 planning 정책은 아직 구현하지 않았습니다. 외부 파일은 변경하지 않습니다.

## 미구현 목록

실차 lateral PID tuning, LANE guidance, HYBRID lane correction 및 fallback,
차량 제어 RQT UI, 센서 health 통합 status,
실제 traffic light/sign recognition, LiDAR detection/clustering, RGB lane extraction/correction,
Depth 보조, GNSS+IMU fusion/EKF, 복합 mission 진행/재출발 규칙, obstacle avoidance,
vehicle speed PID, physical brake model, actuator calibration/raw conversion, UDP sender,
실제 keyboard/manual adapter, hardware E-STOP/watchdog, 실차 검증.

## LanePose 계약 변경 검증 (2026-10-15, 이전 작업)

- 최종 상태 메시지: **GlobalPose / LanePose / MotionState**.
- `External Lane AI → sensor_interface → lane_observation → lane_state_estimator → lane_pose → path_planning`.
  observation과 pose의 타입은 모두 `control_manager/msg/LanePose`입니다.
- C++ type/custom msg/생성 include/CMake/YAML/테스트/README·문서·diagram의 이름을 정리했습니다.
  종오차 대신 외부에서 받은 `heading_error_rad`를 사용하며 곡률/polyline/geometry 필드는 없습니다.
- package 내부 산출물로 clean configure/build 성공. 기존 생성 타입도 제거한 뒤 메시지 11개를 재생성했습니다.
  executable 11개와 XML/config 경로, `global_state_estimator` / `lane_state_estimator` 명칭 일치를 확인했습니다.
- **GTest 34개 + CTest 4개: 38 tests / 0 errors / 0 failures / 0 skipped**.
- **5개 smoke suite, 총 308개 확인 항목 통과**: ingress 49, estimator modes 54,
  state split 78, pipeline 68, tracking 59.
- 추가 검증: 정확한 LanePose 필드 구성, observation/final topic type, 양수·음수 heading rad 보존,
  Header/측정시각 보존. 기존 NaN/Inf/confidence/time 거부와 planner timeout도 통과했습니다.
- Lane/Hybrid 알고리즘은 계속 미구현이며 GNSS estimator, PID/PP, motion/safety/UDP 계산과
  vehicle config, launch는 이번 변경 전 source snapshot과 동일합니다. dry_run 및 PID 안전 기본값을 유지합니다.
- 결과 로그: `log/lane_pose_*_smoke_result.txt`, `build/control_manager/test_results/`.
  이번 변경 목록: `log/lane_pose_changed_files.txt`; 시작 시점 소스: `log/lane_pose_before_sources.json`.
- 외부 39,697개 항목의 mode/size/mtime_ns/SHA256/symlink target 및 외부 Git 상태·diff가 시작 시점과 동일합니다.
  `log/guidance_audit/lane_pose_before.json` / `lane_pose_after.json` 비교 결과:
  **Modified files outside control_manager: 0**. 기존 외부 변경 및 과거 미복구 symlink 상태는 보존했습니다.

## Estimator 통합 검증 (이전 작업)

최신 architecture와 mode matrix/검증 결과는 [Estimator 보고서](doc/estimators.md)에 기록합니다.

## Ingress 변경 검증 (이전 작업)

현재 변경 파일·테스트·외부 scope 비교는 [Ingress 보고서](doc/sensor_ingress.md)에 기록합니다.

## 상태 분리 검증 (0.3.0, 이전 작업)

검증 결과와 변경 파일 전체 목록은 [상태 분리 보고서](doc/state_split.md)에 기록합니다.
이 버전의 smoke는 새 메시지 구조를 사용합니다. 기존 검증 기록의 구 버전 node/message 수는 아래에 보존합니다.

## 이전 버전 검증 기록: Tracking 추가 (2026-10-15)

- ROS 2 Humble package-only configure/build 성공. node 10개, custom msg 9개.
- **GTest 26개 통과** (기존 core 8 + Guidance 4 + 신규 Tracking/PID 14).
  CTest 포함 **29 tests / 0 errors / 0 failures / 0 skipped**.
- 기존 smoke **65개 확인 항목** 통과: 기존 PP/Guidance fixture는 PURE_PURSUIT를 명시하여 회귀 검증.
- 신규 Tracking smoke **48개 확인 항목** 통과: default PID 미설정 차단, configured PID, I/clamp/reset,
  PP steering 수치, mode 변경, unsupported/invalid 요청, MANUAL 독립성, stale/NaN/frame/path/stop 처리.
- config의 PID gain/한계는 placeholder로 유지하며 양의 값은 hardware-free test fixture에서만 사용했습니다.
- 기존 `src/pure_pursuit.cxx` 알고리즘 소스는 수정하지 않았습니다.
- 로그: `log/tracking_legacy_smoke_result.txt`, `log/tracking_smoke_result.txt`,
  `build/control_manager/test_results/`.
- 빌드/테스트 산출물은 package 내부에만 두었으며 scope 비교 기록은
  `log/guidance_audit/tracking_before.json`, `tracking_after.json`입니다.

## 이전 버전 검증 기록: Guidance 추가 (2026-10-14)

- Humble isolated configure/build 성공. GuidanceStatus C++/Python 생성 및 기존 10 executable 유지.
- GTest 12개 (기존 8 + Guidance 4), CTest 포함 **14 tests / 0 errors / 0 failures / 0 skipped**.
- 확대된 smoke **63개 확인 항목 통과**: 요청/actual 구분, GPS_ONLY의 lane 무시,
  LANE_ONLY/HYBRID/잘못된 요청의 AUTO 정지, MANUAL 독립성, empty/wrong-frame path,
  stale/future/zero GNSS timestamp, 기존 E-STOP 및 safety 검증.
- localhost DDS/domain 179 테스트 fixture만 사용했으며 실제 actuator UDP는 없습니다.
- 결과: `log/guidance_smoke_result.txt`, `build/control_manager/test_results/`.
- 외부 변경 검사는 `log/guidance_audit/`의 before/after metadata·SHA256·Git 기록으로 비교합니다.
  이전 작업의 unresolved symlink 문제는 아래 기록에 유지하며 이번 작업에서 복구하지 않습니다.

## 최초 skeleton 검증 기록 (이전 작업)

2026-10-13, 기존 `main` 컨테이너의 ROS 2 Humble에서 검증했습니다.

- isolated configure/build 성공, 최종 compile error/warning 없음.
- 10개 installed executable, custom msg 7개 Python/C++ 생성, XML/package/config 경로 확인.
- `colcon test-result`: **9 tests, 0 errors, 0 failures, 0 skipped** (CTest 집계 포함).
- hardware-free smoke: 기본 10-node launch 및 25개 확인 항목 통과.
  GNSS relay→Path→mission→MotionPlan→Pure Pursuit 흐름, AUTO/MANUAL 분리,
  E-STOP, command/GNSS/object timeout, range/NaN 거부, 미구현 설정 시작 거부 확인.
- 액추에이터 UDP/하드웨어 접속 시험 없음. 테스트의 DDS는 localhost 전용 domain 179.

설정한 환경 변수와 install을 source한 셸에서 smoke 재실행:

```bash
export ROS_DOMAIN_ID=179
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/src/control_manager/test/dds_local.xml
python3 -B src/control_manager/test/smoke_test.py
python3 -B src/control_manager/test/tracking_smoke_test.py
python3 -B src/control_manager/test/state_split_smoke_test.py
python3 -B src/control_manager/test/ingress_smoke_test.py
python3 -B src/control_manager/test/estimator_modes_smoke_test.py
python3 -B src/control_manager/test/command_path_smoke_test.py
```

결과 로그: package 내부 `log/smoke_result.txt`, `log/smoke_default.log`,
`log/smoke_integration.log`, `build/control_manager/test_results/`.

### WRITE SCOPE 검증 미충족 사항

구현 소스와 build/install은 이 패키지 내부에 있습니다. 다만 작업 중 초기
`colcon test-result` 두 호출에 `--log-base`를 누락하여 **외부 로그 변경이 발생했습니다**:

- `main/log/test-result_2026-10-13_02-38-36/logger_all.log` 생성
- `main/log/test-result_2026-10-13_02-39-51/logger_all.log` 생성
- 기존 `main/log/COLCON_IGNORE` (빈 파일)의 수정 시각 갱신
- `main/log/latest`, `main/log/latest_test-result` 링크 갱신 (기존 대상은 기록하지 못함)

사용자의 “알려진 변경만 복구 허용” 승인에 따라 위 로그 디렉터리 2개를 삭제하고,
`COLCON_IGNORE`의 기록된 수정 시각을 복원했습니다. 기존 파일 내용은 변경하지 않았습니다.
`latest`, `latest_test-result`는 원래 대상을 기록하지 못했으므로 승인 범위에 따라 그대로
두었습니다. 현재 두 링크는 삭제된 로그 디렉터리를 가리키는 dangling link입니다.
이 두 외부 링크 변경이 남아 있으므로 코드·기능 검증 통과와 별개로
**원래의 전체 작업 완료 조건은 아직 미충족**입니다.
위 build/test 문서는 모든 colcon 호출에 package 내부 `--log-base`를 명시하도록 수정했습니다.
외부 package의 기존 Git 변경사항은 보존되었습니다.
