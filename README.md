# control_manager

ROS 2 Humble / C++17 / ament_cmake, `.cxx` / `.hxx`, XML launch 기반 상위제어 skeleton입니다.
**실차 주행 준비 완료를 뜻하지 않습니다.** 기본 IDLE, PID disabled/zero gains, 미확정 운용 한계는 0,
`dry_run=true`를 유지합니다. UDP backend가 없으며 `dry_run=false`는 시작 오류입니다.

## 현재 architecture (0.3.0: 상태 분리)

이전 VehicleState에 섞여 있던 global localization과 motion validity를 독립 메시지/estimator로 분리했습니다.
기존에는 planner, motion planning, safety가 모두 같은 localization 조건에 묶여 No-GNSS LANE을
표현하기 어려웠습니다. 현재 LANE 입력 검사는 GNSS/지도 없이 가능하지만 실제 Lane 경로 생성은 미구현입니다.

```text
Raw GNSS / optional IMU / RGB / LiDAR / optional Depth
                         |
                 sensor_interface
                  |             +--> object_manager --> ObjectState
                  +-- GNSS --> gnss_state_estimator [GNSS / GNSS_IMU TODO]
                  |                         +--> GlobalPose
                  +-- GNSS --> motion_state_estimator [GNSS / WHEEL TODO]
                                            +--> MotionState
Future actual wheel speed + steering feedback --(contract TODO)--> WHEEL input

GlobalPose + GlobalPath + external LaneState (direct, no sensor relay)
                 |
       path_planning [GPS / LANE / HYBRID: per-mode requirements]
                 |
           ReferencePath
                 +-- MotionState + ObjectState + MissionLimit
                 v
          motion_planning (NO GlobalPose subscription)
                 |
             MotionPlan + MotionState + GlobalPose*
                 v
          motion_control [PID / legacy PP; algorithms preserved]
                 |
              auto_cmd + manual_cmd
                 v
          control_manager [IDLE / MANUAL / AUTO; FSM]
                 |
             selected_cmd
                 v
          safety_limit [MotionState always; GlobalPose mode-dependent;
                        GuidanceStatus + ObjectState + E-STOP]
                 |
              final_cmd --> vehicle_interface [dry_run only]

* Existing PID/PP still require map pose. Local Lane tracking-frame adaptation is a blocker.
```

## Node별 subscribe / publish

표의 제어 topic은 `/control/` 접두사입니다. 원본 topic/config default는 `ros_support.hxx`와 YAML에 모았습니다.
기존 외부 node는 launch하지 않습니다. launch는 아래 **11개 executable**만 시작합니다.

| Executable | Subscribe | Publish / 책임 |
|---|---|---|
| sensor_interface_node | 원본 GNSS/IMU/RGB/LiDAR/Depth | sensors/*; 원본 stamp/frame/covariance 보존, IMU/Depth optional |
| gnss_state_estimator_node | sensors/gnss_odom; GNSS_IMU 선택 시 sensors/imu | global_pose; 위치·yaw·uncertainty 검증 |
| motion_state_estimator_node | GNSS mode: sensors/gnss_odom | motion_state; 속도 검증, WHEEL은 invalid placeholder |
| path_planning_node | global_pose, global_path, lane_state, guidance_mode_request | reference_path, guidance_status; Guidance owner |
| object_manager_node | sensors/rgb, sensors/lidar, optional sensors/depth | object_state; detection 없음, UNKNOWN/invalid |
| mission_based_limit_node | object_state | mission_limit; opt-in speed cap/신호 정책 |
| motion_planning_node | reference_path, motion_state, object_state, mission_limit | motion_plan; 경로와 제한 속도, obstacle stop |
| motion_control_node | global_pose, motion_state, motion_plan, status, tracking_mode_request | auto_cmd, tracking_status; PID/PP, speed pass-through |
| control_manager_node | auto_cmd, manual_cmd, final_cmd, mode_request | selected_cmd, status; arbitration/FSM |
| safety_limit_node | global_pose, motion_state, guidance_status, selected_cmd, object_state, estop | final_cmd; 최종 위험/시스템 제한 |
| vehicle_interface_node | final_cmd | 로그/timeout 감시; actuator 송신 없음 |

## Message / 시간 계약

- **GlobalPose** (`/control/global_pose`): Header, `x_m, y_m, yaw_rad`,
  `position_variance_x_m2, position_variance_y_m2, yaw_variance_rad2`, `valid`, `reason`.
  map 기준 rear-axle pose 계약. uncertainty는 GNSS marginal variance이며 full fusion covariance가 아닙니다.
- **MotionState** (`/control/motion_state`): Header, `speed_mps, yaw_rate_radps`,
  `speed_valid, yaw_rate_valid`, `reason`. body-frame longitudinal speed와 motion validity를 분리합니다.
  GNSS speed는 기존 `Odometry.twist.twist.linear.x`를 그대로 사용합니다. yaw-rate는 항상 invalid입니다.
- **LaneState** (`/control/lane_state`): Header, `lateral_error_m`, `longitudinal_error_m`, `confidence`.
  **타 팀이 제공할 네 정보는 횡오차·종오차·confidence·측정 timestamp(header.stamp)**입니다.
  기존 `heading_error_rad`와 `valid` 필드는 제거했습니다. heading/곡률/polyline은 가정하지 않습니다.
  오차 부호·기준점·frame의 구체적 의미는 외부 팀과 합의가 필요합니다. 현재는 경로 변환에 사용하지 않습니다.
  finite 값, confidence 범위 [0,1]와 `lane_min_confidence`, source/receipt freshness로 내부 가용성을 판단합니다.
- **ObjectState**: traffic light/sign/obstacle 상태. `obstacle_valid=false`는 장애물 없음이 아닙니다.
  현재 perception은 미구현이며 invalid 관측만 발행합니다. LiDAR가 obstacle primary, Depth는 optional 보조입니다.
- **ReferencePath / GlobalPath**: 표준 `nav_msgs/Path`. global_path는 검증된 외부 정적 map snapshot;
  reliable transient-local depth 1이며 시간 만료 대신 datum/frame/finite/비퇴화 경로를 검사합니다.
  reference_path는 계획 생성 시각을 사용하며 invalid이면 빈 경로입니다.
- **MotionPlan**: Header, Path, target_speed_mps, stop_request, valid, reason.
  motion planning은 map 또는 base_frame Path를 전달할 수 있고 Path frame을 plan에 보존합니다.
  비기하학적 MissionLimit의 기존 map Header 계약은 유지합니다.
- **VehicleCommand**: 좌측 양수 `target_steer_rad`, 전진 `target_speed_mps`, bool brake, valid, reason.
  내부 물리 단위이며 motor raw/normalized joystick/실제 feedback과 다릅니다.
- **GuidanceStatus**: requested_mode / actual_mode / active_source / valid / degraded / reason.
  0=GPS, 1=LANE, 2=HYBRID, actual NONE=255. 미구현이면 actual=NONE, valid=false, degraded=false.
- **ControlStatus / TrackingStatus**: 기존 모드/FSM과 Tracking 요청·실제 상태 계약 유지.

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
| Vehicle Control | IDLE / MANUAL / AUTO | control_manager |
| Guidance | GPS / LANE / HYBRID | path_planning |
| Global estimation | GNSS / GNSS_IMU | gnss_state_estimator |
| Motion estimation | GNSS / WHEEL | motion_state_estimator |
| Tracking | PID / PURE_PURSUIT / ADAPTIVE_PURE_PURSUIT | motion_control |

### Estimator 동작

`gnss_state_estimator` 기본 `estimation_mode=GNSS`:
원래 position/yaw 추출 및 quaternion norm, parent map/child base_link, timestamp,
finite XY, position variance 각각 및 합, yaw variance, `localization_reference_verified` 검사를 보존했습니다.
미확정 GNSS lever arm을 valid로 가정하지 않도록 reference_verified 기본값은 false입니다.
`GNSS_IMU`는 IMU 입력 자리만 두고 **GNSS_IMU_NOT_IMPLEMENTED** invalid heartbeat를 발행합니다.
EKF/가짜 fusion이나 GNSS fallback은 없습니다. 잘못된 mode 문자열은 시작 오류입니다.

`motion_state_estimator` 기본 `estimation_mode=GNSS`:
Odometry twist speed, speed variance, frame/child-frame, finite, source/receipt age를 검사합니다.
위치 covariance가 나쁘더라도 유효 speed는 독립적으로 사용 가능합니다. GNSS yaw-rate는 계산하지 않습니다.
`WHEEL`은 **WHEEL_FEEDBACK_CONTRACT_NOT_IMPLEMENTED**, speed/yaw-rate invalid, stamp=0입니다.
actual wheel/motor speed와 실제 steering angle의 topic/type/calibration/time 계약이 없으므로
외부 계약을 만들지 않았습니다. 입력 접속 위치는 `MotionStateEstimatorNode` 내부 TODO입니다.
향후 검증된 입력에 `v/L*tan(delta)`를 적용할 수 있으며 L은 `vehicle.yaml`에서 읽습니다.
현재 WHEEL 분기에서는 이 수식을 계산하지 않습니다. standalone에서 wheelbase 미설정은 0=unavailable입니다.

### Guidance required input

| Guidance | GlobalPose | GlobalPath | LaneState | 현재 실제 결과 |
|---|---|---|---|---|
| GPS | 필수 | 필수 | 무시 | 검증 후 global Path pass-through |
| LANE | 불필요 | 불필요 | 필수 | Lane 검사 후 LANE_NOT_IMPLEMENTED, empty Path |
| HYBRID | 필수 | 필수 | 필수 | 세 입력 검사 후 HYBRID_NOT_IMPLEMENTED, empty Path |

GPS/HYBRID는 GlobalPose → GlobalPath 순으로 결손을 보고합니다. LANE은 이 두 조건을 검사하지 않습니다.
LANE/HYBRID Lane 오류: `LANE_STATE_UNAVAILABLE`, `LANE_VALUES_INVALID`, `LANE_CONFIDENCE_LOW`.
Lane이 정상이어도 현재 네 정보만으로 검증된 ReferencePath를 만들 수 없어 미구현 상태로 닫습니다.
GPS로 silent fallback하거나 fake lane correction을 만들지 않습니다. HYBRID는 GPS+Lane이며 IMU fusion과 다릅니다.
`guidance_mode_request`는 UInt8이며 잘못된 값은 보존/INVALID_GUIDANCE_MODE; 초기 잘못된 문자열은 시작 오류입니다.
Status 및 Path는 기본 20 Hz 생성 시각으로 발행합니다. GUI는 request source이며 safety authority가 아닙니다.

### Downstream / safety / architecture blocker

Motion planning은 **GlobalPose를 구독하지 않습니다**. ReferencePath + 유효 MotionState + MissionLimit + ObjectState로
계획합니다. mission_based_limit는 정상 주행 속도/신호 정책이고, safety_limit는 선택 명령 이후 최종 제한입니다.
Obstacle detected이면 motion planning은 회피 대신 정지합니다. speed PID는 없습니다.

**현재 PID/PP는 map pose와 map 경로가 필요합니다.** local/base_link ReferencePath가 오면
`LOCAL_REFERENCE_PATH_NOT_SUPPORTED`로 명시적으로 차단합니다. 0 pose/identity TF로 우회하지 않습니다.
LANE planner의 No-GNSS dependency 분리는 완료됐지만 No-GNSS end-to-end 주행은 아직 구현되지 않았습니다.
향후 local reference frame/시간 기준과 controller tracking-frame adapter가 합의되어야 합니다.

Safety는 MotionState speed validity/freshness/finite를 모든 모드에서 확인합니다.
planner의 **신선한 GuidanceStatus.requested_mode=LANE**일 때 GlobalPose를 요구하지 않습니다.
actual=NONE이어도 LANE 요청의 dependency 정책은 유지됩니다. 미구현 LANE의 AUTO 정지는 empty Path →
invalid MotionPlan → invalid AutoCommand로 처리합니다. invalid guidance가 MANUAL source를 대체하지 않습니다.
GPS/HYBRID 및 status missing/stale/unknown일 때는 보수적으로 GlobalPose를 요구합니다.
이 정책은 MANUAL에도 적용됩니다. Guidance 전환은 callback/timer/timeout 단위이며 원자적 전환 보장은 아닙니다.

기존 E-STOP, command timeout, object freshness, invalid/NaN command, 범위 초과, 미확정 한계 차단은 유지합니다.
`require_obstacle_state=true` 기본값이며 최종 범위 초과는 clamp 대신 stop입니다. E-STOP true는 explicit false까지 유지합니다.
물리 E-STOP/재시작 latch/인증/하드웨어 watchdog은 미구현입니다. 안전 범위 및 PID gain은 모두 기존 값 그대로입니다.

AUTO는 auto_cmd만, MANUAL은 manual_cmd만 선택합니다. invalid ControlMode 요청은 IDLE입니다.
FSM INIT/READY/DRIVING/STOPPED/FAULT는 그대로이며 DRIVING은 명령 단계이지 actual motion feedback이 아닙니다.
RGB는 traffic light/sign/Lane 입력용이며 실제 recognition은 없습니다. IMU/Depth는 config opt-in이고 runtime toggle은 없습니다.
Pure Pursuit는 GNSS 패키지 책임이 아니라 Motion Control 내부 lateral controller입니다.

## Breaking changes / 다음 구현 경계

- `VehicleState.msg`, `/control/vehicle_state`, `state_estimator_node` 제거. 별도 compatibility bridge 없음.
- `gnss_state_estimator_node` 및 `/control/gnss_state_estimator` YAML로 rename;
  `motion_state_estimator_node` 추가. 두 메시지 소비자는 새 계약으로 이행해야 합니다.
- Guidance 문자열/constants GPS_ONLY/LANE_ONLY → GPS/LANE (wire 값 0/1/2 유지).
- LaneState heading_error/valid 제거, longitudinal_error 추가. 외부 Lane 팀과 빌드/계약 동기화 필요.
- Localization GNSS_ONLY/fusion_enable → estimation_mode=GNSS/GNSS_IMU.
- 외부 launch/GUI/센서/펌웨어는 수정하지 않았습니다. 이름 변경 반영은 외부 담당 팀 작업입니다.
- 우선순위: (1) 실제 wheel speed/steering 및 Lane 오차 기준 계약, (2) local-path frame/controller adapter,
  (3) 검증된 Lane/Hybrid path algorithm, (4) 검증된 fusion 및 실차 튜닝/actuator 계약.

## Tracking Controller (2026-10-15)

11-node 구조에서 `motion_control_node`만 Tracking 선택과 ROS I/O를 소유합니다.
Guidance(GPS/LANE/HYBRID), Vehicle Control(IDLE/MANUAL/AUTO), Localization은 별개입니다.
`GlobalPose.x_m/y_m/yaw_rad`, `MotionState.speed_mps` 및 각각의 stamp/validity와
`MotionPlan.reference_path`의 `poses[].pose.position.x/y`, frame/stamp를 그대로 사용합니다.
Path의 quaternion heading은 사용하지 않고 가장 가까운 유효 선분의 tangent를 계산합니다.
`MotionPlan.valid/stop_request/target_speed_mps`와 AutoCommand의 VehicleCommand 계약은 유지합니다.

| Tracking request | 구현 / 실제 동작 |
|---|---|
| PID (0, 기본) | GNSS 경로 lateral PID + heading P. 기본 tuning은 비활성 placeholder |
| PURE_PURSUIT (1) | 기존 speed-adaptive Pure Pursuit를 그대로 호출 |
| ADAPTIVE_PURE_PURSUIT (2) | 추가 lateral/heading-error 적응형은 NOT_IMPLEMENTED; invalid stop |
| 나머지 값 | 요청 값 보존, actual=NONE, INVALID_TRACKING_MODE; fallback 없음 |

`TrackingStatus`는 GuidanceStatus와 동일한 `requested_mode`, `actual_mode`, `valid`, `reason` naming을
사용합니다. NONE=255이며 actual은 **이번 steering 계산에 성공한 controller**입니다.
단순 PLAN_STOP은 controller를 계산하지 않으므로 actual=NONE/valid=false/PLAN_STOP이지만
AutoCommand는 기존 정책대로 유효한 speed=0/brake=true 정지 명령입니다. goal 판정에 성공한 경우에는
Tracking valid=true/GOAL_REACHED입니다. 상태 유효성은 실제 조향 feedback이나 운행 허가를 의미하지 않습니다.
mode 요청은 reliable volatile UInt8이며 상태는 기존 rate_hz(20 Hz)로 발행합니다.
잘못된 초기 tracking_mode 문자열은 시작 오류입니다. runtime 요청 변경 시 먼저 invalid stop을 발행하고
다음 tick에 새 controller를 계산합니다. 같은 mode 반복 요청은 적분을 지우지 않습니다.

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
PID는 조향만 반환하며 속도는 MotionPlan 값을 전달하고, brake는 node/plan/safety가 결정합니다.

`dt`는 node timer 사이의 steady-clock 실제 경과 시간입니다. 첫 sample/reset 직후에는 I=D=0이며
그 다음부터 누적합니다. integral clamp가 최소 anti-windup입니다. NaN/Inf, 계산 overflow, dt<=0 또는
max_dt_s 초과는 invalid와 reset입니다. forward-only baseline으로 끝점 뒤나 반대 방향 경로는 거부하고,
끝점 도달은 기존 goal_tolerance_m를 사용합니다. loop/교차점 progress 판정은 구현하지 않았습니다.

다음 경우 PID 이력을 reset합니다: Tracking mode 변경, 입력 invalid/stale, stop plan, goal,
경로 점 좌표 변경, 긴 dt. 같은 경로의 stamp 갱신만으로는 reset하지 않습니다.
`/control/status`를 추가 구독하여 IDLE/MANUAL/FAULT/INIT 및 stale/missing status에서는
PID 이력을 지우고 **P + heading P preview만** 계산합니다. 이때 reason=PID_PREVIEW_STATE_RESET입니다.
AUTO가 READY/DRIVING/STOPPED이고 status도 신선할 때만 I/D 이력을 유지합니다.
이 구독은 ControlMode를 바꾸거나 safety 권한을 이전하지 않습니다. FAULT의 원인인 final command를
기다리며 모든 AUTO 출력을 막는 순환 대기 없이 기존 안전 조건이 회복될 수 있도록 preview는 허용합니다.
preview AutoCommand도 실제로 선택할지는 기존 manager/safety가 결정합니다.

GlobalPose/MotionState/MotionPlan은 source stamp와 수신 timeout을 검사하고, 주행 계산에서는 nested
reference_path.header.stamp도 검사합니다. 잘못되거나 만료된 경로로 유효 steering을 만들지 않습니다.
ReferencePath를 주기적으로 생성하는 기존 planner 계약은 유지되며 upstream Global Path는 정적 snapshot입니다.

### Tracking parameters

모든 값은 `/control/motion_control`의 `config/control_manager.yaml`에 있습니다.

| Parameter | 기본값 | 의미 / 단위 |
|---|---:|---|
| tracking_mode | PID | 초기 선택, runtime 전환은 request topic |
| pid.enabled | false | tuning 후 명시적으로 활성화 |
| pid.kp_lateral | 0.0 | rad/m |
| pid.ki_lateral | 0.0 | rad/(m*s) |
| pid.kd_lateral | 0.0 | rad*s/m |
| pid.kp_heading | 0.0 | rad/rad |
| pid.integral_limit_m_s | 0.0 | 적분 상한, m*s |
| pid.max_steering_rad | 0.0 | controller 출력 clamp; 실제 차량 최대 조향각이 아님 |
| pid.max_dt_s | 0.5 | software timeout, s |

PID가 disabled, max_steering_rad=0, 또는 모든 gain=0이면 PID_NOT_CONFIGURED로 닫습니다.
양의 test fixture gain은 실차 추천값이 아닙니다. 기존 최종 safety의 operating_limits_verified와
steering/speed 범위 검사는 별도로 유지합니다. 실측 운용 한계나 actuator calibration을 추정하지 않았습니다.
기존 `lookahead_min_m/max_m/gain_s`, `goal_tolerance_m`, measured wheelbase 설정은 유지합니다.
추가 Adaptive PP는 현재 mode/interface/거부 테스트만 있으며 가짜 오차 적응 gain이나 PP fallback은 없습니다.

## Configuration / 측정값

새 stack의 local source of truth는 이 패키지의 `config/`입니다.
`control_manager.yaml`에 topic/timing/control policy, `vehicle.yaml`에 geometry를 모았습니다.
launch가 두 파일을 함께 로드합니다. node-specific parameter key는 `/control/<node name>`입니다.
standalone `ros2 run`에 파일을 적용할 때도 `-r __ns:=/control`을 사용합니다.

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

**미확정**: actual max steering angle, wheel-angle/actuator calibration, physical max operating speed,
GNSS/IMU xyz/RPY, camera/LiDAR rotation, motor raw↔m/s, brake physical model.
따라서 max_steer/max_speed는 0과 `operating_limits_verified=false`로 비활성화했습니다.
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
`test/smoke_test.py`는 11-node launch의 기본 invalid stop, 생성된 msg, topic 연결,
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
- 기존 teleop 정책은 localization gate를 제외하지만 신규 최종 safety는 MANUAL에도 Guidance별 GlobalPose 조건을 적용합니다.
  실내 수동 주행 예외 정책은 팀 결정 사항이며 여기서 외부 정책을 수정하지 않습니다.
- 기존 wheelbase 기본 1.04 m 등은 이 패키지에 복사하지 않고 사용자 실측 1.328 m를 사용합니다.
  GNSS lever arm 수치와 LiDAR x=0.559 m를 혼동해서 적용하지 않습니다.
- manual_control의 `cmd` Float32MultiArray, keyboard_control의 steering_cmd/speed_cmd는
  단위 및 제동 계약이 다릅니다. 확인된 calibration을 갖춘 adapter가 필요합니다.
- guidance raw speed와 trans/펌웨어 raw 범위는 m/s 계약이 아닙니다. raw conversion은 미구현입니다.
- dashboard final-map JSON과 guidance CSV의 datum/mission/path 계약을 통일해야 합니다.
  map loader는 placeholder이며 외부 파일을 변경하지 않았습니다.

## 미구현 목록

추가 error-adaptive PP, 실차 lateral PID tuning, LANE guidance, HYBRID lane correction 및 fallback,
차량 제어 RQT UI, 센서 health 통합 status,
실제 traffic light/sign recognition, LiDAR detection/clustering, RGB lane extraction/correction,
Depth 보조, GNSS+IMU fusion/EKF, map loader/mission section rules, obstacle avoidance,
vehicle speed PID, physical brake model, actuator calibration/raw conversion, UDP sender,
실제 keyboard/manual adapter, hardware E-STOP/watchdog, 실차 검증.

## 상태 분리 검증 (0.3.0)

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
