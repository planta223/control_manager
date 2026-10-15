# control_manager

ROS 2 Humble / C++17 / ament_cmake 기반 상위제어 **1차 architecture와 실행 가능한 skeleton**입니다.
기존 기본 패키지를 확장했으며 `.cxx` / `.hxx`, XML launch를 사용합니다.
실차 주행 준비가 완료된 패키지가 아닙니다. 기본 실행은 `IDLE`, `dry_run=true`이고,
미확정 조건에서는 `final_cmd`가 속도 0 / brake=true / valid=false로 닫힙니다.
**액추에이터 UDP 구현 자체가 없으며 `dry_run=false`는 시작 오류입니다.**

## Architecture / node 책임

```text
/gnss/odom ───────┐
/imu/data (opt) ─┤
/camera/image_raw│  sensor_interface_node → /control/sensors/*
/lidar/points ───┤       ├── GNSS → state_estimator_node → VehicleState
/depth (opt) ────┘       └── RGB + LiDAR (+Depth) → object_manager_node → ObjectState

external LaneState ─────────────┐
external global_path + VehicleState → path_planning_node → ReferencePath
ObjectState + configured baseline → mission_based_limit_node → MissionLimit
VehicleState + ReferencePath + ObjectState + MissionLimit
  → motion_planning_node → MotionPlan
VehicleState + MotionPlan
  → motion_control_node [PID / legacy Pure Pursuit / additional Adaptive PP TODO] → AutoCommand

ManualCommand ─┐
AutoCommand ───┴→ control_manager_node [ControlMode + arbitration + FSM/status]
  → selected_cmd → safety_limit_node [VehicleState + ObjectState + E-STOP]
  → final_cmd → vehicle_interface_node [DRY RUN ONLY]
```

| Executable | 책임 / 현재 구현 |
|---|---|
| `sensor_interface_node` | 원본 Odometry/Image/PointCloud2/optional Imu relay. stamp/frame/covariance 보존, 추정 TF 없음 |
| `state_estimator_node` | GNSS Odometry를 VehicleState로 변환; frame, covariance, quaternion, timestamp, 속도 validity 확인 |
| `object_manager_node` | RGB / LiDAR / optional Depth 입력 자리, invalid/UNKNOWN ObjectState 발행. detection 없음 |
| `path_planning_node` | Guidance 요청/상태 소유. GPS_ONLY에서 global Path 검증·전달; invalid/미지원 Guidance는 empty Path |
| `mission_based_limit_node` | 명시적으로 활성화한 baseline speed cap, 유효 sign cap, 신호 적용 구간의 GREEN 외 stop |
| `motion_planning_node` | 신선하고 유효한 입력에서 경로 + 제한된 목표 속도 전달. obstacle detected이면 stop. 조향 계산 없음 |
| `motion_control_node` | 독립 Tracking 선택(PID 기본), lateral PID / 기존 PP + m/s pass-through, invalid 입력은 stop |
| `control_manager_node` | AUTO/MANUAL source 선택, 모드 요청, final command 기반 FSM/status |
| `safety_limit_node` | 선택 이후 최종 timeout/validity/range/E-STOP/obstacle 제한 |
| `vehicle_interface_node` | 최종 물리 명령 수신·로그 / final timeout 감시. socket 생성 없음 |

launch는 위 10개 노드만 `/control` namespace로 시작합니다. 기존 센서, guidance, trans,
manual reader를 시작하지 않습니다. 내부 topic 이름과 원본 sensor topic 이름은
`topics.*` parameter이며 기본 문자열은 `ros_support.hxx` 한 곳에 모았습니다.

## Message / topic contract

| Topic (`/control/` prefix) | Type | Publisher → Subscriber |
|---|---|---|
| `sensors/gnss_odom` | `nav_msgs/Odometry` | sensor_interface → state_estimator |
| `sensors/imu` | `sensor_msgs/Imu` | optional relay → future assist |
| `sensors/rgb`, `sensors/depth` | `sensor_msgs/Image` | relay → object_manager / future lane |
| `sensors/lidar` | `sensor_msgs/PointCloud2` | relay → object_manager |
| `vehicle_state` | `control_manager/VehicleState` | estimator → path / motion planning / motion control / safety |
| `lane_state` | `control_manager/LaneState` | **future external publisher** → path planning |
| `object_state` | `control_manager/ObjectState` | object manager → mission / motion planning / safety |
| `global_path` | `nav_msgs/Path` | **external validated map provider** → path planning |
| `reference_path` | `nav_msgs/Path` | path planning → motion planning |
| `mission_limit` | `control_manager/MissionLimit` | mission → motion planning |
| `motion_plan` | `control_manager/MotionPlan` | motion planning → motion control |
| `auto_cmd` | `control_manager/VehicleCommand` | motion control → manager |
| `manual_cmd` | `control_manager/VehicleCommand` | **future physical-unit adapter** → manager |
| `selected_cmd` | `control_manager/VehicleCommand` | manager → safety |
| `final_cmd` | `control_manager/VehicleCommand` | safety → vehicle interface / manager status |
| `status` | `control_manager/ControlStatus` | manager → future GUI (기존 monitor는 미연결) |
| `mode_request` | `std_msgs/UInt8` | operator → manager; 0=IDLE, 1=MANUAL, 2=AUTO |
| `guidance_mode_request` | `std_msgs/UInt8` | operator → path planning; 0=GPS_ONLY, 1=LANE_ONLY, 2=HYBRID |
| `guidance_status` | `control_manager/GuidanceStatus` | path planning → future GUI |
| `tracking_mode_request` | `std_msgs/UInt8` | operator → motion control; 0=PID, 1=PURE_PURSUIT, 2=ADAPTIVE_PURE_PURSUIT |
| `tracking_status` | `control_manager/TrackingStatus` | motion control → future GUI |
| `estop` | `std_msgs/Bool` | safety input → safety; true=set, false=explicit reset |

원본 기본값: `/gnss/odom`, `/imu/data`, `/camera/image_raw`,
`/camera_dep/depth/image_raw`, `/lidar/points`. 기존 guidance의 `/localization/odom` remap이
필요하면 `topics.gnss`만 변경합니다. 원본/relay는 SensorDataQoS, 제어 topic은 reliable volatile입니다.
`global_path`는 reliable transient-local depth 1이고 **publisher도 transient-local**이어야 합니다.
`estop`도 reliable transient-local depth 1이므로 **publisher에 같은 QoS**를 지정합니다.
E-STOP true는 explicit false 전까지 유지됩니다. 기본 false는 물리 E-STOP 정상 상태의 증명이 아니며,
현재는 하드웨어 E-STOP, heartbeat, 인증, 재시작 시 latch 복원 기능을 구현하지 않았습니다.

### 상태 분리 / 단위 / 시간

- **VehicleState**: rear axle centre의 ENU `x_m, y_m, yaw_rad, speed_mps, yaw_rate_radps`,
  `localization_valid`, `speed_valid`, `yaw_rate_valid`, `header.stamp`.
  독립 C++ `core::VehicleState`도 제공하며 ROS 변환은 `ros_support.hxx`에 있습니다.
- **LaneState**: `lateral_error_m, heading_error_rad, confidence, valid, header.stamp`.
  양의 lateral은 좌측, heading은 CCW. 도로 상대 상태이며 VehicleState에 섞지 않습니다.
  현재 lane_AI overlay를 숫자 차선 상태로 변환하지 않습니다.
- **ObjectState**: UNKNOWN/RED/YELLOW/GREEN, optional speed limit, obstacle detected/distance,
  전체 및 필드별 validity, timestamp. **false detected + obstacle_valid=false는 장애물 없음이 아닙니다.**
  현재 모든 관측은 invalid이며 timestamp는 unavailable 보고 시각입니다. 미래 detector는 acquisition
  timestamp와 필드별 freshness를 검사해 aggregate해야 합니다. 거리 기준은 차량 외곽까지의 거리로
  계약 확정이 필요합니다. 단일 공통 stamp 한계 때문에 비동기 detector 통합 때 계약 재검토가 필요합니다.
- **VehicleCommand**: 실제 전륜 road-wheel `target_steer_rad` (좌측 +),
  `target_speed_mps` (전진 >=0), `target_brake` bool, `valid`, `reason`.
  normalized joystick / steering actuator raw / motor raw와 호환되지 않습니다.
  `valid=false`도 speed=0 / brake=true인 정지 요구로 취급해야 합니다.
- **MotionPlan**: Path + target_speed_mps + stop_request + valid/reason.
  **ReferencePath**는 표준 `nav_msgs/Path`; 빈 경로는 unavailable입니다.

GNSS observation timestamp는 heartbeat 때 갱신하지 않습니다. 선택된 명령도 source timestamp를
보존하므로 상위 노드가 죽으면 반복 전달로 timeout이 숨겨지지 않습니다. 소비자는 receipt monotonic
age와 ROS source stamp를 함께 확인합니다. 0 / 미래 / 만료 stamp는 거부합니다. `use_sim_time` 사용 시
입력과 모든 노드의 clock을 일치시켜야 합니다. 서로 다른 호스트도 clock 동기화가 필요합니다.
Map Path는 정적 snapshot이라 시간 만료 없이 유지하며, ReferencePath는 매 tick 현재 계획 시각으로
발행합니다. map provider는 좌표 원점·version을 별도 검증해야 합니다.

## GNSS / IMU / camera / LiDAR 정책

GNSS primary, IMU optional assist. 기본 `GNSS_ONLY`, `fusion_enable=false`입니다.
`GNSS_IMU` / `GNSS_IMU_WHEEL` 또는 fusion=true 요청은 미구현 오류로 종료하며 조용히 GNSS-only로 위장하지 않습니다.
GNSS-only의 yaw-rate는 항상 invalid입니다. GNSS speed covariance도 확인합니다.
GNSS frame은 `map`, child frame은 rear axle `base_link` 계약입니다.
`localization_reference_verified=false` 기본값은 미실측 antenna lever arm을 정상으로 가정하지 않습니다.
현장 측정 및 외부 GNSS 설정 확인 후에만 이 parameter를 변경해야 합니다.

**Pure Pursuit는 Motion Control / Lateral Control 소속**입니다. GNSS package의 책임이 아닙니다.
`gnss_tools`는 읽기 전용 참고했으며 독립 구현에는 빌드/런타임 의존성이 없습니다.
`PURE_PURSUIT` 선택은 기존 `AdaptivePurePursuit` class를 그대로 사용합니다. 이 기존 구현은 이미
**speed-adaptive**이며 이번에 알고리즘을 재작성하지 않았습니다. nearest segment projection에서
`clamp(min + gain * speed, min, max)`만큼 경로를 따라 target을 찾고 실제 target chord 길이로
`atan2(2 * wheelbase * lateral, chord²)`를 계산합니다. 뒤쪽 target, 퇴화 경로는 invalid입니다.
전진 단순 경로 baseline이며 loop/교차 경로 progress, off-path gate, smoothing, steering rate 제한은
아직 없습니다. 실제 steering calibration이나 물리 한계값을 추측하지 않습니다.

RGB → traffic light / sign / future LaneState. **Obstacle primary는 LiDAR**입니다.
`object_manager/{traffic_light,traffic_sign,obstacle}.hxx`는 교체 가능한 detector interface만 제공합니다.
Depth는 optional이며 기본 구독/relay도 비활성화됩니다. near-field 보조와 object validation은 TODO입니다.

## Tracking Controller (2026-10-15)

기존 10-node 구조에서 `motion_control_node`만 Tracking 선택과 ROS I/O를 소유합니다.
Guidance(GPS_ONLY/LANE_ONLY/HYBRID), Vehicle Control(IDLE/MANUAL/AUTO), Localization은 별개입니다.
`VehicleState.x_m/y_m/yaw_rad/speed_mps`, `header.stamp`, localization/speed validity와
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

VehicleState/MotionPlan은 source stamp와 수신 timeout을 검사하고, 주행 계산에서는 nested
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

## Planning / mission / safety / mode

Path planner가 Guidance Mode의 요청과 실제 상태를 소유합니다. 기본 `guidance_mode=GPS_ONLY`이며
runtime 요청은 `/control/guidance_mode_request`입니다. ControlMode/FSM 및 localization 모드와 독립적입니다.

| 구분 | 모드 | 현재 구현 |
|---|---|---|
| Vehicle Control | IDLE / MANUAL / AUTO | manager의 명령 source 선택 |
| Guidance | GPS_ONLY / LANE_ONLY / HYBRID | path planner; GPS_ONLY만 실제 지원 |
| Localization | GNSS_ONLY / GNSS_IMU / GNSS_IMU_WHEEL | GNSS_ONLY만 지원; 나머지는 future enum, 시작 오류 |
| Optional sensors | IMU / LiDAR / Depth | 추정 보조 / 장애물 primary / optional 보조; Guidance와 독립 |

- **GPS_ONLY**: datum 확인 + frame/유한 수치/신선도/localization_valid 검사 + valid Global Path일 때
  경로를 그대로 전달합니다. LaneState의 존재/validity/내용은 결과에 영향을 주지 않습니다.
- **LANE_ONLY**: INTERFACE_READY / NOT_IMPLEMENTED. 항상 빈 경로, actual=NONE, valid=false,
  reason=LANE_ONLY_NOT_IMPLEMENTED입니다. GPS로 fallback하지 않습니다.
- **HYBRID**: INTERFACE_READY / NOT_IMPLEMENTED. 입력 상태와 관계없이 빈 경로,
  actual=NONE, valid=false, reason=HYBRID_NOT_IMPLEMENTED입니다. lane correction 및 degraded
  GPS fallback은 이번 버전에 없습니다. future HYBRID는 GPS+Lane이며 GNSS+IMU fusion이 아닙니다.
- **잘못된 요청 값**: requested_mode에 원래 값을 남기고 actual=NONE/valid=false,
  reason=INVALID_GUIDANCE_MODE로 닫습니다. 이후 유효한 요청이 오기 전까지 유지합니다.
  잘못된 초기 문자열 parameter는 시작 오류입니다.

`GuidanceStatus`의 requested_mode는 마지막 요청(시작 시 parameter), actual_mode는 실제 유효하게
생성한 Guidance이며 경로가 invalid이면 NONE(255)입니다. active_source는 GLOBAL_PATH 또는 NONE입니다.
`degraded`는 **유효한 축소 기능 fallback이 실제 실행될 때만 true**로 사용할 필드이며 지금은 항상 false입니다.
향후 HYBRID/GPS fallback은 requested=HYBRID, actual=GPS_ONLY, degraded=true, reason=LANE_INVALID로
표현할 수 있습니다. lane-only fallback은 실제 알고리즘이 구현된 뒤에만 허용해야 합니다.
현재는 어느 조합도 미구현 algorithm을 성공으로 표시하지 않습니다.

Status와 ReferencePath는 동일한 계획 시각 stamp를 사용하며 기본 20 Hz로 발행합니다.
UI는 늦거나 끊긴 status를 STALE/UNKNOWN으로 표시해야 합니다(기존 input_timeout_s=0.5 s 정책 참고).
Status는 출력 설명이며 safety authority가 아닙니다. 빈 ReferencePath가 MotionPlan invalid/stop_request,
AutoCommand invalid/brake로 전파됩니다. 전달과 정지는 ROS callback/timer/기존 timeout 범위에서 처리되며
원자적 전환이나 실시간 deadline 보장을 의미하지 않습니다. path planner 중단 시 reference_path timeout으로
정지합니다. Guidance 요청은 MANUAL command 선택이나 ControlMode를 변경하지 않습니다.
MANUAL에도 기존 safety 조건은 그대로 적용됩니다.

Global Path는 기존 계약대로 **정적 transient-local snapshot**이며 관측 센서처럼 시간 만료시키지 않습니다.
유효하지 않은 경로로 교체하면 즉시 사용할 수 없게 됩니다. provider는 datum/version을 보증해야 합니다.
VehicleState와 생성된 ReferencePath는 source timestamp와 수신 age를 모두 검사하며,
새 callback으로 과거/미래/0 timestamp를 정상으로 만들지 않습니다. lane은 현재 사용하지 않으므로
stale lane을 유효 Guidance 입력으로 판단하지 않습니다.

`lane_correction_enabled=true`와 nonempty `map_file`은 시작 오류입니다.
`MapLoader` / `LaneCorrection` interface만 있으며 final-map JSON/CSV는 읽지 않습니다.
IMU/Depth 활성 설정은 시작 시 read-only입니다. runtime sensor toggle과 센서 health 종합 status는 추가하지 않았습니다.
GUI가 종료되어도 요청 상태와 freshness/safety 처리는 ROS node 안에 남습니다. GUI heartbeat 기반 정지는 구현하지 않았습니다.
기존 GUI READ ONLY 조사 및 BEFORE/AFTER 설계는 [Guidance 설계 문서](doc/guidance_modes.md)에 있습니다.

**mission_based_limit**는 정상 주행 정책입니다. opt-in baseline max speed, 유효 speed sign cap,
명시적으로 `traffic_light_applies=true`인 context의 red/yellow stop 및 unknown stop만 구현했습니다.
동적 map mission section, curve/turn 감속, sign 적용 해제/수명은 TODO입니다. baseline은 기본 비활성입니다.
Motion planner는 state/path/mission/object가 모두 유효해야 정상 plan을 내며, obstacle detected이면
거리와 무관하게 정지합니다. avoidance/trajectory optimizer는 없습니다.

**safety_limit**는 선택 이후 최종 위험/시스템 이상 제한입니다. AUTO/MANUAL 모두 localization과
speed validity, timeout, object freshness, E-STOP, NaN/Inf, 음의 속도, brake+양의 속도,
steer/speed 범위, 검증되지 않은 운용 한계를 확인합니다. 범위 초과는 clamp 대신 정지합니다.
`require_obstacle_state=true` 기본값으로 perception 미구현 상태는 주행을 차단합니다.
검증된 장애물이 있고 emergency distance가 미설정(0)이면 정지합니다.
어떤 설정도 이 버전에서 액추에이터 송신을 활성화할 수 없습니다.

ControlMode는 IDLE/MANUAL/AUTO입니다. 초기 `mode` parameter는 read-only이고 runtime 전환은
`mode_request`를 사용합니다. 다른 mode의 drive command로 fallback하지 않습니다.
전환 시 입력 cache를 지워 다음 source callback을 기다리고 즉시 stop을 발행합니다.
잘못된 mode 요청은 IDLE로 전환합니다. 다른 parameter도 read-only이며 변경은 재시작이 필요합니다.

Vehicle FSM은 별도의 INIT/READY/DRIVING/STOPPED/FAULT입니다. 최초 final 대기=INIT,
invalid/stale final=FAULT, 유효 final + IDLE=READY, 유효 brake/zero speed=STOPPED,
유효 양의 command=DRIVING의 최소 상태 관리입니다. **DRIVING은 실제 차량 운동 확인이 아니라 명령 단계**입니다.
FAULT는 조건 해소 시 자동 회복하는 skeleton이며 fault latch/reset 및 운전자 승인 전이는 후속 설계 대상입니다.

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
- 기존 teleop 정책은 localization gate를 제외하지만 신규 최종 safety는 MANUAL에도 적용합니다.
  실내 수동 주행 예외 정책은 팀 결정 사항이며 여기서 외부 정책을 수정하지 않습니다.
- 기존 wheelbase 기본 1.04 m 등은 이 패키지에 복사하지 않고 사용자 실측 1.328 m를 사용합니다.
  GNSS lever arm 수치와 LiDAR x=0.559 m를 혼동해서 적용하지 않습니다.
- manual_control의 `cmd` Float32MultiArray, keyboard_control의 steering_cmd/speed_cmd는
  단위 및 제동 계약이 다릅니다. 확인된 calibration을 갖춘 adapter가 필요합니다.
- guidance raw speed와 trans/펌웨어 raw 범위는 m/s 계약이 아닙니다. raw conversion은 미구현입니다.
- dashboard final-map JSON과 guidance CSV의 datum/mission/path 계약을 통일해야 합니다.
  map loader는 placeholder이며 외부 파일을 변경하지 않았습니다.

## 미구현 목록

추가 error-adaptive PP, 실차 lateral PID tuning, LANE_ONLY guidance, HYBRID lane correction 및 fallback,
차량 제어 RQT UI, 센서 health 통합 status,
실제 traffic light/sign recognition, LiDAR detection/clustering, RGB lane extraction/correction,
Depth 보조, GNSS+IMU fusion/EKF, map loader/mission section rules, obstacle avoidance,
vehicle speed PID, physical brake model, actuator calibration/raw conversion, UDP sender,
실제 keyboard/manual adapter, hardware E-STOP/watchdog, 실차 검증.

## Tracking 추가 검증 (2026-10-15)

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

## Guidance Mode 추가 검증 (2026-10-14)

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
