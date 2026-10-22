Last Updated : 2026.10.23

===========================================
control_manager의 mode를 관리하는 yaml 4개
===========================================

test.yaml
- 학교용. 10.22 혹은 10.23
- map : /workspace/decision/script/map_generator/test/review/maps/MAP_test.json
- 지도는 후륜축 기준 1,217점 / 약 121.6 m. 실제 GNSS datum으로 재투영한다.
- 목표속도는 시험 시 정한다. control_console --cruise-speed-kph로 해당 실행에만 적용 가능.
- 차량상태 : gps odom topic 사용


brake_inspection.yaml (10.29)
- 제동검차용. 10.29
- map : 사용할수도 있고 사용안할수도 있음 (타 팀원 추가구현 가능)
- 차량상태 : gps odom topic 사용
- 라바콘 감지 : 별도의 camera 혹은 lidar topic 사용 예정 (팀원 추가구현 예정)
- 40km/h 로 등속주행 후 정지 계획


preliminary_lap.yaml
- 예선용. 10.29
- 차량상태 : gps odom topic 사용
- map : main/decision/script/map_generator/test/review/maps/MAP_preliminary.json
- 신호 감지 : 별도의 camera topic 사용 예정 (타 팀원 추가구현 예정)


final_lap.yaml
- 본선용. 10.30
- map : main/decision/script/map_generator/test/review/maps/MAP_final.json
- 차량상태 : gps odom topic 사용
- 신호 감지 : 별도의 camera topic 사용 예정 (타 팀원 추가구현 예정)
- 장애물 감지 : 일단 gps odom topic을 신뢰하나 별도의 lidar 혹은 camera 토픽 사용할수도 있음. (타 팀원 추가구현 가능)

===========================================
필요한 추가 토픽
===========================================


1. 라바콘 인식 결과 (필수 : 조성준)
- 토픽명 : /sensors/detectCone
- 타입명 : control_manager/msg/LidarObjects
- 발행 주기 : 본인이 결정해 주세요.
- 필드는 다음과 같이 구현할 것.
'''
# 인식에 사용한 원본 라이다 측정 시각
# 인식 완료 시각이나 발행 시각으로 덮어쓰지 않음
builtin_interfaces/Time timestamp

# 입력·인식 처리가 정상적으로 완료됐는지
# 정상 미검출: true + objects=[]
# 입력·처리 실패: false + objects=[]
bool valid

# 검출한 라바콘 목록
LidarObject[] objects
'''
- 배열 원소인 LidarObject.msg의 필드는 다음과 같이 구성할 것.
'''
# 검출한 물체 경계 박스의 중심 위치 [m]
geometry_msgs/Point center

# 해당 좌표계의 x/y/z축에 평행한 경계 박스 크기 [m]
geometry_msgs/Vector3 size
'''
- center와 size는 차량 좌표계 base_link 기준이며, 인식 측에서 해당 좌표계로 변환하여 발행한다.
- 좌표계는 후륜축 중심 기준 전방 +x, 좌측 +y, 상방 +z를 사용한다.
- 제동 기준으로 사용할 라바콘 선택 및 제동 시점 판단은 제어단에서 수행한다.


2. 장애물 인식 결과 (선택 : 조성준)
- 토픽명 : /sensors/detectObstacle
- 타입명 : control_manager/msg/LidarObjects
- 발행 주기 : 본인이 결정해 주세요.
- 필드 및 상세 사항은 1번과 동일하게 구성하면 되며, 검출 대상만 라바콘이 아닌 장애물로 바꾼다.


3. 신호 인식 결과 (필수 : 정재은)
- 토픽명 : /sensors/detectTraffic
- 타입명 : control_manager/msg/TrafficLight
- 발행 주기 : 본인이 결정해 주세요.
- 필드는 다음과 같이 구성할 것.
'''
# 신호 상태: R / Y / G / Unknown
# 예: "R" → 적색으로 판단
# 예: "G" → 청색 및 좌회전 (미션상 청색, 좌회전이 따로 켜지는 경우는 없음을 참고)
string status

# 인식에 사용한 원본 영상의 촬영 시각
# 형식 예: {sec: 123, nanosec: 450000000} → ROS 시각 123.45초
builtin_interfaces/Time timestamp

# 해당 영상의 수신·인식 처리가 정상적으로 수행됐는지
# 예: 정상 처리=true, 영상 입력 오류·추론 실패=false
# 정상 처리했지만 색을 판별하지 못했다면 true + status="Unknown"
bool valid

# status에 해당하는 색 판단 점수, 범위 0.0~1.0
# 예: status="R", color_confidence=0.93 → 적색 판단 점수 0.93
# valid = false 일때는 0.0 으로 한다.
# 예: 판별 불가 상황에서 valid=true, status="Unknown", color_confidence=0.0
# 예: 처리 실패 상황에서 valid=false, status="Unknown", color_confidence=0.0
float32 color_confidence
'''
- 카메라로 현재 프레임이 어떤 교차로의 신호등인지 구분할 필요는 없다.
- 단 여러 신호등이 관측되는 상황에서는 목표 신호등의 status를 추출할 수 있어야 한다.
- 청색과 좌회전은 항상 같이 켜지므로 굳이 좌회전과 청색을 구분할 필요는 없다.