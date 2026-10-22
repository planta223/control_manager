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
- /sensors/detectCone
- 라이다로 할지 카메라로 할지는 미정.
- 필드는 다음과 같이 구현할 것.
'''

'''


2. 신호 인식 결과 (필수 : 정재은)
- /sensors/detectTraffic 토픽으로 발행할 것.
- 메시지 타입은 TrafficLight.msg 으로 할 것.
- 필드는 다음과 같이 구성할 것.
'''
# 신호 대상 식별자: 1~5, 미지정은 0
# 1: 예선 #300 교차로 북서 방향 — 직진
# 2: 본선 #200 교차로 남서 방향 — 직진
# 3: 본선 #300 교차로 남서 방향 — 직진
# 4: 본선 #400 교차로 남 방향 — 좌회전
# 5: 본선 #1400 교차로 북동 방향 — 직진
# 예: traffic_id=1 → 예선 #300 교차로 북서 방향 신호의 인식 결과
# 해당 ROI에서 미검출되어도 ID는 유지하고 status="Unknown"으로 발행
uint8 traffic_id

# 신호 상태: R / Y / G / Unknown
# 예: "R" → 적색으로 판단
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
float32 color_confidence
'''


3. 장애물 인식 결과 (선택)
- /sensors/detectObstacle 로 발행할 것
- 구현할지는 미정
