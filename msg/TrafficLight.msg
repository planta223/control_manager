# 신호 대상: 1=예선 #300 북서 직진, 2=본선 #200 남서 직진,
# 3=본선 #300 남서 직진, 4=본선 #400 남 좌회전, 5=본선 #1400 북동 직진.
# 미지정은 0. ROI 미검출 시에도 ID를 유지하고 status="Unknown".
uint8 traffic_id
# R / Y / G / Unknown
string status
# 원본 영상 촬영 시각. 수신 시각으로 덮어쓰지 않음.
builtin_interfaces/Time timestamp
# 정상 처리 + 색 판단 불가: true / Unknown. 입력·추론 실패: false.
bool valid
# status의 색 판단 점수, 0.0~1.0.
float32 color_confidence
