# GlobalPose / MotionState 분리 보고서 (0.3.0, 당시 작업 기록)

후속 ingress 변경으로 Lane 직접 입력과 외부 feedback callback 위치는 변경되었습니다.
현재 연결은 [sensor_ingress.md](sensor_ingress.md)를 기준으로 합니다. 아래 검증 수치와 scope 기록은 당시 결과입니다.

## 의미상 상태 분리 (현재도 유지)

GlobalPose는 map pose/uncertainty, MotionState는 vehicle motion/validity입니다.
현재 두 메시지는 global_state_estimator에서 독립 생성합니다. Lane 상대 상태는 lane_state_estimator가 담당합니다.
이 문서의 검증 수치는 node 통합 이전의 이력이며 현재 모드/이름/흐름은 [estimators.md](estimators.md)가 기준입니다.

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

## 당시 변경 이력

당시 변경 파일은 Git 이력과 패키지 내부 log/estimators_before_sources.json에 보존되어 있습니다.
현재 제거/추가 파일과 canonical executable 명칭은 estimators.md를 참고합니다.

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
