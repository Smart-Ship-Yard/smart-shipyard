# Graph Report - smart-shipyard  (2026-08-28)

## Corpus Check
- cluster-only mode — file stats not available

## Summary
- 916 nodes · 1259 edges · 88 communities (68 shown, 20 thin omitted)
- Extraction: 96% EXTRACTED · 3% INFERRED · 0% AMBIGUOUS · INFERRED: 44 edges (avg confidence: 0.86)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `6d4f434b`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- ShipyardTwinDashboard.jsx
- ekf_global (map->odom EKF)
- ShipSurveyNode
- main.py
- SLAM-맵 강체정합
- HeadingComplementaryFilter
- PatrolMissionNode
- 모션 컨트롤러
- WheelOdomBridge
- 프론트엔드 빌드 의존성
- check_patrol_space.py
- WebSocketClient
- UwbMapCalibration
- UwbDwm1001Driver
- 브랜치 + PR 필수 작업 흐름
- Nav2 작업 정리 (결정사항·이슈·단계)
- EventGateNode
- 이벤트 스냅샷 · 핑 위치 표기
- ChangePointDetector
- 가짜 글로벌 로컬라이제이션
- YoloDepthPublisher
- 사각주행 검증 노드
- ship_survey_node 배 중심좌표 측량 요청서
- finalize_map.py (매핑 후처리 일괄 스크립트)
- patrol_mission_node (원형 순찰 순회)
- finalize_map.py
- 가짜 센서 퍼블리셔
- Three.js 3D 관제 대시보드
- YOLO 폴리곤 라벨 변환
- fake_frontend.py
- fake_jetson.py
- 가짜 SLAM 퍼블리셔
- IMU 축 보정
- 좌표계 구조 (map·odom·uwb_frame·slam_map)
- 엣지 하드웨어 실측 튜닝
- launch_setup
- 데모 월드 생성
- ShipPosePublisher
- 프론트엔드 정적 에셋
- ⑤ 실시간 영상 — 중앙 미디어 서버를 거친다
- RTSP 영상 전송 프로토콜
- 캘리브레이션 재현성 이슈
- 프로젝트 도구 설정
- 프로세스 일괄 종료
- udev 규칙 설치
- setup-firewall.sh
- KeepoutFilter 금지영역 마스크 (글로벌·로컬)
- motor/pymongo 비동기 MongoDB 드라이버 의존성
- /ws/frontend 웹소켓 엔드포인트
- CctvPopup 확인 버튼 (프론트)
- ByteTrack 트래커 설정
- GET /api/history (과거 이벤트 50건)
- GET /api/init-data (3D 맵 초기 정보)
- 실시간 이벤트 중계 서버 (backend)
- uvicorn --host 0.0.0.0 필수 규칙
- /ws/jetson 웹소켓 엔드포인트
- 이벤트 확인 기능 요청서 (정지/재개)
- Vite 진입 HTML (#root / main.jsx)
- ShipyardTwinDashboard (3D 관제 대시보드)
- FastAPI 비동기 중계 서버
- 젯슨
- AmclSeedNode
- estop.sh
- resume.sh

## God Nodes (most connected - your core abstractions)
1. `SceneManager` - 29 edges
2. `PatrolMissionNode` - 20 edges
3. `WebSocketClient` - 17 edges
4. `YoloDepthPublisher` - 17 edges
5. `MotionControllerNode` - 17 edges
6. `WheelOdomBridge` - 17 edges
7. `ShipyardTwinDashboard()` - 17 edges
8. `UwbDwm1001Driver` - 15 edges
9. `UwbMapCalibration` - 14 edges
10. `SlamMapAlignmentNode` - 12 edges

## Surprising Connections (you probably didn't know these)
- `젯슨에서 git add . 금지 (wit_ros2_imu 서브모듈)` --semantically_similar_to--> `.gitignore 정책 (비밀값·재생성물 제외)`  [INFERRED] [semantically similar]
  edge/docs/젯슨_인계_노트 — Step8_실물이식.md → CONTRIBUTING.md
- `저장소 내부 상대경로 심볼릭 링크 규칙` --semantically_similar_to--> `맵 폴더를 ROS2 패키지 안으로 이동`  [INFERRED] [semantically similar]
  CONTRIBUTING.md → edge/docs/nav2_작업_정리.md
- `프론트 없이 전체 경로 연동 시험 (fake_send_event_ack)` --semantically_similar_to--> `CctvPopup 확인 버튼 (프론트)`  [INFERRED] [semantically similar]
  edge/docs/전체_실행_명령어_요약본 — 매핑부터_자율주행까지.md → docs/이벤트_확인_기능_요청.md
- `Graphify 우선 조회 워크플로 규칙` --semantically_similar_to--> `Serena 프로젝트 설정 (python LSP, utf-8)`  [INFERRED] [semantically similar]
  CLAUDE.md → .serena/project.yml
- `motor/pymongo 비동기 MongoDB 드라이버 의존성` --implements--> `MongoDB 이벤트 로그 저장`  [INFERRED]
  backend/requirements.txt → README.md

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **감지 사진 전 구간 흐름 (젯슨 base64 → 서버 파일 → 프론트 URL)** — docs_interface_event_snapshot, docs_interface_image_url, backend_readme_snapshots_static, frontend_readme_snapshot_in_popup, backend_readme_snapshot_no_commit [EXTRACTED 1.00]
- **한 세션 매핑 절차 (캘리브레이션 → 매핑 → 후처리 → 커밋)** — edge_docs_calibration_tape_markers, edge_docs_full_ring_mapping_rule, edge_docs_temp_box_for_ship_mapping, edge_docs_map_naming_convention, edge_docs_nav2_finalize_map, edge_docs_nav2_map_origin_bake [EXTRACTED 1.00]
- **Nav2 파라미터 겹치기(공통 + 장소별 + 기능별) 패턴** — edge_ros2_ws_src_ship_ugv_navigation_config_nav2_params_layered_params_pattern, edge_ros2_ws_src_ship_ugv_navigation_config_space_narrow_overlay, edge_ros2_ws_src_ship_ugv_navigation_config_space_wide_overlay, edge_ros2_ws_src_ship_ugv_navigation_config_keepout_on_keepout_overlay [EXTRACTED 1.00]
- **배 측량 → 순찰 설정 · keepout 마스크 생성 파이프라인** — docs_ship_survey_node_request, docs_ship_survey_pose_topic, docs_ship_survey_size_xy, edge_docs_nav2_finalize_map, edge_docs_nav2_check_patrol_space, edge_docs_nav2_keepout_filter [EXTRACTED 1.00]
- **EKF 단일 TF 권위자 체계** — edge_ros2_ws_src_ship_ugv_localization_config_ekf_local_ekf_local, edge_ros2_ws_src_ship_ugv_localization_config_ekf_global_ekf_global, edge_ros2_ws_src_ship_ugv_localization_config_slam_toolbox_mapping_single_tf_authority, edge_ros2_ws_src_ship_ugv_localization_config_slam_toolbox_mapping_slam_map_frame_rename, edge_ros2_ws_src_ship_ugv_navigation_config_nav2_params_no_amcl [EXTRACTED 1.00]
- **영상 경로: 젯슨 mediamtx → 중앙 미디어 서버 이전 (2026-08-28)** — docs_interface_live_video_media_server, docs_interface_jetson_cpu_constraint, docs_interface_why_not_mjpeg_relay, server_streaming_readme_media_server, server_streaming_readme_firewall_ports, frontend_readme_media_server_video [EXTRACTED 1.00]
- **순찰 여유 공간 설계 제약 묶음** — edge_ros2_ws_src_ship_ugv_navigation_config_patrol_demo_room_patrol_mission_node, edge_ros2_ws_src_ship_ugv_navigation_config_patrol_shipyard_map_jg_room_v2_patrol_mission_node, edge_ros2_ws_src_ship_ugv_navigation_config_nav2_params_rotate_to_heading, edge_ros2_ws_src_ship_ugv_navigation_config_nav2_params_polygon_footprint, edge_ros2_ws_src_ship_ugv_navigation_config_space_narrow_overlay [INFERRED 0.95]

## Communities (88 total, 20 thin omitted)

### Community 0 - "ShipyardTwinDashboard.jsx"
Cohesion: 0.05
Nodes (35): plugins, rules, react/only-export-components, react/rules-of-hooks, $schema, App(), BLOCKS, CctvPopup() (+27 more)

### Community 1 - "ekf_global (map->odom EKF)"
Cohesion: 0.08
Nodes (40): ship_ugv_description package, ship_ugv_localization package, ekf_global (map->odom EKF), odom 프레임 위치를 map EKF에 넣으면 안 된다, slam_toolbox pose는 운영 모드에서만 pose1로 켠다, ekf_local (odom->base_link EKF), 속도만 fuse하고 위치는 필터가 적분, angle_filter_back (후방 180도 제거) (+32 more)

### Community 2 - "ShipSurveyNode"
Cohesion: 0.08
Nodes (31): coverage_bin_count(), fit_rect(), main(), Node, String, 점들이 배 둘레 몇 방향에서 관측됐는지 센다 (한 바퀴 완료 판정용). 중앙값 중심에서 각 점을 바라본 방위각을 bin_deg 간격으로 묶어…, 기존 결과 파일 중 가장 큰 회차 번호를 찾는다 (uwb_map_calibration과 동일 방식)., 카메라 좌표계 3D 점을 map 좌표 2D로 변환. change_point.py / websocket_client.py와 동일한 보정 순서를… (+23 more)

### Community 3 - "main.py"
Cohesion: 0.06
Nodes (39): _active_danger_events(), _attach_snapshot_url(), ConnectionManager, get_history(), get_init_data(), handle_event_snapshot(), lifespan(), main.py — 스마트 조선소 FastAPI 백엔드 서버 프로젝트 : 스마트 조선소 선박 건조 공정 트래킹 및 디지털 트윈 관제 시스템 모듈… (+31 more)

### Community 4 - "SLAM-맵 강체정합"
Cohesion: 0.10
Nodes (18): apply_transform(), estimate_rigid_transform_2d(), ransac_rigid_transform_2d(), rigid_transform_2d.py ---------------------- 두 점 집합(대응점 쌍) 사이의 2D 강체변환(회전+평행이동,…, Kabsch/Umeyama 알고리즘 (2D, scale=1 고정). src_points, dst_points: shape (N, 2), N…, RANSAC으로 이상치(잘못 짝지어진 대응점, UWB 튐 잔차 등)에 강건한 변환 추정. 반환: (theta, tx, ty,…, main(), Node (+10 more)

### Community 5 - "HeadingComplementaryFilter"
Cohesion: 0.09
Nodes (17): HeadingComplementaryFilter, main(), Imu, Node, Odometry, PoseWithCovarianceStamped, quaternion_to_yaw(), UWB pose(uwb_frame)를 map 프레임 좌표로 변환. 반환: (map_x, map_y, tf_yaw) 또는 None (TF 미가용… (+9 more)

### Community 6 - "PatrolMissionNode"
Cohesion: 0.07
Nodes (17): Bool, KeyboardTeleopNode, main(), Node, 터미널을 raw 모드로 바꿔서 한 글자씩 즉시 읽는다 (엔터 안 눌러도 됨)., 마지막 키 입력 후 idle_timeout이 지나면 자동으로 정지시킨다., main(), PatrolMissionNode (+9 more)

### Community 7 - "모션 컨트롤러"
Cohesion: 0.12
Nodes (14): main(), MotionControllerNode, MotionState, Empty, Node, Odometry, quaternion_to_yaw(), 새 동작을 시작해도 되는지 확인. 오도메트리가 아직 없으면 거부한다 (시작 위치를 모르는 채로 움직이면 목표 거리를 계산할 수 없음). (+6 more)

### Community 8 - "WheelOdomBridge"
Cohesion: 0.09
Nodes (15): EncoderDiagLogger, main(), Node, main(), Node, Odometry, 포트를 연 직후 Arduino가 리셋되어 부팅 중인 구간인지 판정., ekf_local 의 융합 yaw 를 받아 둔다 (heading_hold 전용). 이 노드는 오도메트리를 스스로 적분해 /wheel/odom… (+7 more)

### Community 9 - "프론트엔드 빌드 의존성"
Cohesion: 0.07
Nodes (27): dependencies, react, react-dom, three, devDependencies, oxlint, @types/react, @types/react-dom (+19 more)

### Community 10 - "check_patrol_space.py"
Cohesion: 0.08
Nodes (42): main(), parse_origin(), 맵 yaml의 단순 key: value를 읽는다 (전체 파싱 아님, 줄은 보존)., 맵 그림 자체를 yaw 만큼 돌린다. (새 폭, 새 높이, 픽셀, da, db) 를 준다. ★ 왜 필요한가 (2026-08-20 실측으로…, read_yaml(), rotate_pixels(), w_h_txt(), write_pgm() (+34 more)

### Community 11 - "WebSocketClient"
Cohesion: 0.13
Nodes (10): extract_level(), main(), Node, Odometry, String, 원본 검출 스트림 — 이제 block_level(조립 단계) 판정에만 쓴다. 위험 이벤트(fire 등)는 위치 기준 중복 제거를 거친…, change_point_detector 가 위치 기준 중복 제거를 마친 위험 이벤트. 같은 자리의 같은 클래스는…, change_point_detector 가 "치워짐"을 확인했을 때. 서버로 그대로 넘겨 프론트엔드가 해당 위치의 빨간 핑을 지우게 한다.… (+2 more)

### Community 12 - "UwbMapCalibration"
Cohesion: 0.14
Nodes (10): CalibState, main(), Node, PoseWithCovarianceStamped, 저장해둔 캘리브레이션 결과를 그대로 되살린다. 되살릴 수 있는 이유는 map<-uwb_frame 이 방에 고정된 값이기 때문이다. 로봇의 현재…, 주의: 서비스 콜백 안에서 rclpy.spin_once()로 블로킹 대기하면 안 된다. (이미 executor가 이 콜백을 실행 중이므로…, 주기 타이머: 시간이 아니라 '실제 이동거리(min_travel)'에 도달하면 계산 수행. (로봇 최고속도로는 고정된 5초 안에…, UwbMapCalibration (+2 more)

### Community 13 - "UwbDwm1001Driver"
Cohesion: 0.09
Nodes (10): FakeSerial, main(), Node, pyserial.Serial과 같은 인터페이스를 흉내내는 가짜 시리얼 포트. 실제 하드웨어 대신, 미리 정해둔 DWM1001 lec 출력…, ResultCollector, main(), Node, 무조건 'lec' 토글 명령을 보내지 않는다. 먼저 probe_seconds 동안 들어오는 라인이 이미 POS,... 형식인지 확인하고, 이미… (+2 more)

### Community 14 - "브랜치 + PR 필수 작업 흐름"
Cohesion: 0.67
Nodes (4): 브랜치 + PR 필수 작업 흐름, Conventional Commits 커밋 메시지 규칙, .gitignore 정책 (비밀값·재생성물 제외), 젯슨에서 git add . 금지 (wit_ros2_imu 서브모듈)

### Community 15 - "Nav2 작업 정리 (결정사항·이슈·단계)"
Cohesion: 0.21
Nodes (13): 저장소 내부 상대경로 심볼릭 링크 규칙, 전체 실행 명령어 요약본 (매핑~자율주행), 주행 1.5 m / 판정 1.4 m 마진 쌍, 맵 폴더를 ROS2 패키지 안으로 이동, 원형 근사 금지 — 다각형 footprint, Regulated Pure Pursuit + NavFn 선택, space:=narrow|wide 장소 프리셋 오버레이, stop_all.sh 좀비 프로세스 정리 (+5 more)

### Community 16 - "EventGateNode"
Cohesion: 0.21
Nodes (7): EventGateNode, main(), Empty, Node, String, 상태를 주기적으로 다시 알린다. 구독자가 나중에 떠도 현재 상태를 받게 하려는 것이다. transient_local 만으로도 대부분…, 서버에서 온 것을 그대로 받는다. event_ack 만 우리 관심사다.

### Community 17 - "이벤트 스냅샷 · 핑 위치 표기"
Cohesion: 0.12
Nodes (15): 1. 위치 표기 규칙, 2. 스냅샷 — 젯슨이 보내는 것과 백엔드가 할 일, 3. 프론트가 할 일, 4. 젯슨 CPU 부하 — 걱정할 수준이 아니다, 5. 사진 최신화 — 보류, `.gitignore`, 무엇을 하려는 것인가, 백엔드가 할 일 (+7 more)

### Community 18 - "ChangePointDetector"
Cohesion: 0.12
Nodes (13): ChangePointDetector, clear_verdict(), in_camera_fov(), main(), Node, String, quat_to_yaw(), 같은 클래스이면서 반경 안에 있는 기존 이벤트를 찾아 반환 (없으면 None). (+5 more)

### Community 19 - "가짜 글로벌 로컬라이제이션"
Cohesion: 0.23
Nodes (10): compose(), FakeGlobalLocalization, invert(), main(), Node, Odometry, 참값을 버퍼에 넣고 곧바로 발행을 시도한다. ★★ "TF가 실제로 있는 시각"에 맞추는 이유 (2026-08-07 실측) ★★ 앞서 두 방식이…, 쿼터니언 -> yaw(rad). 2D 주행이라 yaw만 쓴다. (+2 more)

### Community 20 - "YoloDepthPublisher"
Cohesion: 0.14
Nodes (9): frame_to_bgr_image(), is_level_class(), main(), Node, ['no_helmet:0.15', 'fire:0.45'] -> {'no_helmet': 0.15, 'fire': 0.45} 빈 문자열/형식…, 이 클래스에 적용할 confidence 임계값. 지정 없으면 전역값., 실행 중 파라미터 변경을 캐시된 값에 반영한다., 화면 정중앙 ROI 의 뎁스 중앙값을 카메라 좌표로 내보낸다. 배 중심 좌표를 잴 때… (+1 more)

### Community 21 - "사각주행 검증 노드"
Cohesion: 0.23
Nodes (5): main(), Empty, Node, String, SquareTestNode

### Community 22 - "ship_survey_node 배 중심좌표 측량 요청서"
Cohesion: 0.29
Nodes (7): /server/inbound 무해석 중계 토픽, cv2.minAreaRect 도형 피팅 (centroid 대신), ship_survey_node 배 중심좌표 측량 요청서, /ship_survey/pose 토픽 (1회 발행), TRANSIENT_LOCAL latched QoS 명시, 원본 /event_detection/uvd 구독 (map_point 금지), PR #21 이후 localization.launch.py 가 인지·서버 노드까지 기동

### Community 23 - "finalize_map.py (매핑 후처리 일괄 스크립트)"
Cohesion: 0.29
Nodes (7): 중앙 물체 주위 완전한 한 바퀴 매핑, 맵 이름 규칙 shipyard_map_<장소>_v<번호>, check_patrol_space.py (순찰 가능성 검사·마스크 생성), finalize_map.py (매핑 후처리 일괄 스크립트), free_thresh 0.196 상수 교정, 맵 origin 을 map 좌표계로 굽기 (slam_map 회전 보정), 웨이포인트 개수 자동 계산 규칙 (40도·1.5배)

### Community 24 - "patrol_mission_node (원형 순찰 순회)"
Cohesion: 0.25
Nodes (8): event_gate_node (/event/active 판정기), 정지 = 목표 취소 + 0속도 (복구행동 유발 회피), patrol_mission_node (원형 순찰 순회), 현재 위치 기준 웨이포인트 재동기화, Pose Estimation 기반 낙상 판별 (관절 17점), 안전 관리 (안전모·위험구역·낙상 감지), 스마트 조선소 디지털 트윈 시스템, YOLOv8 실시간 객체 검출

### Community 25 - "finalize_map.py"
Cohesion: 0.11
Nodes (32): copy_record(), fmt_time(), main(), newest(), pick_align(), pick_survey_center(), print_recovery_options(), 패턴에 맞는 파일 중 가장 최근에 수정된 것. 없으면 None. 번호(align_001, align_002...)가 아니라 수정 시각으로… (+24 more)

### Community 26 - "가짜 센서 퍼블리셔"
Cohesion: 0.31
Nodes (4): FakeSensorPublisher, main(), Node, t(경과초)에 따라 (선속도, 각속도) 반환 — 직진 -> 정지 -> 회전

### Community 28 - "YOLO 폴리곤 라벨 변환"
Cohesion: 0.43
Nodes (7): bbox_to_polygon(), convert_lines(), main(), 변환이 정보를 잃지 않는지 + 섞임을 잡아내는지 확인., cls cx cy w h -> cls x1 y1 x2 y2 x3 y3 x4 y4 (좌상->우상->우하->좌하)., 라벨 파일 한 개의 행들을 (변환된 행들, bbox행수, 폴리곤행수)로 돌려준다., selftest()

### Community 29 - "fake_frontend.py"
Cohesion: 0.67
Nodes (3): json_channel(), main(), fake_frontend.py — 가짜 프론트엔드 (백엔드 검증용 뷰어) 브라우저 대시보드 없이 백엔드가 뿌리는 데이터를 눈으로 확인하기 위한…

### Community 30 - "fake_jetson.py"
Cohesion: 0.36
Nodes (7): event_id_of(), json_channel(), main(), fake_jetson.py — 가짜 젯슨 (목업) 실물 젯슨/RC카 없이 백엔드 서버를 테스트하기 위한 스크립트.…, docs/interface.md ② 의 규칙 그대로: "<class>@<x>,<y>" (소수 2자리). ②-b·②-c 가 이 문자열로 이벤트를…, 배 기준 (앞뒤, 좌우) 오프셋을 map 절대좌표로., ship_local_to_map()

### Community 31 - "가짜 SLAM 퍼블리셔"
Cohesion: 0.33
Nodes (4): FakeSlamPublisher, main(), Node, PoseWithCovarianceStamped

### Community 32 - "IMU 축 보정"
Cohesion: 0.33
Nodes (4): ImuAxisCorrectionNode, main(), Imu, Node

### Community 33 - "좌표계 구조 (map·odom·uwb_frame·slam_map)"
Cohesion: 0.50
Nodes (4): 좌표계 구조 (map·odom·uwb_frame·slam_map), TF 최종 권위자는 EKF 하나뿐, Option A — robot_state_publisher 미기동 (TF 단일 발행자), UWB 앵커 배치 원칙 (사각형·고소·3m 이상)

### Community 34 - "엣지 하드웨어 실측 튜닝"
Cohesion: 0.33
Nodes (6): Arduino DTR 자동 리셋 논블로킹 유예, CH341 클론칩 in_waiting 항상 0 버그, 엣지 설치·운영 가이드 (하드웨어 실측), 후방 180도 제거는 필터 2개로 분할, ticks_per_rev 330 (스펙 1320의 1/4) 실측 역산, udev KERNELS 기반 시리얼 장치 고정

### Community 35 - "launch_setup"
Cohesion: 0.36
Nodes (7): generate_launch_description(), launch_setup(), 맵 이름 또는 경로를 실제 yaml 경로로 바꾼다. 이름만 준 경우 패키지의 maps/ 안에서 찾는다. 노트북과 젯슨의 설치 경로가 같아지므로…, 인자를 실제 값으로 확정한 뒤 노드를 만든다. OpaqueFunction 을 쓰는 이유: space 프리셋 선택, 맵 이름 해석,…, wheel_odom_bridge 의 enable_heading_hold 를 확실하게 바꾼다. `ros2 param set` 은 상대 노드를…, resolve_map(), _set_heading_hold()

### Community 36 - "데모 월드 생성"
Cohesion: 0.53
Nodes (5): build_map(), build_world(), main(), 정적 벽 하나. Gazebo Classic SDF의 link 블록., wall_link()

### Community 37 - "ShipPosePublisher"
Cohesion: 0.27
Nodes (6): _age_text(), main(), Node, 실측한 배 중심좌표를 켜질 때마다 자동으로 서버에 보낸다. 왜 필요한가 ----------- 배 위치는 매핑 때 한 번 실측해서…, 지금 어떤 배 위치가 서버에 올라가 있는지 5초마다 상기시킨다. 전송 자체는 시작할 때 **딱 한 번**이다.…, ShipPosePublisher

### Community 38 - "프론트엔드 정적 에셋"
Cohesion: 0.50
Nodes (5): Vite Lightning Bolt Favicon, UI Icon Sprite Sheet, Isometric Layered Slab Hero Image, React Atom Logo, Vite Wordmark Logo

### Community 39 - "⑤ 실시간 영상 — 중앙 미디어 서버를 거친다"
Cohesion: 0.06
Nodes (34): 대시보드 재접속 시 위험 핑 복원, ⚠️ 스냅샷 사진은 커밋 금지, /snapshots 정적 서빙 (감지 순간 사진), 영상은 FastAPI 를 거치지 않는다, ③ 조립 단계 변화 (block_level), ② 위험 이벤트 (fallen_person·fire·no_helmet·ship_defect), ekf_global 키 (map 기준 차체 절대좌표), ⑧ event_ack (관제 확인 → 자율주행 재개) (+26 more)

### Community 41 - "캘리브레이션 재현성 이슈"
Cohesion: 0.50
Nodes (4): 캘리브레이션 시작점·끝점 바닥 테이프 표시, UWB 캘리브레이션 원점·방향 재현성 문제, SVD 캘리브레이션 변경으로 옛 맵 재사용 불가, 캘리브레이션 누락은 항등 TF로 조용히 통과한다

### Community 70 - "KeepoutFilter 금지영역 마스크 (글로벌·로컬)"
Cohesion: 0.50
Nodes (4): size_xy (배 가로·세로, 파일에만 기록), KeepoutFilter 금지영역 마스크 (글로벌·로컬), 모형 배는 라이다 평면에 안 잡힌다, 1단계 대응 — 매핑 중에만 임시 상자

### Community 71 - "motor/pymongo 비동기 MongoDB 드라이버 의존성"
Cohesion: 0.67
Nodes (3): motor/pymongo 비동기 MongoDB 드라이버 의존성, 백엔드 venv 세팅 및 실행 절차, MongoDB 이벤트 로그 저장

### Community 119 - "젯슨"
Cohesion: 0.14
Nodes (13): 0. 모든 터미널 첫 세팅, Nav2 실물 명령어 (compact), 꼭 기억할 것, 백엔드 노트북, 서버 터미널 1 — 서버 실행, 서버 터미널 2 — 이벤트 확인 후 재개 (프론트 "확인" 버튼 대역), 젯슨, 터미널 1 — 로컬라이제이션 (+5 more)

### Community 141 - "AmclSeedNode"
Cohesion: 0.24
Nodes (5): AmclSeedNode, main(), Node, Odometry, PoseWithCovarianceStamped

## Ambiguous Edges - Review These
- `UI Icon Sprite Sheet` → `Isometric Layered Slab Hero Image`  [AMBIGUOUS]
  frontend/public/icons.svg · relation: conceptually_related_to

## Knowledge Gaps
- **101 isolated node(s):** `MotionState`, `react/rules-of-hooks`, `$schema`, `BLOCKS`, `CLASS_META` (+96 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **20 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **What is the exact relationship between `UI Icon Sprite Sheet` and `Isometric Layered Slab Hero Image`?**
  _Edge tagged AMBIGUOUS (relation: conceptually_related_to) - confidence is low._
- **What connects `MotionState`, `react/rules-of-hooks`, `$schema` to the rest of the system?**
  _101 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `ShipyardTwinDashboard.jsx` be split into smaller, more focused modules?**
  _Cohesion score 0.05029838022165388 - nodes in this community are weakly interconnected._
- **Should `ekf_global (map->odom EKF)` be split into smaller, more focused modules?**
  _Cohesion score 0.07692307692307693 - nodes in this community are weakly interconnected._
- **Should `ShipSurveyNode` be split into smaller, more focused modules?**
  _Cohesion score 0.08048780487804878 - nodes in this community are weakly interconnected._
- **Should `main.py` be split into smaller, more focused modules?**
  _Cohesion score 0.057971014492753624 - nodes in this community are weakly interconnected._
- **Should `SLAM-맵 강체정합` be split into smaller, more focused modules?**
  _Cohesion score 0.10317460317460317 - nodes in this community are weakly interconnected._