이 폴더에 학습된 YOLO 가중치 파일(best.pt)을 넣어야 합니다.

예:
  cp ~/Downloads/best.pt ./best.pt

[2026-07-08 변경] setup.py가 glob('weights/*.pt') 방식으로 바뀌어서,
best.pt가 없어도 colcon build는 성공합니다 (clone 직후 팀원 빌드 실패 방지).
단, 가중치 없이 빌드하면 yolo_depth_publisher 실행 시
"YOLO 가중치 파일이 없습니다" 에러로 안내됩니다.

가중치를 이 폴더에 넣은 뒤에는 반드시 colcon build를 다시 실행해야
install(share) 폴더로 복사되어 노드가 찾을 수 있습니다:
  cd ~/smart-shipyard/edge/ros2_ws
  colcon build --packages-select ship_ugv_perception
