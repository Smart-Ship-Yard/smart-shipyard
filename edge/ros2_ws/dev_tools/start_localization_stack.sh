#!/bin/bash
cd ~/smart-shipyard/edge/ros2_ws

# 혹시 이미 떠 있는 노드가 있으면 먼저 정리 (중복 방지)
pkill -f ekf_node 2>/dev/null
pkill -f complementary_filter_node 2>/dev/null
pkill -f calibration_node 2>/dev/null
pkill -f change_point 2>/dev/null
pkill -f slam_map_alignment_node 2>/dev/null
sleep 1

source install/setup.bash

ros2 run heading_complementary_filter complementary_filter_node &
ros2 run uwb_map_calibration calibration_node &
ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_local --params-file src/ship_ugv_localization/config/ekf_local.yaml -r odometry/filtered:=/odometry/local &
ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_global --params-file src/ship_ugv_localization/config/ekf_global.yaml -r odometry/filtered:=/odometry/global &
ros2 run ship_ugv_perception change_point &
ros2 run slam_map_alignment slam_map_alignment_node &
echo "6개 노드 기동 시도 완료. 확인: ros2 node list"
wait
