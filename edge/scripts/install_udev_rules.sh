#!/bin/bash
# smart-shipyard UGV - 시리얼 장치 고정 이름 규칙 설치 스크립트
#
# 새 Jetson에 세팅하거나 OS를 재설치했을 때 반드시 한 번 실행해야 한다.
# 이 규칙이 없으면 im10a.yml의 'port: imu' 와
# localization.launch.py의 '/dev/wheel_mcu' 가 동작하지 않는다.

set -e

RULE_SRC="$(dirname "$0")/99-robot-serial.rules"
RULE_DST="/etc/udev/rules.d/99-robot-serial.rules"

if [ ! -f "$RULE_SRC" ]; then
    echo "오류: $RULE_SRC 를 찾을 수 없습니다."
    exit 1
fi

echo "[1/3] udev 규칙 복사: $RULE_DST"
sudo cp "$RULE_SRC" "$RULE_DST"

echo "[2/3] 규칙 리로드"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "[3/3] 확인"
sleep 1
if [ -e /dev/imu ] && [ -e /dev/wheel_mcu ]; then
    ls -l /dev/imu /dev/wheel_mcu
    echo ""
    echo "성공: /dev/imu 와 /dev/wheel_mcu 가 생성되었습니다."
else
    echo ""
    echo "경고: 링크가 아직 안 보입니다. 다음을 확인하세요."
    echo "  1) IMU가 USB-C 포트에, Arduino Mega가 USB-A 포트에 꽂혀 있는가"
    echo "  2) ch341 드라이버가 로드되어 있는가:  lsmod | grep ch341"
    echo "  3) 현재 포트 경로가 규칙과 맞는가:"
    echo "     udevadm info -a -n /dev/ttyCH341USB0 | grep 'KERNELS==' | head -5"
    echo "     (값이 다르면 99-robot-serial.rules 의 KERNELS 값을 수정 후 재실행)"
fi
