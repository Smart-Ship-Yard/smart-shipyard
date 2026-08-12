import os
from glob import glob

from setuptools import setup

package_name = 'ship_ugv_navigation'

# ★ data_files = "colcon build 할 때 install/ 로 복사할 파일 목록"
#   파이썬 코드(.py)는 자동으로 복사되지만, yaml/pgm/launch/world 같은 데이터
#   파일은 여기 적어주지 않으면 복사되지 않는다. 빌드는 성공하는데 실행 시
#   get_package_share_directory()로 찾으면 "파일 없음"이 나는 대표적 함정.
#   형식: (설치될 경로, [복사할 원본 파일들])
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # ★ glob('maps/*')로 쓰면 하위 폴더(calibration_records)까지 잡혀
        #   "can't copy ...: not a regular file" 빌드 에러가 난다. 파일만 지정한다.
        #   calibration_records는 맵 제작 기록이라 런타임에 읽지 않으므로 설치 제외.
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.yaml') + glob('maps/*.pgm')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # 행동트리(BT). navigation.launch.py가 절대경로로 bt_navigator에 넘긴다.
        (os.path.join('share', package_name, 'behavior_trees'),
            glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jg',
    maintainer_email='dhswjd2003@gmail.com',
    description='Nav2 autonomous patrol for ship_ugv (sim + real shared params)',
    license='Apache-2.0',
    # ※ entry_points는 "이 패키지에 어떤 실행 파일이 있는지" 등록하는 목록일 뿐,
    #   여기 적혀 있다고 자동으로 실행되지는 않는다.
    #   따라서 실물 배포 시 아래 항목을 지우거나 주석 처리할 필요가 없다.
    #   무엇이 실제로 뜨는지는 "어떤 launch 파일을 실행하느냐"가 결정한다.
    #   전체 실행 순서는 edge/docs/nav2_작업_정리.md "9. 실행 순서" 참조.
    entry_points={
        'console_scripts': [
            # ── [시뮬 전용] 아래 한 줄(fake_global_localization)만 해당 ──────
            #   map->odom TF 발행. 실물에서는 ekf_global이 같은 일을 한다.
            #   ⚠️ 실물에서 실행하면 ekf_global과 이중 발행되어 TF 트리가 깨진다.
            #   기동 위치가 sim_bringup.launch.py 하나뿐이고 그 launch를
            #   실물에서 실행하지 않으므로, 여기서 지울 필요는 없다.
            'fake_global_localization = '
            'ship_ugv_navigation.fake_global_localization:main',

            # ── [시뮬·실물 공용] 아래 둘은 양쪽에서 그대로 실행한다 ─────────
            'patrol_mission_node = '
            'ship_ugv_navigation.patrol_mission_node:main',
            'event_gate_node = '
            'ship_ugv_navigation.event_gate_node:main',
        ],
    },
)
