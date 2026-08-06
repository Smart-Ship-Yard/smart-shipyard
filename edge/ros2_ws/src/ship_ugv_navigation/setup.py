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
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jg',
    maintainer_email='dhswjd2003@gmail.com',
    description='Nav2 autonomous patrol for ship_ugv (sim + real shared params)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'patrol_mission_node = ship_ugv_navigation.patrol_mission_node:main',
        ],
    },
)
