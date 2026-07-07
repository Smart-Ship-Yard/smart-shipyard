from glob import glob

from setuptools import setup

package_name = 'ship_ugv_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # glob 사용: weights/*.pt가 없으면 빈 목록이 되어 빌드가 실패하지 않음.
        # (best.pt는 바이너리라 git에 커밋하지 않으므로, clone 직후엔 파일이 없는
        #  것이 정상. 하드코딩 ['weights/best.pt'] 방식은 이때 빌드 전체를 죽였음)
        # 가중치를 넣은 뒤에는 colcon build를 다시 실행해야 share로 복사됨.
        ('share/' + package_name + '/weights', glob('weights/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jh',
    maintainer_email='you@example.com',
    description='Depth camera event to map-point conversion via TF (change_point.py)'
                ' + YOLO/Depth publisher (yolo_depth_publisher.py)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'change_point = ship_ugv_perception.change_point:main',
            'yolo_depth_publisher = ship_ugv_perception.yolo_depth_publisher:main',
        ],
    },
)
