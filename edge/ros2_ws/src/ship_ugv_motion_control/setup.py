from setuptools import setup

package_name = 'ship_ugv_motion_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/motion_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jh',
    maintainer_email='you@example.com',
    description='거리/각도 지정 이동 컨트롤러 (odometry 피드백 기반)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_controller_node = ship_ugv_motion_control.motion_controller_node:main',
            'square_test_node = ship_ugv_motion_control.square_test_node:main',
            'keyboard_teleop_node = ship_ugv_motion_control.keyboard_teleop_node:main',
        ],
    },
)
