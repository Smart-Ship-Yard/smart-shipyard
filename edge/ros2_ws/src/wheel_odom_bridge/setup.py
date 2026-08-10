from setuptools import setup

package_name = 'wheel_odom_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jh',
    maintainer_email='you@example.com',
    description='Arduino(BTS7960+JGB37-520 엔코더) 시리얼 브리지: '
                 'cmd_vel 수신 -> Arduino 전달, 엔코더 델타 수신 -> /wheel/odom 발행',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_odom_node = wheel_odom_bridge.wheel_odom_node:main',
        ],
    },
)
