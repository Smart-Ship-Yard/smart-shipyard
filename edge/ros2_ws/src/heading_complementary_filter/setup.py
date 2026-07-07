from setuptools import setup

package_name = 'heading_complementary_filter'

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
    description='Circular complementary filter fusing IMU gyro yaw rate with UWB course-over-ground',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'complementary_filter_node = heading_complementary_filter.complementary_filter_node:main',
        ],
    },
)
