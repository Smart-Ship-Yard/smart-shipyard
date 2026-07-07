from setuptools import setup

package_name = 'uwb_dwm1001_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='jh',
    maintainer_email='you@example.com',
    description='DWM1001 UWB UART driver publishing gated pose to uwb_frame',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'uwb_ros2_publisher = uwb_dwm1001_driver.uwb_ros2_publisher:main',
        ],
    },
)
