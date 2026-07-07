from setuptools import setup

package_name = 'uwb_map_calibration'

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
    description='map <-> uwb_frame calibration with repeatable Trigger service',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'calibration_node = uwb_map_calibration.calibration_node:main',
        ],
    },
)
