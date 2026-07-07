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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jh',
    maintainer_email='you@example.com',
    description='Depth camera event to map-point conversion via TF (change_point.py)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'change_point = ship_ugv_perception.change_point:main',
        ],
    },
)
