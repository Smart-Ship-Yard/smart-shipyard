from setuptools import setup

package_name = 'slam_map_alignment'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='jh',
    maintainer_email='you@example.com',
    description='Trajectory-correspondence based slam_map <-> map alignment (RANSAC rigid transform)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_map_alignment_node = slam_map_alignment.slam_map_alignment_node:main',
        ],
    },
)
