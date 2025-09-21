from setuptools import setup, find_packages
package_name = 'tennisbuddy_perception'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', [
            'launch/perception_gazebo_gt.launch.py',
            'launch/ball_spawner.launch.py',
        ]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Perception helpers for ros_gz: ball spawner & groundtruth',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ball_spawner = tennisbuddy_perception.nodes.ball_spawner:main',
            'ball_groundtruth = tennisbuddy_perception.nodes.ball_groundtruth:main',
        ],
    },
)