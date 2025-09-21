from setuptools import setup, find_packages
from pathlib import Path

ros_pkg_name = 'tennisbuddy_perception'
py_pkg_root = 'src'
py_pkg_name = 'perception'

Path('resource').mkdir(exist_ok=True)
Path(f'resource/{ros_pkg_name}').touch(exist_ok=True)

setup(
    name=ros_pkg_name,
    version='0.1.0',
    packages=find_packages(where=py_pkg_root, include=[py_pkg_name, py_pkg_name + '.*']),
    package_dir={'': py_pkg_root},
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{ros_pkg_name}']),
        ('share/' + ros_pkg_name + '/launch', [
            'launch/ball_spawner.launch.py',
            'launch/perception_gazebo_gt.launch.py',
        ]),
        ('share/' + ros_pkg_name + '/configs', []),
        ('share/' + ros_pkg_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ball_spawner = perception.nodes.ball_spawner:main',
            'ball_groundtruth = perception.nodes.ball_groundtruth:main',
            'mock_spawn_service = perception.nodes.mock_spawn_service:main'
        ],
    },
)
