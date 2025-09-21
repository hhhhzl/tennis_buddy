from setuptools import setup, find_packages
from pathlib import Path

ros_pkg_name = 'tennisbuddy_planning'
py_pkg_root = 'src'
py_pkg_name = 'planning'

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
            'launch/planning_tennisbuddy.launch.py',
            'launch/planning_nav2_path.launch.py',
            'launch/planning_nav2_goal.launch.py',
        ]),
        ('share/' + ros_pkg_name + '/configs', []),
        ('share/' + ros_pkg_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'planner_node = planning.nodes.planner_node:main',
            'trajectory_tracker = planning.nodes.trajectory_tracker:main',
            'nav2_goal_pusher = planning.nodes.nav2_goal_pusher:main',
            'nav2_path_bridge = planning.nodes.nav2_path_bridge:main',
        ],
    },
)