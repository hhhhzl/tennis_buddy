from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'tennisbuddy_planning'

# Install launch and config files into share/<pkg>/
launch_files = glob(os.path.join('launch', '*.launch.py')) + [
    os.path.join('launch', 'navigation_launch.py'),
    os.path.join('launch', 'nav2_backend.py'),
]
config_files = glob(os.path.join('config', '*.yaml'))

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', launch_files),
    ('share/' + package_name + '/config', config_files),
]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    description='Planning stack for TennisBuddy: planner/tracker and Nav2 bridges.',
    license='Apache-2.0',
    tests_require=['pytest'],
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: Apache Software License'
    ],
    entry_points={
        'console_scripts': [
            # nodes/
            'planner_node = tennisbuddy_planning.nodes.planner_node:main',
            'trajectory_tracker = tennisbuddy_planning.nodes.trajectory_tracker:main',
            'nav2_goal_pusher = tennisbuddy_planning.nodes.nav2_goal_pusher:main',
            'nav2_path_bridge = tennisbuddy_planning.nodes.nav2_path_bridge:main',
        ],
    },
)
