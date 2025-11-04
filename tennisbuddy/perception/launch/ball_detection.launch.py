#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # --- Launch args ---
    model = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(
            get_package_share_directory('tennisbuddy_perception'),
            'resource', 'best.pt'
        ),
        description='Path to YOLO model .pt'
    )
    show_viz = DeclareLaunchArgument(
        'show_viz', default_value='false',
        description='Whether to show OpenCV visualization window'
    )
    start_realsense = DeclareLaunchArgument(
        'start_realsense', default_value='true',
        description='Start realsense2_camera driver in this launch'
    )
    color_topic = DeclareLaunchArgument(
        'color_topic', default_value='/camera/color/image_raw',
        description='RGB topic'
    )
    depth_topic = DeclareLaunchArgument(
        'depth_topic', default_value='/camera/aligned_depth_to_color/image_raw',
        description='Aligned depth topic'
    )
    camera_info_topic = DeclareLaunchArgument(
        'camera_info_topic', default_value='/camera/color/camera_info',
        description='Camera info topic for intrinsics'
    )
    conf = DeclareLaunchArgument(
        'conf', default_value='0.25',
        description='Detection confidence threshold'
    )

    # --- Substitutions ---
    model_path = LaunchConfiguration('model')
    show_v = LaunchConfiguration('show_viz')
    start_rs = LaunchConfiguration('start_realsense')
    color = LaunchConfiguration('color_topic')
    depth = LaunchConfiguration('depth_topic')
    caminfo = LaunchConfiguration('camera_info_topic')
    conf_thr = LaunchConfiguration('conf')

    # --- Detector node ---
    detector = Node(
        package='tennisbuddy_perception',
        executable='yolo_realsense_detector',
        output='screen',
        parameters=[{
            'model': model_path,
            'show_viz': show_v,
            'color_topic': color,
            'depth_topic': depth,
            'camera_info_topic': caminfo,
            'conf': conf_thr,
        }]
    )

    # --- Receiver node ---
    receiver = Node(
        package='tennisbuddy_perception',
        executable='detector_receiver',
        output='screen'
    )

    # --- Optional RealSense driver (align depth=true) ---
    rs_share = get_package_share_directory('realsense2_camera')
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rs_share, 'launch', 'rs_launch.py')),
        condition=IfCondition(start_rs),
        launch_arguments={
            'align_depth': 'true',
            # add more rs_launch args here if needed (e.g., enable_color, enable_depth, etc.)
        }.items()
    )

    return LaunchDescription([
        model, show_viz, start_realsense, color_topic, depth_topic, camera_info_topic, conf,
        rs_launch,
        detector,
        receiver,
    ])