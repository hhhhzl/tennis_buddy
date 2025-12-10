#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Exploration Waypoint Manager
generate and manage exploration waypoint sequence
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterUninitializedException
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry


def yaw_to_quat(yaw: float) -> Quaternion:
    """Convert yaw angle to quaternion"""
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class ExplorationWaypointManager(Node):
    """
    manage the generation and update of exploration waypoint
    support predefined waypoint list or automatic generation based on grid
    """
    
    def __init__(self):
        super().__init__('exploration_waypoint_manager')
        
        # Parameters
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('waypoint_spacing', 3.0)  # meters
        self.declare_parameter('exploration_area_min_x', -10.0)
        self.declare_parameter('exploration_area_max_x', 10.0)
        self.declare_parameter('exploration_area_min_y', -10.0)
        self.declare_parameter('exploration_area_max_y', 10.0)
        self.declare_parameter('use_predefined_waypoints', True)
        self.declare_parameter('predefined_waypoints', [])  # List of [x, y, yaw]
        
        self.frame_id = self.get_parameter('frame_id').value
        self.waypoint_spacing = float(self.get_parameter('waypoint_spacing').value)
        self.area_min_x = float(self.get_parameter('exploration_area_min_x').value)
        self.area_max_x = float(self.get_parameter('exploration_area_max_x').value)
        self.area_min_y = float(self.get_parameter('exploration_area_min_y').value)
        self.area_max_y = float(self.get_parameter('exploration_area_max_y').value)
        self.use_predefined = bool(self.get_parameter('use_predefined_waypoints').value)
        
        # Handle predefined_waypoints parameter - may not be set in YAML
        try:
            self.predefined_waypoints = self.get_parameter('predefined_waypoints').value
        except ParameterUninitializedException:
            self.predefined_waypoints = []
            self.get_logger().warn(
                '[exploration_waypoint_manager] predefined_waypoints parameter not found, using empty list'
            )
        
        # Publisher
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pub_waypoints = self.create_publisher(PoseArray, '/exploration_waypoints', qos)
        
        # Subscriber (for robot position, optional)
        self.sub_odom = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.on_odom,
            qos
        )
        
        self.robot_xy = None
        self.current_waypoints = None
        
        # Generate initial waypoints
        self.generate_waypoints()
        
        # Publish waypoints periodically
        self.create_timer(1.0, self.publish_waypoints)
        
        self.get_logger().info(
            f'[exploration_waypoint_manager] Initialized. '
            f'Area: [{self.area_min_x}, {self.area_min_y}] to [{self.area_max_x}, {self.area_max_y}], '
            f'Spacing: {self.waypoint_spacing}m'
        )
    
    def on_odom(self, msg: Odometry):
        """Update robot position"""
        self.robot_xy = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ], dtype=float)
    
    def generate_waypoints(self):
        """generate exploration waypoint sequence"""
        if self.use_predefined and len(self.predefined_waypoints) > 0:
            # use predefined waypoint
            self.current_waypoints = self._parse_predefined_waypoints()
            self.get_logger().info(
                f'[exploration_waypoint_manager] Using {len(self.current_waypoints)} predefined waypoints'
            )
        else:
            # automatically generate grid waypoint
            self.current_waypoints = self._generate_grid_waypoints()
            self.get_logger().info(
                f'[exploration_waypoint_manager] Generated {len(self.current_waypoints)} grid waypoints'
            )
    
    def _parse_predefined_waypoints(self):
        """parse predefined waypoint list"""
        waypoints = []
        for wp in self.predefined_waypoints:
            if len(wp) >= 2:
                x = float(wp[0])
                y = float(wp[1])
                yaw = float(wp[2]) if len(wp) >= 3 else 0.0
                
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = 0.0
                pose.orientation = yaw_to_quat(yaw)
                waypoints.append(pose)
        return waypoints
    
    def _generate_grid_waypoints(self):
        """automatically generate grid waypoint"""
        waypoints = []
        
        # calculate grid size
        width = self.area_max_x - self.area_min_x
        height = self.area_max_y - self.area_min_y
        
        num_x = max(2, int(width / self.waypoint_spacing) + 1)
        num_y = max(2, int(height / self.waypoint_spacing) + 1)
        
        # generate grid points
        xs = np.linspace(self.area_min_x, self.area_max_x, num_x)
        ys = np.linspace(self.area_min_y, self.area_max_y, num_y)
        
        # zigzag pattern to improve exploration efficiency
        for i, y in enumerate(ys):
            if i % 2 == 0:
                x_list = xs
            else:
                x_list = reversed(xs)
            
            for j, x in enumerate(x_list):
                pose = Pose()
                pose.position.x = float(x)
                pose.position.y = float(y)
                pose.position.z = 0.0

                # calculate orientation: towards the next waypoint
                if j < len(x_list) - 1:
                    # not the last one, towards the next waypoint
                    next_x = x_list[j+1] if i % 2 == 0 else x_list[j-1]
                    yaw = math.atan2(y - y, next_x - x) if i % 2 == 0 else math.atan2(y - y, x - next_x)
                else:
                    # the last one, towards the moving direction
                    yaw = 0.0 if i % 2 == 0 else math.pi
                pose.orientation = yaw_to_quat(0.0)
                waypoints.append(pose)
        
        return waypoints
    
    def get_waypoints(self) -> PoseArray:
        """get current waypoint sequence"""
        if self.current_waypoints is None:
            self.generate_waypoints()
        
        pose_array = PoseArray()
        pose_array.header.frame_id = self.frame_id
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.poses = self.current_waypoints
        
        return pose_array
    
    def publish_waypoints(self):
        """publish waypoint sequence"""
        if self.current_waypoints is None:
            return
        
        pose_array = self.get_waypoints()
        self.pub_waypoints.publish(pose_array)
    
    def get_next_unvisited_waypoint(self, visited_positions, threshold=1.0):
        """
        get the next unvisited waypoint
        
        Args:
            visited_positions: visited position list [[x1, y1], [x2, y2], ...]
            threshold: threshold to determine if a position is visited (meters)
        
        Returns:
            Pose or None
        """
        if self.current_waypoints is None:
            return None
        
        visited_set = set()
        for vp in visited_positions:
            visited_set.add((round(vp[0] / threshold), round(vp[1] / threshold)))
        
        for wp in self.current_waypoints:
            wp_key = (round(wp.position.x / threshold), round(wp.position.y / threshold))
            if wp_key not in visited_set:
                return wp
        
        return None  # all waypoints are visited


def main():
    rclpy.init()
    node = ExplorationWaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

