#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import EntityState


class BallGroundTruthRosGz(Node):
    def __init__(self):
        super().__init__('ball_groundtruth_rosgz')
        self.declare_parameter('world', 'tennis_world')
        self.declare_parameter('in_topic', '/world/default/pose/info')
        self.declare_parameter('out_topic', '/ball_positions')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('z_max', 0.25)
        self.declare_parameter('xmin', -4.0)
        self.declare_parameter('xmax', 4.0)
        self.declare_parameter('ymin', -7.0)
        self.declare_parameter('ymax', 7.0)
        self.declare_parameter('max_balls', 50)
        self.declare_parameter('robot_odom_topic', '/odometry/filtered')
        self.declare_parameter('robot_exclusion_radius', 0.6)
        self.declare_parameter('ball_name_prefix', 'tennis_ball_')
        self.declare_parameter('entity_topic', '')

        self.world = self.get_parameter('world').get_parameter_value().string_value
        self.in_topic = f'/world/{self.world}/pose/info'
        self.out_topic = self.get_parameter('out_topic').value
        self.frame_id = self.get_parameter('target_frame').value
        self.z_max = float(self.get_parameter('z_max').value)
        self.xmin = float(self.get_parameter('xmin').value)
        self.xmax = float(self.get_parameter('xmax').value)
        self.ymin = float(self.get_parameter('ymin').value)
        self.ymax = float(self.get_parameter('ymax').value)
        self.max_balls = int(self.get_parameter('max_balls').value)

        self.robot_odom_topic = self.get_parameter('robot_odom_topic').get_parameter_value().string_value
        self.robot_exclusion_radius = float(self.get_parameter('robot_exclusion_radius').value)
        self.ball_name_prefix = self.get_parameter('ball_name_prefix').get_parameter_value().string_value
        entity_topic_param = self.get_parameter('entity_topic').get_parameter_value().string_value
        self.entity_topic = entity_topic_param or f'/world/{self.world}/dynamic_pose/info'

        self.robot_xy = None

        self.sub = self.create_subscription(EntityState, self.entity_topic, self.on_entity_state, 10)
        self.pub = self.create_publisher(PoseArray, self.out_topic, 10)

        if self.robot_odom_topic:
            self.sub_odom = self.create_subscription(
                Odometry, self.robot_odom_topic, self.on_odom, 10)
            self.get_logger().info(
                f'[gt_rosgz] tracking robot pose from {self.robot_odom_topic} (exclusion radius={self.robot_exclusion_radius:.2f} m)')
        else:
            self.sub_odom = None
            self.get_logger().warn('[gt_rosgz] robot_odom_topic empty; robot exclusion disabled')

        self.get_logger().info(
            f'[gt_rosgz] listen {self.entity_topic} -> publish {self.out_topic} (frame={self.frame_id})')

    def on_odom(self, msg: Odometry):
        self.robot_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def on_entity_state(self, msg: EntityState):
        if not msg.entities:
            return

        filtered = PoseArray()
        filtered.header.frame_id = self.frame_id
        filtered.header.stamp = self.get_clock().now().to_msg()

        candidates = []
        prefix = self.ball_name_prefix or ''
        for entity in msg.entities:
            name = getattr(entity, 'name', '')
            if prefix and not name.startswith(prefix):
                continue
            pose = getattr(entity, 'pose', None)
            if pose is None:
                continue

            px = pose.position.x
            py = pose.position.y
            pz = pose.position.z

            if pz > self.z_max:
                continue
            if not (self.xmin <= px <= self.xmax and
                    self.ymin <= py <= self.ymax):
                continue

            if self.robot_xy is not None:
                dx = px - self.robot_xy[0]
                dy = py - self.robot_xy[1]
                dist_robot = math.hypot(dx, dy)
                if dist_robot < self.robot_exclusion_radius:
                    continue
                distance_metric = dist_robot
            else:
                distance_metric = math.hypot(px, py)

            q = Pose()
            q.position.x = px
            q.position.y = py
            q.position.z = 0.0
            q.orientation.x = pose.orientation.x
            q.orientation.y = pose.orientation.y
            q.orientation.z = pose.orientation.z
            q.orientation.w = pose.orientation.w or 1.0
            candidates.append((distance_metric, q))

        if not candidates:
            return

        candidates.sort(key=lambda item: item[0])
        selected = [pose for _, pose in candidates[:self.max_balls]]

        if not selected:
            return

        filtered.poses.extend(selected)

        self.pub.publish(filtered)


def main():
    rclpy.init()
    node = BallGroundTruthRosGz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
