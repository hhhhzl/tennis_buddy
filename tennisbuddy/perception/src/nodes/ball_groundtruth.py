#!/usr/bin/env python3
import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry


class BallGroundTruthRosGz(Node):
    def __init__(self):
        super().__init__('ball_groundtruth_rosgz')
        self.declare_parameter('world', 'tennis_world')
        self.declare_parameter('in_topic', '/world/default/pose/info')
        self.declare_parameter('out_topic', '/ball_positions')
        self.declare_parameter('spawned_truth_topic', '/ball_spawned_truth')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('z_max', 0.25)
        self.declare_parameter('xmin', -4.0)
        self.declare_parameter('xmax', 4.0)
        self.declare_parameter('ymin', -7.0)
        self.declare_parameter('ymax', 7.0)
        self.declare_parameter('max_balls', 50)
        self.declare_parameter('robot_odom_topic', '/odometry/filtered')
        self.declare_parameter('robot_exclusion_radius', 0.6)
        self.declare_parameter('truth_match_tolerance', 0.05)

        self.world = self.get_parameter('world').get_parameter_value().string_value
        self.in_topic = f'/world/{self.world}/pose/info'
        self.out_topic = self.get_parameter('out_topic').value
        self.truth_topic = self.get_parameter('spawned_truth_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('target_frame').value
        self.z_max = float(self.get_parameter('z_max').value)
        self.xmin = float(self.get_parameter('xmin').value)
        self.xmax = float(self.get_parameter('xmax').value)
        self.ymin = float(self.get_parameter('ymin').value)
        self.ymax = float(self.get_parameter('ymax').value)
        self.max_balls = int(self.get_parameter('max_balls').value)
        self.robot_odom_topic = self.get_parameter('robot_odom_topic').get_parameter_value().string_value
        self.robot_exclusion_radius = float(self.get_parameter('robot_exclusion_radius').value)
        self.truth_match_tol = float(self.get_parameter('truth_match_tolerance').value)

        self.robot_xy: Optional[tuple[float, float]] = None
        self.truth_positions: Optional[List[tuple[float, float]]] = None

        self.sub = self.create_subscription(PoseArray, self.in_topic, self.on_poses, 10)
        self.pub = self.create_publisher(PoseArray, self.out_topic, 10)

        if self.robot_odom_topic:
            self.sub_odom = self.create_subscription(
                Odometry, self.robot_odom_topic, self.on_odom, 10)
            self.get_logger().info(
                f'[gt_rosgz] tracking robot pose from {self.robot_odom_topic} '
                f'(exclusion radius={self.robot_exclusion_radius:.2f} m)')
        else:
            self.sub_odom = None
            self.get_logger().warn('[gt_rosgz] robot_odom_topic empty; robot exclusion disabled')

        if self.truth_topic:
            self.sub_truth = self.create_subscription(
                PoseArray, self.truth_topic, self.on_truth, 10)
            self.get_logger().info(
                f'[gt_rosgz] receiving {len(self.truth_positions)} ball truth from {self.truth_topic} (tol={self.truth_match_tol:.2f} m)')
        else:
            self.sub_truth = None

        self.get_logger().info(
            f'[gt_rosgz] listen {self.in_topic} -> publish {self.out_topic} (frame={self.frame_id})')

    def on_odom(self, msg: Odometry):
        self.robot_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def on_truth(self, msg: PoseArray):
        positions = []
        for p in msg.poses:
            positions.append((float(p.position.x), float(p.position.y)))
        self.truth_positions = positions
        self.get_logger().info(f'[gt_rosgz] updated truth list ({len(positions)} balls)')

    def _matches_truth(self, x: float, y: float) -> bool:
        if not self.truth_positions:
            return True
        for tx, ty in self.truth_positions:
            if x - tx <= self.truth_match_tol and y - ty <= self.truth_match_tol:
                return True
        return False

    def on_poses(self, msg: PoseArray):
        filtered = PoseArray()
        filtered.header = msg.header
        filtered.header.frame_id = self.frame_id

        candidates = []
        for p in msg.poses:
            px = float(p.position.x)
            py = float(p.position.y)
            pz = float(p.position.z)

            if pz > self.z_max:
                continue
            if not (self.xmin <= px <= self.xmax and
                    self.ymin <= py <= self.ymax):
                continue
            if not self._matches_truth(px, py):
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
            q.orientation.w = 1.0
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
