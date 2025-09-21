#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist


def quat_to_yaw(q):
    # tf_transformations
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TrajectoryTracker(Node):
    def __init__(self):
        super().__init__('trajectory_tracker')

        self.declare_parameter('lookahead', 0.6)
        self.declare_parameter('v_max', 1.0)
        self.declare_parameter('vy_max', 0.0)
        self.declare_parameter('w_max', 2.0)
        self.declare_parameter('end_slow_radius', 0.2)
        self.lookahead = float(self.get_parameter('lookahead').value)
        self.vx_max = float(self.get_parameter('v_max').value)
        self.vy_max = float(self.get_parameter('vy_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.end_slow = float(self.get_parameter('end_slow_radius').value)

        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        self.sub_path = self.create_subscription(Path, '/planned_path', self.on_path, qos_reliable)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.on_odom, qos_reliable)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.path_pts = []
        self.idx = 0
        self.robot_xy = None
        self.yaw = 0.0
        self.create_timer(1.0 / 50.0, self.tick)  # 50 Hz
        self.get_logger().info('[trajectory_tracker] ready.')

    def on_path(self, msg: Path):
        self.path_pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.idx = 0

    def on_odom(self, msg: Odometry):
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.yaw = quat_to_yaw(msg.pose.pose.orientation)

    def tick(self):
        if not self.path_pts or self.robot_xy is None:
            return

        px, py = self.robot_xy
        window = self.path_pts[self.idx:self.idx + 60] if self.idx < len(self.path_pts) else []
        if window:
            dists = [(i, (px - x) ** 2 + (py - y) ** 2) for i, (x, y) in enumerate(window)]
            self.idx += min(dists, key=lambda t: t[1])[0]

        goal = self.path_pts[-1]
        for j in range(self.idx, len(self.path_pts)):
            if math.hypot(self.path_pts[j][0] - px, self.path_pts[j][1] - py) >= self.lookahead:
                goal = self.path_pts[j]
                self.idx = j
                break

        dx, dy = goal[0] - px, goal[1] - py
        ex = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy
        ey = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy
        yaw_err = math.atan2(ey, ex)

        cmd = Twist()
        cmd.linear.x = float(np.clip(0.9 * ex, -self.vx_max, self.vx_max))
        cmd.linear.y = float(np.clip(0.9 * ey, -self.vy_max, self.vy_max))
        cmd.angular.z = float(np.clip(1.8 * yaw_err, -self.w_max, self.w_max))

        end_dist = math.hypot(self.path_pts[-1][0] - px, self.path_pts[-1][1] - py)
        if end_dist < self.end_slow:
            cmd.linear.x *= 0.3
            cmd.linear.y *= 0.3
            cmd.angular.z *= 0.5

        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = TrajectoryTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
