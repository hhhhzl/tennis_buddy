#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path, Odometry


def straight_path(start_xy, goal_xy, n=40):
    xs = np.linspace(start_xy[0], goal_xy[0], n)
    ys = np.linspace(start_xy[1], goal_xy[1], n)
    return list(zip(xs, ys))


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        self.declare_parameter('replan_rate', 5.0)
        self.declare_parameter('pickup_distance', 0.35)
        self.declare_parameter('frame_id', 'map')
        self.replan_rate = float(self.get_parameter('replan_rate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.pickup_d = float(self.get_parameter('pickup_distance').value)

        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5)

        # SUB
        self.sub_balls = self.create_subscription(
            PoseArray, '/ball_positions', self.on_balls, qos_sensor)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.on_odom, qos_reliable)

        # PUB
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)

        # State
        self.robot_xy = None
        self.targets = np.zeros((0, 2), dtype=float)
        self.current_idx = None

        self.timer = self.create_timer(1.0 / max(1e-3, self.replan_rate), self.tick)
        self.get_logger().info('[planner_node] ready. replan_rate=%.2f Hz' % self.replan_rate)

    def on_balls(self, msg: PoseArray):
        arr = []
        for p in msg.poses:
            arr.append([p.position.x, p.position.y])
        self.targets = np.array(arr, dtype=float) if arr else np.zeros((0, 2))
        if self.targets.size == 0:
            self.current_idx = None
        elif self.current_idx is None or self.current_idx >= len(self.targets):
            self.current_idx = int(np.argmin(
                np.sum((self.targets - (self.robot_xy if self.robot_xy is not None else 0)) ** 2,
                       axis=1))) if self.robot_xy is not None else 0

    def on_odom(self, msg: Odometry):
        self.robot_xy = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y], dtype=float)

    def choose_next_goal(self):
        if self.targets.size == 0 or self.robot_xy is None:
            return None
        if self.current_idx is None:
            self.current_idx = int(np.argmin(np.sum((self.targets - self.robot_xy) ** 2, axis=1)))
        goal = self.targets[self.current_idx]
        return goal

    def compute_pickup_pose(self, robot_xy, ball_xy, d):
        theta = math.atan2(ball_xy[1] - robot_xy[1], ball_xy[0] - robot_xy[0])
        gx = ball_xy[0] - d * math.cos(theta)
        gy = ball_xy[1] - d * math.sin(theta)
        return np.array([gx, gy, theta], dtype=float)

    def tick(self):
        if self.robot_xy is None:
            return
        goal_xy = self.choose_next_goal()
        if goal_xy is None:
            return

        gpose = self.compute_pickup_pose(self.robot_xy, goal_xy, self.pickup_d)

        # TODO: use our paths
        points = straight_path(self.robot_xy, gpose[:2], n=40)

        # Pub Path
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        for (x, y) in points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_path.publish(path)


def main():
    rclpy.init()
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
