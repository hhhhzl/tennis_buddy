#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseArray, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry


def yaw_to_quat(yaw: float) -> Quaternion:
    import math
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class Nav2GoalPusher(Node):
    def __init__(self):
        super().__init__('nav2_goal_pusher')
        self.declare_parameter('pickup_distance', 0.35)
        self.declare_parameter('frame_id', 'map')
        self.pickup_d = float(self.get_parameter('pickup_distance').value)
        self.frame_id = self.get_parameter('frame_id').value

        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        qos_rel = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        self.sub_balls = self.create_subscription(PoseArray, '/ball_positions', self.on_balls, qos_sensor)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.on_odom, qos_rel)

        # Action Client
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.robot_xy = None
        self.targets = np.zeros((0, 2))
        self.work_queue = []
        self.sending = False

        self.get_logger().info('[nav2_goal_pusher] ready. Waiting for navigate_to_pose server...')
        self.ac.wait_for_server()

        self.create_timer(0.5, self.tick)

    def on_odom(self, msg: Odometry):
        self.robot_xy = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y], dtype=float)

    def on_balls(self, msg: PoseArray):
        arr = []
        for p in msg.poses:
            arr.append([p.position.x, p.position.y])
        self.targets = np.array(arr, dtype=float) if arr else np.zeros((0, 2))
        self.rebuild_queue()

    def rebuild_queue(self):
        """Multi_target algorithm"""
        # TODO
        self.work_queue = []
        if self.targets.size == 0 or self.robot_xy is None:
            return
        remaining = list(range(len(self.targets)))
        pos = self.robot_xy.copy()
        while remaining:
            i = min(remaining, key=lambda k: np.sum((self.targets[k] - pos) ** 2))
            self.work_queue.append(i)
            pos = self.targets[i]
            remaining.remove(i)

    def compute_pickup_pose(self, robot_xy, ball_xy):
        theta = math.atan2(ball_xy[1] - robot_xy[1], ball_xy[0] - robot_xy[0])
        gx = ball_xy[0] - self.pickup_d * math.cos(theta)
        gy = ball_xy[1] - self.pickup_d * math.sin(theta)
        return gx, gy, theta

    def tick(self):
        if self.sending:
            return
        if not self.work_queue or self.robot_xy is None:
            return
        idx = self.work_queue.pop(0)
        ball = self.targets[idx]
        gx, gy, yaw = self.compute_pickup_pose(self.robot_xy, ball)

        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = float(gx)
        ps.pose.position.y = float(gy)
        ps.pose.orientation = yaw_to_quat(yaw)
        goal.pose = ps

        self.sending = True
        self.get_logger().info(f'[nav2_goal_pusher] Send goal -> ({gx:.2f}, {gy:.2f})')
        send_future = self.ac.send_goal_async(goal, feedback_callback=self.on_feedback)
        send_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, fut):
        goal_handle = fut.result()
        if not goal_handle.accepted:
            self.get_logger().warn('[nav2_goal_pusher] Goal rejected')
            self.sending = False
            return
        self.get_logger().info('[nav2_goal_pusher] Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_feedback(self, fb):
        pass

    def on_result(self, fut):
        result = fut.result().result
        self.get_logger().info(f'[nav2_goal_pusher] Goal finished with result: {result}')
        self.sending = False


def main():
    rclpy.init()
    node = Nav2GoalPusher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
