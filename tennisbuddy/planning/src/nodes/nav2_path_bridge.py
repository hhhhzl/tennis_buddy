#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path


class Nav2PathBridge(Node):
    """
    Sub /planned_path，path update：(Only Do Planning)
      1) cancel last FollowPath task
      2) send FollowPath.Goal(path=msg) use tennisbuddy algorithms
    Nav2 （Regulated PP / TEB / MPC）outputs /cmd_vel。
    """

    def __init__(self):
        super().__init__('nav2_path_bridge')
        self.declare_parameter('controller_id', '')
        self.declare_parameter('goal_checker_id', '')
        self.controller_id = self.get_parameter('controller_id').value
        self.goal_checker_id = self.get_parameter('goal_checker_id').value

        qos_rel = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        self.sub_path = self.create_subscription(Path, '/planned_path', self.on_path, qos_rel)

        self.ac = ActionClient(self, FollowPath, 'follow_path')
        self.current_goal_handle = None
        self.ready = False

        self.get_logger().info('[nav2_path_bridge] waiting for follow_path server...')
        self.ac.wait_for_server()
        self.ready = True
        self.get_logger().info('[nav2_path_bridge] ready.')

    def on_path(self, msg: Path):
        if not self.ready:
            return

        # Cancel last task
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.current_goal_handle = None

        goal = FollowPath.Goal()
        goal.path = msg
        if self.controller_id:
            goal.controller_id = self.controller_id
        if self.goal_checker_id:
            goal.goal_checker_id = self.goal_checker_id

        self.get_logger().info(f'[nav2_path_bridge] Send FollowPath (poses={len(msg.poses)})')
        future = self.ac.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn('[nav2_path_bridge] FollowPath goal rejected')
            self.current_goal_handle = None
            return
        self.current_goal_handle = gh
        result_future = gh.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_feedback(self, fb):
        pass

    def on_result(self, fut):
        res = fut.result()
        self.get_logger().info(f'[nav2_path_bridge] FollowPath finished: {res}')
        self.current_goal_handle = None


def main():
    rclpy.init()
    node = Nav2PathBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
