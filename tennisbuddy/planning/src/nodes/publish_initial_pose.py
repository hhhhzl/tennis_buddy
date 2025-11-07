#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw_to_quat(yaw: float):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_once')
        self.declare_parameter('x', -5.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_delay', 1.0)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        delay = float(self.get_parameter('publish_delay').value)
        self.timer = self.create_timer(delay, self.publish_initial_pose)
        self.published = False

    def publish_initial_pose(self):
        if self.published:
            return
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        yaw = float(self.get_parameter('yaw').value)
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quat(yaw)
        # basic covariance
        msg.pose.covariance = [0.25, 0, 0, 0, 0, 0,
                               0, 0.25, 0, 0, 0, 0,
                               0, 0, 0.0, 0, 0, 0,
                               0, 0, 0, 0.0, 0, 0,
                               0, 0, 0, 0, 0.0, 0,
                               0, 0, 0, 0, 0, 0.0685]
        self.pub.publish(msg)
        self.get_logger().info(f'[initial_pose_once] Published initial pose ({x:.2f}, {y:.2f}, yaw={yaw:.2f}) in {frame_id}')
        self.published = True
        self.destroy_timer(self.timer)


def main():
    rclpy.init()
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
