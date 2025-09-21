#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose


class BallGroundTruthRosGz(Node):
    def __init__(self):
        super().__init__('ball_groundtruth_rosgz')
        self.declare_parameter('world', 'default')
        self.declare_parameter('in_topic', '/world/default/pose/info')
        self.declare_parameter('out_topic', '/ball_positions')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('z_max', 0.25)
        self.declare_parameter('xmin', -4.0)
        self.declare_parameter('xmax', 4.0)
        self.declare_parameter('ymin', -7.0)
        self.declare_parameter('ymax', 7.0)
        self.declare_parameter('max_balls', 50)

        self.in_topic = self.get_parameter('in_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.frame_id = self.get_parameter('target_frame').value
        self.z_max = float(self.get_parameter('z_max').value)
        self.xmin = float(self.get_parameter('xmin').value)
        self.xmax = float(self.get_parameter('xmax').value)
        self.ymin = float(self.get_parameter('ymin').value)
        self.ymax = float(self.get_parameter('ymax').value)
        self.max_balls = int(self.get_parameter('max_balls').value)

        self.sub = self.create_subscription(PoseArray, self.in_topic, self.on_poses, 10)
        self.pub = self.create_publisher(PoseArray, self.out_topic, 10)

        self.get_logger().info(
            f'[gt_rosgz] listen {self.in_topic} -> publish {self.out_topic} (frame={self.frame_id})')

    def on_poses(self, msg: PoseArray):
        filtered = PoseArray()
        filtered.header = msg.header
        filtered.header.frame_id = self.frame_id

        for p in msg.poses:
            if p.position.z > self.z_max:
                continue
            if not (self.xmin <= p.position.x <= self.xmax and
                    self.ymin <= p.position.y <= self.ymax):
                continue
            q = Pose()
            q.position.x, q.position.y, q.position.z = p.position.x, p.position.y, 0.0
            q.orientation.w = 1.0
            filtered.poses.append(q)
            if len(filtered.poses) >= self.max_balls:
                break

        if filtered.poses:
            self.pub.publish(filtered)


def main():
    rclpy.init()
    node = BallGroundTruthRosGz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
