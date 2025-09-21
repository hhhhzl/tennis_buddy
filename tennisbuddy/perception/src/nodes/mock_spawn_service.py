#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity


class MockSpawnService(Node):
    def __init__(self):
        super().__init__('mock_spawn_service')
        self.srv = self.create_service(
            SpawnEntity, '/world/default/create', self.cb)
        self.get_logger().info('Mock spawn service ready at /world/default/create')

    def cb(self, request, response):
        self.get_logger().info(f"Received spawn request for {request.entity_factory.name}")
        response.success = True
        response.status_message = "mock spawn success"
        return response


def main():
    rclpy.init()
    node = MockSpawnService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
