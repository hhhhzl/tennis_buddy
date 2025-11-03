#!/usr/bin/env python3
import random
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SpawnEntity  # /world/<world>/create

SDF_TPL = """<?xml version='1.0'?>
<sdf version='1.9'>
  <model name='{name}'>
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name='link'>
      <inertial><mass>{mass}</mass></inertial>
      <collision name='col'>
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        <surface>
          <bounce><restitution_coefficient>0.6</restitution_coefficient></bounce>
        </surface>
      </collision>
      <visual name='vis'>
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class BallSpawnerRosGz(Node):
    def __init__(self):
        super().__init__('ball_spawner_rosgz')
        self.declare_parameter('world', 'default')
        self.declare_parameter('count', 15)
        self.declare_parameter('xmin', -3.5)
        self.declare_parameter('xmax', 3.5)
        self.declare_parameter('ymin', -6.0)
        self.declare_parameter('ymax', 6.0)
        self.declare_parameter('z', 0.065)
        self.declare_parameter('radius', 0.065)
        self.declare_parameter('mass', 0.057)
        self.declare_parameter('name_prefix', 'tennis_ball_')
        self.declare_parameter('seed', 0)
        self.declare_parameter('color', [1.0, 1.0, 0.0])
        self.declare_parameter('allow_renaming', False)

        self.world = self.get_parameter('world').get_parameter_value().string_value
        self.count = int(self.get_parameter('count').value)
        self.xmin = float(self.get_parameter('xmin').value)
        self.xmax = float(self.get_parameter('xmax').value)
        self.ymin = float(self.get_parameter('ymin').value)
        self.ymax = float(self.get_parameter('ymax').value)
        self.z = float(self.get_parameter('z').value)
        self.radius = float(self.get_parameter('radius').value)
        self.mass = float(self.get_parameter('mass').value)
        self.prefix = str(self.get_parameter('name_prefix').value)
        self.seed = int(self.get_parameter('seed').value)
        color = self.get_parameter('color').value
        self.r, self.g, self.b = [float(c) for c in color]
        self.allow_renaming = bool(self.get_parameter('allow_renaming').value)

        if self.seed > 0:
            random.seed(self.seed)

        # ros_gz ：/world/<world>/create
        service_name = f'/world/{self.world}/create'
        self.cli = self.create_client(SpawnEntity, service_name)
        self.get_logger().info(f'[spawner] waiting for {service_name} ...')
        self.cli.wait_for_service()
        self.get_logger().info(f'[spawner] service available: {service_name}')

        self.spawn_all()

    def spawn_all(self):
        successes = 0
        for i in range(self.count):
            x = random.uniform(self.xmin, self.xmax)
            y = random.uniform(self.ymin, self.ymax)
            name = f'{self.prefix}{i:02d}'
            sdf = SDF_TPL.format(
                name=name, x=x, y=y, z=self.z, mass=self.mass, radius=self.radius,
                r=self.r, g=self.g, b=self.b
            )
            req = SpawnEntity.Request()
            req.entity_factory.name = name
            req.entity_factory.allow_renaming = self.allow_renaming
            req.entity_factory.sdf = sdf
            req.entity_factory.pose.position.x = x
            req.entity_factory.pose.position.y = y
            req.entity_factory.pose.position.z = self.z
            req.entity_factory.pose.orientation.w = 1.0

            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            res = future.result()
            if res and res.success:
                successes += 1
                self.get_logger().info(f'[spawner] spawned {name} at ({x:.2f},{y:.2f})')
            else:
                self.get_logger().warn(f'[spawner] FAILED: {name}')

        self.get_logger().info(f'[spawner] done. success={successes}/{self.count}')


def main():
    rclpy.init()
    node = BallSpawnerRosGz()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
