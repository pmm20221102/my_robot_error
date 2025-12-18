import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.executors import MultiThreadedExecutor
import random
import math

class FakeLidarPublisher(Node):
    def __init__(self, topic_name, angle_min, angle_max):
        super().__init__('fake_lidar_' + topic_name)
        self.publisher = self.create_publisher(LaserScan, topic_name, 10)
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.timer = self.create_timer(5, self.publish_scan)

    def publish_scan(self):
        msg = LaserScan()
        msg.header.frame_id = 'base_link'
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        num_points = int((self.angle_max - self.angle_min) / (math.pi / 6))+ 1 
        msg.angle_increment = (self.angle_max - self.angle_min) / (num_points - 1)
        msg.range_min = 0.1
        msg.range_max = 5.0
        msg.ranges = [random.uniform(0.9, 1.9) for _ in range(num_points)]

        self.publisher.publish(msg)
        self.get_logger().info(f'Published fake scan on {self.publisher.topic_name}')
        ranges_str = ', '.join(f'{r:.2f}' for r in msg.ranges)
        self.get_logger().info(f'[{self.get_name()}] Ranges: {ranges_str}')


def main():
    rclpy.init()
    lidar1 = FakeLidarPublisher('lidar1',math.radians(30), math.pi)
    lidar2 = FakeLidarPublisher('lidar2',math.radians(210), math.radians(360))

    executor = MultiThreadedExecutor()
    executor.add_node(lidar1)
    executor.add_node(lidar2)
   
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        lidar1.destroy_node()
        lidar2.destroy_node()
        rclpy.shutdown()
