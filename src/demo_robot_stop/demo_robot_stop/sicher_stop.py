import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class SicherStopNode(Node):
    def __init__(self,node_name):
        super().__init__(node_name)

        self.lidar1_data = []
        self.lidar2_data = []

        self.create_subscription(LaserScan, 'lidar1', self.lidar1_callback, 10)
        self.create_subscription(LaserScan, 'lidar2', self.lidar2_callback, 10)
        self.publisher = self.create_publisher(Bool, 'sicher_stop', 10)
        self.create_timer(5, self.check_obstacles)

    def lidar1_callback(self, msg):
        self.lidar1_data = msg.ranges

    def lidar2_callback(self, msg):
        self.lidar2_data = msg.ranges

    def check_obstacles(self):
        stop = False
        if any(0.0 < r <= 1.0 for r in self.lidar1_data):
            stop = True
        if any(0.0 < r <= 1.0 for r in self.lidar2_data):
            stop = True

        msg = Bool()
        msg.data = stop
        self.publisher.publish(msg)
        self.get_logger().warn(f"Sicher_stop: {stop}")

def main():
    rclpy.init()
    node = SicherStopNode("sicher_stop")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()   
        rclpy.shutdown()