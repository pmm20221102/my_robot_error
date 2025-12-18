# sicher_marker_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

class SafetyMarkerNode(Node):
    def __init__(self):
        super().__init__('sicher_marker')
        self.sub = self.create_subscription(Bool, 'sicher_stop', self.callback, 10)
        self.pub = self.create_publisher(Marker, 'sicher_marker', 10)

    def callback(self, msg):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sicher"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        
        if msg.data:
            marker.color.r = 1.0  
            marker.color.g = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0  
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.pub.publish(marker)

def main():
    rclpy.init()
    node = SafetyMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()