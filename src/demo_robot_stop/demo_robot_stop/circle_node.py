import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math


class CircleMarkerNode(Node):
    def __init__(self):
        super().__init__('circle_marker_node')
        self.publisher = self.create_publisher(Marker, 'circle_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_circle_marker)
        self.get_logger().info("Circle Marker Node started.")

    def publish_circle_marker(self):
        marker = Marker()
        marker.header.frame_id = 'base_link' 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "circle"
        marker.id = 0
        marker.type = Marker.LINE_STRIP  
        marker.action = Marker.ADD

        
        marker.scale.x = 0.01  
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 1.0

        
        radius = 1.0
        points = []
        for i in range(361):  
            angle = math.radians(i)
            p = Point()
            p.x = radius * math.cos(angle)
            p.y = radius * math.sin(angle)
            p.z = 0.0
            points.append(p)
        marker.points = points

        self.publisher.publish(marker)


def main():
    rclpy.init()
    node = CircleMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
