#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        
        # Subscribe to the trajectory topic
        self.create_subscription(
            PoseArray,
            '/trajectory',
            self.trajectory_callback,
            10
        )
        
        # Create a publisher for the marker
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 1)

    def trajectory_callback(self, msg):
        # Create a line strip marker for the path
        line_strip = Marker()
        line_strip.header = msg.header
        line_strip.ns = "path_line"
        line_strip.id = 0
        line_strip.type = Marker.LINE_STRIP
        line_strip.action = Marker.ADD
        line_strip.scale.x = 0.02  # Line width
        line_strip.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # Orange color
        line_strip.pose.orientation.w = 1.0
        
        # Add all points to the line strip
        for pose in msg.poses:
            line_strip.points.append(pose.position)
        
        # Publish the line strip marker
        self.marker_pub.publish(line_strip)
        self.get_logger().info('Published path visualization marker')

def main(args=None):
    rclpy.init(args=args)
    path_visualizer = PathVisualizer()
    try:
        rclpy.spin(path_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        path_visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()