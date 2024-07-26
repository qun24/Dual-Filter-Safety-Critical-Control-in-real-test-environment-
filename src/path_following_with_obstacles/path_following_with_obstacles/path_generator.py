#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from transforms3d.euler import euler2quat

def generate_sine_trajectory(start_x, end_x, num_points, amplitude, frequency):
    x = np.linspace(start_x, end_x, num_points)
    y = amplitude * np.sin(frequency * x)
    dx = np.diff(x)
    dy = np.diff(y)
    theta = np.arctan2(dy, dx)
    theta = np.append(theta, theta[-1])
    return [(x[i], y[i], theta[i]) for i in range(num_points)]


# def generate_elliptical_trajectory(a, b, num_points, start_angle_deg=30, smooth_entry_points=10):
#     # Convert start angle to radians
#     start_angle = np.radians(start_angle_deg)
    
#     # Generate points for a full ellipse
#     t = np.linspace(0, 2*np.pi, num_points, endpoint=False)
    
#     x = a * np.cos(t)
#     y = b * np.sin(t)
    
#     # Calculate the heading angle (theta)
#     dx = -a * np.sin(t)
#     dy = b * np.cos(t)
#     theta = np.arctan2(dy, dx)
    
#     # Find the starting point (30 degrees to the right of bottom)
#     start_x = a * np.cos(start_angle)
#     start_y = -b * np.sin(start_angle)  # Negative because y increases upwards
#     start_index = np.argmin((x - start_x)**2 + (y - start_y)**2)
    
#     # Shift the starting point
#     x = np.roll(x, -start_index)
#     y = np.roll(y, -start_index)
#     theta = np.roll(theta, -start_index)
    
#     # Adjust theta to be relative to the x-axis
#     theta = (theta - np.pi/2 + start_angle) % (2*np.pi)
    
#     # Generate smooth entry path
#     entry_t = np.linspace(0, 1, smooth_entry_points)
#     entry_x = entry_t * x[0]
#     entry_y = entry_t * y[0]
#     entry_theta = np.linspace(np.arctan2(y[0], x[0]), theta[0], smooth_entry_points)
    
#     # Combine entry path with elliptical trajectory
#     x = np.concatenate((entry_x, x))
#     y = np.concatenate((entry_y, y))
#     theta = np.concatenate((entry_theta, theta))
    
#     # Ensure smooth angle transitions
#     for i in range(1, len(theta)):
#         while theta[i] - theta[i-1] > np.pi:
#             theta[i] -= 2*np.pi
#         while theta[i] - theta[i-1] < -np.pi:
#             theta[i] += 2*np.pi
    
#     trajectory = [(x[i], y[i], theta[i]) for i in range(len(x))]
#     return trajectory
def bezier_curve(p0, p1, p2, t):
    return (1-t)**2 * p0 + 2*(1-t)*t * p1 + t**2 * p2

def generate_elliptical_trajectory(a, b, num_points, start_angle_deg=30, smooth_entry_points=20):
    # Convert start angle to radians
    start_angle = np.radians(start_angle_deg)
    
    # Generate points for a full ellipse
    t = np.linspace(0, 2*np.pi, num_points, endpoint=False)
    
    x = a * np.cos(t)
    y = b * np.sin(t)
    
    # Calculate the heading angle (theta)
    dx = -a * np.sin(t)
    dy = b * np.cos(t)
    theta = np.arctan2(dy, dx)
    
    # Find the starting point (30 degrees to the right of bottom)
    start_x = a * np.cos(start_angle)
    start_y = -b * np.sin(start_angle)  # Negative because y increases upwards
    start_index = np.argmin((x - start_x)**2 + (y - start_y)**2)
    
    # Shift the starting point
    x = np.roll(x, -start_index)
    y = np.roll(y, -start_index)
    theta = np.roll(theta, -start_index)
    
    # Adjust theta to be relative to the x-axis
    theta = (theta - np.pi/2 + start_angle) % (2*np.pi)
    
    # Generate smooth entry path using Bezier curve
    p0 = np.array([0, 0])  # Start point
    p2 = np.array([x[0], y[0]])  # End point (first point of ellipse)
    p1 = p2 * 0.7  # Control point
    
    entry_t = np.linspace(0, 1, smooth_entry_points)
    entry_points = np.array([bezier_curve(p0, p1, p2, t) for t in entry_t])
    entry_x, entry_y = entry_points[:, 0], entry_points[:, 1]
    
    # Calculate entry path angles
    entry_dx = np.diff(entry_x, append=x[0]-entry_x[-1])
    entry_dy = np.diff(entry_y, append=y[0]-entry_y[-1])
    entry_theta = np.arctan2(entry_dy, entry_dx)
    
    # Combine entry path with elliptical trajectory
    x = np.concatenate((entry_x, x))
    y = np.concatenate((entry_y, y))
    theta = np.concatenate((entry_theta, theta))
    
    # Ensure smooth angle transitions
    for i in range(1, len(theta)):
        while theta[i] - theta[i-1] > np.pi:
            theta[i] -= 2*np.pi
        while theta[i] - theta[i-1] < -np.pi:
            theta[i] += 2*np.pi
    
    trajectory = [(x[i], y[i], theta[i]) for i in range(len(x))]
    return trajectory
class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.path_pub = self.create_publisher(PoseArray, 'trajectory', 10)
        self.timer = self.create_timer(0.1, self.publish_trajectory)  # 10hz

        # Generate the elliptical trajectory
        # Parameters: a (semi-major axis), b (semi-minor axis), number of points
        self.trajectory = generate_elliptical_trajectory(2.5, 1.5, 150)

        # Create PoseArray message
        self.pose_array = PoseArray()
        self.pose_array.header.frame_id = "odom"  
        for point in self.trajectory:
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            q = euler2quat(0, 0, point[2])
            pose.orientation.x = q[1]
            pose.orientation.y = q[2]
            pose.orientation.z = q[3]
            pose.orientation.w = q[0]
            self.pose_array.poses.append(pose)

    def publish_trajectory(self):
        self.pose_array.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.pose_array)
        self.get_logger().info('Published trajectory')

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    try:
        rclpy.spin(path_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        path_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()