import re
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class ObstacleProcessorNode(Node):
    def __init__(self):
        super().__init__('obstacle_processor')
        self.obstacle_tracks = []
        self.timestamps = []
        # self.DECAY_TIME_LIMIT = 2.5#jingzhi
        self.DECAY_TIME_LIMIT = 2.0#path



        self.sub = self.create_subscription(String, '/obstacle_info', self.obstacle_info_callback, 10)
        self.pub = self.create_publisher(String, '/processed_obstacles', 10)

    def extract_and_format_data(self, text):
        numbers = re.findall(r"[-+]?\d*\.\d+|\d+", text)
        formatted_data = []
        for i in range(0, len(numbers), 3):
            obstacle_data = [round(float(numbers[i]), 2), round(float(numbers[i+1]), 2),
                             round(float(numbers[i+2]), 2), 0.0, 0.0]
            formatted_data.append(obstacle_data)
        return formatted_data

    def obstacle_info_callback(self, msg):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if msg.data:
            formatted_list = self.extract_and_format_data(msg.data)
        else:
            formatted_list = []
        self.update_obstacle_history(formatted_list, current_time)

    def update_obstacle_history(self, current_scan, current_time):
        valid_indices = [i for i in range(len(self.obstacle_tracks)) if (current_time - self.timestamps[i][-1]) < self.DECAY_TIME_LIMIT]
        self.obstacle_tracks = [self.obstacle_tracks[i] for i in valid_indices]
        self.timestamps = [self.timestamps[i] for i in valid_indices]

        if current_scan:
            if not self.obstacle_tracks:
                self.obstacle_tracks = [[obstacle] for obstacle in current_scan]
                self.timestamps = [[current_time] for _ in current_scan]
                return

            matched_current_scan = set()

            for i in range(len(self.obstacle_tracks)):
                track = self.obstacle_tracks[i]
                last_obstacle = track[-1]

                for j in range(len(current_scan)):
                    if j in matched_current_scan:
                        continue
                    current_obstacle = current_scan[j]
                    distance = np.linalg.norm(np.array(last_obstacle[:2]) - np.array(current_obstacle[:2]))
                    if distance < 0.2:
                        matched_current_scan.add(j)
                        track.append(current_obstacle)
                        self.timestamps[i].append(current_time)

                        if len(track) > 15:
                            track.pop(0)
                            self.timestamps[i].pop(0)
                        break

            for j, obstacle in enumerate(current_scan):
                if j not in matched_current_scan:
                    self.obstacle_tracks.append([obstacle])
                    self.timestamps.append([current_time])

        for i in range(len(self.obstacle_tracks)):
            track = self.obstacle_tracks[i]
            time_list = self.timestamps[i]
            obs_vx, obs_vy = self.calculate_velocity(track, time_list)
            track[-1][3] = obs_vx
            track[-1][4] = obs_vy

        self.publish_all_latest_obstacles()

    def initialize_kalman(self):
        x = np.array([0, 0])  
        P = np.eye(2)  
        F = np.eye(2)  
        H = np.array([[1, 0]])  
        R = np.array([[1.0]])  
        Q = np.array([[1e-4, 0], [0, 1e-4]])  
        return x, P, F, H, R, Q

    def kalman_filter_last_velocity(self, positions, obstime):
        x, P, F, H, R, Q = self.initialize_kalman()
        for i in range(1, len(positions)):
            dt = obstime[i] - obstime[i - 1]
            F[0, 1] = dt
            x = F.dot(x)
            P = F.dot(P).dot(F.T) + Q
            z = np.array([positions[i]])
            y = z - H.dot(x)
            S = H.dot(P).dot(H.T) + R
            K = P.dot(H.T).dot(np.linalg.inv(S))
            x = x + K.dot(y)
            P = (np.eye(2) - K.dot(H)).dot(P)
        
        return x[1]

    def calculate_velocity(self, track, obstime):
        if len(obstime) <= 1:
            return 0.0, 0.0  
        else:
            x_positions = np.array([pos[0] for pos in track])
            x_positions = x_positions - x_positions[0]
            y_positions = np.array([pos[1] for pos in track])
            y_positions = y_positions - y_positions[0]
            vx = self.kalman_filter_last_velocity(x_positions, obstime)
            vy = self.kalman_filter_last_velocity(y_positions, obstime)
            if abs(vx) < 0.01:
                vx = 0
            if abs(vy) < 0.01:
                vy = 0
            return vx, vy

    def publish_all_latest_obstacles(self):
        latest_obstacles = [track[-1] for track in self.obstacle_tracks]
        obstacle_data = json.dumps(latest_obstacles)
        self.pub.publish(String(data=obstacle_data))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
