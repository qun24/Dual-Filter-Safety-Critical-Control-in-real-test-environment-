#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import casadi as ca
import cvxpy as cp
import json
import numpy as np
import time
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from transforms3d.euler import quat2euler

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance_node")
        self.create_timer(0.025, self.control_loop)  
        # 初始化属性
        self.x_real = 0.0
        self.y_real = 0.0
        self.theta_real = 0.0
        self.latest_obstacles = []
        
        # MPC 参数
        self.T = 0.05  # 时间步长
        self.N = 30    # 预测步数
        self.rob_diam = 0.3 
        self.v_max = 0.2
        self.omega_max = np.pi / 8.0 
        self.l = 0.1  # 相机距离小车中心的偏移量
        
        # 创建订阅者和发布者
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(String, "/processed_obstacles", self.obstacle_callback, 10)
        self.create_subscription(PoseArray, "/trajectory", self.trajectory_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        
        # 初始化 MPC
        self.setup_mpc()
        
        # 其他初始化
        self.speed = Twist()
        self.target_points = []
        self.current_target_index = 0
        self.start_time = time.time()
        
        # 数据记录
        self.x_path = []
        self.y_path = []
        self.theta_path = []
        self.u0_real_list = []
        self.u1_real_list = []
        self.execution_times = []
        self.T_zong_list = []
        self.min_dist_list = []
        self.cost_values = []
        self.x_c = []
        self.u_c = []
        self.t_c = []
        self.index_t = []

        self.t0 = 0.0
        self.u0 = np.array([0.0, 0.0] * self.N).reshape(-1, 2)

    def odom_callback(self, msg):
        self.x_real = msg.pose.pose.position.x
        self.y_real = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (_, _, self.theta_real) = quat2euler([rot_q.w, rot_q.x, rot_q.y, rot_q.z])
        self.get_logger().info(f"Current position: x={self.x_real:.2f}, y={self.y_real:.2f}, theta={self.theta_real:.2f}")

    def obstacle_callback(self, msg):
        # self.get_logger().info("Received obstacle message")
        self.latest_obstacles = json.loads(msg.data)
        # self.get_logger().info(f"Received obstacles: {self.latest_obstacles}")

    def trajectory_callback(self, msg):
        self.target_points = [(pose.position.x, pose.position.y, quat2euler([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])[2]) for pose in msg.poses]

    def is_obstacle_near_target(self, target, obstacles, threshold=0.55):
        target_x, target_y, _ = target
        for obs in obstacles:
            obs_x, obs_y, obs_r, _, _ = obs
            distance = np.sqrt((target_x - obs_x)**2 + (target_y - obs_y)**2)
            if distance < (obs_r + threshold):
                return True
        return False

    def setup_mpc(self):
        # MPC 设置代码（保持原有逻辑不变）
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        v = ca.SX.sym('v')
        omega = ca.SX.sym('omega')

        states = ca.vertcat(x, y, theta)
        controls = ca.vertcat(v, omega)
        n_states = states.size()[0]
        n_controls = controls.size()[0]

        rhs = ca.vertcat(v * ca.cos(theta), v * ca.sin(theta), omega)

        self.f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])

        U = ca.SX.sym('U', n_controls, self.N)
        X = ca.SX.sym('X', n_states, self.N+1)
        P = ca.SX.sym('P', n_states + n_states)

        X[:, 0] = P[:3]
        for i in range(self.N):
            f_value = self.f(X[:, i], U[:, i])
            X[:, i+1] = X[:, i] + f_value * self.T

        self.ff = ca.Function('ff', [U, P], [X], ['input_U', 'target_state'], ['horizon_states'])

        Q = np.array([[15.0, 0.0, 0.0], [0.0, 15.0, 0.0], [0.0, 0.0, 0.0025]]) # x、y、theta权重
        R = np.array([[0.1, 0.0], [0.0, 0.01]])# 线速度v和角速度w的权重
        # Rd = np.diag([10.0, 90.0])#更小丢丢震荡
        # Rd = np.diag([10.0, 80.0])#更小丢丢震荡 前几个场景横好
        # Rd = np.diag([10.0, 70.0])#更小丢丢震荡 动态场景比较好
        # Rd = np.diag([10.0, 75.0])#更小丢丢震荡 动态场景比较好
        Rd = np.diag([10.0, 65.0])#更小丢丢震荡 动态场景比较好



        obj = 0
        g = []
        for i in range(self.N):
            state_error = X[:, i] - P[3:]
            obj += ca.mtimes([state_error.T, Q, state_error]) + ca.mtimes([U[:, i].T, R, U[:, i]])
            if i < self.N-1:
                control_change = U[:, i+1] - U[:, i]
                obj += ca.mtimes([control_change.T, Rd, control_change])
        self.cost_func = ca.Function('cost_func', [U, P], [obj], ['control_input', 'params'], ['cost'])

        nlp_prob = {'f': obj, 'x': ca.reshape(U, -1, 1), 'p': P, 'g': ca.vertcat(*g)}
        opts_setting = {'ipopt.max_iter': 100, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-8, 'ipopt.acceptable_obj_change_tol': 1e-6}
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

        self.lbg = []
        self.ubg = []
        self.lbx = []
        self.ubx = []
        for _ in range(self.N):
            self.lbx.append(0)
            self.ubx.append(self.v_max)
            self.lbx.append(-self.omega_max)
            self.ubx.append(self.omega_max)

    def process_obstacle(self, last_obstacle, robot_state, control_params):
        x_real, y_real, theta_real, l = robot_state
        v_max, omega_max, u_real = control_params
        obs_x, obs_y, obs_r, obs_vx, obs_vy = last_obstacle
        
        # 预计算和固定参数
        cos_theta = np.cos(theta_real)
        sin_theta = np.sin(theta_real)
        delta_x = x_real - obs_x 
        delta_y = y_real - obs_y
        delta_x_offset = delta_x + l * cos_theta
        delta_y_offset = delta_y + l * sin_theta
        distance = np.sqrt(delta_x_offset ** 2 + delta_y_offset ** 2) - obs_r - self.rob_diam/2 -0.05

        if distance <= 0.4:
            # self.get_logger().info(f"Obstacle within avoidance range. Distance: {distance:.2f}")
            CBF_Condition = distance
            Q_cbf = np.array([[1000, 0], [0, 1]])
            c_cbf = np.zeros(2)

            stop = False
            if distance <= 0.3:
                obs_rel_y_1 = (delta_x + obs_vx*1) * sin_theta - (delta_y + obs_vy*1) * cos_theta 
                obs_rel_y_2 = (delta_x - obs_vx*1) * sin_theta - (delta_y - obs_vy*1) * cos_theta  
                obs_mul = obs_rel_y_1 * obs_rel_y_2

                obs_rel_y = delta_x * sin_theta - delta_y * cos_theta
                obs_rel_vy = obs_vy * cos_theta - obs_vx * sin_theta

                if (0 < obs_rel_y < 0.2 and obs_rel_vy < -0.05) or (-0.2 < obs_rel_y < 0 and obs_rel_vy > 0.05) or obs_mul < 0:
                    stop = True
                    self.get_logger().info(f"stop!!!!!!!!!!!!")
                

            dist_sq = delta_x_offset ** 2 + delta_y_offset ** 2
            e1 = -1.0 / np.sqrt(dist_sq) * (delta_x_offset * cos_theta + delta_y_offset * sin_theta)
            e2 = -1.0 / np.sqrt(dist_sq) * (delta_x_offset * sin_theta * -l + delta_y_offset * cos_theta * l)
            e3 = 1.0 / np.sqrt(dist_sq) * (delta_x_offset * obs_vx + delta_y_offset * obs_vy)

            CBF_Condition = distance - e3
            A = np.array([[e1, e2], [1, 0], [0, 1], [0, -1]])
            b_cbf = np.array([CBF_Condition, v_max, omega_max, omega_max]).reshape(-1, 1)

            u1 = cp.Variable()
            u2 = cp.Variable()
            if u_real[0] >= 0:
                objective = cp.Minimize(0.5 * cp.quad_form(cp.vstack([u1 - 0.2, u2]), Q_cbf) + c_cbf @ cp.vstack([u1, u2]))
            else:
                objective = cp.Minimize(0.5 * cp.quad_form(cp.vstack([u1 + 0.2, u2]), Q_cbf) + c_cbf @ cp.vstack([u1, u2]))

            constraints = [cp.matmul(A, cp.vstack([u1, u2])) <= b_cbf]
            problem = cp.Problem(objective, constraints)
            problem.solve()

            rate = distance / 0.4
            rate = max(0, min(rate, 1))
            u_real[0] = u1.value * (1 - rate) + rate * u_real[0]
            u_real[1] = u2.value * (1 - rate) + rate * u_real[1]

            if stop:
                u_real[0] = 0
                u_real[1] = 0

        return distance

    def control_loop(self):
        if not self.target_points:
            self.get_logger().info("Waiting for trajectory...")
            return
        # 检查并跳过被障碍物占据的路径点
        while self.current_target_index < len(self.target_points):
            if self.latest_obstacles and self.is_obstacle_near_target(self.target_points[self.current_target_index], self.latest_obstacles):
                self.current_target_index += 1
            else:
                break
        if self.current_target_index < len(self.target_points):
            xs = np.array(self.target_points[self.current_target_index]).reshape(-1, 1)
            # self.get_logger().info(f"Current target: {xs.flatten()}")
            x_idk = np.array([self.x_real, self.y_real, self.theta_real]).reshape(-1, 1)
            ref_trajectory = np.concatenate((x_idk, xs))
            init_control = ca.reshape(self.u0, -1, 1)
            start_time = time.time()
            res = self.solver(x0=init_control, p=ref_trajectory, lbx=self.lbx, ubx=self.ubx)
            self.index_t.append(time.time() - start_time)

            u_sol = ca.reshape(res['x'], 2, self.N)  # 获取MPC输入
            ff_value = self.ff(u_sol, ref_trajectory)
            u_real = u_sol[:, 0]
            self.x_c.append(ff_value)
            self.u_c.append(u_sol[:, 0])
            self.t_c.append(self.t0)
                        
            self.get_logger().info(f"MPC solution: v={float(u_real[0]):.2f}, omega={float(u_real[1]):.2f}")
            
            # Process obstacles
            if self.latest_obstacles:
                # self.get_logger().info(f"Processing {len(self.latest_obstacles)} obstacles")
                distlist = []
                robot_state = (self.x_real, self.y_real, self.theta_real, self.l)
                control_params = (self.v_max, self.omega_max, u_real)

                for obstacle in self.latest_obstacles:
                    distance = self.process_obstacle(obstacle, robot_state, control_params)
                    distlist.append(distance)

                mindist = min(distlist)
                self.min_dist_list.append(mindist)
            else:
                self.get_logger().info("No obstacles detected")
            
            # Publish command
            self.speed.linear.x = float(u_real[0])
            self.speed.angular.z = float(u_real[1])
            self.cmd_vel_pub.publish(self.speed)
            
            # Record data
            self.x_path.append(self.x_real)
            self.y_path.append(self.y_real)
            self.theta_path.append(self.theta_real)
            self.u0_real_list.append(float(u_real[0]))
            self.u1_real_list.append(float(u_real[1]))
            current_cost = self.cost_func(u_sol,ref_trajectory)
            self.cost_values.append(float(current_cost))
            
            end_time = time.time()
            execution_time = end_time - start_time
            self.execution_times.append(execution_time)
            t_zong = end_time - self.start_time
            self.T_zong_list.append(t_zong)
            
            # Check if target is reached
            distance_to_target = np.linalg.norm(x_idk[:2] - xs[:2])
            # if distance_to_target < 0.1 and angle_to_target < 0.1:
            if distance_to_target < 0.2:#第一版
                self.get_logger().info(f"Reached target {self.current_target_index}: {xs.flatten()}")
                self.current_target_index += 1
                
                if self.current_target_index < len(self.target_points):
                    next_target = self.target_points[self.current_target_index]
                    self.get_logger().info(f"Switching to next target: {next_target}")
                else:
                    self.get_logger().info("All targets reached")

        else:
            # Finished and stopped
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0
            self.cmd_vel_pub.publish(self.speed)
            self.save_data()
            self.get_logger().info("Mission completed. Shutting down.")
            self.destroy_node()
            rclpy.shutdown()

    def save_data(self):
        data_to_save = {
            "xy_path": {"x_path": self.x_path, "y_path": self.y_path, "theta_path": self.theta_path, "T": self.T_zong_list},
            "t_path": {"t_path": self.execution_times, "T": self.T_zong_list},
            "cost_values": {"cost_values": self.cost_values, "T": self.T_zong_list},
            "input": {"v": self.u0_real_list, "w": self.u1_real_list, "T": self.T_zong_list},
            "min_dist": {"min": self.min_dist_list, "T": self.T_zong_list}
        }

        for filename, data in data_to_save.items():
            with open(f"{filename}.json", "w") as file:
                json.dump(data, file)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()