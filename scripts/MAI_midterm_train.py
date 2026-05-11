#!/usr/bin/env python3
import math
import random

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

random.seed(42)


class QLearningTrainer:
    def __init__(self):
        rospy.init_node('tb3_qlearning_trainer', anonymous=True)
        rospy.on_shutdown(self.on_shutdown)

        # ROS 訂閱與發佈
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # 等待 Gazebo 重置服務上線
        rospy.loginfo('Waiting for /gazebo/set_model_state service...')
        rospy.wait_for_service('/gazebo/set_model_state', timeout=10.0)
        self.set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.loginfo('Service connected!')

        # 機器人狀態
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.laser_data = np.full(360, 10.0)

        self.goal_x = 1.5
        self.goal_y = 2.5

        # self.W = np.array([0.0, -1.0, -1.0, -5.0])
        self.W = np.array([0.1, -1.0, -1.0, -5.0])

        # RL 學習參數
        self.alpha = 0.1
        self.gamma = 0.9
        self.max_episodes = 10
        self.epsilon_init = 0.5
        self.epsilon_end = 0.01
        self.decay_episodes = int(self.max_episodes * 0.5)

        # Sim-to-Real 對應參數
        self.unit_to_meter = 0.01
        self.max_sim_dist = 300.0
        self.laser_threshold = 70.0
        self.inflation_radius_cm = 15.0

        self.v = 0.15
        self.w_rad = [math.radians(-30), math.radians(-15), 0.0, math.radians(15), math.radians(30)]
        self.dt = 0.1  # 動作執行時間

    def on_shutdown(self):
        """ROS shutdown 時的清理工作，確保機器人停下來"""
        rospy.loginfo('Shutting down, stopping the robot...')
        self.reset_environment()
        rospy.sleep(1)  # 確保命令送達

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        self.robot_theta = yaw

    def scan_callback(self, msg):
        """讀取光達資料，並進行雷射膨脹以解決車體死角碰撞"""
        scan = np.array(msg.ranges)
        # 處理 Inf 與 0 的無效值
        scan[np.isinf(scan)] = 3.5
        scan[np.isnan(scan)] = 3.5
        scan[scan == 0.0] = 0.01

        # 雷射視角膨脹 (Laser Inflation)
        inflated_scan = np.copy(scan)
        # 迴圈處理 360 度的每一根雷射
        for i in range(360):
            # 抓取左右各 20 度的範圍 (共 41 度的扇形)
            # 找出這個扇形範圍內「最近」的距離
            window = [scan[(i + j) % 360] for j in range(-20, 21)]
            inflated_scan[i] = min(window)

        self.laser_data = inflated_scan

    def piangle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def reset_environment(self):
        """傳送機器人回到起點"""
        # 先發佈 0 速度，確保機器人停下
        self.cmd_pub.publish(Twist())

        state_msg = ModelState()
        state_msg.model_name = 'turtlebot3_burger'
        state_msg.pose.position.x = 1.5
        state_msg.pose.position.y = 0.5
        state_msg.pose.position.z = 0.0

        # 設定朝向為 90 度 (1.57 rad)
        q = quaternion_from_euler(0, 0, 1.57)
        state_msg.pose.orientation.x = q[0]
        state_msg.pose.orientation.y = q[1]
        state_msg.pose.orientation.z = q[2]
        state_msg.pose.orientation.w = q[3]

        try:
            self.set_state_service(state_msg)
        except rospy.ServiceException as e:
            rospy.logerr(f'Reset failed: {e}')

        rospy.sleep(0.5)  # 等待感測器資料穩定

    def get_reward(self):
        """計算 Reward 與判斷是否撞牆 (Terminal)"""
        # 計算轉換回模擬單位的距離
        dist_sim = (
            math.hypot(self.goal_x - self.robot_x, self.goal_y - self.robot_y) / self.unit_to_meter
        )
        obs_dist_sim = math.hypot(1.5 - self.robot_x, 1.5 - self.robot_y) / self.unit_to_meter

        D_RB_x = min(self.robot_x, 3.0 - self.robot_x) / self.unit_to_meter
        D_RB_y = min(self.robot_y, 3.0 - self.robot_y) / self.unit_to_meter
        D_RB = min(D_RB_x, D_RB_y)

        Terminal = 0
        R = -0.05

        if obs_dist_sim < 20:
            R = -10
            Terminal = 1
        elif D_RB < 13:
            R = -10
            Terminal = 1
        elif dist_sim < 10:
            R = 10
            Terminal = 1

        return R, Terminal

    def predict_state(self, action_idx):
        """利用運動學模型預測 0.1 秒後的狀態 (完美還原 MATLAB 的 motion_model)"""
        w = self.w_rad[action_idx]

        # 預測未來的 x, y, theta
        next_x = self.robot_x + self.v * math.cos(self.robot_theta) * self.dt
        next_y = self.robot_y + self.v * math.sin(self.robot_theta) * self.dt
        next_theta = self.piangle(self.robot_theta + w * self.dt)

        return next_x, next_y, next_theta

    def get_features(self, action_idx):
        f = np.zeros(4)

        # f1: 常數項
        f[0] = 1.0

        next_x, next_y, next_theta = self.predict_state(action_idx)

        # f2: 距離
        dist_m = math.hypot(self.goal_x - next_x, self.goal_y - next_y)
        f[1] = dist_m / 4.0

        # f3: 角度
        if dist_m < 0.2:
            # 距離終點小於 20 公分時，直接把角度誤差歸零
            f[2] = 0.0
        else:
            angle_to_goal = math.atan2(self.goal_y - next_y, self.goal_x - next_x)
            angle_diff = abs(self.piangle(angle_to_goal - next_theta))
            # 當距離大於 1.0 公尺時，正常計算角度懲罰。當距離小於 1.0 公尺時，距離越近，角度懲罰越小。
            distance_discount = min(1.0, dist_m / 1.0)
            f[2] = (angle_diff / math.pi) * distance_discount

        # f4: 危險指數
        angle_deg = [-30, -15, 0, 15, 30]
        center_angle = angle_deg[action_idx]

        sector_dists = []
        for offset in range(-15, 16):
            idx = int((center_angle + offset) % 360)
            sector_dists.append(self.laser_data[idx])

        min_dist_m = min(sector_dists)
        safe_margin = 0.8

        if min_dist_m < safe_margin:
            if min_dist_m < (dist_m + 0.15):
                f[3] = (safe_margin - min_dist_m) / safe_margin
            else:
                f[3] = 0.0
        else:
            f[3] = 0.0

        return f

    def loop(self):
        for epi in range(self.max_episodes):
            self.reset_environment()

            if epi <= self.decay_episodes:
                epsilon = self.epsilon_init - (self.epsilon_init - self.epsilon_end) * (epi / self.decay_episodes)
            else:
                epsilon = self.epsilon_end

            self.alpha = max(0.01, 0.1 * (1.0 - epi / self.max_episodes))

            step = 0
            total_reward = 0
            Terminal = 0

            # 初始動作選擇
            if random.random() < epsilon:
                a_t = random.randint(0, 4)
            else:
                q_vals = [np.dot(self.W, self.get_features(a)) for a in range(5)]
                a_t = np.argmax(q_vals)

            while not Terminal and not rospy.is_shutdown() and step < 1000:
                step += 1

                f_t = self.get_features(a_t)
                Q_prev = np.dot(self.W, f_t)

                # 執行動作
                cmd = Twist()
                cmd.linear.x = self.v
                cmd.angular.z = self.w_rad[a_t]
                self.cmd_pub.publish(cmd)

                # 等待 dt (對應 MATLAB 的 0.1 秒)
                rospy.sleep(self.dt)

                # 取得 Reward 與新狀態
                R, Terminal = self.get_reward()
                total_reward += R

                # Q-learning 更新核心
                if Terminal == 1:
                    max_Q_next = 0.0
                else:
                    q_vals_next = [np.dot(self.W, self.get_features(a)) for a in range(5)]
                    max_Q_next = max(q_vals_next)

                # TD Error 計算與權重更新
                J = R + self.gamma * max_Q_next - Q_prev
                self.W = self.W + self.alpha * J * f_t

                # 選擇下一步動作
                if Terminal == 0:
                    if random.random() < epsilon:
                        a_t = random.randint(0, 4)
                    else:
                        a_t = np.argmax(q_vals_next)

            status = '✅ Reached' if R == 10 else '❌ Crashed'
            rospy.loginfo(
                f'Episode {epi} | {status} | Steps: {step} | Reward: {total_reward:.2f} | Alpha: {self.alpha:.2f} | Epsilon: {epsilon:.2f} | Current Weights (W): {np.round(self.W, 4)}'
            )


if __name__ == '__main__':
    try:
        trainer = QLearningTrainer()
        trainer.loop()
    except rospy.ROSInterruptException:
        pass