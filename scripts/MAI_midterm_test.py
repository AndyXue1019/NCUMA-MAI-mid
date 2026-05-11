#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class QLearningTester:
    def __init__(self):
        rospy.init_node('tb3_qlearning_tester', anonymous=True)

        # ROS 訂閱與發佈
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        rospy.loginfo('Waiting for /gazebo/set_model_state service...')
        rospy.wait_for_service('/gazebo/set_model_state', timeout=10.0)
        self.set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.loginfo('Service connected!')

        # 註冊 Ctrl+C 的清理動作
        rospy.on_shutdown(self.on_shutdown)

        # 機器人狀態
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.laser_data = np.full(360, 10.0)

        # 目標座標
        self.goal_x = 1.5
        self.goal_y = 2.5

        # 訓練好的權重
        self.W = np.array([0.3076, -1.1648, -0.6121, -4.8315])
        # self.W = np.array([0.7128, -1.154,  -0.8127, -4.9204])

        # 物理參數
        self.v = 0.15
        self.w_rad = [math.radians(-30), math.radians(-15), 0.0, math.radians(15), math.radians(30)]
        self.dt = 0.1

    def on_shutdown(self):
        """ROS shutdown 時確保機器人停下來"""
        rospy.loginfo('Shutting down, stopping the robot...')
        self.reset_environment()
        rospy.sleep(1)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        self.robot_theta = yaw

    def scan_callback(self, msg):
        """讀取並膨脹光達資料"""
        scan = np.array(msg.ranges)
        scan[np.isinf(scan)] = 3.5
        scan[np.isnan(scan)] = 3.5
        scan[scan == 0.0] = 0.01

        inflated_scan = np.copy(scan)
        for i in range(360):
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
        self.cmd_pub.publish(Twist())
        state_msg = ModelState()
        state_msg.model_name = 'turtlebot3_burger'
        state_msg.pose.position.x = 1.5
        state_msg.pose.position.y = 0.5
        state_msg.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, 1.57)
        state_msg.pose.orientation.x = q[0]
        state_msg.pose.orientation.y = q[1]
        state_msg.pose.orientation.z = q[2]
        state_msg.pose.orientation.w = q[3]

        try:
            self.set_state_service(state_msg)
        except rospy.ServiceException as e:
            rospy.logerr(f'Reset failed: {e}')
        rospy.sleep(0.5)

    def predict_state(self, action_idx):
        w = self.w_rad[action_idx]
        next_x = self.robot_x + self.v * math.cos(self.robot_theta) * self.dt
        next_y = self.robot_y + self.v * math.sin(self.robot_theta) * self.dt
        next_theta = self.piangle(self.robot_theta + w * self.dt)
        return next_x, next_y, next_theta

    def get_features(self, action_idx):
        f = np.zeros(4)
        f[0] = 1.0

        next_x, next_y, next_theta = self.predict_state(action_idx)

        dist_m = math.hypot(self.goal_x - next_x, self.goal_y - next_y)
        f[1] = dist_m / 4.0

        if dist_m < 0.2:
            f[2] = 0.0
        else:
            angle_to_goal = math.atan2(self.goal_y - next_y, self.goal_x - next_x)
            angle_diff = abs(self.piangle(angle_to_goal - next_theta))
            distance_discount = min(1.0, dist_m / 1.0)
            f[2] = (angle_diff / math.pi) * distance_discount

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
        rospy.loginfo('Starting Inference Mode (Testing)...')
        self.reset_environment()

        step = 0
        rate = rospy.Rate(10)  # 10Hz 對應 dt = 0.1

        while not rospy.is_shutdown():
            step += 1

            # 檢查是否到達終點 (距離小於 0.1 公尺)
            current_dist = math.hypot(self.goal_x - self.robot_x, self.goal_y - self.robot_y)
            if current_dist < 0.1:
                rospy.loginfo(f'🎉 成功抵達終點！總共花了 {step} 步。')
                self.reset_environment()
                break

            # 純利用 W 計算每個動作的 Q 值，直接選擇最大值 (無隨機探索)
            q_vals = [np.dot(self.W, self.get_features(a)) for a in range(5)]
            best_action = np.argmax(q_vals)

            # 執行最佳動作
            cmd = Twist()
            cmd.linear.x = self.v
            cmd.angular.z = self.w_rad[best_action]
            self.cmd_pub.publish(cmd)

            rate.sleep()


if __name__ == '__main__':
    try:
        tester = QLearningTester()
        tester.loop()
    except rospy.ROSInterruptException:
        pass
