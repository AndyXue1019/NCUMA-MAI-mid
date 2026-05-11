#!/usr/bin/env python3
import math

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


class RealTurtlebot3Navigator:
    def __init__(self):
        rospy.init_node('tb3_real_robot_nav', anonymous=True)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        rospy.on_shutdown(self.on_shutdown)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.laser_data = np.full(360, 10.0)
        self.odom_ready = False
        self.scan_ready = False

        # real world
        self.goal_x = 2.0
        self.goal_y = 0.0
        # Gazebo world
        # self.goal_x = 1.5
        # self.goal_y = 2.5

        # 訓練好的權重
        self.W = np.array([0.3076, -1.1648, -0.6121, -4.8315])

        # 物理限制
        self.v = 0.15
        self.w_rad = [math.radians(-30), math.radians(-15), 0.0, math.radians(15), math.radians(30)]
        self.dt = 0.1

        rospy.loginfo('Waiting for real sensor data (/odom and /scan)...')

    def on_shutdown(self):
        """緊急煞車機制：確保程式關閉時實體車不會暴衝"""
        rospy.loginfo('Shutting down! Stopping the real robot...')
        self.cmd_pub.publish(Twist())
        rospy.sleep(1)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        self.robot_theta = yaw
        self.odom_ready = True

    def scan_callback(self, msg):
        """讀取實體光達資料並進行安全膨脹"""
        scan = np.array(msg.ranges)
        scan[np.isinf(scan)] = 3.5
        scan[np.isnan(scan)] = 3.5
        scan[scan == 0.0] = 0.01

        # 實體車身盲區膨脹
        inflated_scan = np.copy(scan)
        for i in range(360):
            window = [scan[(i + j) % 360] for j in range(-20, 21)]
            inflated_scan[i] = min(window)

        self.laser_data = inflated_scan
        self.scan_ready = True

    def piangle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

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

        # 安全距離設定
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
        # 等待實體感測器上線
        while not (self.odom_ready and self.scan_ready) and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.loginfo('Sensors ready! Commencing Real Robot Navigation...')

        rate = rospy.Rate(10)  # 10Hz 控制頻率

        while not rospy.is_shutdown():
            # 檢查是否到達終點 (誤差 10 公分內算抵達)
            current_dist = math.hypot(self.goal_x - self.robot_x, self.goal_y - self.robot_y)
            if current_dist < 0.1:
                rospy.loginfo('🎉 成功抵達終點！')
                self.cmd_pub.publish(Twist())  # 煞車
                break

            # 大腦決策：計算每個動作的 Q 值
            q_vals = [np.dot(self.W, self.get_features(a)) for a in range(5)]
            best_action = np.argmax(q_vals)

            # 發送物理指令給實體馬達
            cmd = Twist()
            cmd.linear.x = self.v
            cmd.angular.z = self.w_rad[best_action]
            self.cmd_pub.publish(cmd)

            rate.sleep()


if __name__ == '__main__':
    try:
        navigator = RealTurtlebot3Navigator()
        navigator.loop()
    except rospy.ROSInterruptException:
        pass
