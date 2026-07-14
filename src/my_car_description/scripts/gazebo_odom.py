#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
import tf.transformations as tft
import tf2_ros

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def normalize_angle(angle):
    """將角度限制在 [-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


class GazeboOdomPublisher:
    def __init__(self):
        # ==============================
        # ROS 參數
        # ==============================
        self.model_name = rospy.get_param(
            "~model_name",
            "my_car_ackermann"
        )

        self.odom_topic = rospy.get_param(
            "~odom_topic",
            "/odom"
        )

        self.odom_frame = rospy.get_param(
            "~odom_frame",
            "odom"
        )

        self.base_frame = rospy.get_param(
            "~base_frame",
            "base_footprint"
        )

        self.publish_tf = rospy.get_param(
            "~publish_tf",
            True
        )

        self.reset_origin = rospy.get_param(
            "~reset_origin",
            True
        )

        # ==============================
        # Publisher / TF
        # ==============================
        self.odom_pub = rospy.Publisher(
            self.odom_topic,
            Odometry,
            queue_size=20
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # ==============================
        # 狀態
        # ==============================
        self.initialized = False

        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_z = 0.0
        self.initial_yaw = 0.0

        self.last_stamp = rospy.Time(0)

        # ==============================
        # Subscriber
        # ==============================
        self.model_states_sub = rospy.Subscriber(
            "/gazebo/model_states",
            ModelStates,
            self.model_states_callback,
            queue_size=1
        )

        rospy.loginfo(
            "gazebo_odom_publisher started, model=%s, odom=%s, base=%s",
            self.model_name,
            self.odom_frame,
            self.base_frame
        )

    def model_states_callback(self, msg):
        # 找出指定模型
        try:
            model_index = msg.name.index(self.model_name)
        except ValueError:
            rospy.logwarn_throttle(
                5.0,
                "Model '%s' not found in /gazebo/model_states",
                self.model_name
            )
            return

        stamp = rospy.Time.now()

        # Gazebo 還沒開始發布 /clock
        if stamp == rospy.Time(0):
            return

        # 避免相同或更舊的時間重複發布 TF
        if stamp <= self.last_stamp:
            return

        pose = msg.pose[model_index]
        twist = msg.twist[model_index]

        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

        _, _, current_yaw = tft.euler_from_quaternion(quaternion)

        # 第一次收到資料時，記錄原點
        if not self.initialized:
            self.initial_x = pose.position.x
            self.initial_y = pose.position.y
            self.initial_z = pose.position.z
            self.initial_yaw = current_yaw

            self.initialized = True

            rospy.loginfo(
                "Odom origin initialized: x=%.3f, y=%.3f, yaw=%.3f",
                self.initial_x,
                self.initial_y,
                self.initial_yaw
            )

        if self.reset_origin:
            # 相對於啟動位置的位移
            dx_world = pose.position.x - self.initial_x
            dy_world = pose.position.y - self.initial_y

            cos_initial = math.cos(self.initial_yaw)
            sin_initial = math.sin(self.initial_yaw)

            # 將世界座標位移轉換成 odom 初始座標
            odom_x = (
                cos_initial * dx_world
                + sin_initial * dy_world
            )

            odom_y = (
                -sin_initial * dx_world
                + cos_initial * dy_world
            )

            odom_z = pose.position.z - self.initial_z
            odom_yaw = normalize_angle(
                current_yaw - self.initial_yaw
            )
        else:
            odom_x = pose.position.x
            odom_y = pose.position.y
            odom_z = pose.position.z
            odom_yaw = current_yaw

        odom_quaternion = tft.quaternion_from_euler(
            0.0,
            0.0,
            odom_yaw
        )

        # Gazebo ModelStates 的速度通常是世界座標，
        # 轉換成車體座標速度
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)

        linear_x_body = (
            cos_yaw * twist.linear.x
            + sin_yaw * twist.linear.y
        )

        linear_y_body = (
            -sin_yaw * twist.linear.x
            + cos_yaw * twist.linear.y
        )

        # ==============================
        # 發布 /odom
        # ==============================
        odom_msg = Odometry()

        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = odom_x
        odom_msg.pose.pose.position.y = odom_y
        odom_msg.pose.pose.position.z = odom_z

        odom_msg.pose.pose.orientation.x = odom_quaternion[0]
        odom_msg.pose.pose.orientation.y = odom_quaternion[1]
        odom_msg.pose.pose.orientation.z = odom_quaternion[2]
        odom_msg.pose.pose.orientation.w = odom_quaternion[3]

        odom_msg.twist.twist.linear.x = linear_x_body
        odom_msg.twist.twist.linear.y = linear_y_body
        odom_msg.twist.twist.linear.z = twist.linear.z

        odom_msg.twist.twist.angular.x = twist.angular.x
        odom_msg.twist.twist.angular.y = twist.angular.y
        odom_msg.twist.twist.angular.z = twist.angular.z

        # Ground-truth odometry covariance
        odom_msg.pose.covariance = [
            0.001, 0.0,   0.0,   0.0, 0.0,   0.0,
            0.0,   0.001, 0.0,   0.0, 0.0,   0.0,
            0.0,   0.0,   999.0, 0.0, 0.0,   0.0,
            0.0,   0.0,   0.0,   999.0, 0.0, 0.0,
            0.0,   0.0,   0.0,   0.0, 999.0, 0.0,
            0.0,   0.0,   0.0,   0.0, 0.0,   0.001
        ]

        odom_msg.twist.covariance = [
            0.001, 0.0,   0.0,   0.0, 0.0,   0.0,
            0.0,   0.001, 0.0,   0.0, 0.0,   0.0,
            0.0,   0.0,   999.0, 0.0, 0.0,   0.0,
            0.0,   0.0,   0.0,   999.0, 0.0, 0.0,
            0.0,   0.0,   0.0,   0.0, 999.0, 0.0,
            0.0,   0.0,   0.0,   0.0, 0.0,   0.001
        ]

        self.odom_pub.publish(odom_msg)

        # ==============================
        # 發布 odom → base_footprint
        # ==============================
        if self.publish_tf:
            transform = TransformStamped()

            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame

            transform.transform.translation.x = odom_x
            transform.transform.translation.y = odom_y
            transform.transform.translation.z = odom_z

            transform.transform.rotation.x = odom_quaternion[0]
            transform.transform.rotation.y = odom_quaternion[1]
            transform.transform.rotation.z = odom_quaternion[2]
            transform.transform.rotation.w = odom_quaternion[3]

            self.tf_broadcaster.sendTransform(transform)

        self.last_stamp = stamp


if __name__ == "__main__":
    rospy.init_node("gazebo_odom_publisher")

    try:
        GazeboOdomPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass