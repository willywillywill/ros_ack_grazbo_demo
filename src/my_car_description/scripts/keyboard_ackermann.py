#!/usr/bin/env python3

import math
import select
import sys
import termios
import tty

import rospy
import tf

from sensor_msgs.msg import JointState


class KeyboardAckermann:
    def __init__(self):
        rospy.init_node("keyboard_ackermann")

        # 車輛尺寸，需和 URDF 相同
        self.wheelbase = rospy.get_param("~wheelbase", 0.35)
        self.track_width = rospy.get_param("~track_width", 0.26)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.065)

        # 控制參數
        self.max_speed = rospy.get_param("~max_speed", 1.0)
        self.max_steering = rospy.get_param("~max_steering", 0.60)

        self.speed_step = rospy.get_param("~speed_step", 0.10)
        self.steering_step = rospy.get_param("~steering_step", 0.05)

        # 車輛狀態
        self.speed = 0.0
        self.steering_angle = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.wheel_position = 0.0

        self.last_time = rospy.Time.now()

        self.joint_pub = rospy.Publisher(
            "/joint_states",
            JointState,
            queue_size=10
        )

        self.tf_broadcaster = tf.TransformBroadcaster()

        self.terminal_settings = termios.tcgetattr(sys.stdin)

        rospy.loginfo("Ackermann keyboard controller started")
        self.print_help()

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(value, maximum))

    def print_help(self):
        print(
            "\n"
            "========== Ackermann 鍵盤控制 ==========\n"
            "W：增加前進速度\n"
            "S：增加後退速度\n"
            "A：向左轉\n"
            "D：向右轉\n"
            "X：方向盤回正\n"
            "空白鍵：停止車輛\n"
            "R：重設位置\n"
            "Q：離開\n"
            "========================================\n"
        )

    def read_key(self):
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)

        if readable:
            return sys.stdin.read(1).lower()

        return ""

    def process_key(self, key):
        if key == "w":
            self.speed += self.speed_step

        elif key == "s":
            self.speed -= self.speed_step

        elif key == "a":
            self.steering_angle += self.steering_step

        elif key == "d":
            self.steering_angle -= self.steering_step

        elif key == "x":
            self.steering_angle = 0.0

        elif key == " ":
            self.speed = 0.0

        elif key == "r":
            self.speed = 0.0
            self.steering_angle = 0.0

            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
            self.wheel_position = 0.0

        elif key == "q":
            rospy.signal_shutdown("Keyboard quit")

        elif key == "h":
            self.print_help()

        self.speed = self.clamp(
            self.speed,
            -self.max_speed,
            self.max_speed
        )

        self.steering_angle = self.clamp(
            self.steering_angle,
            -self.max_steering,
            self.max_steering
        )

        if key:
            print(
                "\r速度：{:+.2f} m/s｜中央轉向角：{:+.1f}°       ".format(
                    self.speed,
                    math.degrees(self.steering_angle)
                ),
                end="",
                flush=True
            )

    def calculate_ackermann_angles(self):
        """
        根據中央虛擬轉向角，計算左右前輪角度。

        正角度：左轉
        負角度：右轉
        """

        if abs(self.steering_angle) < 1e-5:
            return 0.0, 0.0

        turning_radius = self.wheelbase / math.tan(
            self.steering_angle
        )

        left_angle = math.atan(
            self.wheelbase /
            (turning_radius - self.track_width / 2.0)
        )

        right_angle = math.atan(
            self.wheelbase /
            (turning_radius + self.track_width / 2.0)
        )

        return left_angle, right_angle

    def update_odometry(self, dt):
        if abs(self.steering_angle) < 1e-5:
            yaw_rate = 0.0
        else:
            yaw_rate = (
                self.speed *
                math.tan(self.steering_angle) /
                self.wheelbase
            )

        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt
        self.yaw += yaw_rate * dt

        # 將 yaw 保持在 -pi 到 pi
        self.yaw = math.atan2(
            math.sin(self.yaw),
            math.cos(self.yaw)
        )

        wheel_speed = self.speed / self.wheel_radius
        self.wheel_position += wheel_speed * dt

    def publish_joint_states(self, stamp):
        left_steering, right_steering = (
            self.calculate_ackermann_angles()
        )

        joint_msg = JointState()
        joint_msg.header.stamp = stamp

        joint_msg.name = [
            "left_front_steering_joint",
            "right_front_steering_joint",
            "left_front_wheel_joint",
            "right_front_wheel_joint",
            "left_rear_wheel_joint",
            "right_rear_wheel_joint",
        ]

        joint_msg.position = [
            left_steering,
            right_steering,
            self.wheel_position,
            self.wheel_position,
            self.wheel_position,
            self.wheel_position,
        ]

        joint_msg.velocity = [
            0.0,
            0.0,
            self.speed / self.wheel_radius,
            self.speed / self.wheel_radius,
            self.speed / self.wheel_radius,
            self.speed / self.wheel_radius,
        ]

        self.joint_pub.publish(joint_msg)

    def publish_tf(self, stamp):
        quaternion = tf.transformations.quaternion_from_euler(
            0.0,
            0.0,
            self.yaw
        )

        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            quaternion,
            stamp,
            "base_footprint",
            "odom"
        )

    def run(self):
        rate = rospy.Rate(30)

        try:
            tty.setcbreak(sys.stdin.fileno())

            while not rospy.is_shutdown():
                key = self.read_key()
                self.process_key(key)

                now = rospy.Time.now()
                dt = (now - self.last_time).to_sec()
                self.last_time = now

                if dt < 0.0 or dt > 0.5:
                    dt = 0.0

                self.update_odometry(dt)
                self.publish_joint_states(now)
                self.publish_tf(now)

                rate.sleep()

        finally:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                self.terminal_settings
            )

            print("\n鍵盤控制結束")


if __name__ == "__main__":
    try:
        controller = KeyboardAckermann()
        controller.run()

    except rospy.ROSInterruptException:
        pass