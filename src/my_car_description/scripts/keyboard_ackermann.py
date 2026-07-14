#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import select
import sys
import termios
import tty

import rospy
from std_msgs.msg import Float64


class KeyboardAckermann:
    def __init__(self):
        rospy.init_node("keyboard_ackermann")

        # 車輛尺寸
        self.wheelbase = rospy.get_param("~wheelbase", 0.35)
        self.track_width = rospy.get_param("~track_width", 0.26)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.065)

        # 控制限制
        self.max_speed = rospy.get_param("~max_speed", 1.0)
        self.max_steering = rospy.get_param("~max_steering", 0.60)

        self.speed_step = rospy.get_param("~speed_step", 0.10)
        self.steering_step = rospy.get_param("~steering_step", 0.05)

        self.speed = 0.0
        self.steering_angle = 0.0

        # 前輪轉向位置 controller
        self.left_steering_pub = rospy.Publisher(
            "/left_front_steering_controller/command",
            Float64,
            queue_size=10
        )

        self.right_steering_pub = rospy.Publisher(
            "/right_front_steering_controller/command",
            Float64,
            queue_size=10
        )

        # 後輪速度 controller
        self.left_rear_wheel_pub = rospy.Publisher(
            "/left_rear_wheel_controller/command",
            Float64,
            queue_size=10
        )

        self.right_rear_wheel_pub = rospy.Publisher(
            "/right_rear_wheel_controller/command",
            Float64,
            queue_size=10
        )

        if not sys.stdin.isatty():
            rospy.logfatal(
                "此節點需要鍵盤終端，請使用 rosrun 在獨立終端執行"
            )
            raise RuntimeError("stdin is not a terminal")

        self.terminal_settings = termios.tcgetattr(sys.stdin)

        rospy.on_shutdown(self.stop_vehicle)

        rospy.loginfo("Gazebo Ackermann keyboard controller started")
        self.print_help()

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(value, maximum))

    def print_help(self):
        print(
            "\n"
            "========== Gazebo Ackermann 控制 ==========\n"
            "W：增加前進速度\n"
            "S：增加後退速度\n"
            "A：向左轉\n"
            "D：向右轉\n"
            "X：方向盤回正\n"
            "空白鍵：停止\n"
            "Q：離開\n"
            "H：顯示說明\n"
            "===========================================\n"
        )

    def read_key(self):
        readable, _, _ = select.select([sys.stdin], [], [], 0.05)

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
                "\r速度：{:+.2f} m/s｜轉向：{:+.1f}°          ".format(
                    self.speed,
                    math.degrees(self.steering_angle)
                ),
                end="",
                flush=True
            )

    def calculate_ackermann_angles(self):
        if abs(self.steering_angle) < 1e-6:
            return 0.0, 0.0

        turning_radius = (
            self.wheelbase / math.tan(self.steering_angle)
        )

        left_denominator = (
            turning_radius - self.track_width / 2.0
        )

        right_denominator = (
            turning_radius + self.track_width / 2.0
        )

        left_angle = math.atan2(
            self.wheelbase,
            left_denominator
        )

        right_angle = math.atan2(
            self.wheelbase,
            right_denominator
        )

        # atan2 在右轉時可能產生接近 pi 的角度，重新限制
        left_angle = math.atan(
            self.wheelbase / left_denominator
        )

        right_angle = math.atan(
            self.wheelbase / right_denominator
        )

        return left_angle, right_angle

    def publish_commands(self):
        left_steering, right_steering = (
            self.calculate_ackermann_angles()
        )

        wheel_angular_velocity = (
            self.speed / self.wheel_radius
        )

        self.left_steering_pub.publish(
            Float64(data=left_steering)
        )

        self.right_steering_pub.publish(
            Float64(data=right_steering)
        )

        self.left_rear_wheel_pub.publish(
            Float64(data=wheel_angular_velocity)
        )

        self.right_rear_wheel_pub.publish(
            Float64(data=wheel_angular_velocity)
        )

    def stop_vehicle(self):
        try:
            self.left_steering_pub.publish(Float64(data=0.0))
            self.right_steering_pub.publish(Float64(data=0.0))
            self.left_rear_wheel_pub.publish(Float64(data=0.0))
            self.right_rear_wheel_pub.publish(Float64(data=0.0))
        except rospy.ROSException:
            pass

    def run(self):
        rate = rospy.Rate(30)

        try:
            tty.setcbreak(sys.stdin.fileno())

            while not rospy.is_shutdown():
                key = self.read_key()
                self.process_key(key)
                self.publish_commands()
                rate.sleep()

        finally:
            self.stop_vehicle()

            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                self.terminal_settings
            )

            print("\n鍵盤控制結束")


if __name__ == "__main__":
    try:
        KeyboardAckermann().run()

    except rospy.ROSInterruptException:
        pass

    except RuntimeError as error:
        rospy.logerr(str(error))