#!/usr/bin/env python3

import rospy
import sys
import tty
import termios
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class OmniWheelControl:
    def __init__(self):
        rospy.init_node('omni_wheel_control', anonymous=True)
        
        # Tạo publisher để gửi vận tốc đến 3 bánh xe
        self.wheel_front_pub = rospy.Publisher('/front_wheel_joint_velocity_controller/command', Float64, queue_size=10)
        self.wheel_left_pub = rospy.Publisher('/left_wheel_joint_velocity_controller/command', Float64, queue_size=10)
        self.wheel_right_pub = rospy.Publisher('/right_wheel_joint_velocity_controller/command', Float64, queue_size=10)

        # Nhận lệnh từ /cmd_vel (Twist)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        rospy.sleep(1)  # Đợi publisher khởi tạo

        # Thông số robot
        self.R = 0.25  # Khoảng cách từ tâm robot đến bánh (m)
        self.speed_limit = 5.0  # Giới hạn vận tốc tối đa

        # Biến lưu vận tốc bánh xe
        self.wheel_front_vel = 0.0
        self.wheel_left_vel = 0.0
        self.wheel_right_vel = 0.0

        # Biến điều khiển vòng lặp
        self.running = True

    def cmd_vel_callback(self, msg):
        """Nhận lệnh Twist và tính toán vận tốc bánh xe"""
        Vx = msg.linear.x   # Tiến/lùi
        Vy = msg.linear.y   # Sang trái/phải
        Wz = msg.angular.z  # Xoay tại chỗ

        # Cập nhật công thức tính vận tốc của các bánh xe theo công thức từ file test.py
        self.wheel_left_vel = (0.5 * Vx) + ((math.sqrt(3) / 2) * Vy) + (self.R * Wz)
        self.wheel_right_vel = (0.5 * Vx) - ((math.sqrt(3) / 2) * Vy) + (self.R * Wz)
        self.wheel_front_vel = (0.5 * Vx) + (self.R * Wz)

        # Giới hạn vận tốc
        self.wheel_front_vel = max(min(self.wheel_front_vel, self.speed_limit), -self.speed_limit)
        self.wheel_left_vel = max(min(self.wheel_left_vel, self.speed_limit), -self.speed_limit)
        self.wheel_right_vel = max(min(self.wheel_right_vel, self.speed_limit), -self.speed_limit)

        self.publish_wheel_velocities()

    def publish_wheel_velocities(self):
        """Gửi vận tốc đến các bánh xe"""
        self.wheel_front_pub.publish(self.wheel_front_vel)
        self.wheel_left_pub.publish(self.wheel_left_vel)
        self.wheel_right_pub.publish(self.wheel_right_vel)

        rospy.loginfo(f"Front: {self.wheel_front_vel:.2f}, Left: {self.wheel_left_vel:.2f}, Right: {self.wheel_right_vel:.2f}")

    def get_key(self):
        """Đọc phím từ terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        """Chạy chế độ điều khiển bằng bàn phím"""
        rospy.loginfo("Điều khiển xe omni 3 bánh:")
        rospy.loginfo("w: Tiến, s: Lùi, a: Sang trái, d: Sang phải, q: Xoay trái, e: Xoay phải, x: Dừng")

        while not rospy.is_shutdown() and self.running:
            key = self.get_key()

            Vx, Vy, Wz = 0.0, 0.0, 0.0

            if key == 's':  # Tiến
                Vx = self.speed_limit
            elif key == 'a':  # Lùi
                Vx = -self.speed_limit
            elif key == 'w':  # Sang trái
                Vy = self.speed_limit
            elif key == 'd':  # Sang phải
                Vy = -self.speed_limit
            elif key == 'q':  # Xoay trái
                Wz = -self.speed_limit
            elif key == 'e':  # Xoay phải
                Wz = self.speed_limit
            elif key == 'x':  # Dừng
                rospy.loginfo("Dừng robot")
                self.wheel_front_vel = 0.0
                self.wheel_left_vel = 0.0
                self.wheel_right_vel = 0.0
                self.publish_wheel_velocities()
                self.running = False
                break

            twist_msg = Twist()
            twist_msg.linear.x = Vx
            twist_msg.linear.y = Vy
            twist_msg.angular.z = Wz
            self.cmd_vel_callback(twist_msg)

            rospy.sleep(0.1)

        self.cmd_vel_callback(Twist())

if __name__ == '__main__':
    try:
        controller = OmniWheelControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
