#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import sys
import tty
import termios

class OmniKeyboardControl:
    def __init__(self):
        # Khởi tạo node ROS
        rospy.init_node('omni_keyboard_control', anonymous=True)

        # Publisher cho các lệnh vận tốc bánh xe
        self.pub_left = rospy.Publisher('/left_wheel_joint_velocity_controller/command', Float64, queue_size=10)
        self.pub_right = rospy.Publisher('/right_wheel_joint_velocity_controller/command', Float64, queue_size=10)
        self.pub_front = rospy.Publisher('/front_wheel_joint_velocity_controller/command', Float64, queue_size=10)

        # Tốc độ tối đa (radian/s)
        self.max_speed = 10.0  # Giới hạn từ URDF: velocity="10"

        # Vận tốc hiện tại của từng bánh
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.front_speed = 0.0

    def get_key(self):
        # Đọc phím từ bàn phím mà không cần nhấn Enter
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        rospy.loginfo("Điều khiển robot omni 3 bánh bằng bàn phím:")
        rospy.loginfo("w: Tiến, s: Lùi, a: Trái, d: Phải, q: Xoay trái, e: Xoay phải")
        rospy.loginfo("x: Dừng và thoát")

        while not rospy.is_shutdown():
            key = self.get_key()

            # Đặt lại vận tốc về 0 trước khi tính toán
            self.left_speed = 0.0
            self.right_speed = 0.0
            self.front_speed = 0.0

            # Điều khiển chuyển động
            if key == 'w':  # Tiến
                self.left_speed = self.max_speed
                self.right_speed = -self.max_speed
                self.front_speed = 0.0
            elif key == 's':  # Lùi
                self.left_speed = -self.max_speed
                self.right_speed = self.max_speed
                self.front_speed = 0.0
            elif key == 'a':  # Sang trái
                self.left_speed = -self.max_speed
                self.right_speed = self.max_speed
                self.front_speed = -self.max_speed
            elif key == 'd':  # Sang phải
                self.left_speed = self.max_speed
                self.right_speed = -self.max_speed
                self.front_speed = self.max_speed
            elif key == 'q':  # Xoay trái
                self.left_speed = -self.max_speed
                self.right_speed  = self.max_speed
                self.front_speed = self.max_speed
            elif key == 'e':  # Xoay phải
                self.left_speed = self.max_speed
                self.right_speed = -self.max_speed
                self.front_speed = -self.max_speed
            elif key == 'z':  # Dừng và thoát
                self.left_speed = 0.0
                self.right_speed = 0.0
                self.front_speed = 0.0
            elif key == 'x':  # Dừng và thoát
                self.left_speed = 0.0
                self.right_speed = 0.0
                self.front_speed = 0.0
                self.pub_left.publish(self.left_speed)
                self.pub_right.publish(self.right_speed)
                self.pub_front.publish(self.front_speed)
                rospy.loginfo("Dừng robot và thoát")
                break

            # Gửi lệnh vận tốc
            self.pub_left.publish(self.left_speed)
            self.pub_right.publish(self.right_speed)
            self.pub_front.publish(self.front_speed)

            # Hiển thị trạng thái
            rospy.loginfo(f"Left: {self.left_speed:.2f}, Right: {self.right_speed:.2f}, Front: {self.front_speed:.2f}")

            rospy.sleep(0.1)  # Tránh đọc phím quá nhanh

if __name__ == '__main__':
    try:
        controller = OmniKeyboardControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass