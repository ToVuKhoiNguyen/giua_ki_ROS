#!/usr/bin/env python3

import rospy
import sys
import tty
import termios
import select
from std_msgs.msg import Float64

class ArmTeleopControl:
    def __init__(self):
        # Khởi tạo node ROS
        rospy.init_node('arm_teleop_control', anonymous=True)

        # Tạo publisher cho hai khớp của tay máy
        self.joint_arm_1_pub = rospy.Publisher('/arm_1_joint_controller/command', Float64, queue_size=10)
        self.joint_arm_2_pub = rospy.Publisher('/arm_2_joint_controller/command', Float64, queue_size=10)
 
        # Bước điều chỉnh góc mỗi lần nhấn phím (radians)
        self.angle_step = 0.1
        self.angle_limit = 2.0  # Giới hạn góc tối đa

        # Biến lưu giá trị góc hiện tại
        self.joint_arm_1_angle = 0.0
        self.joint_arm_2_angle = 0.0

        # Biến điều khiển vòng lặp
        self.running = True

    def get_key(self):
        """Đọc phím từ terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)  # Thiết lập chế độ đọc phím ngay lập tức
            select.select([sys.stdin], [], [], 0)  # Kiểm tra phím nhấn
            key = sys.stdin.read(1)  # Đọc một ký tự từ bàn phím
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # Khôi phục cài đặt bàn phím ban đầu
        return key

    def cmd_callback(self, key):
        """Xử lý phím nhập để điều khiển tay máy"""
        if key == 'w':  # Tăng góc joint_arm_1
            self.joint_arm_1_angle = min(self.joint_arm_1_angle + self.angle_step, self.angle_limit)
        elif key == 's':  # Giảm góc joint_arm_1
            self.joint_arm_1_angle = max(self.joint_arm_1_angle - self.angle_step, -self.angle_limit)
        elif key == 'a':  # Tăng góc joint_arm_2
            self.joint_arm_2_angle = min(self.joint_arm_2_angle + self.angle_step, self.angle_limit)
        elif key == 'd':  # Giảm góc joint_arm_2
            self.joint_arm_2_angle = max(self.joint_arm_2_angle - self.angle_step, -self.angle_limit)
        elif key == 'x':  # Thoát khi nhấn phím 'x'
            rospy.loginfo("Thoát teleop!")
            self.running = False

        # Xuất lệnh điều khiển
        self.joint_arm_1_pub.publish(self.joint_arm_1_angle)
        self.joint_arm_2_pub.publish(self.joint_arm_2_angle)

        rospy.loginfo(f"Joint Arm 1: {self.joint_arm_1_angle:.2f}, Joint Arm 2: {self.joint_arm_2_angle:.2f}")

    def run(self):
        """Chạy chế độ điều khiển bằng bàn phím"""
        rospy.loginfo("Điều khiển tay máy:")
        rospy.loginfo("w: Tăng joint_arm_1, s: Giảm joint_arm_1, a: Tăng joint_arm_2, d: Giảm joint_arm_2, x: Thoát")

        while not rospy.is_shutdown() and self.running:
            key = self.get_key()  # Đọc phím từ bàn phím
            self.cmd_callback(key)  # Xử lý lệnh điều khiển
            rospy.sleep(0.1)  # Tránh spam lệnh quá nhanh

        rospy.signal_shutdown("Tắt teleop")  # Tắt ROS node khi thoát

if __name__ == '__main__':
    try:
        controller = ArmTeleopControl()  # Tạo đối tượng điều khiển
        controller.run()  # Chạy điều khiển
    except rospy.ROSInterruptException:
        pass  # Nếu có lỗi ROS, không làm gì
