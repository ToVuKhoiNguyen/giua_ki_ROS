#!/usr/bin/env python3
import rospy
import sys
import tty
import termios
import select
from std_msgs.msg import Float64

class ArmTeleopControl:
    def __init__(self):
        rospy.init_node('arm_teleop_control', anonymous=True)
        self.joint_arm_1_pub = rospy.Publisher('/arm_1_joint_controller/command', Float64, queue_size=10)
        self.joint_arm_2_pub = rospy.Publisher('/arm_2_joint_controller/command', Float64, queue_size=10)
 
        self.angle_step = 0.1
        self.angle_limit = 2.0  
        self.joint_arm_1_angle = 0.0
        self.joint_arm_2_angle = 0.0
        self.running = True

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)  
            select.select([sys.stdin], [], [], 0)  
            key = sys.stdin.read(1)  
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  
        return key

    def cmd_callback(self, key):
        if key == 'w':  
            self.joint_arm_1_angle = min(self.joint_arm_1_angle + self.angle_step, self.angle_limit)
        elif key == 's':  
            self.joint_arm_1_angle = max(self.joint_arm_1_angle - self.angle_step, -self.angle_limit)
        elif key == 'a':  
            self.joint_arm_2_angle = min(self.joint_arm_2_angle + self.angle_step, self.angle_limit)
        elif key == 'd':  
            self.joint_arm_2_angle = max(self.joint_arm_2_angle - self.angle_step, -self.angle_limit)
        elif key == 'x':  
            self.running = False

        self.joint_arm_1_pub.publish(self.joint_arm_1_angle)
        self.joint_arm_2_pub.publish(self.joint_arm_2_angle)
        rospy.loginfo(f"Joint Arm 1: {self.joint_arm_1_angle:.2f}, Joint Arm 2: {self.joint_arm_2_angle:.2f}")

    def run(self):
        rospy.loginfo("w: Tăng joint_arm_1, s: Giảm joint_arm_1, a: Tăng joint_arm_2, d: Giảm joint_arm_2, x: Thoát")
        while not rospy.is_shutdown() and self.running:
            key = self.get_key()  
            self.cmd_callback(key)  
            rospy.sleep(0.1)  
        rospy.signal_shutdown("Tắt teleop")  

if __name__ == '__main__':
    try:
        controller = ArmTeleopControl()  
        controller.run()  
    except rospy.ROSInterruptException:
        pass  