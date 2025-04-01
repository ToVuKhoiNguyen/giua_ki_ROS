#!/usr/bin/env python3

import rospy
import sys
import tty
import termios
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class OmniWheelControl:
    def __init__(self):
        rospy.init_node('omni_wheel_control', anonymous=True)
        self.wheel_front_pub = rospy.Publisher('/front_wheel_joint_velocity_controller/command', Float64, queue_size=10)
        self.wheel_left_pub = rospy.Publisher('/left_wheel_joint_velocity_controller/command', Float64, queue_size=10)
        self.wheel_right_pub = rospy.Publisher('/right_wheel_joint_velocity_controller/command', Float64, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.sleep(1)  

        self.R = 0.25  
        self.speed_limit = 4.0  
        self.wheel_front_vel = 0.0
        self.wheel_left_vel = 0.0
        self.wheel_right_vel = 0.0
        self.running = True

    def cmd_vel_callback(self, msg):
        Vx = msg.linear.x    
        Vy = msg.linear.y   
        Wz = msg.angular.z  

        if Wz != 0:  
            self.wheel_front_vel = Wz
            self.wheel_left_vel = Wz
            self.wheel_right_vel = Wz
        elif Vx != 0:
            self.wheel_front_vel = -Wz * self.R
            self.wheel_left_vel = Vx - Wz * self.R
            self.wheel_right_vel = -Vx - Wz * self.R
        elif Vy > 0: 
            self.wheel_front_vel = -Vy
            self.wheel_left_vel = 0.0
            self.wheel_right_vel = Vy
        elif Vy < 0:  
            self.wheel_front_vel = -Vy
            self.wheel_left_vel = Vy
            self.wheel_right_vel = 0.0

        self.wheel_front_vel = max(min(self.wheel_front_vel, self.speed_limit), -self.speed_limit)
        self.wheel_left_vel = max(min(self.wheel_left_vel, self.speed_limit), -self.speed_limit)
        self.wheel_right_vel = max(min(self.wheel_right_vel, self.speed_limit), -self.speed_limit)
        self.publish_wheel_velocities()

    def publish_wheel_velocities(self):
        self.wheel_front_pub.publish(self.wheel_front_vel)
        self.wheel_left_pub.publish(self.wheel_left_vel)
        self.wheel_right_pub.publish(self.wheel_right_vel)
        rospy.loginfo(f"Front: {self.wheel_front_vel:.2f}, Left: {self.wheel_left_vel:.2f}, Right: {self.wheel_right_vel:.2f}")

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def stop_robot(self):
        self.wheel_front_vel = 0.0
        self.wheel_left_vel = 0.0
        self.wheel_right_vel = 0.0
        self.publish_wheel_velocities()

    def run(self):
        rospy.loginfo("w: Tiến, s: Lùi, a: Trái, d: Phải, q: Xoay trái, e: Xoay phải, k: Dừng, x: Thoát")

        while not rospy.is_shutdown() and self.running:
            key = self.get_key()

            rospy.loginfo(f"Phím đang nhấn: {key}")
            Vx, Vy, Wz = 0.0, 0.0, 0.0

            if key == 'w':  
                Vx = self.speed_limit
            elif key == 's':  
                Vx = -self.speed_limit
            elif key == 'a':  
                Vy = self.speed_limit
            elif key == 'd':  
                Vy = -self.speed_limit
            elif key == 'q':  
                Wz = -self.speed_limit
            elif key == 'e':  
                Wz = self.speed_limit
            elif key == 'k':  
                self.stop_robot()
                continue
            elif key == 'x':  
                self.stop_robot()
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
