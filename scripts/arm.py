#!/usr/bin/env python3

import rospy
import sys, select, termios, tty
from std_msgs.msg import Float64

msg = """
Điều khiển tay máy:

    Nhấn 'w' để tăng joint_1
    Nhấn 's' để giảm joint_1
    Nhấn 'a' để tăng joint_2
    Nhấn 'd' để giảm joint_2
    Nhấn 'q' để thoát

"""

# Mức điều chỉnh góc mỗi lần nhấn phím (radians)
step_size = 0.1

# Giá trị ban đầu của các joint
joint_1_angle = 0.0
joint_2_angle = 0.0

def get_key():
    """Đọc phím từ terminal"""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def teleop_control():
    global joint_1_angle, joint_2_angle

    rospy.init_node('arm_teleop')
    joint1_pub = rospy.Publisher('/arm_1_joint_controller/command', Float64, queue_size=10)
    joint2_pub = rospy.Publisher('/arm_2_joint_controller/command', Float64, queue_size=10)
    rospy.sleep(1)

    print(msg)

    while not rospy.is_shutdown():
        key = get_key()

        if key == 'w':
            joint_1_angle += step_size
        elif key == 's':
            joint_1_angle -= step_size
        elif key == 'a':
            joint_2_angle += step_size
        elif key == 'd':
            joint_2_angle -= step_size
        elif key == 'q':
            print("\nThoát teleop!")
            break

        rospy.loginfo(f"Joint 1: {joint_1_angle}, Joint 2: {joint_2_angle}")
        joint1_pub.publish(joint_1_angle)
        joint2_pub.publish(joint_2_angle)

    rospy.signal_shutdown("Tắt teleop")

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        teleop_control()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
