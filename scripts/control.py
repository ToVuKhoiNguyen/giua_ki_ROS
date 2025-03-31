#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import sys
import tty
import termios



class OmniWheelControl:
  def __init__(self):
      rospy.init_node('omni_wheel_control', anonymous=True)


      # Tạo publisher để gửi vận tốc đến 3 bánh xe
      self.pub_front = rospy.Publisher('/front_wheel_joint_velocity_controller/command', Float64, queue_size=10)
      self.pub_left = rospy.Publisher('/left_wheel_joint_velocity_controller/command', Float64, queue_size=10)
      self.pub_right = rospy.Publisher('/right_wheel_joint_velocity_controller/command', Float64, queue_size=10)


      # Nhận lệnh từ /cmd_vel (Twist)
      rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)


      # Bán kính bánh xe
      self.R = 0.25  # Khoảng cách từ tâm robot đến bánh (m)


      # Vận tốc tối đa
      self.max_speed = 5.0


      # Biến lưu vận tốc bánh xe
      self.v1 = 0.0  # Bánh trước
      self.v2 = 0.0  # Bánh trái
      self.v3 = 0.0  # Bánh phải


      # Biến điều khiển vòng lặp
      self.running = True


  def cmd_vel_callback(self, msg):
      """Nhận lệnh Twist và tính vận tốc bánh xe"""
      Vx = msg.linear.x   # Tiến/lùi
      Vy = msg.linear.y   # Sang trái/phải
      Wz = msg.angular.z  # Xoay tại chỗ


      if Wz != 0:  # Xoay tại chỗ
          self.v1 = Wz  # Bánh trước quay cùng chiều
          self.v2 = Wz  # Bánh trái quay cùng chiều
          self.v3 = Wz  # Bánh phải quay cùng chiều
      elif Vx != 0:
          self.v1 = -Wz * self.R  # Bánh trước chỉ di chuyển theo trục quay
          self.v2 = Vx - Wz * self.R  # Bánh trái khi tiến/lùi
          self.v3 = -Vx - Wz * self.R  # Bánh phải khi tiến/lùi
      elif Vy > 0:  # Dịch trái
          self.v1 = -Vy  # Bánh trước quay ngược chiều với hướng dịch
          self.v2 = 0.0  # Bánh trái đứng yên
          self.v3 = Vy  # Bánh phải quay cùng tốc độ nhưng ngược chiều bánh trước
      elif Vy < 0:  # Dịch phải
          self.v1 = -Vy  # Bánh trước quay cùng chiều với hướng dịch
          self.v2 = Vy  # Bánh trái quay cùng tốc độ nhưng ngược chiều bánh trước
          self.v3 = 0.0  # Bánh phải đứng yên


      # Giới hạn vận tốc
      self.v1 = max(min(self.v1, self.max_speed), -self.max_speed)
      self.v2 = max(min(self.v2, self.max_speed), -self.max_speed)
      self.v3 = max(min(self.v3, self.max_speed), -self.max_speed)


      # Xuất lệnh điều khiển
      self.pub_front.publish(self.v1)
      self.pub_left.publish(self.v2)
      self.pub_right.publish(self.v3)


      rospy.loginfo(f"V1 (Front): {self.v1:.2f}, V2 (Left): {self.v2:.2f}, V3 (Right): {self.v3:.2f}")


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
      rospy.loginfo("w: Tiến, s: Lùi, a: Trái, d: Phải, q: Xoay trái, e: Xoay phải, x: Dừng")


      while not rospy.is_shutdown() and self.running:
          key = self.get_key()


          Vx = 0.0
          Vy = 0.0
          Wz = 0.0


          if key == 'w':  # Tiến
              Vx = self.max_speed
          elif key == 's':  # Lùi
              Vx = -self.max_speed
          elif key == 'a':  # Sang trái
              Vy = self.max_speed
          elif key == 'd':  # Sang phải
              Vy = -self.max_speed
          elif key == 'q':  # Xoay trái
              Wz = -self.max_speed
          elif key == 'e':  # Xoay phải
              Wz = self.max_speed
          elif key == 'x':  # Dừng
              rospy.loginfo("Dừng robot")
              self.running = False
              break


          # Gửi lệnh cmd_vel
          twist_msg = Twist()
          twist_msg.linear.x = Vx
          twist_msg.linear.y = Vy
          twist_msg.angular.z = Wz
          self.cmd_vel_callback(twist_msg)


          rospy.sleep(0.1)  # Tránh spam lệnh


      # Đảm bảo robot dừng khi thoát vòng lặp
      stop_msg = Twist()
      self.cmd_vel_callback(stop_msg)


if __name__ == '__main__':
  try:
      controller = OmniWheelControl()
      controller.run()
  except rospy.ROSInterruptException:
      pass
