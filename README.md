# MID-TERM 
# Mô tả
Mô phỏng Robot Omni ba bánh và tay máy trong Gazebo và mô tả trong Rviz. Xe có thể di chuyển bằng bàn phím (bao gồm cả xe và tay máy), đồng thời có thể đọc được các cảm biến IMU, LidDAR, Camera.
# 1. Setup môi trường
   ROS, Gazebo, Rviz
   Download source: ```
    git clone https://github.com/ToVuKhoiNguyen/omni
    ```
   Đảm bảo không gian làm việc: ```
     catkin_make
     source devel/setup.bash
    ```
# Các bước thực hiện :
# 1. Chạy Gazebo và Rviz
```
roslaunch omni omni.launch
```
# 3. Điều khiển xe 
```
rosrun omni control.py 
``
# 3. Điều khiển tay máy
```
rosrun omni arm.py
``
# 4. Link video demo : 
https://drive.google.com/file/d/1ku3pGa2G42McpiYHarjgyWkxFSMXIxWk/view?usp=drive_link
