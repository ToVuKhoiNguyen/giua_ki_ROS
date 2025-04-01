## Mở Gazebo và Rviz
Mở một terminal mới:  
roslaunch omni omni.launch

## Điều khiển xe  
Mở một terminal mới, chạy lệnh:  
rosrun omni control.py

## Điều khiển tay máy
Mở một terminal khác:  
rosrun omni arm.py   
  
## Check /scan – vật cản và camera trong map  
Có thể tắt IMU để quan sát rõ hơn. 
RViz sẽ hiển thị vật cản khi robot quét.  
Khi robot di chuyển, TF xoay theo chuyển động bánh xe và tay máy. Tay máy lúc này vẫn đang chạy trong vòng lặp.  
Có thể mở terminal khác để kiểm tra các topic của cảm biến
