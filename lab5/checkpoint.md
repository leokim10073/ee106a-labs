### Checkpoint 1
```bash
terminal 

ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true rgb_camera.color_profile:=1920x1080x30
rviz2
    (set fixframe to camera_depth_optical_frame)
    (add point topic)
ros2 run ur7e_utils enable_comms
ros2 run planning static_tf_transform.py
ros2 run tf2_tools view_frames
============================================================
xdg-open frames_*.pdf


```

### Checkpoint 2
```bash

ros2 run perception process_pointcloud
rviz2 
    (set fixframe to baselink)
    (Add /filter_planes、/filtered_points、/cube_pose topic)

Adjustment:    
ros2 param set /realsense_pc_subscriber min_z -0.17
ros2 param set /realsense_pc_subscriber max_z -0.10

```

1. plane normal to along +z vector



### Checkpoint 3

```bash
Task 5:
ros2 run ur7e_utils enable_comms
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur7e launch_rviz:=true
ros2 run planning ik

Task 6:
measure GRIPPER_LENGTH, and modify it in main.py
colcon build && source install/setup.bash

ros2 run ur7e_utils enable_comms
ros2 launch planning lab5_bringup.launch.py
ros2 run ur7e_utils freedrive   (move to desire position Ctrl+C)
(ensure gripper open, and emergency stop stadn by)
ros2 run planning main
(no tape response)

Task 7:
(enable_comms 和 lab5_bringup.launch.py)
ros2 run perception hsv_tuner
H = color, S =, V = brightness
(press p to get command, execute ros2 param set)
(Add topic /tape_points, /tape_pose, /filtered_points)

Task 8:
ros2 run ur7e_utils enable_comms
ros2 launch planning lab5_bringup.launch.py
ros2 run ur7e_utils freedrive 
ros2 run planning main
```
