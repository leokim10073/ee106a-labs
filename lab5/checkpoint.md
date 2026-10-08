### Checkpoint 1
```bash
terminal 

ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true rgb_camera.color_profile:=1920x1080x30
rviz2
    (set fixframe to camera_depth_optical_frame)
    (add point topic)
ros2 run ur7e_utils enable_comms
ros2 run planning tf
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



