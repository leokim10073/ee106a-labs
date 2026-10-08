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
