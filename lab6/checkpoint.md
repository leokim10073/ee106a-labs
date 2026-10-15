### Checkpoint 1
```bash
ros2 launch turtlebot3_bringup robot.launch.py
ros2 launch usb_cam usb_cam.launch.py

ros2 run lab6_capture capture_pose --ros-args \
-p with_pose:=false -p image_topic:=/ROBOTNAME/image_raw
(Hold the board in front of the camera and press space to save a picture. Save path: ~/ros_workspaces/lab6/captures/calib_<date>_<time>/)

```

1. fx and fy is the focal length scaled into pixel dimensions along the horizontal and vertical axes, and cx, cy is the principal point where the optical axis intersects the image sensor in pixel coordinates.
2. We have to undistort images because optical lenses bend light rays which cause straight lines to appear curved near the image edges. We model image distortion with radial (k1, k2, k3) and tangential (p1, p2) distortion parameters.
3. We need pictures with the board at different angles to constrain all degrees of freedom in the intrinsic matrix. If every picture had the board straight-on in the center, geometric equations fail to constrain the focal length and distortion parameters accurately.
4. Reprojection error measures the distance in pixels between the detected calibration corners in an image and where the mathematical camera model predicts those corners should appear based on the estimated intrinsics and pose.

### Checkpoint 2
```bash
(first terminal)
source ~/ros_workspaces/lab4/install/setup.bash
ros2 launch turtlebot_controller slam_launch.py

(second terminal)
source ~/ros_workspaces/lab6/install/setup.bash
ros2 run lab6_capture capture_pose --ros-args -p image_topic:=/ROBOTNAME/image_raw

(third terminal)
source ~/ros_workspaces/lab6/install/setup.bash
ros2 run lab6_capture hold_teleop

Task2:
python3 features.py ../captures/run_<date>_<time> # matches between neighbors
python3 features.py ../captures/run_<date>_<time> --image 5 # draw image 5's keypoints
python3 features.py ../captures/run_<date>_<time> --pair 5 6 # draw matches between 5 and 6

Task3: 
python3 two_view.py ../captures/run_<date>_<time> 5 6
```
1. Explain why the carpet has so few matches, even though it has many keypoints: 
The carpet consists of repetitive and similar patterns where many descriptor vectors look nearly identical, causing Lowe's ratio test to reject candidate matches as ambiguous.
2. Geometrical meaning of the equation is the ray from camera 1 through pixel x1 projects onto image 2 as a line (l2 = Fx1). And, a match has to lie on its epipolar line because x1, x2, and both camera centers all lie within the same 3D epipolar plane.
3. The epipole sits where one camera center projects into the other camera's image plane, and the epipolar lines appear nearly parallel when the camera moves sideways translation relative to its viewing direction.
4. We can’t fit F to all the matches with least squares, without RANSAC: Standard least squares minimizes the squared distance for all points without distinction, meaning a outlier will disproportionately skew the entire estimation of F.
5. We black out the upper part of the image to prevent moving or irrelevant distant features from forming incorrect matches. A decrease in the total
number of matches doesn't mean the result is worse since remaining matches are more reliable, and K remains unchanged because image dimensions and pixel coordinate geometry are strictly preserved.

### Checkpoint 3
```bash
Task4:
python3 two_view.py ../captures/run_<date>_<time> 5 6
python3 view_viser.py ../captures/run_<date>_<time> --two-view

Task5:
python3 multiview.py ../captures/run_<date>_<time>


python3 bundle_adjust.py ../captures/run_<date>_<time>
python3 view_viser.py ../captures/run_<date>_<time>
```

1.
2.
3.