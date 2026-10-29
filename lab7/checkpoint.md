### Setup
```bash
(in ros_workspaces)
git pull starter main

(static_tf_transform.py, MUST do before running anything)
line 62: self.transform_goal.header.frame_id   = 'ar_marker_<small tag>'
line 81: self.transform_center.header.frame_id = 'ar_marker_<large tag>'

cd ~/ros_workspaces/lab7
colcon build && source install/setup.bash
```

### Checkpoint 1
```bash
Task 3.1 (AR tag in RViz):
(first terminal)
ros2 run ur7e_utils enable_comms
(second terminal)
ros2 launch visual_servoing lab7.launch.py
    (Add -> By display type -> TF)
    (Add -> By topic -> /camera/camera/color/image_raw -> Image)
    (check ar_marker_N, goal_pose, center_pose in TF)

Task 3.2 (trajectories, offline):
python3 trajectories.py --task line --animate
python3 trajectories.py --task circle --animate

Task 3.2.4 (joint space controller):
ros2 run visual_servoing JointTrajectory --task line --ar_marker <large tag>
ros2 run visual_servoing JointTrajectory --task circle --ar_marker <large tag>
    (too slow / barely moves -> reduce trajectory time at bottom of JointTrajectory.py)

Task 3.3 (workspace setpoint, tune Kp only, Ki = Kd = 0):
(main.py line 67: self.Kp = 0.5, then colcon build)
(large tag lettering facing the computer, hand on e-stop)
ros2 run visual_servoing main --task pose --controller pid --ar_marker <large tag>
    (too slow / "Could not reach trajectory start" -> Kp up)
    (overshoot / oscillation -> Kp down)
    ("Point Reached" -> done, write down Kp)
```

1. Joint space vs workspace: joint space runs IK on every waypoint first, then tracks joint angles (slow, path between waypoints not guaranteed straight). Workspace computes the pose error twist xi_e = log(g_d g^-1)^vee every step, PID -> V_s, then theta_dot = pinv(J_s) V_s (no IK, real time, but blows up near singularities and ignores joint limits).
2. Kp tuning: started at 0.5, increased until fast with little overshoot. Final Kp = ____
3. V_ff for pose = 0, because the goal pose is fixed, g_d_dot = 0, so V_ff = (g_d_dot g_d^-1)^vee = 0.

### Checkpoint 2
```bash
Task 3.4 (workspace trajectory tracking, tune Kd then Ki):
(main.py line 67-69: Kp, Kd, Ki)
ros2 run visual_servoing main --task line --controller pid --ar_marker <large tag> \
--total_time 20 --circle_radius 0.05
ros2 run visual_servoing main --task circle --controller pid --ar_marker <large tag> \
--total_time 20 --circle_radius 0.05

Task 3.4.2 (feedforward off, then change back!):
(main.py line ~318: Vff = np.zeros(6))
ros2 run visual_servoing main --task circle --controller pid --ar_marker <large tag> \
--total_time 20 --circle_radius 0.05

Task 3.5 (visual servoing, hold small tag on a stick under the camera):
ros2 run visual_servoing main --controller pid --ar_marker <small tag>
```

1. PID tuning: Kp = ____, Kd = ____, Ki = ____
2. Feedforward: V_ff = [p_dot, 0] lets the robot move with the trajectory before error builds up; without it the controller lags behind (sim: 0.7 mm vs 3 cm max error on circle). Visual servoing can't use it because the future tag motion is unknown, so g_d_dot is unknown -> V_ff = 0.
3. Pose error -> joint velocities: xi_e = log(g_d g^-1)^vee, V_s = Kp xi_e + Kd d(xi_e)/dt + Ki nu, theta_dot = pinv(J_s(theta)) V_s, capped at 1 rad/s, published to /forward_velocity_controller/commands.
