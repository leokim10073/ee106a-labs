# Lab 3 Checkpoints Summary

## Checkpoint 0: Initial Setup

### Requirements
1. Set up Distrobox with `ros_workspaces` folder (both `origin` and `starter` remotes, and SSH setup configured)
2. Set up VS Code for development (Python, C/C++ Extension Pack, CMake Tools, XML, YAML extensions)
3. Complete the Robot Usage Quiz on Gradescope
4. Share personal repository with partner

### Commands
```bash
# Check and remove existing distroboxes
distrobox list
distrobox rm -f <container_name>

# Create and enter the ROS2 container
inst-containers-setup ee106a/create-ros2-container
distrobox enter ros2

# Git identity setup
git config --global user.name "Your Name"
git config --global user.email "your_email@example.com"

# SSH key generation & GitHub setup
ssh-keygen -t ed25519 -C "your_email@example.com"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
cat ~/.ssh/id_ed25519.pub
# Add public key to GitHub -> Settings -> SSH and GPG keys
ssh -T git@github.com

# Clone personal repo inside container
cd ~
git clone git@github.com:GITHUB_USERNAME/YOUR_REPO.git ros_workspaces
cd ~/ros_workspaces

# Add starter remote & sync
git remote add starter [https://github.com/ucb-ee106/106a-student-labs-starter.git](https://github.com/ucb-ee106/106a-student-labs-starter.git)
git fetch starter

# Clean up outdated directories and restore Fall 2026 lab versions
git rm -rf --ignore-unmatch lab3 lab3_ur7e lab4 lab5 lab6 lab7 lab8
git restore --source=starter/main --staged --worktree lab3 lab4
git commit -m "Update Labs 3 and 4 for Fall 2026"
git push origin main

```
## Checkpoint 1: Initial Setup

```bash
# Enable communication with the robot (KEEP RUNNING in background!)
ros2 run ur7e_utils enable_comms

# Validate safe trajectories (KEEP RUNNING in background!)
ros2 run joint_control validate_trajectory

# Return arm to tuck position
ros2 run ur7e_utils tuck

# Option A: Move joints with keyboard
ros2 run ur7e_utils keyboard_controller

# Option B: Move manually in freedrive mode (move slowly and gently!)
ros2 run ur7e_utils freedrive

# Observe joint state updates
ros2 topic echo /joint_states

# Visualize UR7e model in RViz2
rviz2 -d /opt/ros/humble/share/ur_description/rviz/view_robot.rviz

# Run your FK node to test output
ros2 run forward_kinematics forward_kinematics_node
```

# Checkpoint 2: TF and Coordinate Frames

The goal of Checkpoint 2 is to implement a custom ROS 2 TF listener node, query coordinate transformations between links, and verify that the results match your Forward Kinematics (FK) implementation.

---

## 1. Prerequisites (Keep Running in Background)

Do **NOT** close the core services from Checkpoint 1. Ensure the following nodes remain active:

```bash
# Terminal 1: Robot communication (keep running!)
ros2 run ur7e_utils enable_comms

# Terminal 2: Safety validation (keep running!)
ros2 run joint_control validate_trajectory

# Option A: Move the arm gently by hand
ros2 run ur7e_utils freedrive

# Option B: Move joints using keyboard
ros2 run ur7e_utils keyboard_controller

# Launch RViz2 to inspect frames
# (In RViz: click 'Add' -> 'TF', then expand 'TF -> Frames' to view 'base_link' and 'wrist_3_link')
rviz2 -d /opt/ros/humble/share/ur_description/rviz/view_robot.rviz
```
### Testing

```bash
# Step 1: Verify TF broadcast using the built-in ROS 2 tool
ros2 run tf2_ros tf2_echo base_link wrist_3_link

# Step 2: Run your completed TF listener node
ros2 run tf_listener tf_listener base_link wrist_3_link

# Step 3: Run your FK node side-by-side to compare numerical values
ros2 run forward_kinematics forward_kinematics_node

```
# Checkpoint 3: Publish trajectory

```bash
# Echo joint angles to record Pose A and Pose B
ros2 topic echo /joint_states

# Run direct joint controller to move between Pose A and Pose B
ros2 run joint_control joint_controller

# Launch MoveIt with RViz
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur7e launch_rviz:=true

# Run your straight-line action server
ros2 run straight_line straight_line_server

# Send action goal to test straight-line Cartesian path
ros2 action send_goal /move_straight straight_line_interface/action/MoveStraight "{target: {position: {x: 0.3, y: 0.1, z: 0.4}}, max_step: 0.01}" --feedback

```