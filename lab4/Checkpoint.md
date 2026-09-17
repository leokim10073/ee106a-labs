# EECS 106A Lab 4 - Checkpoint 1 指令彙整

---

## 0. Initial Setup (Lab Computer & Distrobox)

### 0.1 重設 Distrobox 容器
```bash
distrobox list

distrobox rm -f <container_name>

inst-containers-setup ee106a/create-ros2-container
distrobox enter ros2
```

### 0.2 設定 SSH Key 與 GitHub 連線
```bash
git config --global user.name "Your Name"
git config --global user.email "your_email@example.com"

ssh-keygen -t ed25519 -C "your_email@example.com"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519

cat ~/.ssh/id_ed25519.pub
ssh -T git@github.com
```

### 0.3 Git
```bash
cd ~
git clone git@github.com:GITHUB_USERNAME/YOUR_REPO.git ros_workspaces
cd ~/ros_workspaces

git remote add starter [https://github.com/ucb-ee106/106a-student-labs-starter.git](https://github.com/ucb-ee106/106a-student-labs-starter.git)
git fetch starter

git rm -rf --ignore-unmatch lab3 lab3_ur7e lab4 lab5 lab6 lab7 lab8
git restore --source starter/main --staged --worktree lab3 lab4
git commit -m "Update Labs 3 and 4 for Fall 2026"
git push origin main
```

---
## 1. Introduction to the TurtleBots 

### 1.1 TurtleBot Launch
```bash
ping fruitname
ssh fruitname@fruitname
ros2 launch turtlebot3_bringup robot.launch.py
```

### 1.2 Check
```bash
ssh fruitname@fruitname
ros2 launch usb_cam usb_cam.launch.py
rviz2
ros2 topic list
ros2 topic echo /imu
ros2 topic echo /odom
ros2 run turtlebot3_teleop teleop_keyboard
```

---

## 2. Checkpoint1: 
```bash
ros2 run turtlebot_controller square_drive
```

## 3. Checkpoint2: 
```bash
ros2 launch turtlebot_controller slam_launch.py
ros2 run nav2_map_server map_saver_cli -f ~/ros_workspaces/lab4/slam_map
```

## 4. Checkpoint3: 
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/ros_workspaces/lab4/slam_map.yaml
ros2 launch turtlebot_controller tag_launch.py
ros2 topic echo /aruco_markers
ros2 run tf2_ros tf2_echo camera ar_marker_<ID>
## Just Test Nav Client
ros2 run turtlebot_controller nav_client --ros-args -p x:=1.5 -p y:=0.0
## Pure Navigate to tag
ros2 run turtlebot_controller nav_client --ros-args -p marker_id:=<ID>
##Put three obstacles test
ros2 run turtlebot_controller nav_client --ros-args -p marker_id:=<ID>
```
