import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/leo/Documents/eecs_106/ros_workspaces/lab2/install/lab2_turtlesim'
