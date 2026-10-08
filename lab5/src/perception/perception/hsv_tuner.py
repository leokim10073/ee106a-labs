# Tool for finding the HSV range of your tape:
#   ros2 run perception hsv_tuner
# Drag the sliders until only the tape is white in the mask. Press p to print the
# ros2 param set commands for process_pointcloud, and q to quit.
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

WINDOW = 'hsv_tuner'
SLIDERS = [('H min', 100, 179), ('H max', 130, 179), ('S min', 120, 255), ('V min', 50, 255)]


class HSVTuner(Node):
    def __init__(self):
        super().__init__('hsv_tuner')
        topic = self.declare_parameter('image_topic', '/camera/camera/color/image_raw').value
        self.bridge = CvBridge()
        self.image = None
        self.create_subscription(Image, topic, self.on_image, 1)
        self.get_logger().info(f'listening on {topic}')

    def on_image(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


def main(args=None):
    rclpy.init(args=args)
    node = HSVTuner()
    cv2.namedWindow(WINDOW)
    for name, value, top in SLIDERS:
        cv2.createTrackbar(name, WINDOW, value, top, lambda _: None)

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        h_min, h_max, s_min, v_min = [cv2.getTrackbarPos(name, WINDOW) for name, _, _ in SLIDERS]
        if node.image is not None:
            small = cv2.resize(node.image, (640, 360))
            hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([h_min, s_min, v_min]), np.array([h_max, 255, 255]))
            cv2.imshow(WINDOW, np.hstack([small, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)]))
        key = cv2.waitKey(10) & 0xFF
        if key == ord('p'):
            print('\nros2 param set /process_pointcloud tape_h_min', h_min)
            print('ros2 param set /process_pointcloud tape_h_max', h_max)
            print('ros2 param set /process_pointcloud tape_s_min', s_min)
            print('ros2 param set /process_pointcloud tape_v_min', v_min, flush=True)
        elif key == ord('q'):
            break

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
