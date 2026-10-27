import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO


MODEL_FILES = {
    'cones': 'segment_bounding_box_cones.pt',
    'coco': 'yolov8n-seg.pt',
}

# Cone silhouette area, m^2
CONE_AREA = 0.0208227849


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.bridge = CvBridge()

        self.declare_parameter('model', 'cones')
        # For coco, pass a comma-separated list: -p target_class:="cup,apple"
        self.declare_parameter('target_class', 'all')
        self.declare_parameter('confidence', 0.25)
        self.declare_parameter('robot_name', os.environ.get('ROBOT_NAME', ''))
        self.declare_parameter('image_topic', '')
        self.declare_parameter('camera_info_topic', '')
        self.declare_parameter('reject_truncated', True)

        model_key = self.get_parameter('model').value
        self.target_class = self.get_parameter('target_class').value
        self.confidence = self.get_parameter('confidence').value
        self.reject_truncated = self.get_parameter('reject_truncated').value

        robot_name = self.get_parameter('robot_name').value.strip('/')
        prefix = f'/{robot_name}' if robot_name else ''
        image_topic = (
            self.get_parameter('image_topic').value or f'{prefix}/image_raw'
        )
        camera_info_topic = (
            self.get_parameter('camera_info_topic').value or f'{prefix}/camera_info'
        )

        self.model = self._load_model(model_key)
        self.class_filter = self._resolve_class_filter(self.target_class)
        self.object_area = CONE_AREA if model_key == 'cones' else None

        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 1
        )
        self.cones_pub = self.create_publisher(PoseArray, '/cones', 1)

        self.annotated_image_pub = self.create_publisher(
            Image,
            '/yolo/annotated_image',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.last_count = None
        self.last_detections = None

        mode = ('boxes + depth on /cones' if self.object_area
                else 'boxes only, no depth')
        self.get_logger().info(
            f'model={model_key} target={self.target_class} ({mode}) '
            f'image={image_topic} camera_info={camera_info_topic} '
            f'boxes=/yolo/annotated_image'
        )

    def _load_model(self, model_key):
        model_file = MODEL_FILES.get(model_key, model_key)
        model_path = os.path.join(
            get_package_share_directory('perception'), 'utilities', model_file
        )
        if not os.path.exists(model_path):
            self.get_logger().warn(
                f'{model_path} not found; letting ultralytics resolve '
                f'"{model_file}" instead (downloads on first use).'
            )
            model_path = model_file
        return YOLO(model_path)

    def _resolve_class_filter(self, target_class):
        """Comma-separated class names -> ids. None means every class."""
        names = self.model.names
        if target_class.lower() in ('', 'all'):
            return None

        lookup = {name.lower(): class_id for class_id, name in names.items()}
        wanted = [t.strip() for t in target_class.split(',') if t.strip()]
        ids = [lookup[w.lower()] for w in wanted if w.lower() in lookup]
        missing = [w for w in wanted if w.lower() not in lookup]

        if missing:
            self.get_logger().warn(
                f'not a class in this model: {", ".join(missing)}. '
                f'Available: {sorted(names.values())}.'
            )
        if not ids:
            self.get_logger().warn('Detecting all classes.')
            return None
        return ids

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(
            cv_image,
            classes=self.class_filter,
            conf=self.confidence,
            verbose=False,
        )
        result = results[0]

        # Bounding boxes go out for every model -- this is how detection gets
        # checked in rviz. Only the cone model goes on to compute depth.
        annotated_msg = self.bridge.cv2_to_imgmsg(result.plot(), encoding='bgr8')
        annotated_msg.header = msg.header
        self.annotated_image_pub.publish(annotated_msg)

        if self.object_area is None:
            self._log_detections(result)
            return

        if self.fx is None:
            return

        masks = self._valid_masks(result, cv_image.shape[:2])
        if len(masks) != self.last_count:
            self.get_logger().info(f'{len(masks)} cone(s) detected')
            self.last_count = len(masks)
        if not masks:
            return

        cones = PoseArray()
        cones.header.stamp = msg.header.stamp
        cones.header.frame_id = 'base_link'

        for mask in masks:
            # TODO: Count the non-zero pixels in the mask to get A_pixels.
            pixel_count = 0.

            # TODO: Recover depth with Equation 10. self.object_area is A_real.
            depth = 0.

            # TODO: Mask centroid in image coordinates. np.nonzero returns
            #       (rows, cols): rows index v, columns index u.
            u, v = 0., 0.

            # TODO: Back-project to the camera frame (Equations 1 & 2).
            X = 0.
            Y = 0.
            Z = 0.

            # Convert to turtlebot frame
            # There's no camera frame for the turtlebots, so we just do this instead
            G = np.array([[0, 0, 1, 0.115],
                          [-1, 0, 0, 0],
                          [0, -1, 0, 0],
                          [0, 0, 0, 1]])
            cone_point = (G @ np.array([X, Y, Z, 1]).reshape(4, 1)).flatten()

            pose = Pose()
            pose.position.x = float(cone_point[0])
            pose.position.y = float(cone_point[1])
            pose.position.z = float(cone_point[2])
            pose.orientation.w = 1.0
            cones.poses.append(pose)

            self.get_logger().info(
                f'cone at ({cone_point[0]:.2f}, {cone_point[1]:.2f}) '
                f'depth={depth:.3f}m'
            )

        self.cones_pub.publish(cones)

    def camera_info_callback(self, msg):
        # TODO: Store fx, fy, cx, cy from the K matrix as self.fx, self.fy,
        #       self.cx, self.cy. K is a flat row-major 9-element array.
        self.get_logger().info('Received Camera Info',
                               throttle_duration_sec=10.0)

    def _log_detections(self, result):
        found = sorted({self.model.names[int(c)] for c in result.boxes.cls})
        if found != self.last_detections:
            self.get_logger().info(f'detected: {", ".join(found) or "nothing"}')
            self.last_detections = found

    def _valid_masks(self, result, image_shape):
        if result.masks is None:
            if len(result.boxes) == 0:
                return []          # nothing in frame -- not an error
            self.get_logger().error(
                'Got boxes but no masks: this checkpoint has no segmentation '
                'head. Use a "-seg" model.',
                throttle_duration_sec=10.0,
            )
            return []

        img_height, img_width = image_shape
        valid = []
        for mask in result.masks.data.cpu().numpy():
            # Masks come back at inference resolution, not the resolution
            # fx/fy were calibrated against.
            if mask.shape != (img_height, img_width):
                mask = cv2.resize(
                    mask, (img_width, img_height), interpolation=cv2.INTER_NEAREST
                )
            # A truncated object has a small mask and reads as too far away,
            # but a cone on the floor often runs off the bottom of the frame.
            if self.reject_truncated and self._touches_border(mask):
                self.get_logger().warn(
                    'dropped a mask touching the frame edge; '
                    'set reject_truncated:=false to keep it',
                    throttle_duration_sec=10.0,
                )
                continue
            valid.append(mask)
        return valid

    @staticmethod
    def _touches_border(mask):
        return bool(
            mask[0, :].any() or mask[-1, :].any()
            or mask[:, 0].any() or mask[:, -1].any()
        )


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
