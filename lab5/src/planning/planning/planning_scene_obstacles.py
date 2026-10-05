#!/usr/bin/env python3
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


def _box(obj_id, frame, size, center):
    co = CollisionObject()
    co.id = obj_id
    co.header.frame_id = frame
    co.operation = CollisionObject.ADD
    prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size])
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in center)
    pose.orientation.w = 1.0
    co.primitives.append(prim)
    co.primitive_poses.append(pose)
    return co


class PlanningSceneObstacles(Node):
    def __init__(self):
        super().__init__('planning_scene_obstacles')
        default = os.path.join(get_package_share_directory('planning'), 'config', 'obstacles.yaml')
        self.declare_parameter('obstacles_config', default)
        self._cfg_path = self.get_parameter('obstacles_config').get_parameter_value().string_value
        self._cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self._done = False
        self._future = None
        self.create_timer(1.0, self._apply_once)

    def _apply_once(self):
        if self._done:
            return
        # Poll the asynchronous response without blocking the timer's callback group.
        # Keep at most one request in flight, and retry a rejected/failed request.
        if self._future is not None:
            if not self._future.done():
                return
            try:
                res = self._future.result()
            except Exception as exc:
                self.get_logger().error(f'ApplyPlanningScene failed: {exc}')
            else:
                if res and res.success:
                    self._done = True
                    self.get_logger().info(f'Applied obstacles from {self._cfg_path}')
                else:
                    self.get_logger().error('ApplyPlanningScene failed')
            self._future = None
            return
        if not self._cli.service_is_ready():
            return
        with open(self._cfg_path, encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        frame = cfg.get('frame_id', 'base_link')
        scene = PlanningScene(is_diff=True)
        for w in cfg.get('world_obstacles', []):
            scene.world.collision_objects.append(_box(w['id'], frame, w['size'], w['center']))
        attached = cfg.get('attached_obstacles', [])
        if attached:
            scene.robot_state.is_diff = True
            for a in attached:
                link = a.get('link_name', 'wrist_3_link')
                aco = AttachedCollisionObject(
                    link_name=link,
                    object=_box(a['id'], link, a['size'], a['center']),
                    touch_links=list(a.get('touch_links', [link])),
                )
                scene.robot_state.attached_collision_objects.append(aco)
        self._future = self._cli.call_async(ApplyPlanningScene.Request(scene=scene))


def main():
    rclpy.init()
    node = PlanningSceneObstacles()
    try:
        rclpy.spin(node)  # stay alive so bringup OnProcessExit does not shut down
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
