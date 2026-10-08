#!/usr/bin/env python3
# Copyright 2026 Space and Terrestrial Autonomous Robotic Systems (STARS) Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Apply a scene file to move_group through /apply_planning_scene and exit.

The file lists world objects and attached objects with primitive shapes, in the format of
config/fr3_shelf_scene.yaml. The script exits after the service confirms that the scene is
applied.

    apply_scene.py --ros-args -p scene_file:=<path>
"""

import sys

from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
import yaml

TYPES = {
    'box': SolidPrimitive.BOX,
    'sphere': SolidPrimitive.SPHERE,
    'cylinder': SolidPrimitive.CYLINDER,
}


def to_pose(entry):
    """Build a Pose from a mapping with position and orientation lists."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = entry['position']
    (pose.orientation.x, pose.orientation.y, pose.orientation.z,
     pose.orientation.w) = entry['orientation']
    return pose


def to_object(entry):
    """Build a CollisionObject from a scene-file entry."""
    obj = CollisionObject()
    obj.id = entry['id']
    obj.header.frame_id = entry['frame_id']
    obj.pose = to_pose(entry['pose'])
    obj.operation = CollisionObject.ADD
    for prim in entry['primitives']:
        shape = SolidPrimitive()
        shape.type = TYPES[prim['type']]
        shape.dimensions = [float(d) for d in prim['dimensions']]
        obj.primitives.append(shape)
        obj.primitive_poses.append(to_pose(prim))
    return obj


def main():
    rclpy.init()
    node = Node('apply_scene')
    path = node.declare_parameter('scene_file', '').value
    if not path:
        node.get_logger().error('parameter scene_file is not set')
        sys.exit(1)
    with open(path) as fh:
        doc = yaml.safe_load(fh)

    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    for entry in doc.get('world', []):
        scene.world.collision_objects.append(to_object(entry))
    for entry in doc.get('attached', []):
        attached = AttachedCollisionObject()
        attached.link_name = entry['link']
        attached.touch_links = list(entry.get('touch_links', []))
        attached.object = to_object(entry)
        scene.robot_state.attached_collision_objects.append(attached)

    client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    if not client.wait_for_service(timeout_sec=60.0):
        node.get_logger().error('/apply_planning_scene did not appear')
        sys.exit(1)
    future = client.call_async(ApplyPlanningScene.Request(scene=scene))
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    if not future.done() or not future.result().success:
        node.get_logger().error('applying the scene failed')
        sys.exit(1)
    names = [o['id'] for o in doc.get('world', [])] + [o['id'] for o in doc.get('attached', [])]
    node.get_logger().info(f"scene applied: {', '.join(names)}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
