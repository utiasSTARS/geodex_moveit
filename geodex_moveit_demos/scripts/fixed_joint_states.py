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
Publish one fixed joint state, for headless planning without ros2_control.

Usage: fixed_joint_states.py --ros-args -p joint_state_file:=<path> [-p rate:=30.0]
"""

from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml


def main():
    rclpy.init()
    node = Node('fixed_joint_states')
    path = node.declare_parameter('joint_state_file', '').value
    rate = node.declare_parameter('rate', 30.0).value
    joints = yaml.safe_load(Path(path).read_text())['joints']
    publisher = node.create_publisher(JointState, '/joint_states', 10)

    def tick():
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = list(joints)
        msg.position = [float(v) for v in joints.values()]
        publisher.publish(msg)

    node.create_timer(1.0 / rate, tick)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # A shutdown that races the executor's wait surfaces as an RCL error. Re-raise every other
        # error.
        if rclpy.ok():
            raise
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
