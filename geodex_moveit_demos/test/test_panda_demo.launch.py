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
"""Smoke-test the headless Panda demo with geodex and OMPL plans executed on mock hardware."""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = [f'panda_joint{i}' for i in range(1, 8)]
GOAL = [1.0, 0.3, 0.0, -1.8, 0.0, 2.1, 0.785]
CONTROLLERS = {'joint_state_broadcaster', 'panda_arm_controller', 'panda_hand_controller'}


@pytest.mark.launch_test
def generate_test_description():
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('geodex_moveit_demos'), 'launch', 'panda_demo.launch.py')),
        launch_arguments={'rviz': 'false'}.items())
    return LaunchDescription([demo, launch_testing.actions.ReadyToTest()])


class TestPandaDemo(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('geodex_moveit_demo_smoke_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def request(self, pipeline):
        req = MotionPlanRequest()
        req.group_name = 'panda_arm'
        req.pipeline_id = pipeline
        # Plan from the current state move_group monitors.
        req.start_state.is_diff = True
        req.allowed_planning_time = 2.0
        req.max_velocity_scaling_factor = 1.0
        req.max_acceleration_scaling_factor = 1.0
        goal = Constraints()
        for name, value in zip(JOINTS, GOAL):
            goal.joint_constraints.append(JointConstraint(
                joint_name=name, position=value, tolerance_above=1e-3, tolerance_below=1e-3,
                weight=1.0))
        req.goal_constraints.append(goal)
        return GetMotionPlan.Request(motion_plan_request=req)

    def call(self, future, timeout):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done())
        return future.result()

    def wait_until_ready(self, timeout):
        """Wait until every Panda joint has a state and the three controllers are active."""
        seen = set()
        sub = self.node.create_subscription(
            JointState, '/joint_states', lambda msg: seen.update(msg.name), 10)
        lister = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.assertTrue(lister.wait_for_service(timeout_sec=timeout))
        deadline = time.monotonic() + timeout
        active = set()
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            future = lister.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            if future.done() and future.result() is not None:
                active = {c.name for c in future.result().controller if c.state == 'active'}
            if set(JOINTS) <= seen and CONTROLLERS <= active:
                break
            time.sleep(0.2)
        self.node.destroy_subscription(sub)
        self.assertLessEqual(set(JOINTS), seen, 'no joint state for every Panda joint')
        self.assertLessEqual(CONTROLLERS, active, 'controllers not active')

    def test_plan_with_both_pipelines_and_execute(self):
        plan = self.node.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.assertTrue(plan.wait_for_service(timeout_sec=120.0))
        self.wait_until_ready(60.0)
        responses = {}
        for pipeline in ('geodex', 'ompl'):
            res = self.call(plan.call_async(self.request(pipeline)), 60.0).motion_plan_response
            self.assertEqual(res.error_code.val, MoveItErrorCodes.SUCCESS, pipeline)
            self.assertGreater(len(res.trajectory.joint_trajectory.points), 1, pipeline)
            responses[pipeline] = res

        execute = ActionClient(self.node, ExecuteTrajectory, '/execute_trajectory')
        self.assertTrue(execute.wait_for_server(timeout_sec=60.0))
        goal = ExecuteTrajectory.Goal(trajectory=responses['geodex'].trajectory)
        handle = self.call(execute.send_goal_async(goal), 30.0)
        self.assertTrue(handle.accepted)
        result = self.call(handle.get_result_async(), 120.0).result
        self.assertEqual(result.error_code.val, MoveItErrorCodes.SUCCESS)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        # Accept clean exits and SIGINT or SIGTERM, and SIGSEGV from move_group at shutdown.
        for info in proc_info:
            codes = [0, -2, -15]
            if info.process_name.startswith('move_group'):
                codes.append(-11)
            self.assertIn(info.returncode, codes, info.process_name)
