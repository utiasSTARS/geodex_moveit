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
Plan and execute the FR3 shelf move through the geodex pipeline of MoveIt.

The script runs against fr3_shelf.launch.py on mock hardware. It sets the metric, the demo's
G-RRT* range for that metric, a seed and an iteration budget, plans the move of the box from the
middle compartment of the shelf to the top compartment, and executes it.

    ros2 run geodex_moveit_demos fr3_shelf_plan.py [--metric kinetic_energy|euclidean]
"""

import argparse
import json
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (Constraints, DisplayTrajectory, JointConstraint, MotionPlanRequest,
                             MoveItErrorCodes, RobotState)
from moveit_msgs.srv import GetMotionPlan
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

ENDPOINTS = json.loads((Path(get_package_share_directory('geodex_moveit_demos')) / 'robots'
                        / 'fr3_arm_gripper' / 'shelf_endpoints.json').read_text())
JOINTS = ENDPOINTS['joint_order']
START = ENDPOINTS['configurations']['middle']
GOAL = ENDPOINTS['configurations']['top']
# The G-RRT* range of the demo for each metric.
RANGE = {'kinetic_energy': 0.6, 'euclidean': 1.2}
PIPELINE_PARAMETERS = {'geodex.planner.seed': 1, 'geodex.planner.max_iterations': 1500}


class Client:
    """Service and action clients of move_group."""

    def __init__(self):
        self.node = rclpy.create_node('geodex_moveit_shelf_client')
        self.plan_srv = self.node.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.param_srv = self.node.create_client(SetParameters, '/move_group/set_parameters')
        self.execute = ActionClient(self.node, ExecuteTrajectory, '/execute_trajectory')
        self.display = self.node.create_publisher(DisplayTrajectory, '/display_planned_path', 1)
        self.current = None
        self.node.create_subscription(JointState, '/joint_states', self._joint_state, 10)

    def _joint_state(self, msg):
        if set(JOINTS) <= set(msg.name):
            position = dict(zip(msg.name, msg.position))
            self.current = [position[j] for j in JOINTS]

    def call(self, future, timeout):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('no answer in time')
        return future.result()

    def wait(self):
        for client in (self.plan_srv, self.param_srv):
            if not client.wait_for_service(timeout_sec=120.0):
                raise RuntimeError(f'{client.srv_name} is not available')
        if not self.execute.wait_for_server(timeout_sec=60.0):
            raise RuntimeError('/execute_trajectory is not available')
        time.sleep(5.0)  # the controller spawners and the scene finish after move_group starts
        while self.current is None:
            rclpy.spin_once(self.node, timeout_sec=1.0)

    def set_parameters(self, values):
        def value(v):
            if isinstance(v, str):
                return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=v)
            if isinstance(v, float):
                return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=v)
            return ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=v)

        request = SetParameters.Request(parameters=[
            Parameter(name=name, value=value(v)) for name, v in values.items()])
        for result in self.call(self.param_srv.call_async(request), 10.0).results:
            if not result.successful:
                raise RuntimeError(f'setting a pipeline parameter failed: {result.reason}')

    def plan(self, goal, start=None):
        """Plan to `goal`, from `start` when given and from the current state otherwise."""
        request = MotionPlanRequest(group_name='fr3_arm', pipeline_id='geodex',
                                    planner_id='GreedyRRTstar', num_planning_attempts=1,
                                    allowed_planning_time=30.0,
                                    max_velocity_scaling_factor=1.0,
                                    max_acceleration_scaling_factor=1.0)
        request.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=v, tolerance_above=1e-3,
                            tolerance_below=1e-3, weight=1.0) for j, v in zip(JOINTS, goal)])]
        if start is not None:
            request.start_state = RobotState(
                joint_state=JointState(name=JOINTS, position=list(start)), is_diff=True)
        response = self.call(self.plan_srv.call_async(
            GetMotionPlan.Request(motion_plan_request=request)), 60.0).motion_plan_response
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'planning failed with error code {response.error_code.val}')
        self.display.publish(DisplayTrajectory(trajectory_start=response.trajectory_start,
                                               trajectory=[response.trajectory]))
        return response

    def run(self, trajectory):
        handle = self.call(self.execute.send_goal_async(
            ExecuteTrajectory.Goal(trajectory=trajectory)), 30.0)
        result = self.call(handle.get_result_async(), 120.0).result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'execution failed with error code {result.error_code.val}')


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--metric', default='kinetic_energy',
                        choices=['kinetic_energy', 'euclidean'])
    args = parser.parse_args()

    rclpy.init()
    client = Client()
    try:
        client.wait()
        client.set_parameters({'geodex.metric': args.metric,
                               'geodex.planner.config.grrtstar.range': RANGE[args.metric],
                               **PIPELINE_PARAMETERS})
        if max(abs(a - b) for a, b in zip(client.current, START)) > 1e-3:
            client.run(client.plan(START).trajectory)
        response = client.plan(GOAL, start=START)
        trajectory = response.trajectory.joint_trajectory
        last = trajectory.points[-1].time_from_start
        print(f'{args.metric}: planning_time={1000 * response.planning_time:.0f} ms '
              f'points={len(trajectory.points)} '
              f'duration={last.sec + last.nanosec * 1e-9:.3f} s')
        client.run(response.trajectory)
    finally:
        client.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
