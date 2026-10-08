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
"""Panda with mock hardware, move_group serving the geodex and OMPL pipelines, and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

PANDA_CONFIG = get_package_share_directory('moveit_resources_panda_moveit_config')


def load(package, *path):
    """Load a YAML file from a package's share directory."""
    with open(os.path.join(get_package_share_directory(package), *path)) as fh:
        return yaml.safe_load(fh)


def generate_launch_description():
    """Launch the Panda demo with the geodex and OMPL pipelines."""
    moveit_config = (
        MoveItConfigsBuilder('moveit_resources_panda')
        .robot_description(file_path='config/panda.urdf.xacro',
                           mappings={'ros2_control_hardware_type': 'mock_components'})
        .robot_description_semantic(file_path='config/panda.srdf')
        .trajectory_execution(file_path='config/gripper_moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    # The geodex pipeline is the plugin's pipeline file plus the Panda's parameter file.
    geodex = load('geodex_moveit', 'config', 'geodex_planning.yaml')
    geodex.update(load('geodex_moveit_demos', 'config', 'panda_geodex.yaml'))
    pipelines = moveit_config.planning_pipelines
    pipelines['geodex'] = geodex
    pipelines['planning_pipelines'] = ['geodex', 'ompl']
    pipelines['default_planning_pipeline'] = 'geodex'

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description],
    )
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='log',
        arguments=['--frame-id', 'world', '--child-frame-id', 'panda_link0'],
    )
    ros2_control = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[os.path.join(PANDA_CONFIG, 'config', 'ros2_controllers.yaml')],
        remappings=[('/controller_manager/robot_description', '/robot_description')],
        output='screen',
    )
    # Pass the controller parameters through the spawner. Every ros2_control release reads them
    # there.
    spawners = [
        Node(package='controller_manager', executable='spawner',
             arguments=[name, '--controller-manager', '/controller_manager', '--param-file',
                        os.path.join(PANDA_CONFIG, 'config', 'ros2_controllers.yaml')])
        for name in ('joint_state_broadcaster', 'panda_arm_controller', 'panda_hand_controller')
    ]
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', os.path.join(PANDA_CONFIG, 'launch', 'moveit.rviz')],
        parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines, moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        robot_state_publisher, static_tf, ros2_control, *spawners, move_group, rviz,
    ])
