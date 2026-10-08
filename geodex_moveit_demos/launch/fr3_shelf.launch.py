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
Start move_group for the FR3 shelf demo, headless by default.

The shelf and the attached box come from config/fr3_shelf_scene.yaml. The configs argument takes
comma-separated geodex_moveit parameter files, and later files override earlier ones.
mock_hardware:=true runs the arm on ros2_control mock components, and rviz:=true starts RViz.

    ros2 launch geodex_moveit_demos fr3_shelf.launch.py mock_hardware:=true rviz:=true
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

SHARE = get_package_share_directory('geodex_moveit_demos')
ROBOT = os.path.join(SHARE, 'robots', 'fr3_arm_gripper')


def load(path):
    """Load a YAML file."""
    with open(path) as fh:
        return yaml.safe_load(fh)


def merge(base, overlay):
    """Merge nested mappings, with the values of overlay taking precedence."""
    out = dict(base)
    for key, value in overlay.items():
        out[key] = merge(out[key], value) if isinstance(value, dict) and isinstance(
            out.get(key), dict) else value
    return out


def ros2_control(joints):
    """Return a ros2_control tag that runs the joints on mock components from their positions."""
    tags = ''.join(
        f'<joint name="{name}"><command_interface name="position"/>'
        f'<state_interface name="position"><param name="initial_value">{value!r}</param>'
        '</state_interface><state_interface name="velocity"/></joint>'
        for name, value in joints.items())
    return ('<ros2_control name="fr3_mock" type="system"><hardware>'
            f'<plugin>mock_components/GenericSystem</plugin></hardware>{tags}</ros2_control>')


def sphere_visuals(urdf):
    """Return the URDF with a light gray visual copy of every collision shape, for RViz."""
    return re.sub(r'<collision>(.*?)</collision>',
                  r'<collision>\1</collision><visual>\1<material name="fr3_spheres">'
                  r'<color rgba="0.8 0.8 0.8 1"/></material></visual>', urdf, flags=re.S)


def mesh_visuals(urdf):
    """Return the URDF with the visual mesh of every link, or with sphere visuals without them."""
    meshes = load(os.path.join(ROBOT, 'visual_meshes.yaml'))
    if not all(os.path.exists(os.path.join(SHARE, 'meshes', f)) for f in meshes.values()):
        return sphere_visuals(urdf)
    for link, mesh in meshes.items():
        visual = ('<visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename='
                  f'"package://geodex_moveit_demos/meshes/{mesh}"/></geometry></visual>')
        urdf, count = re.subn(rf'(<link name="{link}">)', rf'\1{visual}', urdf)
        if count != 1:
            raise RuntimeError(f'link {link} of visual_meshes.yaml is not in the URDF')
    return urdf


def setup(context):
    """Build the nodes once the launch arguments are known."""
    geodex = load(os.path.join(get_package_share_directory('geodex_moveit'), 'config',
                               'geodex_planning.yaml'))
    for path in LaunchConfiguration('configs').perform(context).split(','):
        geodex = merge(geodex, load(path.strip()))
    joint_state_file = os.path.join(SHARE, 'config', 'fr3_shelf_joint_state.yaml')
    mock = LaunchConfiguration('mock_hardware').perform(context) == 'true'
    with open(os.path.join(ROBOT, 'fr3_arm_gripper.urdf')) as fh:
        urdf = fh.read()
    if mock:
        body, end = urdf.rsplit('</robot>', 1)
        urdf = body + ros2_control(load(joint_state_file)['joints']) + '</robot>' + end
    with open(os.path.join(ROBOT, 'fr3_arm_gripper.srdf')) as fh:
        srdf = fh.read()
    description = [
        {'robot_description': urdf, 'robot_description_semantic': srdf},
        {'robot_description_kinematics': load(os.path.join(ROBOT, 'kinematics.yaml'))},
        {'robot_description_planning': load(os.path.join(ROBOT, 'fr3_joint_limits.yaml'))},
    ]
    move_group_params = description + [
        {'geodex': geodex, 'planning_pipelines': ['geodex'],
         'default_planning_pipeline': 'geodex'},
        {'publish_planning_scene': True, 'publish_geometry_updates': True,
         'publish_state_updates': True, 'publish_transforms_updates': True},
    ]
    controllers = os.path.join(SHARE, 'config', 'fr3_shelf_ros2_controllers.yaml')
    if mock:
        move_group_params.append(load(os.path.join(SHARE, 'config',
                                                   'fr3_shelf_moveit_controllers.yaml')))
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': urdf}]),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=move_group_params),
    ]
    if mock:
        nodes.append(Node(package='controller_manager', executable='ros2_control_node',
                          output='screen', parameters=[controllers],
                          remappings=[('/controller_manager/robot_description',
                                       '/robot_description')]))
        nodes += [Node(package='controller_manager', executable='spawner',
                       arguments=[name, '--controller-manager', '/controller_manager',
                                  '--param-file', controllers])
                  for name in ('joint_state_broadcaster', 'fr3_arm_controller')]
    else:
        nodes.append(Node(package='geodex_moveit_demos', executable='fixed_joint_states.py',
                          output='screen', parameters=[{'joint_state_file': joint_state_file}]))
    nodes.append(Node(package='geodex_moveit_demos', executable='apply_scene.py', output='screen',
                      parameters=[{'scene_file': os.path.join(SHARE, 'config',
                                                              'fr3_shelf_scene.yaml')}]))
    if LaunchConfiguration('rviz').perform(context) == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', output='log',
                          arguments=['-d', os.path.join(SHARE, 'config', 'fr3_shelf.rviz')],
                          parameters=[{**description[0],
                                       'robot_description': mesh_visuals(urdf)},
                                      *description[1:]]))
    return nodes


def generate_launch_description():
    """Launch the FR3 shelf demo."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'configs', default_value=os.path.join(SHARE, 'config', 'fr3_shelf_geodex.yaml'),
            description='Comma-separated geodex_moveit parameter files'),
        DeclareLaunchArgument('mock_hardware', default_value='false',
                              description='Run the arm on ros2_control mock components'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz'),
        OpaqueFunction(function=setup),
    ])
