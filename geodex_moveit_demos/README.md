# geodex_moveit_demos

Demos and robot configurations for the `geodex_moveit` planner plugin.

| file | purpose |
|---|---|
| `launch/fr3_shelf.launch.py` | the FR3 shelf demo (`mock_hardware:=true`, `rviz:=true`) |
| `launch/panda_demo.launch.py` | the Panda demo with the geodex and OMPL pipelines |
| `scripts/fr3_shelf_plan.py` | plans and executes the move of the box in the FR3 shelf demo |
| `scripts/attached_object_spheres.py` | turns foam's spheres into `attached_object.spheres` |
| `scripts/apply_scene.py`, `scripts/fixed_joint_states.py` | helpers of the FR3 shelf launch |
| `config/` | parameter files, the shelf scene, the controllers and the RViz configuration |
| `robots/fr3_arm_gripper/` | the FR3 with a Robotiq 2F-85 and its compartment configurations |

## The FR3 files

| file | source |
|---|---|
| `fr3_arm_gripper.urdf` | geodex `data/robots/fr3_gripper/fr3_arm_gripper_spherized.urdf` (sha256 a843fea8...) |
| `fr3_arm_gripper.srdf` | the `fr3_arm` group with the `disable_collisions` entries of geodex's SRDF |
| `visual_meshes.yaml` | the visual meshes of franka_description 2.9.0 (`7aeeddc`) and ros2_robotiq_gripper `a74d007` |
| `kinematics.yaml`, `fr3_joint_limits.yaml` | `franka_fr3_moveit_config/config`, without the finger joints |
| `shelf_endpoints.json` | one joint configuration per shelf compartment, with the box at mid-height |

The build downloads the visual meshes and installs them with their license files.
`-DGEODEX_MOVEIT_DEMOS_MESHES=OFF` builds without them, and RViz then draws the collision spheres.

The Franka description files are Apache-2.0 (franka_description), and the Robotiq 2F-85 files are
BSD-3-Clause (ros2_robotiq_gripper).
