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
cmake_minimum_required(VERSION 3.22)
project(geodex_moveit_demos NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch robots DESTINATION share/${PROJECT_NAME})

# RViz draws the FR3 and its Robotiq 2F-85 with their visual meshes, which the build downloads from
# the upstream descriptions at the commits geodex pins. Each entry is a source path, the installed
# path under share/geodex_moveit_demos/meshes and the file's sha256. Without the meshes the FR3
# launch draws the collision spheres instead.
option(GEODEX_MOVEIT_DEMOS_MESHES "Download the FR3 and 2F-85 visual meshes for RViz" ON)
set(FRANKA_DESCRIPTION_RAW
  "https://raw.githubusercontent.com/frankarobotics/franka_description/7aeeddc449edf8d62b594f9e36a81da53e7796f9")
set(ROBOTIQ_DESCRIPTION_RAW
  "https://raw.githubusercontent.com/PickNikRobotics/ros2_robotiq_gripper/a74d007d8f2f06dc6a503ad21038ba869d4999a6")
set(FR3_VISUAL "${FRANKA_DESCRIPTION_RAW}/meshes/robots/fr3/visual")
set(ROBOTIQ_VISUAL "${ROBOTIQ_DESCRIPTION_RAW}/robotiq_description/meshes/visual/2f_85")
set(VISUAL_MESHES
  "${FR3_VISUAL}/link0.dae|fr3/link0.dae|3c52a112d1610d645934945b80ea8d9bb3d1635f345098480a33a537bd43224e"
  "${FR3_VISUAL}/link1.dae|fr3/link1.dae|5c5846eabc714aefc82ef051a4d09c4fc1c092bca6258b1290227345dd217fcb"
  "${FR3_VISUAL}/link2.dae|fr3/link2.dae|90a1bd38abdc53d0141719cd8e040f788db337b768a845bb6d53c5c52590498f"
  "${FR3_VISUAL}/link3.dae|fr3/link3.dae|1c7a2a8124e39ad07096c0cce8461b0e650f6752900cfd777bf87c181e81749b"
  "${FR3_VISUAL}/link4.dae|fr3/link4.dae|dd134cf13cdaced2b67739e4d9fe2ca011cc533936397d9f6303fc740424e2a2"
  "${FR3_VISUAL}/link5.dae|fr3/link5.dae|cbcabd369e044eba3f1c2cab84b9ed2ebc1b2f424c6104ad9c5d7410e92fabfe"
  "${FR3_VISUAL}/link6.dae|fr3/link6.dae|907b16fea84931f4881174234065bb647700abf105b432c3ff65360e0ad5cf8b"
  "${FR3_VISUAL}/link7.dae|fr3/link7.dae|b9d05ac502ff269b1323705de811d07a8eb9920ce711b32ee0497bf16b74989b"
  "${ROBOTIQ_VISUAL}/ur_to_robotiq_adapter.dae|robotiq_2f_85/ur_to_robotiq_adapter.dae|ed326c745df2942d2efb8dd902ecfafa84c692d360a7c03ce9276ad491614ac4"
  "${ROBOTIQ_VISUAL}/robotiq_base.dae|robotiq_2f_85/robotiq_base.dae|761a79d586a48d367d60f146ca77ef7d2821ad790f7b1b1c93b061ac59799267"
  "${ROBOTIQ_VISUAL}/left_knuckle.dae|robotiq_2f_85/left_knuckle.dae|1b82214363d9548f3562be70e25cd23217a304c72ccd37bc1551cf1a3a64029d"
  "${ROBOTIQ_VISUAL}/right_knuckle.dae|robotiq_2f_85/right_knuckle.dae|532b6f6f8794883ff34995701036ca392461f26127757f92003db127693bc57c"
  "${ROBOTIQ_VISUAL}/left_finger.dae|robotiq_2f_85/left_finger.dae|5fb9e5659c735e48eb55269da431cf61b3b7a6745922c1932a7adefb1db86dc3"
  "${ROBOTIQ_VISUAL}/right_finger.dae|robotiq_2f_85/right_finger.dae|3b2245b5537eb596d7f8a4c64b12e3aedea46e365fc589019184d3b4200e664f"
  "${ROBOTIQ_VISUAL}/left_inner_knuckle.dae|robotiq_2f_85/left_inner_knuckle.dae|2df009ef28868c90a4904370f5f4995c74ec84e0b8ca1371044e0ea669edcd71"
  "${ROBOTIQ_VISUAL}/right_inner_knuckle.dae|robotiq_2f_85/right_inner_knuckle.dae|4cc6de99e177773ba755805b58b67537c3af24b2fd8f3a1bf4bbced87d6240a5"
  "${ROBOTIQ_VISUAL}/left_finger_tip.dae|robotiq_2f_85/left_finger_tip.dae|785ec7a83b4fc6357c211a39bb117833b1cfd4e9bccbef2efc7f35aeb2fdb92e"
  "${ROBOTIQ_VISUAL}/right_finger_tip.dae|robotiq_2f_85/right_finger_tip.dae|568371d25ddbcee8e432c0610bdcc7afcd1393d48d973eed92168e353eb4edf5")
if(GEODEX_MOVEIT_DEMOS_MESHES)
  set(MESH_DIR "${CMAKE_CURRENT_BINARY_DIR}/meshes")
  foreach(entry IN LISTS VISUAL_MESHES)
    string(REPLACE "|" ";" fields "${entry}")
    list(GET fields 0 url)
    list(GET fields 1 path)
    list(GET fields 2 sha256)
    set(file "${MESH_DIR}/${path}")
    if(EXISTS "${file}")
      file(SHA256 "${file}" have)
    else()
      set(have "")
    endif()
    if(NOT have STREQUAL sha256)
      file(DOWNLOAD "${url}" "${file}" EXPECTED_HASH SHA256=${sha256} STATUS status)
      list(GET status 0 code)
      if(NOT code EQUAL 0)
        message(FATAL_ERROR "Downloading ${url} failed (${status}). Configure with "
                            "-DGEODEX_MOVEIT_DEMOS_MESHES=OFF to build without the meshes.")
      endif()
    endif()
  endforeach()
  install(DIRECTORY "${MESH_DIR}/" DESTINATION share/${PROJECT_NAME}/meshes)
  install(FILES robots/fr3_arm_gripper/LICENSE.franka_description
                robots/fr3_arm_gripper/LICENSE.ros2_robotiq_gripper
          DESTINATION share/${PROJECT_NAME}/meshes)
endif()
install(PROGRAMS scripts/apply_scene.py scripts/fixed_joint_states.py scripts/fr3_shelf_plan.py
  scripts/attached_object_spheres.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # Turn off pytest plugin autoload for this test. launch_testing's pytest plugin does not load
  # under the environment's pytest, and this test does not use any plugin.
  ament_add_pytest_test(test_configs test/test_configs.py ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  ament_add_pytest_test(test_attached_object_spheres test/test_attached_object_spheres.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_panda_demo.launch.py
    RUNNER ${ament_cmake_ros_DIR}/run_test_isolated.py
    TIMEOUT 300)
endif()

ament_package()
