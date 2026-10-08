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
find_package(ament_cmake_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moveit_ros_planning REQUIRED)

set(_schema "${PROJECT_SOURCE_DIR}/params/geodex_moveit_parameters.yaml")
set(_example "${PROJECT_SOURCE_DIR}/config/geodex_moveit.example.yaml")
set(_pipeline "${PROJECT_SOURCE_DIR}/config/geodex_planning.yaml")

# Every test runs under its own ROS_DOMAIN_ID.
function(geodex_moveit_gtest name source)
  ament_add_ros_isolated_gtest(${name} ${source} ${ARGN} TIMEOUT 600)
  target_link_libraries(${name} geodex_moveit_core yaml-cpp::yaml-cpp)
  target_compile_definitions(${name} PRIVATE
    GEODEX_MOVEIT_SCHEMA="${_schema}"
    GEODEX_MOVEIT_EXAMPLE="${_example}"
    GEODEX_MOVEIT_PIPELINE_CONFIG="${_pipeline}")
endfunction()

# Unit tests cover the backend without ROS and the MoveIt-facing pieces on the moveit_resources
# Panda.
geodex_moveit_gtest(test_backend unit/test_backend.cpp)
geodex_moveit_gtest(test_attached_box unit/test_attached_box.cpp)
target_link_libraries(test_attached_box moveit_core::moveit_test_utils)
geodex_moveit_gtest(test_scene_translation unit/test_scene_translation.cpp)
target_link_libraries(test_scene_translation moveit_core::moveit_test_utils)

# Parameter tests check the declared set, the defaults, the validators and a value round trip
# through a parameter file.
geodex_moveit_gtest(test_parameters params/test_parameters.cpp integration/ros_main.cpp
  SKIP_LINKING_MAIN_LIBRARIES)
target_link_libraries(test_parameters gtest)
add_rostest_with_parameters_gtest(test_parameters_file params/test_parameters_file.cpp ${_example})
target_link_libraries(test_parameters_file geodex_moveit_core yaml-cpp::yaml-cpp)
target_compile_definitions(test_parameters_file PRIVATE GEODEX_MOVEIT_EXAMPLE="${_example}")

# Integration tests load the plugin through pluginlib and the planning pipeline, alone and next to
# MoveIt's own OMPL planner in one process.
geodex_moveit_gtest(test_planning integration/test_planning.cpp integration/ros_main.cpp
  SKIP_LINKING_MAIN_LIBRARIES)
target_link_libraries(test_planning gtest pluginlib::pluginlib ament_index_cpp::ament_index_cpp
  moveit_ros_planning::moveit_planning_pipeline moveit_ros_planning::moveit_robot_model_loader
  moveit_core::moveit_test_utils)
target_compile_definitions(test_planning PRIVATE
  GEODEX_MOVEIT_REPLAY_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/fixtures/fr3_shelf_replay")

find_package(moveit_planners_ompl QUIET)
if(moveit_planners_ompl_FOUND)
  geodex_moveit_gtest(test_ompl_coexistence integration/test_ompl_coexistence.cpp
    integration/ros_main.cpp SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_ompl_coexistence gtest pluginlib::pluginlib
    ament_index_cpp::ament_index_cpp moveit_ros_planning::moveit_planning_pipeline
    moveit_ros_planning::moveit_robot_model_loader)
  geodex_moveit_gtest(test_ompl_coexistence_geodex_first integration/test_ompl_coexistence.cpp
    integration/ros_main.cpp SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_ompl_coexistence_geodex_first gtest pluginlib::pluginlib
    ament_index_cpp::ament_index_cpp moveit_ros_planning::moveit_planning_pipeline
    moveit_ros_planning::moveit_robot_model_loader)
  target_compile_definitions(test_ompl_coexistence_geodex_first PRIVATE
    GEODEX_MOVEIT_LOAD_GEODEX_FIRST)
endif()

# Plan through the plugin's adapter chain with the Panda demo's parameters and check that
# ValidateSolution accepts every solution.
geodex_moveit_gtest(test_pipeline_validation integration/test_pipeline_validation.cpp
  integration/ros_main.cpp SKIP_LINKING_MAIN_LIBRARIES)
target_link_libraries(test_pipeline_validation gtest ament_index_cpp::ament_index_cpp
  moveit_ros_planning::moveit_planning_pipeline moveit_ros_planning::moveit_robot_model_loader)

# Run the backend's smoother on recorded FR3 shelf inputs and compare with the recorded outputs.
geodex_moveit_gtest(test_smoother_replay integration/test_smoother_replay.cpp)
target_compile_definitions(test_smoother_replay PRIVATE
  GEODEX_MOVEIT_REPLAY_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/fixtures/fr3_shelf_replay")

# Plan with the backend for the FR3 and its attached box at a shelf on the robot's right.
geodex_moveit_gtest(test_fr3_side_shelf integration/test_fr3_side_shelf.cpp)
target_compile_definitions(test_fr3_side_shelf PRIVATE
  GEODEX_MOVEIT_REPLAY_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/fixtures/fr3_shelf_replay"
  GEODEX_MOVEIT_SIDE_SHELF_SCENE="${CMAKE_CURRENT_SOURCE_DIR}/fixtures/fr3_side_shelf/scene.yaml")

# Plan for the FR3 and its attached box in the shelf through pluginlib.
geodex_moveit_gtest(test_fr3_shelf integration/test_fr3_shelf.cpp integration/ros_main.cpp
  SKIP_LINKING_MAIN_LIBRARIES)
target_link_libraries(test_fr3_shelf gtest pluginlib::pluginlib)
target_compile_definitions(test_fr3_shelf PRIVATE
  GEODEX_MOVEIT_REPLAY_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/fixtures/fr3_shelf_replay")

# Check that the plugin and the backend do not export any OMPL, Eigen or geodex symbol.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  add_test(NAME test_exported_symbols
    COMMAND ${CMAKE_COMMAND}
      -DLIBRARIES=$<TARGET_FILE:geodex_moveit>;$<TARGET_FILE:geodex_moveit_backend>
      -P ${CMAKE_CURRENT_SOURCE_DIR}/integration/check_exported_symbols.cmake)
endif()
