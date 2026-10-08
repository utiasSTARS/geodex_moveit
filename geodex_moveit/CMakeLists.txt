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
project(geodex_moveit LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # Default to Release. An unoptimized build of the template-heavy planning code runs orders of
  # magnitude slower.
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
# geodex resolves OMPL to the fork installed in its own prefix, ahead of any stock OMPL on
# CMAKE_PREFIX_PATH.
find_package(geodex 1.0 CONFIG REQUIRED COMPONENTS ompl vamp robots)
find_package(ompl REQUIRED)
find_package(yaml-cpp REQUIRED)

get_target_property(_ompl_type ompl::ompl TYPE)
if(NOT _ompl_type STREQUAL "STATIC_LIBRARY")
  message(FATAL_ERROR
    "geodex_moveit links the OMPL fork statically, but ompl::ompl from ${ompl_DIR} is a "
    "${_ompl_type}. Put the fork's prefix (scripts/build_deps.sh) first on CMAKE_PREFIX_PATH.")
endif()

generate_parameter_library(geodex_moveit_parameters params/geodex_moveit_parameters.yaml)

# The backend holds every geodex call. It sees only geodex's Eigen, links the OMPL fork and
# exports only the plain-data API of include/geodex_moveit/backend.hpp.
add_library(geodex_moveit_backend SHARED
  src/backend/geodex_backend.cpp
  src/backend/validity.cpp)
target_compile_features(geodex_moveit_backend PUBLIC cxx_std_20)
target_include_directories(geodex_moveit_backend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(geodex_moveit_backend PRIVATE geodex::geodex ompl::ompl)
set_target_properties(geodex_moveit_backend PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON)
target_link_options(geodex_moveit_backend PRIVATE
  $<$<PLATFORM_ID:Linux>:LINKER:--exclude-libs,ALL>
  $<$<PLATFORM_ID:Linux>:LINKER:--no-undefined>)

# The plugin and the tests share this MoveIt-facing code. It does not include any geodex header.
add_library(geodex_moveit_core STATIC
  src/attached_box_test.cpp
  src/composite_validity.cpp
  src/environment_cache.cpp
  src/moveit_collision.cpp
  src/planner_manager.cpp
  src/planning_context.cpp
  src/request_setup.cpp
  src/scene_translation.cpp)
target_compile_features(geodex_moveit_core PUBLIC cxx_std_20)
set_target_properties(geodex_moveit_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON)
target_include_directories(geodex_moveit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(geodex_moveit_core PUBLIC
  geodex_moveit_backend
  geodex_moveit_parameters
  ${moveit_core_TARGETS}
  rclcpp::rclcpp)

# The plugin file keeps default visibility. pluginlib matches its factory by type.
add_library(geodex_moveit SHARED src/plugin.cpp)
if(APPLE)
  set_target_properties(geodex_moveit PROPERTIES INSTALL_RPATH "@loader_path")
else()
  set_target_properties(geodex_moveit PROPERTIES INSTALL_RPATH "$ORIGIN")
endif()
target_link_libraries(geodex_moveit PRIVATE geodex_moveit_core pluginlib::pluginlib)
target_link_options(geodex_moveit PRIVATE
  $<$<PLATFORM_ID:Linux>:LINKER:--exclude-libs,ALL>)

pluginlib_export_plugin_description_file(moveit_core plugin_description.xml)

install(TARGETS geodex_moveit geodex_moveit_backend geodex_moveit_parameters
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY config params DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../.clang-format")
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core pluginlib rclcpp)
ament_package()
