// Copyright 2026 Space and Terrestrial Autonomous Robotic Systems (STARS) Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file scene_translation.hpp
/// @brief Translation of a MoveIt planning scene into primitive obstacles and attached objects.
///
/// This is MoveIt-facing code. It reads the scene with MoveIt's Eigen and hands the backend
/// plain arrays.

#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "geodex_moveit/backend.hpp"
#include "moveit/planning_scene/planning_scene.hpp"

namespace geodex_moveit
{

/// @brief Rigidly attached box, for the exact attached-object test.
struct AttachedBox
{
  std::string link;                                                ///< Link that holds the box.
  Eigen::Isometry3d pose_in_link = Eigen::Isometry3d::Identity();  ///< Box center pose.
  Eigen::Vector3d half_extents = Eigen::Vector3d::Zero();
};

/// @brief Inputs of the translation that come from parameters.
struct TranslationOptions
{
  std::string base_link;          ///< Frame of the collision backend, empty for the model frame.
  std::string end_effector_link;  ///< Link in which the attached-object spheres are expressed.
  /// Pose of the collision model's end-effector frame in end_effector_link.
  Eigen::Isometry3d end_effector_offset = Eigen::Isometry3d::Identity();
  double attachment_padding = 0.0;  ///< Meters added to every attached-object sphere radius.
  bool want_attached_box = true;    ///< Look for an attached box for the exact test.
  std::string
    attached_box_link;  ///< Link of the attached box and spheres when given by parameters.
  std::vector<double> attached_box_size;  ///< Empty, or the full box size along x, y, z.
  std::vector<double>
    attached_box_pose;  ///< Empty, or x, y, z, roll, pitch, yaw in the link frame.
  /// Empty, or x, y, z, radius per attached-object sphere in the link frame. The spheres replace
  /// the sphere primitives and the enclosing spheres of the attached bodies.
  std::vector<double> attached_spheres;
};

/// @brief Result of translating one planning scene.
struct TranslatedScene
{
  CollisionScene scene;
  /// World boxes, including mesh and cone bounds, in the model frame for the exact test.
  std::vector<SceneBox> model_frame_boxes;
  std::optional<AttachedBox> attached_box;
  /// One entry per shape the backend cannot represent, as "object:type".
  std::vector<std::string> unsupported;
  /// One entry per shape replaced by its bounding box, as "object:type".
  std::vector<std::string> bounded;
  /// Non-empty when the scene cannot be translated at all.
  std::string error;
};

/// @brief Translate the world of @p scene and the bodies attached in @p state.
///
/// @p state is the request's start state, with the objects attached in the scene and by the
/// request. Boxes, spheres and cylinders translate exactly. Meshes and cones become their oriented
/// bounding boxes, and planes and octrees are reported as unsupported. An attached object becomes
/// the spheres of the options, its own sphere primitives, or one sphere that encloses its shapes.
/// The largest attached box, or the box of the options, goes to the exact test.
TranslatedScene translateScene(
  const planning_scene::PlanningScene & scene, const moveit::core::RobotState & state,
  const moveit::core::JointModelGroup & group, const TranslationOptions & options);

}  // namespace geodex_moveit
