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

/// @file attached_box_test.hpp
/// @brief Exact oriented-box test of a rigidly attached box against the world's boxes.
///
/// The vamp checker checks the spheres of an attached box against every obstacle and against the
/// robot. This test checks the box itself, corners included, against every world box with the
/// 15-axis separating-axis theorem.

#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "geodex_moveit/scene_translation.hpp"
#include "geodex_moveit/validity.hpp"
#include "moveit/robot_model/joint_model_group.hpp"
#include "moveit/robot_state/robot_state.hpp"

namespace geodex_moveit
{

/// @brief Exact attached-box validity term for one planning group.
///
/// The test holds a scratch robot state and is not thread-safe. Build one per request.
class AttachedBoxTest : public ValidityOracle
{
public:
  /// @param state      Current state, fixing every variable outside the planning group.
  /// @param group      Planning group whose variables the configurations hold.
  /// @param box        Attached box and the link that holds it.
  /// @param obstacles  World boxes in the model frame.
  /// @param margin     Meters added to each half extent of the attached box.
  AttachedBoxTest(
    const moveit::core::RobotState & state, const moveit::core::JointModelGroup & group,
    const AttachedBox & box, std::vector<SceneBox> obstacles, double margin);

  bool isValid(const double * q) const override;
  int dimension() const override { return dimension_; }

  /// @brief Pose of the attached box in the model frame at configuration @p q.
  Eigen::Isometry3d boxPose(const double * q) const;

  /// @brief True when the test poses the box with the precomputed kinematic chain instead of the
  /// full state update.
  bool usesChainKinematics() const { return chain_valid_; }

  /// @brief Separating-axis test of two oriented boxes, true when they are disjoint.
  static bool disjoint(
    const Eigen::Vector3d & ca, const Eigen::Matrix3d & ra, const Eigen::Vector3d & ha,
    const Eigen::Vector3d & cb, const Eigen::Matrix3d & rb, const Eigen::Vector3d & hb);

private:
  /// One link of the chain from the root to the attach link. The chain follows RobotState's
  /// recursion global[link] = global[parent] * origin(link) * joint(parent joint).
  struct ChainNode
  {
    const moveit::core::JointModel * joint = nullptr;
    Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
    bool origin_identity = true;
    int q_index = -1;  ///< Group variable index, or -1 for a joint frozen for the request.
    Eigen::Isometry3d constant = Eigen::Isometry3d::Identity();
    bool constant_identity = true;
  };

  Eigen::Isometry3d linkPose(const double * q) const;

  mutable moveit::core::RobotState state_;
  const moveit::core::JointModelGroup * group_;
  const moveit::core::LinkModel * link_ = nullptr;
  Eigen::Isometry3d box_in_link_ = Eigen::Isometry3d::Identity();
  Eigen::Vector3d half_ = Eigen::Vector3d::Zero();
  struct Obb
  {
    Eigen::Vector3d center;
    Eigen::Vector3d half;
    Eigen::Matrix3d rot;
  };
  std::vector<Obb> obstacles_;
  std::vector<ChainNode> chain_;
  bool chain_valid_ = false;
  int dimension_ = 0;
};

}  // namespace geodex_moveit
