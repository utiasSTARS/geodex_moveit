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

/// @file moveit_collision.hpp
/// @brief Validity through the planning scene's own collision checker.

#pragma once

#include "geodex_moveit/validity.hpp"
#include "moveit/planning_scene/planning_scene.hpp"
#include "moveit/robot_state/robot_state.hpp"

namespace geodex_moveit
{

/// @brief Validity through the planning scene's collision checker, the robot against the world
/// and against itself, attached objects included.
///
/// A positive padding inflates every robot link by that amount in the check against the world.
/// The self-collision check uses the unpadded links. The oracle holds a scratch robot state and
/// is not thread-safe. Build one per request.
class MoveItCollisionOracle : public ValidityOracle
{
public:
  MoveItCollisionOracle(
    planning_scene::PlanningSceneConstPtr scene, const moveit::core::RobotState & state,
    const moveit::core::JointModelGroup & group, double padding = 0.0);

  bool isValid(const double * q) const override;
  int dimension() const override { return dimension_; }

private:
  planning_scene::PlanningSceneConstPtr scene_;
  planning_scene::PlanningScenePtr padded_;  ///< A diff of the scene with padded links, or null.
  const moveit::core::JointModelGroup * group_;
  mutable moveit::core::RobotState state_;
  int dimension_;
};

}  // namespace geodex_moveit
