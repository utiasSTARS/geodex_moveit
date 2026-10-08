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

/// @file request_setup.hpp
/// @brief The settings and the validity oracle one request plans with.

#pragma once

#include <memory>
#include <string>

#include "geodex_moveit/backend.hpp"
#include "geodex_moveit/environment_cache.hpp"
#include "geodex_moveit/geodex_moveit_parameters.hpp"
#include "moveit/planning_scene/planning_scene.hpp"
#include "moveit/robot_state/robot_state.hpp"

namespace geodex_moveit
{

/// @brief The parameters that reach geodex, as the backend's plain settings.
PlannerSettings plannerSettings(const Params & params);

/// @brief The validity oracle of one request, or nullptr with @p message set.
///
/// The moveit checker uses the planning scene directly. The vamp checker translates the scene,
/// takes the collision environment from @p cache, and adds the exact attached-box test when an
/// attached box is known. With @p with_margin, the obstacle padding and the attached-box margin
/// grow by smoothingMargin(), the room of the smoother's evenly spaced waypoints.
std::shared_ptr<const ValidityOracle> buildValidity(
  const planning_scene::PlanningSceneConstPtr & scene, const moveit::core::RobotState & start,
  const moveit::core::JointModelGroup & group, const Params & params, EnvironmentCache & cache,
  std::string & message, bool with_margin = false);

}  // namespace geodex_moveit
