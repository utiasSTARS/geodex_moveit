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

/// @file group_state.hpp
/// @brief Conversion between robot states and planning-group configurations.
///
/// A configuration holds the group's active variables in the order of its active joint models.
/// Mimic and fixed joints are not part of it.

#pragma once

#include <cmath>
#include <string>
#include <vector>

#include "moveit/robot_model/joint_model_group.hpp"
#include "moveit/robot_state/robot_state.hpp"

namespace geodex_moveit
{

/// @brief Number of variables in a configuration of @p group.
inline int configurationSize(const moveit::core::JointModelGroup & group)
{
  return static_cast<int>(group.getActiveVariableCount());
}

/// @brief The configuration of @p group in @p state.
inline std::vector<double> configuration(
  const moveit::core::RobotState & state, const moveit::core::JointModelGroup & group)
{
  std::vector<double> q;
  q.reserve(group.getActiveVariableCount());
  for (const auto * joint : group.getActiveJointModels()) {
    const double * p = state.getJointPositions(joint);
    q.insert(q.end(), p, p + joint->getVariableCount());
  }
  return q;
}

/// @brief Write configuration @p q of @p group into @p state, mimic joints included.
inline void setConfiguration(
  moveit::core::RobotState & state, const moveit::core::JointModelGroup & group, const double * q)
{
  for (const auto * joint : group.getActiveJointModels()) {
    state.setJointPositions(joint, q);
    q += joint->getVariableCount();
  }
}

/// @brief Why @p group cannot be planned for, or an empty string.
///
/// A group with a planar or a floating joint, or with a variable without finite bounds, cannot be
/// planned for. Continuous joints are planned inside their bounds, [-pi, pi], without wrapping.
inline std::string unsupportedGroupReason(const moveit::core::JointModelGroup & group)
{
  for (const auto * joint : group.getActiveJointModels()) {
    const auto type = joint->getType();
    if (type == moveit::core::JointModel::PLANAR || type == moveit::core::JointModel::FLOATING) {
      return "joint '" + joint->getName() + "' of group '" + group.getName() + "' is " +
             (type == moveit::core::JointModel::PLANAR ? "planar" : "floating") +
             ", which needs a geodex model of its space";
    }
    const auto & bounds = joint->getVariableBounds();
    for (std::size_t k = 0; k < bounds.size(); ++k) {
      if (!std::isfinite(bounds[k].min_position_) || !std::isfinite(bounds[k].max_position_)) {
        return "variable '" + joint->getVariableNames()[k] + "' of group '" + group.getName() +
               "' has no finite position bounds";
      }
    }
  }
  return {};
}

/// @brief Per-variable position bounds of @p group, as lower and upper vectors.
inline void configurationBounds(
  const moveit::core::JointModelGroup & group, std::vector<double> & lower,
  std::vector<double> & upper)
{
  lower.clear();
  upper.clear();
  for (const auto * joint : group.getActiveJointModels()) {
    for (const auto & b : joint->getVariableBounds()) {
      lower.push_back(b.min_position_);
      upper.push_back(b.max_position_);
    }
  }
}

}  // namespace geodex_moveit
