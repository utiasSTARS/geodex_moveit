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

/// @file planner_manager.hpp
/// @brief MoveIt planner plugin entry point.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "geodex_moveit/environment_cache.hpp"
#include "geodex_moveit/geodex_moveit_parameters.hpp"
#include "moveit/planning_interface/planning_interface.hpp"

namespace geodex_moveit
{

/// @brief Planner manager that serves the planning groups whose active joints are the joints of
/// the configured robot model, in the model's order. Without a robot model, it serves every group.
///
/// The manager reads parameters through generate_parameter_library under the pipeline's
/// namespace. Each request runs in its own planning context, built from the parameters current at
/// that request. Runtime parameter changes, the robot and the metric included, apply from the
/// next request on.
class GeodexPlannerManager : public planning_interface::PlannerManager
{
public:
  bool initialize(
    const moveit::core::RobotModelConstPtr & model, const rclcpp::Node::SharedPtr & node,
    const std::string & parameter_namespace) override;

  std::string getDescription() const override;

  void getPlanningAlgorithms(std::vector<std::string> & algs) const override;

  /// @brief Accepts requests without path or trajectory constraints, for a group that the manager
  /// serves.
  bool canServiceRequest(const planning_interface::MotionPlanRequest & req) const override;

  /// @brief Warns that planner settings come from this plugin's parameters only.
  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap & pcs) override;

  planning_interface::PlanningContextPtr getPlanningContext(
    const planning_scene::PlanningSceneConstPtr & planning_scene,
    const planning_interface::MotionPlanRequest & req,
    moveit_msgs::msg::MoveItErrorCodes & error_code) const override;

private:
  /// @brief Why @p req cannot be served, or an empty string when it can.
  std::string rejectReason(
    const planning_interface::MotionPlanRequest & req, const Params & params) const;

  moveit::core::RobotModelConstPtr model_;
  std::shared_ptr<ParamListener> param_listener_;
  std::shared_ptr<EnvironmentCache> cache_ = std::make_shared<EnvironmentCache>();
  std::shared_ptr<void> log_sink_;
};

}  // namespace geodex_moveit
