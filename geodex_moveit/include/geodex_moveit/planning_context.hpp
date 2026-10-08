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

/// @file planning_context.hpp
/// @brief One planning request for one planning group.

#pragma once

#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geodex_moveit/backend.hpp"
#include "geodex_moveit/geodex_moveit_parameters.hpp"
#include "moveit/planning_interface/planning_interface.hpp"

namespace geodex_moveit
{

class EnvironmentCache;

MOVEIT_CLASS_FORWARD(GeodexPlanningContext);

/// @brief Planning context that owns every piece of per-request state.
///
/// The planner manager creates a fresh context per request. Concurrent contexts share the
/// manager's environment cache, which is mutex-guarded and hands each request its own validity
/// oracle. terminate() may be called from another thread while solve() runs.
class GeodexPlanningContext : public planning_interface::PlanningContext
{
public:
  /// @param name     Context name.
  /// @param group    Planning group name.
  /// @param params   Parameter snapshot this request runs with.
  /// @param backend  Planner for the group's robot model and metric.
  /// @param cache    Collision environment cache shared by the manager's contexts.
  GeodexPlanningContext(
    const std::string & name, const std::string & group, Params params,
    std::shared_ptr<const PlannerBackend> backend, std::shared_ptr<EnvironmentCache> cache);

  void solve(planning_interface::MotionPlanResponse & res) override;
  void solve(planning_interface::MotionPlanDetailedResponse & res) override;

  /// @brief Ask a running solve() to stop. solve() returns the best exact solution found so far.
  bool terminate() override;

  /// @brief Drop the request and scene this context holds.
  void clear() override;

private:
  /// @brief Run the request, writing the trajectory and error code into @p res.
  void run(planning_interface::MotionPlanDetailedResponse & res);

  /// @brief Goal configuration of the request, or nullopt with @p error set.
  ///
  /// Joint goals are used as given. Other goals are sampled with MoveIt's constraint samplers for
  /// up to planner.ik_timeout seconds. Each solution is checked against @p validity and the bounds
  /// of @p plan, and the valid configuration nearest the start is returned. Several goal sets are
  /// alternatives.
  std::optional<std::vector<double>> goalConfiguration(
    const moveit::core::RobotState & start, const PlanRequest & plan,
    const ValidityOracle & validity, moveit_msgs::msg::MoveItErrorCodes & error,
    std::string & message) const;

  /// @brief Collision oracle for the request, or nullptr with @p message set. With
  /// @p with_margin, it keeps the margin of the smoother's evenly spaced waypoints.
  std::shared_ptr<const ValidityOracle> buildValidity(
    const moveit::core::RobotState & start, std::string & message, bool with_margin = false) const;

  Params params_;
  std::shared_ptr<const PlannerBackend> backend_;
  std::shared_ptr<EnvironmentCache> cache_;
  std::atomic_bool cancel_{false};
};

}  // namespace geodex_moveit
