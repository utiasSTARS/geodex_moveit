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

#include "geodex_moveit/planning_context.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <string>
#include <utility>

#include "geodex_moveit/environment_cache.hpp"
#include "geodex_moveit/group_state.hpp"
#include "geodex_moveit/request_setup.hpp"
#include "moveit/constraint_samplers/constraint_sampler_manager.hpp"
#include "moveit/kinematic_constraints/kinematic_constraint.hpp"
#include "moveit/robot_state/conversions.hpp"
#include "moveit/robot_trajectory/robot_trajectory.hpp"
#include "rclcpp/logging.hpp"

namespace geodex_moveit
{
namespace
{

using moveit_msgs::msg::MoveItErrorCodes;

rclcpp::Logger logger() { return rclcpp::get_logger("geodex_moveit"); }

bool withinBounds(
  const std::vector<double> & q, const std::vector<double> & lo, const std::vector<double> & hi)
{
  for (std::size_t i = 0; i < q.size(); ++i) {
    if (q[i] < lo[i] || q[i] > hi[i]) return false;
  }
  return true;
}

}  // namespace

GeodexPlanningContext::GeodexPlanningContext(
  const std::string & name, const std::string & group, Params params,
  std::shared_ptr<const PlannerBackend> backend, std::shared_ptr<EnvironmentCache> cache)
: planning_interface::PlanningContext(name, group),
  params_(std::move(params)),
  backend_(std::move(backend)),
  cache_(std::move(cache))
{
}

void GeodexPlanningContext::solve(planning_interface::MotionPlanResponse & res)
{
  planning_interface::MotionPlanDetailedResponse detailed;
  solve(detailed);
  res.error_code = detailed.error_code;
  res.planner_id = detailed.planner_id;
  if (!detailed.processing_time.empty()) res.planning_time = detailed.processing_time.front();
  if (!detailed.trajectory.empty()) res.trajectory = detailed.trajectory.front();
}

void GeodexPlanningContext::solve(planning_interface::MotionPlanDetailedResponse & res)
{
  try {
    run(res);
  } catch (const std::exception & e) {
    res.trajectory.clear();
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::CRASH;
    RCLCPP_ERROR(logger(), "planning failed with an exception: %s", e.what());
  }
}

bool GeodexPlanningContext::terminate()
{
  cancel_.store(true);
  return true;
}

void GeodexPlanningContext::clear()
{
  planning_scene_.reset();
  request_ = planning_interface::MotionPlanRequest();
}

std::optional<std::vector<double>> GeodexPlanningContext::goalConfiguration(
  const moveit::core::RobotState & start, const PlanRequest & plan, const ValidityOracle & validity,
  MoveItErrorCodes & error, std::string & message) const
{
  const auto & group = *start.getJointModelGroup(group_);
  const auto & model = *start.getRobotModel();
  if (request_.goal_constraints.empty()) {
    error.val = MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    message = "the request has no goal constraints";
    return std::nullopt;
  }
  // Keep the first refusal reason. It becomes the error when no goal set yields a configuration.
  int refused = 0;
  std::string refused_message;
  const auto admissible = [&](const std::vector<double> & q, bool record) {
    if (!withinBounds(q, plan.lower, plan.upper)) {
      if (record && refused == 0) {
        refused = MoveItErrorCodes::GOAL_STATE_INVALID;
        refused_message = "goal is outside the joint bounds";
      }
      return false;
    }
    if (!validity.isValid(q.data())) {
      if (record && refused == 0) {
        refused = MoveItErrorCodes::GOAL_IN_COLLISION;
        refused_message = "goal is in collision or within the smoothing margin of an obstacle";
      }
      return false;
    }
    return true;
  };

  // Goal sets are alternatives. Every valid configuration found is a candidate, and the planner
  // plans to the one nearest the start in joint space.
  std::vector<std::vector<double>> candidates;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                          std::chrono::duration<double>(params_.planner.ik_timeout));
  for (const auto & goal : request_.goal_constraints) {
    const bool joints_only = !goal.joint_constraints.empty() && goal.position_constraints.empty() &&
                             goal.orientation_constraints.empty() &&
                             goal.visibility_constraints.empty();
    if (joints_only) {
      moveit::core::RobotState state(start);
      const auto & names = model.getVariableNames();
      for (const auto & jc : goal.joint_constraints) {
        if (std::find(names.begin(), names.end(), jc.joint_name) == names.end()) {
          error.val = MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
          message = "goal names unknown joint '" + jc.joint_name + "'";
          return std::nullopt;
        }
        state.setVariablePosition(jc.joint_name, jc.position);
      }
      state.update();
      const auto q = configuration(state, group);
      if (admissible(q, true)) candidates.push_back(q);
      continue;
    }
    for (const auto & pc : goal.position_constraints) {
      if (!model.hasLinkModel(pc.link_name)) {
        error.val = MoveItErrorCodes::INVALID_LINK_NAME;
        message = "position goal names unknown link '" + pc.link_name + "'";
        return std::nullopt;
      }
    }
    for (const auto & oc : goal.orientation_constraints) {
      if (!model.hasLinkModel(oc.link_name)) {
        error.val = MoveItErrorCodes::INVALID_LINK_NAME;
        message = "orientation goal names unknown link '" + oc.link_name + "'";
        return std::nullopt;
      }
    }
    // MoveIt's goal sampler samples a pose from the constraint regions and solves inverse
    // kinematics from random seeds. The request's validity checks each solution.
    const auto sampler = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(
      planning_scene_, group_, goal);
    if (!sampler) {
      if (refused == 0) {
        refused = MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        refused_message = "no goal sampler for the goal constraints of group '" + group_ + "'";
      }
      continue;
    }
    kinematic_constraints::KinematicConstraintSet satisfied(planning_scene_->getRobotModel());
    satisfied.add(goal, planning_scene_->getTransforms());
    int solutions = 0;
    sampler->setGroupStateValidityCallback([&](
                                             moveit::core::RobotState * s,
                                             const moveit::core::JointModelGroup * g,
                                             const double * values) {
      s->setJointGroupPositions(g, values);
      s->update();
      ++solutions;
      return admissible(configuration(*s, group), false);
    });
    moveit::core::RobotState state(start);
    int found = 0;
    constexpr int kCandidatesPerSet = 4;
    while (found < kCandidatesPerSet && std::chrono::steady_clock::now() < deadline) {
      if (!sampler->sample(state, start, 1)) continue;
      state.update();
      if (!satisfied.decide(state).satisfied) continue;
      const auto q = configuration(state, group);
      if (admissible(q, false)) {
        candidates.push_back(q);
        ++found;
      }
    }
    if (found == 0 && refused == 0) {
      refused =
        solutions > 0 ? MoveItErrorCodes::GOAL_IN_COLLISION : MoveItErrorCodes::NO_IK_SOLUTION;
      refused_message = solutions > 0
                          ? "every inverse kinematics solution of the goal is in collision or "
                            "outside the joint bounds"
                          : "no inverse kinematics solution for the goal constraints";
    }
  }
  if (candidates.empty()) {
    error.val = refused != 0 ? refused : MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    message = refused_message.empty() ? "no goal configuration" : refused_message;
    return std::nullopt;
  }
  const auto distance = [&](const std::vector<double> & q) {
    double d2 = 0.0;
    for (std::size_t i = 0; i < q.size(); ++i)
      d2 += (q[i] - plan.start[i]) * (q[i] - plan.start[i]);
    return d2;
  };
  return *std::min_element(
    candidates.begin(), candidates.end(),
    [&](const auto & a, const auto & b) { return distance(a) < distance(b); });
}

std::shared_ptr<const ValidityOracle> GeodexPlanningContext::buildValidity(
  const moveit::core::RobotState & start, std::string & message, const bool with_margin) const
{
  return geodex_moveit::buildValidity(
    planning_scene_, start, *start.getJointModelGroup(group_), params_, *cache_, message,
    with_margin);
}

void GeodexPlanningContext::run(planning_interface::MotionPlanDetailedResponse & res)
{
  const auto wall0 = std::chrono::steady_clock::now();
  res.error_code.val = MoveItErrorCodes::FAILURE;
  res.planner_id = request_.planner_id.empty() ? plannerIds().front() : request_.planner_id;
  auto fail = [&](int code, const std::string & message) {
    res.error_code.val = code;
    if (res.processing_time.empty()) {
      res.processing_time.push_back(
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count());
    }
    RCLCPP_ERROR(logger(), "%s", message.c_str());
  };
  if (!planning_scene_) return fail(MoveItErrorCodes::FAILURE, "no planning scene");
  const auto * group = planning_scene_->getRobotModel()->getJointModelGroup(group_);
  if (group == nullptr)
    return fail(MoveItErrorCodes::INVALID_GROUP_NAME, "unknown group " + group_);

  moveit::core::RobotState start = planning_scene_->getCurrentState();
  const auto & msg = request_.start_state;
  const bool has_start =
    msg.is_diff || !msg.joint_state.name.empty() || !msg.multi_dof_joint_state.joint_names.empty();
  if (
    has_start &&
    !moveit::core::robotStateMsgToRobotState(planning_scene_->getTransforms(), msg, start)) {
    return fail(MoveItErrorCodes::INVALID_ROBOT_STATE, "invalid start state in the request");
  }
  start.update();

  PlanRequest plan;
  configurationBounds(*group, plan.lower, plan.upper);
  std::vector<double> model_lower, model_upper;
  if (
    !params_.robot.empty() && modelJointBounds(params_.robot, model_lower, model_upper) &&
    model_lower.size() == plan.lower.size()) {
    // Plan inside the group's bounds and the robot model's joint limits.
    for (std::size_t i = 0; i < plan.lower.size(); ++i) {
      plan.lower[i] = std::max(plan.lower[i], model_lower[i]);
      plan.upper[i] = std::min(plan.upper[i], model_upper[i]);
    }
  }
  plan.start = configuration(start, *group);
  if (!withinBounds(plan.start, plan.lower, plan.upper)) {
    return fail(
      MoveItErrorCodes::START_STATE_INVALID,
      "start state is outside the joint bounds of the group or the robot model");
  }

  std::string message;
  const auto validity = buildValidity(start, message);
  if (!validity) return fail(MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE, message);
  if (!validity->isValid(plan.start.data())) {
    return fail(MoveItErrorCodes::START_STATE_IN_COLLISION, "start state is in collision");
  }
  // The planner and the smoother keep a margin for the smoother's evenly spaced waypoints. The
  // returned states pass the request's own validity.
  const auto planning = buildValidity(start, message, true);
  if (!planning) return fail(MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE, message);
  if (!planning->isValid(plan.start.data())) {
    return fail(
      MoveItErrorCodes::START_STATE_IN_COLLISION,
      "start state lies within the smoothing margin of an obstacle; lower "
      "planner.collision_check_resolution for a smaller margin");
  }
  plan.planning_validity = planning.get();

  const auto goal = goalConfiguration(start, plan, *planning, res.error_code, message);
  if (!goal) return fail(res.error_code.val, message);
  plan.goal = *goal;

  plan.planner_id = res.planner_id;
  // PlanningContext::setMotionPlanRequest replaces a non-positive budget. The request's own
  // preparation, goal sampling included, counts against the budget.
  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
  plan.time_budget = std::max(0.0, request_.allowed_planning_time - elapsed);
  plan.cancel = &cancel_;
  const PlanResult result = backend_->plan(plan, *validity, plannerSettings(params_));

  switch (result.status) {
    case PlanResult::Status::kSolved:
      break;
    case PlanResult::Status::kNoExactSolution:
      res.processing_time.push_back(result.reported_time_s);
      return fail(MoveItErrorCodes::TIMED_OUT, result.message);
    case PlanResult::Status::kCanceled:
      res.processing_time.push_back(result.reported_time_s);
      return fail(MoveItErrorCodes::PREEMPTED, result.message);
    case PlanResult::Status::kInvalidInput:
    case PlanResult::Status::kInvalidPath:
      return fail(MoveItErrorCodes::PLANNING_FAILED, result.message);
  }

  auto trajectory =
    std::make_shared<robot_trajectory::RobotTrajectory>(planning_scene_->getRobotModel(), group);
  moveit::core::RobotState waypoint(start);
  for (const auto & q : result.path) {
    setConfiguration(waypoint, *group, q.data());
    // The pipeline's time-parameterization adapter assigns the timing.
    trajectory->addSuffixWayPoint(waypoint, 0.0);
  }
  res.trajectory.push_back(trajectory);
  res.description.emplace_back("geodex");
  res.processing_time.push_back(result.reported_time_s);
  res.error_code.val = MoveItErrorCodes::SUCCESS;

  RCLCPP_INFO(
    logger(),
    "%s solved %s in %.1f ms (search %.1f ms, %s iterations, first solution %.1f ms "
    "at %s iterations, smoothing %.1f ms, smoother stage %d), %zu waypoints",
    result.planner_id.c_str(), group_.c_str(), 1e3 * result.reported_time_s,
    1e3 * result.search_time_s, std::to_string(result.iterations).c_str(),
    1e3 * result.first_solution_time_s, std::to_string(result.first_solution_iterations).c_str(),
    1e3 * result.smooth_time_s, result.smoothing_stage, result.path.size());
  RCLCPP_DEBUG(
    logger(), "request wall clock %.1f ms",
    1e3 * std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count());
}

}  // namespace geodex_moveit
