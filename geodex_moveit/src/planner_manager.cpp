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

#include "geodex_moveit/planner_manager.hpp"

#include <algorithm>
#include <exception>

#include "geodex_moveit/backend.hpp"
#include "geodex_moveit/group_state.hpp"
#include "geodex_moveit/planning_context.hpp"
#include "rclcpp/logging.hpp"

namespace geodex_moveit
{
namespace
{

using moveit_msgs::msg::MoveItErrorCodes;

rclcpp::Logger logger() { return rclcpp::get_logger("geodex_moveit"); }

/// Configuration errors that no request can work around.
std::string configurationError(const Params & p)
{
  if (!p.robot.empty() && !isRegisteredRobot(p.robot)) {
    std::string known;
    for (const auto & r : registeredRobots()) known += (known.empty() ? "" : ", ") + r;
    return "'" + p.robot + "' is not a geodex robot model; known models: " + known;
  }
  if (hasMobileBase(p.robot)) {
    return "'" + p.robot + "' has a mobile base, and this plugin plans fixed-base arms only";
  }
  if (!p.attached_object.box_size.empty() && p.attached_object.box_size.size() != 3) {
    return "attached_object.box_size takes 3 values or none";
  }
  if (!p.attached_object.box_pose.empty() && p.attached_object.box_pose.size() != 6) {
    return "attached_object.box_pose takes 6 values or none";
  }
  if (p.metric == "kinetic_energy" && !hasMassMatrix(p.robot)) {
    return "metric 'kinetic_energy' needs a geodex mass matrix; set 'robot' to a model that has "
           "one";
  }
  if (p.checker == "vamp" && !hasCollisionModel(p.robot)) {
    return "checker 'vamp' needs a geodex collision model; set 'robot' or use checker 'moveit'";
  }
  return {};
}

std::string joined(const std::vector<std::string> & v)
{
  std::string out;
  for (const auto & s : v) out += (out.empty() ? "" : ", ") + s;
  return out;
}

/// Why the vamp checker's end-effector settings do not fit the robot model, or empty.
std::string endEffectorError(const Params & p, const moveit::core::RobotModel & model)
{
  if (p.checker != "vamp" || p.robot.empty()) return {};
  const auto & offset = p.end_effector_offset;
  if (!offset.empty() && offset.size() != 6) return "end_effector_offset takes 6 values or none";
  const std::string frame = modelEndEffectorFrame(p.robot);
  const std::string & link = p.end_effector_link;
  if (link.empty()) {
    return offset.empty() ? std::string() : "end_effector_offset needs end_effector_link";
  }
  if (!model.hasLinkModel(link)) {
    return "end_effector_link '" + link + "' is not in the robot model";
  }
  const bool zero = std::all_of(offset.begin(), offset.end(), [](double v) { return v == 0.0; });
  if (link == frame && !zero) {
    return "end_effector_link is the collision model's end-effector frame '" + frame +
           "', so end_effector_offset must be empty";
  }
  if (link != frame && offset.empty()) {
    return "end_effector_link '" + link + "' is not the collision model's end-effector frame '" +
           frame + "'; set end_effector_offset to the pose of '" + frame + "' in '" + link +
           "', or leave end_effector_link empty if the robot model has '" + frame + "'";
  }
  return {};
}

/// Why the geodex robot model cannot plan for @p group, or empty.
std::string modelMismatch(const Params & p, const moveit::core::JointModelGroup & group)
{
  if (p.robot.empty()) return {};
  const auto names = modelJointNames(p.robot);
  if (!names.empty()) {
    const auto & active = group.getActiveJointModelNames();
    if (active == names) return {};
    return "planning group '" + group.getName() + "' has the joints " + joined(active) +
           ", and robot model '" + p.robot + "' plans " + joined(names) + ", in that order";
  }
  const int nq = robotDimension(p.robot);
  const int n = configurationSize(group);
  if (n == nq) return {};
  return "planning group '" + group.getName() + "' has " + std::to_string(n) +
         " active joints, robot model '" + p.robot + "' has " + std::to_string(nq);
}

}  // namespace

bool GeodexPlannerManager::initialize(
  const moveit::core::RobotModelConstPtr & model, const rclcpp::Node::SharedPtr & node,
  const std::string & parameter_namespace)
{
  model_ = model;
  log_sink_ = addLogSink([](int level, const std::string & message) {
    const auto l = rclcpp::get_logger("geodex_moveit.ompl");
    if (level >= 3) {
      RCLCPP_ERROR(l, "%s", message.c_str());
    } else if (level == 2) {
      RCLCPP_WARN(l, "%s", message.c_str());
    } else {
      RCLCPP_DEBUG(l, "%s", message.c_str());
    }
  });
  try {
    param_listener_ = std::make_shared<ParamListener>(node, parameter_namespace);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger(), "invalid parameters under '%s': %s", parameter_namespace.c_str(), e.what());
    return false;
  }
  const Params params = param_listener_->get_params();
  std::string err = configurationError(params);
  if (err.empty()) err = endEffectorError(params, *model_);
  if (!err.empty()) {
    RCLCPP_ERROR(logger(), "%s", err.c_str());
    return false;
  }

  std::vector<std::string> served;
  std::string refused;
  for (const auto * group : model_->getJointModelGroups()) {
    if (configurationSize(*group) == 0 || !modelMismatch(params, *group).empty()) continue;
    if (const std::string reason = unsupportedGroupReason(*group); !reason.empty()) {
      refused += (refused.empty() ? "" : "; ") + reason;
      continue;
    }
    served.push_back(group->getName());
  }
  if (served.empty()) {
    const std::string joints =
      params.robot.empty() ? std::string("any joints") : joined(modelJointNames(params.robot));
    RCLCPP_ERROR(
      logger(), "no planning group of the robot model plans %s for robot model '%s'%s%s",
      joints.c_str(), params.robot.c_str(), refused.empty() ? "" : ": ", refused.c_str());
    return false;
  }
  std::string list;
  for (const auto & g : served) list += (list.empty() ? "" : ", ") + g;
  RCLCPP_INFO(
    logger(), "robot '%s', metric '%s', checker '%s', obstacle padding %g m, serving groups: %s",
    params.robot.c_str(), params.metric.c_str(), params.checker.c_str(),
    params.checker == "vamp" ? params.obstacle_padding : 0.0, list.c_str());
  return true;
}

std::string GeodexPlannerManager::getDescription() const { return "geodex"; }

void GeodexPlannerManager::getPlanningAlgorithms(std::vector<std::string> & algs) const
{
  algs = plannerIds();
}

void GeodexPlannerManager::setPlannerConfigurations(
  const planning_interface::PlannerConfigurationMap & pcs)
{
  if (!pcs.empty()) {
    RCLCPP_WARN(
      logger(),
      "ignoring %zu planner configurations; geodex_moveit reads its "
      "settings from its own parameters",
      pcs.size());
  }
  planning_interface::PlannerManager::setPlannerConfigurations(pcs);
}

std::string GeodexPlannerManager::rejectReason(
  const planning_interface::MotionPlanRequest & req, const Params & params) const
{
  const auto & pc = req.path_constraints;
  if (
    !pc.joint_constraints.empty() || !pc.position_constraints.empty() ||
    !pc.orientation_constraints.empty() || !pc.visibility_constraints.empty()) {
    return "path constraints are not supported";
  }
  if (!req.trajectory_constraints.constraints.empty()) {
    return "trajectory constraints are not supported";
  }
  if (!req.reference_trajectories.empty()) return "reference trajectories are not supported";
  if (req.group_name.empty()) return "no planning group given";
  const auto * group = model_->getJointModelGroup(req.group_name);
  if (group == nullptr) return "unknown planning group '" + req.group_name + "'";
  const auto & ids = plannerIds();
  if (!req.planner_id.empty() && std::find(ids.begin(), ids.end(), req.planner_id) == ids.end()) {
    return "unknown planner_id '" + req.planner_id + "'";
  }
  if (const std::string err = configurationError(params); !err.empty()) {
    return err;
  }
  if (const std::string err = endEffectorError(params, *model_); !err.empty()) {
    return err;
  }
  if (configurationSize(*group) == 0) {
    return "planning group '" + req.group_name + "' has no active joints";
  }
  if (std::string reason = unsupportedGroupReason(*group); !reason.empty()) {
    return reason;
  }
  return modelMismatch(params, *group);
}

bool GeodexPlannerManager::canServiceRequest(
  const planning_interface::MotionPlanRequest & req) const
{
  return rejectReason(req, param_listener_->get_params()).empty();
}

planning_interface::PlanningContextPtr GeodexPlannerManager::getPlanningContext(
  const planning_scene::PlanningSceneConstPtr & planning_scene,
  const planning_interface::MotionPlanRequest & req,
  moveit_msgs::msg::MoveItErrorCodes & error_code) const
{
  const Params params = param_listener_->get_params();
  if (!planning_scene) {
    error_code.val = MoveItErrorCodes::FAILURE;
    RCLCPP_ERROR(logger(), "no planning scene");
    return nullptr;
  }
  if (const std::string reason = rejectReason(req, params); !reason.empty()) {
    error_code.val = model_->hasJointModelGroup(req.group_name)
                       ? MoveItErrorCodes::PLANNING_FAILED
                       : MoveItErrorCodes::INVALID_GROUP_NAME;
    RCLCPP_ERROR(logger(), "%s", reason.c_str());
    return nullptr;
  }
  std::shared_ptr<const PlannerBackend> backend;
  try {
    backend = PlannerBackend::create(
      params.robot, params.metric, configurationSize(*model_->getJointModelGroup(req.group_name)));
  } catch (const std::exception & e) {
    error_code.val = MoveItErrorCodes::PLANNING_FAILED;
    RCLCPP_ERROR(logger(), "%s", e.what());
    return nullptr;
  }
  RCLCPP_DEBUG(logger(), "%s", backend->describe().c_str());

  auto context = std::make_shared<GeodexPlanningContext>(
    "geodex", req.group_name, params, std::move(backend), cache_);
  context->setPlanningScene(planning_scene);
  context->setMotionPlanRequest(req);
  error_code.val = MoveItErrorCodes::SUCCESS;
  return context;
}

}  // namespace geodex_moveit
