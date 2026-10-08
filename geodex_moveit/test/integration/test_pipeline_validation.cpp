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

// Runs the plugin's pipeline file, with time parameterization and ValidateSolution, on the Panda
// wall problem with the Panda demo's parameters. Request i plans with seed i, and the iteration
// budget ends each search before the time budget. With either checker, every request succeeds
// and ValidateSolution accepts every solution. GEODEX_MOVEIT_PIPELINE_REQUESTS sets the number of
// requests, and GEODEX_MOVEIT_PIPELINE_SWEEP=checker:resolution:padding[,...] measures other
// settings instead of asserting.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometric_shapes/shapes.h"
#include "gtest/gtest.h"
#include "moveit/kinematic_constraints/utils.hpp"
#include "moveit/planning_pipeline/planning_pipeline.hpp"
#include "moveit/planning_scene/planning_scene.hpp"
#include "moveit/robot_model_loader/robot_model_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace
{

using moveit_msgs::msg::MoveItErrorCodes;
constexpr const char * kGroup = "panda_arm";
constexpr std::int64_t kIterations = 5000;

std::string readFile(const std::string & path)
{
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::vector<rclcpp::Parameter> robotParameters()
{
  const std::string desc =
    ament_index_cpp::get_package_share_directory("moveit_resources_panda_description");
  const std::string conf =
    ament_index_cpp::get_package_share_directory("moveit_resources_panda_moveit_config");
  std::vector<rclcpp::Parameter> params{
    {"robot_description", readFile(desc + "/urdf/panda.urdf")},
    {"robot_description_semantic", readFile(conf + "/config/panda.srdf")}};
  const YAML::Node limits = YAML::LoadFile(conf + "/config/joint_limits.yaml")["joint_limits"];
  for (const auto & joint : limits) {
    for (const auto & field : joint.second) {
      const std::string key = field.first.as<std::string>();
      const std::string name =
        "robot_description_planning.joint_limits." + joint.first.as<std::string>() + "." + key;
      if (key.rfind("has_", 0) == 0) {
        params.emplace_back(name, field.second.as<bool>());
      } else {
        params.emplace_back(name, field.second.as<double>());
      }
    }
  }
  return params;
}

/// Plugin parameters of the Panda demo, geodex_moveit_demos/config/panda_geodex.yaml, with the
/// given checker. The moveit checker pads by 0.006 m, the largest motion of a Panda mesh point per
/// radian (1.19 m) times half the resolution. The vamp checker adds 0.0181 m, the largest gap
/// between the Panda meshes and the sphere model.
std::vector<rclcpp::Parameter> pandaDemoParameters(const std::string & checker)
{
  return {
    {"geodex.robot", "panda"},
    {"geodex.checker", checker},
    {"geodex.planner.collision_check_resolution", 0.01},
    {"geodex.obstacle_padding", checker == "vamp" ? 0.0241 : 0.006}};
}

struct Outcome
{
  int requests = 0;
  int solved = 0;
  int rejected = 0;
};

/// Sends alternating left and right requests around the wall through the shipped adapter chain.
Outcome runPipeline(const std::vector<rclcpp::Parameter> & plugin, int n)
{
  static int count = 0;
  auto overrides = robotParameters();
  overrides.insert(overrides.end(), plugin.begin(), plugin.end());
  overrides.emplace_back("geodex.planner.max_iterations", kIterations);
  auto node = std::make_shared<rclcpp::Node>(
    "geodex_moveit_pipeline_validation_" + std::to_string(count++),
    rclcpp::NodeOptions()
      .parameter_overrides(overrides)
      .automatically_declare_parameters_from_overrides(true)
      .start_parameter_services(false));
  robot_model_loader::RobotModelLoader loader(node, "robot_description", false);
  const auto model = loader.getModel();
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  scene->getCurrentStateNonConst().setToDefaultValues(model->getJointModelGroup(kGroup), "ready");
  scene->getCurrentStateNonConst().update();
  scene->getWorldNonConst()->addToObject(
    "wall", std::make_shared<shapes::Box>(0.05, 0.3, 0.4),
    Eigen::Isometry3d(Eigen::Translation3d(0.5, 0.0, 0.45)));
  const YAML::Node wiring = YAML::LoadFile(GEODEX_MOVEIT_PIPELINE_CONFIG);
  planning_pipeline::PlanningPipeline pipeline(
    model, node, "geodex", wiring["planning_plugins"].as<std::vector<std::string>>(),
    wiring["request_adapters"].as<std::vector<std::string>>(),
    wiring["response_adapters"].as<std::vector<std::string>>());
  const std::vector<double> left{1.0, 0.3, 0.0, -1.8, 0.0, 2.1, 0.785};
  const std::vector<double> right{-1.0, 0.3, 0.0, -1.8, 0.0, 2.1, 0.785};
  Outcome out;
  for (int i = 0; i < n; ++i) {
    EXPECT_TRUE(node->set_parameter({"geodex.planner.seed", std::int64_t{i}}).successful);
    planning_interface::MotionPlanRequest req;
    req.group_name = kGroup;
    req.allowed_planning_time = 30.0;
    req.max_velocity_scaling_factor = 1.0;
    req.max_acceleration_scaling_factor = 1.0;
    moveit::core::RobotState goal(scene->getCurrentState());
    goal.setJointGroupPositions(kGroup, (i % 2) ? left : right);
    goal.update();
    req.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints(goal, model->getJointModelGroup(kGroup)));
    req.start_state.is_diff = true;
    req.start_state.joint_state.name =
      model->getJointModelGroup(kGroup)->getActiveJointModelNames();
    req.start_state.joint_state.position = (i % 2) ? right : left;
    planning_interface::MotionPlanResponse res;
    pipeline.generatePlan(scene, req, res);
    ++out.requests;
    if (res.error_code.val == MoveItErrorCodes::SUCCESS) ++out.solved;
    if (res.error_code.val == MoveItErrorCodes::INVALID_MOTION_PLAN) ++out.rejected;
  }
  return out;
}

int requests()
{
  const char * n = std::getenv("GEODEX_MOVEIT_PIPELINE_REQUESTS");
  return n != nullptr ? std::atoi(n) : 60;
}

TEST(PipelineValidation, ShippedPandaConfigurationPassesValidateSolution)
{
  if (const char * sweep = std::getenv("GEODEX_MOVEIT_PIPELINE_SWEEP")) {
    std::stringstream entries(sweep);
    std::string entry;
    while (std::getline(entries, entry, ',')) {
      std::stringstream fields(entry);
      std::string checker, resolution, padding;
      std::getline(fields, checker, ':');
      std::getline(fields, resolution, ':');
      std::getline(fields, padding, ':');
      const Outcome o = runPipeline(
        {{"geodex.robot", "panda"},
         {"geodex.checker", checker},
         {"geodex.planner.collision_check_resolution", std::stod(resolution)},
         {"geodex.obstacle_padding", std::stod(padding)}},
        requests());
      std::printf(
        "checker %s resolution %s padding %s: %d of %d solved, %d rejected\n", checker.c_str(),
        resolution.c_str(), padding.c_str(), o.solved, o.requests, o.rejected);
    }
    return;
  }
  for (const std::string checker : {"vamp", "moveit"}) {
    const Outcome o = runPipeline(pandaDemoParameters(checker), requests());
    std::printf(
      "checker %s: %d of %d solved, %d rejected by ValidateSolution\n", checker.c_str(), o.solved,
      o.requests, o.rejected);
    EXPECT_EQ(o.rejected, 0) << checker;
    EXPECT_EQ(o.solved, o.requests) << checker;
  }
}

}  // namespace
