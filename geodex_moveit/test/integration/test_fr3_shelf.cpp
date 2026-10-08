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

// Plans for the FR3 with an attached box in the shelf of the FR3 shelf demo, through the
// plugin with both metrics. The robot, the scene, the box's sphere cover and the endpoints come
// from the replay fixtures.

#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geodex_moveit/environment_cache.hpp"
#include "geodex_moveit/group_state.hpp"
#include "geodex_moveit/request_setup.hpp"
#include "gtest/gtest.h"
#include "moveit/kinematic_constraints/utils.hpp"
#include "moveit/planning_interface/planning_interface.hpp"
#include "moveit/planning_scene/planning_scene.hpp"
#include "moveit/robot_model/robot_model.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "srdfdom/model.h"
#include "urdf_parser/urdf_parser.h"

namespace fs = std::filesystem;

namespace
{

using moveit_msgs::msg::MoveItErrorCodes;
const fs::path kFixtures = GEODEX_MOVEIT_REPLAY_FIXTURES;

std::string slurp(const fs::path & p)
{
  std::ifstream in(p, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::vector<std::vector<double>> readRows(const fs::path & p)
{
  std::vector<std::vector<double>> rows;
  std::ifstream in(p);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::vector<double> row;
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, ',')) row.push_back(std::strtod(tok.c_str(), nullptr));
    rows.push_back(row);
  }
  return rows;
}

class Fr3Shelf : public ::testing::TestWithParam<std::string>
{
};

std::vector<double> attachedSpheres()
{
  std::vector<double> flat;
  for (const auto & row : readRows(kFixtures / "attached_spheres.csv")) {
    flat.insert(flat.end(), row.begin(), row.end());
  }
  return flat;
}

TEST_P(Fr3Shelf, PlansAroundTheShelfWithTheAttachedBox)
{
  const std::string metric = GetParam();
  const fs::path task = kFixtures / metric / "bottom_top";

  auto urdf = urdf::parseURDF(slurp(kFixtures / "fr3_arm_gripper.urdf"));
  auto srdf = std::make_shared<srdf::Model>();
  srdf->initString(*urdf, slurp(kFixtures / "fr3_arm_gripper.srdf"));
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  {
    const std::string bytes = slurp(task / "scene.bin");
    rclcpp::SerializedMessage sm(bytes.size());
    auto & raw = sm.get_rcl_serialized_message();
    std::copy(bytes.begin(), bytes.end(), reinterpret_cast<char *>(raw.buffer));
    raw.buffer_length = bytes.size();
    moveit_msgs::msg::PlanningScene msg;
    rclcpp::Serialization<moveit_msgs::msg::PlanningScene>().deserialize_message(&sm, &msg);
    scene->setPlanningSceneMsg(msg);
  }

  const std::vector<rclcpp::Parameter> overrides{
    {"geodex.robot", "fr3_arm_gripper"},
    {"geodex.metric", metric},
    {"geodex.end_effector_link", "2f85_tcp"},
    {"geodex.attached_object.spheres", attachedSpheres()},
    {"geodex.planner.collision_check_resolution", 0.002},
    {"geodex.planner.config.grrtstar.range", metric == "euclidean" ? 1.2 : 0.6}};
  auto node = std::make_shared<rclcpp::Node>(
    "geodex_moveit_fr3_shelf_test_" + metric,
    rclcpp::NodeOptions().parameter_overrides(overrides).start_parameter_services(false));
  pluginlib::ClassLoader<planning_interface::PlannerManager> loader(
    "moveit_core", "planning_interface::PlannerManager");
  auto manager = loader.createSharedInstance("geodex_moveit/GeodexPlanner");
  ASSERT_TRUE(manager->initialize(model, node, "geodex"));

  const auto * group = model->getJointModelGroup("fr3_arm");
  const auto raw = readRows(task / "raw.csv");
  moveit::core::RobotState goal(scene->getCurrentState());
  geodex_moveit::setConfiguration(goal, *group, raw.back().data());
  goal.update();
  planning_interface::MotionPlanRequest req;
  req.group_name = "fr3_arm";
  req.allowed_planning_time = 0.25;
  req.start_state.is_diff = true;
  req.start_state.joint_state.name = group->getActiveJointModelNames();
  req.start_state.joint_state.position = raw.front();
  req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal, group));

  moveit_msgs::msg::MoveItErrorCodes code;
  auto context = manager->getPlanningContext(scene, req, code);
  ASSERT_TRUE(context);
  planning_interface::MotionPlanResponse res;
  context->solve(res);
  ASSERT_EQ(res.error_code.val, MoveItErrorCodes::SUCCESS);

  // Every returned waypoint passes the plugin's own validity, the attached-box test included.
  geodex_moveit::Params params;
  params.robot = "fr3_arm_gripper";
  params.end_effector_link = "2f85_tcp";
  params.attached_object.spheres = attachedSpheres();
  geodex_moveit::EnvironmentCache cache;
  std::string message;
  const auto validity =
    geodex_moveit::buildValidity(scene, scene->getCurrentState(), *group, params, cache, message);
  ASSERT_TRUE(validity) << message;
  for (std::size_t i = 0; i < res.trajectory->getWayPointCount(); ++i) {
    const auto q = geodex_moveit::configuration(res.trajectory->getWayPoint(i), *group);
    EXPECT_TRUE(validity->isValid(q.data())) << "waypoint " << i;
  }
  const auto last = geodex_moveit::configuration(res.trajectory->getLastWayPoint(), *group);
  for (std::size_t j = 0; j < last.size(); ++j) EXPECT_NEAR(last[j], raw.back()[j], 1e-9);
}

INSTANTIATE_TEST_SUITE_P(Metrics, Fr3Shelf, ::testing::Values("euclidean", "kinetic_energy"));

}  // namespace
