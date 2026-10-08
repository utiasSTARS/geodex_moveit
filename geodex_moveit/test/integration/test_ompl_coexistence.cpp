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

// Plans with MoveIt's own OMPL planner and this plugin in one process. The plugin links a fork
// of OMPL with the same soname as the stock OMPL that MoveIt loads. In this setup, a leaked symbol
// binds one planner to the other's library.

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

namespace
{

using moveit_msgs::msg::MoveItErrorCodes;
constexpr const char * kGroup = "panda_arm";

std::string readFile(const std::string & path)
{
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

TEST(OmplCoexistence, BothPlannersPlanInOneProcess)
{
  const std::string desc =
    ament_index_cpp::get_package_share_directory("moveit_resources_panda_description");
  const std::string conf =
    ament_index_cpp::get_package_share_directory("moveit_resources_panda_moveit_config");
  const std::vector<rclcpp::Parameter> overrides{
    {"robot_description", readFile(desc + "/urdf/panda.urdf")},
    {"robot_description_semantic", readFile(conf + "/config/panda.srdf")},
    {"ompl.planner_configs.RRTConnectkConfigDefault.type", "geometric::RRTConnect"},
    {"ompl.panda_arm.planner_configs", std::vector<std::string>{"RRTConnectkConfigDefault"}},
    {"geodex.robot", "panda"},
    {"geodex.end_effector_link", "panda_hand"},
    {"geodex.end_effector_offset", std::vector<double>{0.0, 0.0, 0.105, 0.0, 0.0, 0.0}},
    {"geodex.exact_attached_box", false},
    {"geodex.planner.collision_check_resolution", 0.01},
  };
  auto node = std::make_shared<rclcpp::Node>(
    "geodex_moveit_ompl_coexistence_test", rclcpp::NodeOptions()
                                             .parameter_overrides(overrides)
                                             .automatically_declare_parameters_from_overrides(true)
                                             .start_parameter_services(false));
  robot_model_loader::RobotModelLoader loader(node, "robot_description", false);
  const auto model = loader.getModel();
  ASSERT_TRUE(model);

  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  auto & state = scene->getCurrentStateNonConst();
  state.setToDefaultValues(model->getJointModelGroup(kGroup), "ready");
  state.update();
  scene->getWorldNonConst()->addToObject(
    "wall", std::make_shared<shapes::Box>(0.05, 0.3, 0.4),
    Eigen::Isometry3d(Eigen::Translation3d(0.5, 0.0, 0.45)));

  // The build chooses which plugin loads first. A loaded library stays loaded for the life of the
  // process, and each order needs its own test binary.
#ifdef GEODEX_MOVEIT_LOAD_GEODEX_FIRST
  planning_pipeline::PlanningPipeline geodex(
    model, node, "geodex", {"geodex_moveit/GeodexPlanner"});
  planning_pipeline::PlanningPipeline ompl(model, node, "ompl", {"ompl_interface/OMPLPlanner"});
#else
  planning_pipeline::PlanningPipeline ompl(model, node, "ompl", {"ompl_interface/OMPLPlanner"});
  planning_pipeline::PlanningPipeline geodex(
    model, node, "geodex", {"geodex_moveit/GeodexPlanner"});
#endif

  const std::vector<std::vector<double>> goals{
    {1.0, 0.3, 0.0, -1.8, 0.0, 2.1, 0.785}, {-1.0, 0.3, 0.0, -1.8, 0.0, 2.1, 0.785}};
  for (int round = 0; round < 2; ++round) {
    for (const auto & goal : goals) {
      moveit::core::RobotState g(state);
      g.setJointGroupPositions(kGroup, goal);
      g.update();
      planning_interface::MotionPlanRequest req;
      req.group_name = kGroup;
      req.allowed_planning_time = 2.0;
      req.goal_constraints.push_back(
        kinematic_constraints::constructGoalConstraints(g, model->getJointModelGroup(kGroup)));
      for (auto * pipeline : {&ompl, &geodex}) {
        planning_interface::MotionPlanResponse res;
        EXPECT_TRUE(pipeline->generatePlan(scene, req, res));
        EXPECT_EQ(res.error_code.val, MoveItErrorCodes::SUCCESS)
          << (pipeline == &ompl ? "ompl" : "geodex") << " round " << round;
        if (pipeline == &geodex) {
          EXPECT_EQ(res.planner_id, "GreedyRRTstar");
        } else {
          EXPECT_NE(res.planner_id, "GreedyRRTstar");
        }
      }
    }
  }
}

}  // namespace
