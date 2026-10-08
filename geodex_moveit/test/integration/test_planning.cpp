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

// End-to-end tests load the plugin as MoveIt does, through pluginlib, on the Panda of
// moveit_resources, with a node that holds the plugin's parameters.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geodex_moveit/environment_cache.hpp"
#include "geodex_moveit/group_state.hpp"
#include "geodex_moveit/request_setup.hpp"
#include "geometric_shapes/shapes.h"
#include "gtest/gtest.h"
#include "moveit/kinematic_constraints/utils.hpp"
#include "moveit/planning_interface/planning_interface.hpp"
#include "moveit/planning_pipeline/planning_pipeline.hpp"
#include "moveit/planning_scene/planning_scene.hpp"
#include "moveit/robot_model_loader/robot_model_loader.hpp"
#include "moveit/utils/robot_model_test_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "srdfdom/model.h"
#include "urdf_parser/urdf_parser.h"
#include "yaml-cpp/yaml.h"

namespace
{

using moveit_msgs::msg::MoveItErrorCodes;
constexpr const char * kGroup = "panda_arm";
constexpr const char * kPlugin = "geodex_moveit/GeodexPlanner";
// Pose of geodex's Panda end-effector frame, panda_grasptarget, in panda_hand.
const std::vector<double> kGraspTargetInHand{0.0, 0.0, 0.105, 0.0, 0.0, 0.0};

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
    {"robot_description_semantic", readFile(conf + "/config/panda.srdf")},
    {"robot_description_kinematics.panda_arm.kinematics_solver",
     "kdl_kinematics_plugin/KDLKinematicsPlugin"},
    {"robot_description_kinematics.panda_arm.kinematics_solver_timeout", 0.05}};
  // Time parameterization needs joint limits with accelerations.
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

class PandaPlanning : public ::testing::Test
{
protected:
  /// Plugin parameters of the fixture. Tests that need other parameters build their own manager.
  virtual std::vector<rclcpp::Parameter> pluginParameters() const
  {
    return {
      {"geodex.robot", "panda"},
      {"geodex.end_effector_link", "panda_hand"},
      {"geodex.end_effector_offset", kGraspTargetInHand},
      {"geodex.exact_attached_box", false},
      {"geodex.planner.collision_check_resolution", 0.01}};
  }

  void SetUp() override
  {
    static int count = 0;
    auto overrides = robotParameters();
    for (const auto & p : pluginParameters()) overrides.push_back(p);
    node_ = std::make_shared<rclcpp::Node>(
      "geodex_moveit_planning_test_" + std::to_string(count++),
      rclcpp::NodeOptions().parameter_overrides(overrides).start_parameter_services(false));
    model_loader_ =
      std::make_unique<robot_model_loader::RobotModelLoader>(node_, "robot_description", true);
    model_ = model_loader_->getModel();
    ASSERT_TRUE(model_);
    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    auto & state = scene_->getCurrentStateNonConst();
    state.setToDefaultValues(model_->getJointModelGroup(kGroup), "ready");
    state.update();
    loader_ = std::make_unique<pluginlib::ClassLoader<planning_interface::PlannerManager>>(
      "moveit_core", "planning_interface::PlannerManager");
    manager_ = loader_->createSharedInstance(kPlugin);
    ASSERT_TRUE(manager_->initialize(model_, node_, "geodex"));
  }

  void TearDown() override
  {
    manager_.reset();
    loader_.reset();
  }

  /// A wall in front of the robot between the two test configurations.
  void addWall()
  {
    scene_->getWorldNonConst()->addToObject(
      "wall", std::make_shared<shapes::Box>(0.05, 0.3, 0.4),
      Eigen::Isometry3d(Eigen::Translation3d(0.5, 0.0, 0.45)));
  }

  std::vector<double> ready() const
  {
    std::vector<double> q;
    scene_->getCurrentState().copyJointGroupPositions(kGroup, q);
    return q;
  }

  static std::vector<double> left() { return {1.0, 0.3, 0.0, -1.8, 0.0, 2.1, 0.785}; }
  static std::vector<double> right() { return {-1.0, 0.3, 0.0, -1.8, 0.0, 2.1, 0.785}; }

  planning_interface::MotionPlanRequest jointRequest(
    const std::vector<double> & goal, double time = 2.0) const
  {
    planning_interface::MotionPlanRequest req;
    req.group_name = kGroup;
    req.allowed_planning_time = time;
    moveit::core::RobotState g(scene_->getCurrentState());
    g.setJointGroupPositions(kGroup, goal);
    g.update();
    req.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints(g, model_->getJointModelGroup(kGroup)));
    return req;
  }

  planning_interface::MotionPlanResponse solve(const planning_interface::MotionPlanRequest & req)
  {
    moveit_msgs::msg::MoveItErrorCodes code;
    planning_interface::MotionPlanResponse res;
    auto context = manager_->getPlanningContext(scene_, req, code);
    if (!context) {
      res.error_code = code;
      return res;
    }
    context->solve(res);
    return res;
  }

  std::vector<double> endOf(const planning_interface::MotionPlanResponse & res, bool last) const
  {
    std::vector<double> q;
    const auto & wp = last ? res.trajectory->getLastWayPoint() : res.trajectory->getFirstWayPoint();
    wp.copyJointGroupPositions(kGroup, q);
    return q;
  }

  rclcpp::Node::SharedPtr node_;
  // The loader owns the kinematics plugins of the model and outlives it.
  std::unique_ptr<robot_model_loader::RobotModelLoader> model_loader_;
  moveit::core::RobotModelPtr model_;
  planning_scene::PlanningScenePtr scene_;
  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> loader_;
  std::shared_ptr<planning_interface::PlannerManager> manager_;
};

void expectNear(const std::vector<double> & a, const std::vector<double> & b, double tol)
{
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) EXPECT_NEAR(a[i], b[i], tol) << "joint " << i;
}

// The FCL check below uses the robot's meshes, which the vamp sphere model approximates. This test
// plans with the moveit checker, and the next one solves the same request with both checkers.
TEST_F(PandaPlanning, ReachesAJointGoalAroundAWall)
{
  ASSERT_TRUE(node_->set_parameter({"geodex.checker", "moveit"}).successful);
  addWall();
  auto req = jointRequest(right());
  req.start_state.joint_state.name = model_->getJointModelGroup(kGroup)->getActiveJointModelNames();
  req.start_state.joint_state.position = left();
  req.start_state.is_diff = true;
  const auto res = solve(req);
  ASSERT_EQ(res.error_code.val, MoveItErrorCodes::SUCCESS);
  expectNear(endOf(res, false), left(), 1e-12);
  expectNear(endOf(res, true), right(), 1e-12);
  for (std::size_t i = 0; i < res.trajectory->getWayPointCount(); ++i) {
    moveit::core::RobotState s = res.trajectory->getWayPoint(i);
    s.update();
    EXPECT_FALSE(scene_->isStateColliding(s, kGroup)) << "waypoint " << i;
  }
  EXPECT_GT(res.planning_time, 0.0);
}

TEST_F(PandaPlanning, MetricsAndCheckersSolve)
{
  addWall();
  for (const std::string checker : {"vamp", "moveit"}) {
    ASSERT_TRUE(node_->set_parameter({"geodex.checker", checker}).successful);
    for (const std::string metric : {"euclidean", "kinetic_energy"}) {
      ASSERT_TRUE(node_->set_parameter({"geodex.metric", metric}).successful);
      const auto res = solve(jointRequest(left(), 1.0));
      EXPECT_EQ(res.error_code.val, MoveItErrorCodes::SUCCESS) << checker << " " << metric;
      EXPECT_EQ(res.planner_id, "GreedyRRTstar");
    }
  }
}

TEST_F(PandaPlanning, EmptyGoalIsInvalid)
{
  planning_interface::MotionPlanRequest req;
  req.group_name = kGroup;
  req.allowed_planning_time = 1.0;
  EXPECT_EQ(solve(req).error_code.val, MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
}

TEST_F(PandaPlanning, GoalInCollisionIsRejected)
{
  moveit::core::RobotState g(scene_->getCurrentState());
  g.setJointGroupPositions(kGroup, left());
  g.update();
  scene_->getWorldNonConst()->addToObject(
    "block", std::make_shared<shapes::Box>(0.2, 0.2, 0.2),
    Eigen::Isometry3d(Eigen::Translation3d(g.getGlobalLinkTransform("panda_hand").translation())));
  const auto res = solve(jointRequest(left()));
  EXPECT_EQ(res.error_code.val, MoveItErrorCodes::GOAL_IN_COLLISION);
  EXPECT_FALSE(res.trajectory);
}

TEST_F(PandaPlanning, GoalOutsideTheBoundsIsRejected)
{
  auto goal = left();
  goal[3] = 1.0;  // panda_joint4 is bounded above by -0.0698
  planning_interface::MotionPlanRequest req = jointRequest(left());
  for (auto & jc : req.goal_constraints.front().joint_constraints) {
    if (jc.joint_name == "panda_joint4") jc.position = goal[3];
  }
  EXPECT_EQ(solve(req).error_code.val, MoveItErrorCodes::GOAL_STATE_INVALID);
}

TEST_F(PandaPlanning, ExhaustedBudgetIsNotSuccess)
{
  addWall();
  auto req = jointRequest(right(), 1e-6);
  req.start_state.joint_state.name = model_->getJointModelGroup(kGroup)->getActiveJointModelNames();
  req.start_state.joint_state.position = left();
  req.start_state.is_diff = true;
  const auto res = solve(req);
  EXPECT_EQ(res.error_code.val, MoveItErrorCodes::TIMED_OUT);
  EXPECT_FALSE(res.trajectory);
}

TEST_F(PandaPlanning, UnservableRequestsAreRefused)
{
  auto req = jointRequest(left());
  req.planner_id = "NoSuchPlanner";
  EXPECT_FALSE(manager_->canServiceRequest(req));
  EXPECT_NE(solve(req).error_code.val, MoveItErrorCodes::SUCCESS);

  // The hand group has one active joint, the robot model seven.
  planning_interface::MotionPlanRequest hand;
  hand.group_name = "hand";
  EXPECT_FALSE(manager_->canServiceRequest(hand));
  moveit_msgs::msg::MoveItErrorCodes code;
  EXPECT_FALSE(manager_->getPlanningContext(scene_, hand, code));
  EXPECT_NE(code.val, MoveItErrorCodes::SUCCESS);

  auto constrained = jointRequest(left());
  constrained.path_constraints.joint_constraints.push_back(
    constrained.goal_constraints.front().joint_constraints.front());
  EXPECT_FALSE(manager_->canServiceRequest(constrained));

  planning_interface::MotionPlanRequest nogroup;
  nogroup.group_name = "no_such_group";
  EXPECT_FALSE(manager_->getPlanningContext(scene_, nogroup, code));
  EXPECT_EQ(code.val, MoveItErrorCodes::INVALID_GROUP_NAME);
}

TEST_F(PandaPlanning, PoseGoalThroughInverseKinematics)
{
  moveit::core::RobotState g(scene_->getCurrentState());
  g.setJointGroupPositions(kGroup, left());
  g.update();
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = model_->getModelFrame();
  const Eigen::Isometry3d x = g.getGlobalLinkTransform("panda_hand");
  const Eigen::Quaterniond r(x.linear());
  target.pose.position.x = x.translation().x();
  target.pose.position.y = x.translation().y();
  target.pose.position.z = x.translation().z();
  target.pose.orientation.x = r.x();
  target.pose.orientation.y = r.y();
  target.pose.orientation.z = r.z();
  target.pose.orientation.w = r.w();

  planning_interface::MotionPlanRequest req;
  req.group_name = kGroup;
  req.allowed_planning_time = 2.0;
  req.goal_constraints.push_back(
    kinematic_constraints::constructGoalConstraints("panda_hand", target));
  const auto res = solve(req);
  ASSERT_EQ(res.error_code.val, MoveItErrorCodes::SUCCESS);
  const Eigen::Isometry3d reached =
    res.trajectory->getLastWayPoint().getGlobalLinkTransform("panda_hand");
  EXPECT_LT((reached.translation() - x.translation()).norm(), 1e-3);

  target.pose.position.z = 5.0;
  req.goal_constraints = {kinematic_constraints::constructGoalConstraints("panda_hand", target)};
  EXPECT_EQ(solve(req).error_code.val, MoveItErrorCodes::NO_IK_SOLUTION);
}

TEST_F(PandaPlanning, TerminateStopsARunningSolve)
{
  addWall();
  auto req = jointRequest(right(), 30.0);
  req.start_state.joint_state.name = model_->getJointModelGroup(kGroup)->getActiveJointModelNames();
  req.start_state.joint_state.position = left();
  req.start_state.is_diff = true;
  moveit_msgs::msg::MoveItErrorCodes code;
  auto context = manager_->getPlanningContext(scene_, req, code);
  ASSERT_TRUE(context);
  planning_interface::MotionPlanResponse res;
  const auto t0 = std::chrono::steady_clock::now();
  std::thread worker([&] { context->solve(res); });
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_TRUE(context->terminate());
  worker.join();
  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_LT(elapsed, 5.0);
  // An anytime search that already holds an exact solution returns it.
  EXPECT_TRUE(
    res.error_code.val == MoveItErrorCodes::SUCCESS ||
    res.error_code.val == MoveItErrorCodes::PREEMPTED);
}

TEST_F(PandaPlanning, ConcurrentContextsDoNotInterfere)
{
  addWall();
  const std::vector<std::vector<double>> goals{
    left(), right(), {0.5, 0.2, 0.1, -2.0, 0.0, 2.2, 0.5}, {-0.5, -0.2, -0.1, -2.2, 0.1, 1.9, 1.0}};
  std::vector<planning_interface::MotionPlanResponse> results(goals.size());
  std::vector<std::thread> threads;
  for (std::size_t i = 0; i < goals.size(); ++i) {
    threads.emplace_back([&, i] { results[i] = solve(jointRequest(goals[i], 1.0)); });
  }
  for (auto & t : threads) t.join();
  for (std::size_t i = 0; i < goals.size(); ++i) {
    ASSERT_EQ(results[i].error_code.val, MoveItErrorCodes::SUCCESS) << "request " << i;
    expectNear(endOf(results[i], true), goals[i], 1e-12);
  }
}

TEST_F(PandaPlanning, AttachedObjectNeedsTheEndEffectorFrame)
{
  auto & state = scene_->getCurrentStateNonConst();
  state.attachBody(
    "ball", Eigen::Isometry3d::Identity(), {std::make_shared<shapes::Sphere>(0.03)},
    {Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.12))}, std::set<std::string>{},
    "panda_hand");
  state.update();
  // The collision model's own frame, panda_grasptarget, is not a link of this Panda.
  ASSERT_TRUE(
    node_
      ->set_parameters_atomically(
        {{"geodex.end_effector_link", ""}, {"geodex.end_effector_offset", std::vector<double>{}}})
      .successful);
  EXPECT_EQ(
    solve(jointRequest(left())).error_code.val, MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE);
  ASSERT_TRUE(node_
                ->set_parameters_atomically(
                  {{"geodex.end_effector_link", "panda_hand"},
                   {"geodex.end_effector_offset", kGraspTargetInHand}})
                .successful);
  // Without a box, the exact test does not run.
  ASSERT_TRUE(node_->set_parameter({"geodex.exact_attached_box", true}).successful);
  EXPECT_EQ(solve(jointRequest(left())).error_code.val, MoveItErrorCodes::SUCCESS);
  ASSERT_TRUE(
    node_->set_parameter({"geodex.attached_object.box_size", std::vector<double>{0.04, 0.04, 0.04}})
      .successful);
  ASSERT_TRUE(
    node_
      ->set_parameter(
        {"geodex.attached_object.box_pose", std::vector<double>{0.0, 0.0, 0.12, 0.0, 0.0, 0.0}})
      .successful);
  EXPECT_EQ(solve(jointRequest(left())).error_code.val, MoveItErrorCodes::SUCCESS);
}

TEST_F(PandaPlanning, MoveItCheckerWithoutARobotModel)
{
  ASSERT_TRUE(node_->set_parameter({"geodex.checker", "moveit"}).successful);
  ASSERT_TRUE(node_->set_parameter({"geodex.robot", ""}).successful);
  addWall();
  EXPECT_EQ(solve(jointRequest(left(), 2.0)).error_code.val, MoveItErrorCodes::SUCCESS);
  // The kinetic-energy metric needs a robot model.
  ASSERT_TRUE(node_->set_parameter({"geodex.metric", "kinetic_energy"}).successful);
  EXPECT_FALSE(manager_->canServiceRequest(jointRequest(left())));
}

TEST_F(PandaPlanning, ThroughThePlanningPipeline)
{
  // ValidateSolution checks the meshes again at states that time parameterization places between
  // the checked ones. The Panda demo's padding for the moveit checker covers those states.
  ASSERT_TRUE(node_->set_parameter({"geodex.checker", "moveit"}).successful);
  ASSERT_TRUE(node_->set_parameter({"geodex.obstacle_padding", 0.006}).successful);
  addWall();
  auto pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(
    model_, node_, "geodex", std::vector<std::string>{kPlugin},
    std::vector<std::string>{
      "default_planning_request_adapters/CheckStartStateBounds",
      "default_planning_request_adapters/CheckStartStateCollision"},
    std::vector<std::string>{
      "default_planning_response_adapters/AddTimeOptimalParameterization",
      "default_planning_response_adapters/ValidateSolution"});
  auto req = jointRequest(left());
  req.max_velocity_scaling_factor = 1.0;
  req.max_acceleration_scaling_factor = 1.0;
  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(pipeline->generatePlan(scene_, req, res));
  ASSERT_EQ(res.error_code.val, MoveItErrorCodes::SUCCESS);
  EXPECT_GT(res.trajectory->getDuration(), 0.0);
}

moveit_msgs::msg::AttachedCollisionObject attachedBall(double z, double radius)
{
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.link_name = "panda_hand";
  aco.object.id = "ball";
  aco.object.header.frame_id = "panda_hand";
  shape_msgs::msg::SolidPrimitive sphere;
  sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  sphere.dimensions = {radius};
  aco.object.primitives.push_back(sphere);
  geometry_msgs::msg::Pose pose;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  aco.object.primitive_poses.push_back(pose);
  aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  aco.touch_links = {"panda_hand", "panda_leftfinger", "panda_rightfinger"};
  return aco;
}

// Concurrent requests on one manager while an object is attached share the cached environment.
// Every returned waypoint is checked afterwards against a fresh environment.
TEST_F(PandaPlanning, ConcurrentRequestsWithAttachedObject)
{
  addWall();
  auto & state = scene_->getCurrentStateNonConst();
  std::vector<shapes::ShapeConstPtr> balls;
  EigenSTL::vector_Isometry3d poses;
  for (int i = 0; i < 4; ++i) {
    balls.push_back(std::make_shared<shapes::Sphere>(0.03));
    poses.push_back(Eigen::Isometry3d(Eigen::Translation3d(0.0, (i - 1.5) * 0.04, 0.2)));
  }
  state.attachBody(
    "ball", Eigen::Isometry3d::Identity(), balls, poses,
    std::set<std::string>{"panda_hand", "panda_leftfinger", "panda_rightfinger"}, "panda_hand");
  state.update();
  const auto * group = model_->getJointModelGroup(kGroup);
  geodex_moveit::Params params;
  params.robot = "panda";
  params.end_effector_link = "panda_hand";
  params.end_effector_offset = kGraspTargetInHand;
  params.exact_attached_box = false;
  geodex_moveit::EnvironmentCache check_cache;
  std::string message;
  const auto validity = geodex_moveit::buildValidity(
    scene_, scene_->getCurrentState(), *group, params, check_cache, message);
  ASSERT_TRUE(validity) << message;
  std::vector<std::vector<double>> goals;
  for (const auto & c : std::vector<std::vector<double>>{
         left(),
         right(),
         {0.5, 0.2, 0.1, -2.0, 0.0, 2.2, 0.5},
         {-0.5, -0.2, -0.1, -2.2, 0.1, 1.9, 1.0},
         {1.4, -0.3, 0.2, -1.6, 0.0, 1.8, 0.2},
         {-1.4, 0.1, -0.2, -2.4, 0.0, 2.4, 1.2}}) {
    if (validity->isValid(c.data())) goals.push_back(c);
  }
  ASSERT_GE(goals.size(), 3u);
  int solved = 0;
  int invalid = 0;
  const int rounds = 10;
  for (int round = 0; round < rounds; ++round) {
    std::vector<planning_interface::MotionPlanResponse> results(goals.size());
    std::vector<std::thread> threads;
    for (std::size_t i = 0; i < goals.size(); ++i) {
      threads.emplace_back([&, i] {
        auto req = jointRequest(goals[(i + round) % goals.size()], 0.3);
        req.start_state.joint_state.name = group->getActiveJointModelNames();
        req.start_state.joint_state.position = goals[(i + round + 1) % goals.size()];
        req.start_state.is_diff = true;
        results[i] = solve(req);
      });
    }
    for (auto & t : threads) t.join();
    for (const auto & r : results) {
      if (r.error_code.val != MoveItErrorCodes::SUCCESS) continue;
      ++solved;
      for (std::size_t w = 0; w < r.trajectory->getWayPointCount(); ++w) {
        const auto q = geodex_moveit::configuration(r.trajectory->getWayPoint(w), *group);
        if (!validity->isValid(q.data())) ++invalid;
      }
    }
  }
  EXPECT_EQ(invalid, 0);
  EXPECT_EQ(solved, rounds * static_cast<int>(goals.size()));
}

// An object attached in the request's start state reaches the vamp checker as it reaches
// MoveIt's. With the ball inside a post, both refuse the start.
TEST_F(PandaPlanning, AttachedBodyInRequestStartState)
{
  const auto & cur = scene_->getCurrentState();
  const Eigen::Isometry3d hand = cur.getGlobalLinkTransform("panda_hand");
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = hand.linear();
  pose.translation() = hand * Eigen::Vector3d(0.0, 0.0, 0.32);
  scene_->getWorldNonConst()->addToObject(
    "post", std::make_shared<shapes::Box>(0.04, 0.04, 0.3), pose);
  ASSERT_FALSE(scene_->isStateColliding(cur, kGroup)) << "the arm alone is clear";
  auto req = jointRequest(left());
  req.start_state.is_diff = true;
  req.start_state.attached_collision_objects.push_back(attachedBall(0.25, 0.05));
  for (const std::string checker : {"vamp", "moveit"}) {
    ASSERT_TRUE(node_->set_parameter({"geodex.checker", checker}).successful);
    EXPECT_EQ(solve(req).error_code.val, MoveItErrorCodes::START_STATE_IN_COLLISION) << checker;
  }
}

// The vamp checker places an attached ball where MoveIt does. A block at the ball blocks the start
// for both checkers.
TEST_F(PandaPlanning, AttachedBallSitsWhereMoveItHasIt)
{
  auto & state = scene_->getCurrentStateNonConst();
  const Eigen::Vector3d ball =
    state.getGlobalLinkTransform("panda_hand") * Eigen::Vector3d(0.0, 0.0, 0.25);
  scene_->getWorldNonConst()->addToObject(
    "block", std::make_shared<shapes::Box>(0.04, 0.04, 0.04),
    Eigen::Isometry3d(Eigen::Translation3d(ball)));
  ASSERT_FALSE(scene_->isStateColliding(state, kGroup)) << "the arm alone is clear";
  state.attachBody(
    "ball", Eigen::Isometry3d::Identity(), {std::make_shared<shapes::Sphere>(0.05)},
    {Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.25))},
    std::set<std::string>{"panda_hand", "panda_leftfinger", "panda_rightfinger"}, "panda_hand");
  state.update();
  for (const std::string checker : {"vamp", "moveit"}) {
    ASSERT_TRUE(node_->set_parameter({"geodex.checker", checker}).successful);
    EXPECT_EQ(
      solve(jointRequest(left())).error_code.val, MoveItErrorCodes::START_STATE_IN_COLLISION)
      << checker;
  }
}

// A pose goal plans to a collision-free goal when the inverse kinematics solution nearest the
// start collides and other solutions are free.
TEST_F(PandaPlanning, PoseGoalAvoidsACollidingIkSolution)
{
  const auto * group = model_->getJointModelGroup(kGroup);
  moveit::core::RobotState g(scene_->getCurrentState());
  g.setJointGroupPositions(kGroup, left());
  g.update();
  const Eigen::Isometry3d x = g.getGlobalLinkTransform("panda_hand");
  moveit::core::RobotState ik(scene_->getCurrentState());
  ASSERT_TRUE(ik.setFromIK(group, x, "panda_hand", 0.1));
  ik.update();
  bool placed = false;
  for (const std::string link : {"panda_link4", "panda_link3", "panda_link5", "panda_link6"}) {
    for (double radius : {0.02, 0.03, 0.04, 0.05, 0.06}) {
      scene_->getWorldNonConst()->removeObject("elbow_block");
      scene_->getWorldNonConst()->addToObject(
        "elbow_block", std::make_shared<shapes::Sphere>(radius),
        Eigen::Isometry3d(Eigen::Translation3d(ik.getGlobalLinkTransform(link).translation())));
      if (
        scene_->isStateColliding(ik, kGroup) &&
        !scene_->isStateColliding(scene_->getCurrentState(), kGroup) &&
        !scene_->isStateColliding(g, kGroup)) {
        placed = true;
        break;
      }
    }
    if (placed) break;
  }
  ASSERT_TRUE(placed);
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = model_->getModelFrame();
  const Eigen::Quaterniond r(x.linear());
  target.pose.position.x = x.translation().x();
  target.pose.position.y = x.translation().y();
  target.pose.position.z = x.translation().z();
  target.pose.orientation.x = r.x();
  target.pose.orientation.y = r.y();
  target.pose.orientation.z = r.z();
  target.pose.orientation.w = r.w();
  planning_interface::MotionPlanRequest req;
  req.group_name = kGroup;
  req.allowed_planning_time = 5.0;
  req.goal_constraints.push_back(
    kinematic_constraints::constructGoalConstraints("panda_hand", target));
  ASSERT_TRUE(node_->set_parameter({"geodex.checker", "moveit"}).successful);
  ASSERT_TRUE(node_->set_parameter({"geodex.planner.ik_timeout", 1.0}).successful);
  const auto res = solve(req);
  ASSERT_EQ(res.error_code.val, MoveItErrorCodes::SUCCESS);
  EXPECT_FALSE(scene_->isStateColliding(res.trajectory->getLastWayPoint(), kGroup));
}

// A padding set at runtime reaches the checker. A goal clear of a box is in collision once the
// padding inflates the box past the robot.
TEST_F(PandaPlanning, ObstaclePaddingReachesTheChecker)
{
  moveit::core::RobotState g(scene_->getCurrentState());
  g.setJointGroupPositions(kGroup, left());
  g.update();
  const Eigen::Vector3d hand = g.getGlobalLinkTransform("panda_hand").translation();
  int probes = 0;
  for (int axis = 0; axis < 3; ++axis) {
    for (const double sign : {-1.0, 1.0}) {
      scene_->getWorldNonConst()->clearObjects();
      scene_->getWorldNonConst()->addToObject(
        "probe", std::make_shared<shapes::Box>(0.1, 0.1, 0.1),
        Eigen::Isometry3d(Eigen::Translation3d(hand + sign * 0.25 * Eigen::Vector3d::Unit(axis))));
      ASSERT_TRUE(node_->set_parameter({"geodex.obstacle_padding", 0.0}).successful);
      if (solve(jointRequest(left(), 1.0)).error_code.val != MoveItErrorCodes::SUCCESS) continue;
      ++probes;
      ASSERT_TRUE(node_->set_parameter({"geodex.obstacle_padding", 0.2}).successful);
      // The inflated box reaches the goal, and sometimes the start as well.
      const int code = solve(jointRequest(left(), 1.0)).error_code.val;
      EXPECT_TRUE(
        code == MoveItErrorCodes::GOAL_IN_COLLISION ||
        code == MoveItErrorCodes::START_STATE_IN_COLLISION)
        << "axis " << axis << ", sign " << sign << ", code " << code;
    }
  }
  EXPECT_GT(probes, 0);
}

// The whole request, search and smoothing, ends within allowed_planning_time. A fine resolution
// makes smoothing outlast the budget unless it stops at the deadline. The tolerance covers the
// time the smoother takes to stop at the deadline.
TEST_F(PandaPlanning, AnytimeRequestsKeepTheirBudget)
{
  addWall();
  ASSERT_TRUE(node_->set_parameter({"geodex.metric", "kinetic_energy"}).successful);
  ASSERT_TRUE(
    node_->set_parameter({"geodex.planner.collision_check_resolution", 0.002}).successful);
  const double budget = 0.25;
  for (int i = 0; i < 10; ++i) {
    auto req = jointRequest(right(), budget);
    req.start_state.joint_state.name =
      model_->getJointModelGroup(kGroup)->getActiveJointModelNames();
    req.start_state.joint_state.position = left();
    req.start_state.is_diff = true;
    const auto t0 = std::chrono::steady_clock::now();
    const auto res = solve(req);
    const double wall =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    ASSERT_EQ(res.error_code.val, MoveItErrorCodes::SUCCESS) << i;
    std::printf(
      "anytime request %d: planning time %.3f s, wall %.3f s\n", i, res.planning_time, wall);
    EXPECT_LT(res.planning_time, budget + 0.06) << i;
  }
}

// PlannerManager::terminate() stops every live context.
TEST_F(PandaPlanning, ManagerTerminateStopsARunningSolve)
{
  addWall();
  auto req = jointRequest(right(), 30.0);
  req.start_state.joint_state.name = model_->getJointModelGroup(kGroup)->getActiveJointModelNames();
  req.start_state.joint_state.position = left();
  req.start_state.is_diff = true;
  moveit_msgs::msg::MoveItErrorCodes code;
  auto context = manager_->getPlanningContext(scene_, req, code);
  ASSERT_TRUE(context);
  planning_interface::MotionPlanResponse res;
  const auto t0 = std::chrono::steady_clock::now();
  std::thread worker([&] { context->solve(res); });
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  manager_->terminate();
  worker.join();
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), 2.0);
}

// With a seed and an iteration budget, the same request returns the same trajectory.
TEST_F(PandaPlanning, SeededRequestsReplay)
{
  addWall();
  ASSERT_TRUE(node_->set_parameter({"geodex.planner.seed", 5}).successful);
  ASSERT_TRUE(node_->set_parameter({"geodex.planner.max_iterations", 400}).successful);
  auto req = jointRequest(right(), 30.0);
  req.start_state.joint_state.name = model_->getJointModelGroup(kGroup)->getActiveJointModelNames();
  req.start_state.joint_state.position = left();
  req.start_state.is_diff = true;
  const auto a = solve(req);
  const auto b = solve(req);
  ASSERT_EQ(a.error_code.val, MoveItErrorCodes::SUCCESS);
  ASSERT_EQ(b.error_code.val, MoveItErrorCodes::SUCCESS);
  ASSERT_EQ(a.trajectory->getWayPointCount(), b.trajectory->getWayPointCount());
  for (std::size_t w = 0; w < a.trajectory->getWayPointCount(); ++w) {
    expectNear(
      geodex_moveit::configuration(
        a.trajectory->getWayPoint(w), *model_->getJointModelGroup(kGroup)),
      geodex_moveit::configuration(
        b.trajectory->getWayPoint(w), *model_->getJointModelGroup(kGroup)),
      0.0);
  }
}

std::shared_ptr<planning_interface::PlannerManager> loadManager(
  pluginlib::ClassLoader<planning_interface::PlannerManager> & loader,
  const moveit::core::RobotModelConstPtr & model, const std::vector<rclcpp::Parameter> & params,
  bool & initialized)
{
  static int count = 0;
  auto node = std::make_shared<rclcpp::Node>(
    "geodex_moveit_model_test_" + std::to_string(count++),
    rclcpp::NodeOptions().parameter_overrides(params).start_parameter_services(false));
  auto manager = loader.createSharedInstance(kPlugin);
  initialized = manager->initialize(model, node, "geodex");
  return manager;
}

// Another seven-joint robot is not planned with the Panda's model, mass matrix and heuristic.
TEST(PandaPlanningModel, OtherSevenJointRobotIsRefused)
{
  namespace fs = std::filesystem;
  const fs::path fixtures = GEODEX_MOVEIT_REPLAY_FIXTURES;
  auto urdf = urdf::parseURDF(readFile((fixtures / "fr3_arm_gripper.urdf").string()));
  auto srdf = std::make_shared<srdf::Model>();
  srdf->initString(*urdf, readFile((fixtures / "fr3_arm_gripper.srdf").string()));
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  pluginlib::ClassLoader<planning_interface::PlannerManager> loader(
    "moveit_core", "planning_interface::PlannerManager");
  bool initialized = false;
  auto manager = loadManager(
    loader, model, {{"geodex.robot", "panda"}, {"geodex.metric", "kinetic_energy"}}, initialized);
  EXPECT_FALSE(initialized) << "no FR3 group has the Panda's joints";
  planning_interface::MotionPlanRequest req;
  req.group_name = "fr3_arm";
  if (initialized) EXPECT_FALSE(manager->canServiceRequest(req));
  auto fr3 = loadManager(loader, model, {{"geodex.robot", "fr3_arm_gripper"}}, initialized);
  EXPECT_TRUE(initialized);
  EXPECT_TRUE(fr3->canServiceRequest(req));
}

// Without a robot model, a group with a planar joint is refused, and its arm-only group is served.
TEST(PandaPlanningModel, PlanarJointGroupIsRefused)
{
  moveit::core::RobotModelBuilder builder("mobile", "base");
  builder.addVirtualJoint("world", "base", "planar", "base_joint");
  builder.addChain("base->link1", "revolute");
  builder.addCollisionBox("base", {0.2, 0.2, 0.2}, geometry_msgs::msg::Pose());
  builder.addGroup({}, {"base_joint", "base-link1-joint"}, "whole");
  builder.addGroup({}, {"base-link1-joint"}, "arm");
  ASSERT_TRUE(builder.isValid());
  const auto model = builder.build();
  pluginlib::ClassLoader<planning_interface::PlannerManager> loader(
    "moveit_core", "planning_interface::PlannerManager");
  bool initialized = false;
  auto manager =
    loadManager(loader, model, {{"geodex.robot", ""}, {"geodex.checker", "moveit"}}, initialized);
  ASSERT_TRUE(initialized);
  planning_interface::MotionPlanRequest req;
  req.group_name = "whole";
  EXPECT_FALSE(manager->canServiceRequest(req));
  req.group_name = "arm";
  EXPECT_TRUE(manager->canServiceRequest(req));
}

/// Configurations no request can plan with fail the plugin load.
TEST(PandaPlanningLoad, InvalidConfigurationFailsInitialize)
{
  const std::vector<std::vector<rclcpp::Parameter>> bad{
    {{"geodex.robot", ""}, {"geodex.checker", "vamp"}},
    {{"geodex.robot", ""}, {"geodex.checker", "moveit"}, {"geodex.metric", "kinetic_energy"}},
    {{"geodex.robot", "no_such_robot"}},
    {{"geodex.attached_object.box_size", std::vector<double>{0.1, 0.2}}},
    // The Panda's collision model attaches attached objects to panda_grasptarget, not panda_hand.
    {{"geodex.robot", "panda"}, {"geodex.end_effector_link", "panda_hand"}},
    {{"geodex.robot", "panda"}, {"geodex.end_effector_offset", kGraspTargetInHand}},
    {{"geodex.robot", "panda"},
     {"geodex.end_effector_link", "panda_hand"},
     {"geodex.end_effector_offset", std::vector<double>{0.0, 0.0, 0.1}}},
  };
  int count = 0;
  for (const auto & params : bad) {
    auto overrides = robotParameters();
    overrides.insert(overrides.end(), params.begin(), params.end());
    auto node = std::make_shared<rclcpp::Node>(
      "geodex_moveit_load_test_" + std::to_string(count++),
      rclcpp::NodeOptions().parameter_overrides(overrides).start_parameter_services(false));
    robot_model_loader::RobotModelLoader loader(node, "robot_description", false);
    pluginlib::ClassLoader<planning_interface::PlannerManager> cl(
      "moveit_core", "planning_interface::PlannerManager");
    auto manager = cl.createSharedInstance(kPlugin);
    EXPECT_FALSE(manager->initialize(loader.getModel(), node, "geodex")) << count;
  }
}

}  // namespace
