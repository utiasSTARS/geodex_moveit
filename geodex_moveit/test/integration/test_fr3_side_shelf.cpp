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

// Plans the FR3 with the attached box from the middle compartment to the top compartment of a shelf
// at the robot's right, under the kinetic-energy metric and the FR3 datasheet joint limits. The
// endpoints hold joint 4 near -2.8 and -2.0 and joint 6 near 3.9 and 3.0. The backend plans with
// the plugin's own validity, the vamp checker with the box's sphere cover and the exact
// attached-box test. The robot files and the sphere cover come from the replay fixtures, and the
// scene from fixtures/fr3_side_shelf.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geodex_moveit/backend.hpp"
#include "geodex_moveit/environment_cache.hpp"
#include "geodex_moveit/group_state.hpp"
#include "geodex_moveit/request_setup.hpp"
#include "gtest/gtest.h"
#include "moveit/planning_scene/planning_scene.hpp"
#include "moveit/robot_model/robot_model.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "srdfdom/model.h"
#include "urdf_parser/urdf_parser.h"
#include "yaml-cpp/yaml.h"

namespace fs = std::filesystem;

namespace
{

using geodex_moveit::PlanResult;

const fs::path kReplay = GEODEX_MOVEIT_REPLAY_FIXTURES;
const fs::path kScene = GEODEX_MOVEIT_SIDE_SHELF_SCENE;

// FR3 datasheet joint position limits in rad.
const std::vector<double> kDatasheetLower{-2.7437, -1.7837, -2.9007, -3.0421,
                                          -2.8065, 0.5445,  -3.0159};
const std::vector<double> kDatasheetUpper{2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159};

// The box in the middle compartment and in the top compartment, 0.09 m behind the compartment's
// front face, 0.015 m above its floor and upright.
const std::vector<double> kMiddle{
  -1.567123132193691,    -0.5229658048864806, -0.002396875359219103, -2.8227860476223086,
  0.0023968945051037844, 3.8706211510349786,  -0.002984782184914447};
const std::vector<double> kTop{-1.5711737145832498,   -0.6047508165412793,   0.0003163356450217851,
                               -2.02151320818307,     0.0007636032126596944, 2.987558718708925,
                               -0.0005747074374970973};

std::string slurp(const fs::path & p)
{
  std::ifstream in(p, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::vector<double> attachedSpheres()
{
  std::vector<double> flat;
  std::ifstream in(kReplay / "attached_spheres.csv");
  std::string line;
  while (std::getline(in, line)) {
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, ',')) flat.push_back(std::strtod(tok.c_str(), nullptr));
  }
  return flat;
}

geometry_msgs::msg::Pose pose(const YAML::Node & n)
{
  const auto p = n["position"].as<std::vector<double>>();
  const auto o = n["orientation"].as<std::vector<double>>();
  geometry_msgs::msg::Pose out;
  out.position.x = p[0];
  out.position.y = p[1];
  out.position.z = p[2];
  out.orientation.x = o[0];
  out.orientation.y = o[1];
  out.orientation.z = o[2];
  out.orientation.w = o[3];
  return out;
}

/// A collision object of box primitives from one entry of the scene file.
moveit_msgs::msg::CollisionObject object(const YAML::Node & e)
{
  moveit_msgs::msg::CollisionObject o;
  o.id = e["id"].as<std::string>();
  o.header.frame_id = e["frame_id"].as<std::string>();
  o.pose = pose(e["pose"]);
  o.operation = moveit_msgs::msg::CollisionObject::ADD;
  for (const auto & prim : e["primitives"]) {
    shape_msgs::msg::SolidPrimitive sp;
    sp.type = shape_msgs::msg::SolidPrimitive::BOX;
    for (const double v : prim["dimensions"].as<std::vector<double>>()) sp.dimensions.push_back(v);
    o.primitives.push_back(sp);
    o.primitive_poses.push_back(pose(prim));
  }
  return o;
}

/// Forwards to another oracle and logs the configuration of every point check in call order.
class SequenceOracle : public geodex_moveit::ValidityOracle
{
public:
  explicit SequenceOracle(std::shared_ptr<const ValidityOracle> inner) : inner_(std::move(inner)) {}
  bool isValid(const double * q) const override
  {
    log_.emplace_back(q, q + inner_->dimension());
    return inner_->isValid(q);
  }
  int batchWidth() const override { return inner_->batchWidth(); }
  bool allValid(const double * q, int count) const override { return inner_->allValid(q, count); }
  int dimension() const override { return inner_->dimension(); }
  const std::vector<std::vector<double>> & log() const { return log_; }

private:
  std::shared_ptr<const ValidityOracle> inner_;
  mutable std::vector<std::vector<double>> log_;
};

/// The densified path that plan() checked last, the point checks from the last check of the start
/// onwards. Checks that it runs from @p path's start to its goal through every waypoint of @p path
/// in order, along the chords between them, with at most @p resolution between neighbors.
std::vector<std::vector<double>> densifiedPath(
  const std::vector<std::vector<double>> & path, const std::vector<std::vector<double>> & log,
  double resolution)
{
  std::vector<std::vector<double>> dense;
  const auto start = std::find(log.rbegin(), log.rend(), path.front());
  if (start == log.rend()) {
    ADD_FAILURE() << "the start was never checked";
    return dense;
  }
  dense.assign(start.base() - 1, log.end());
  EXPECT_EQ(dense.back(), path.back());
  std::size_t edge = 0;
  for (std::size_t k = 0; k < dense.size(); ++k) {
    if (edge + 1 < path.size() && dense[k] == path[edge + 1]) ++edge;
    if (k > 0) {
      double d2 = 0.0;
      for (std::size_t j = 0; j < dense[k].size(); ++j) {
        d2 += (dense[k][j] - dense[k - 1][j]) * (dense[k][j] - dense[k - 1][j]);
      }
      EXPECT_LE(std::sqrt(d2), resolution * (1.0 + 1e-9)) << "densified state " << k;
    }
    if (edge + 1 < path.size() && path[edge] != path[edge + 1]) {
      // The state lies on the chord from waypoint edge to waypoint edge + 1.
      const auto & a = path[edge];
      const auto & b = path[edge + 1];
      double ab2 = 0.0;
      double t = 0.0;
      for (std::size_t j = 0; j < a.size(); ++j) {
        ab2 += (b[j] - a[j]) * (b[j] - a[j]);
        t += (dense[k][j] - a[j]) * (b[j] - a[j]);
      }
      t /= ab2;
      double off2 = 0.0;
      for (std::size_t j = 0; j < a.size(); ++j) {
        const double e = dense[k][j] - (a[j] + t * (b[j] - a[j]));
        off2 += e * e;
      }
      EXPECT_LE(std::sqrt(off2), 1e-9) << "densified state " << k << " off edge " << edge;
    }
  }
  EXPECT_EQ(edge + 1, path.size()) << "the densified path skips a waypoint";
  return dense;
}

class Fr3SideShelf : public ::testing::Test
{
protected:
  void SetUp() override
  {
    auto urdf = urdf::parseURDF(slurp(kReplay / "fr3_arm_gripper.urdf"));
    auto srdf = std::make_shared<srdf::Model>();
    srdf->initString(*urdf, slurp(kReplay / "fr3_arm_gripper.srdf"));
    model_ = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    scene_->getCurrentStateNonConst().setToDefaultValues();
    const YAML::Node doc = YAML::LoadFile(kScene.string());
    for (const auto & e : doc["world"]) ASSERT_TRUE(scene_->processCollisionObjectMsg(object(e)));
    for (const auto & e : doc["attached"]) {
      moveit_msgs::msg::AttachedCollisionObject att;
      att.link_name = e["link"].as<std::string>();
      att.touch_links = e["touch_links"].as<std::vector<std::string>>();
      att.object = object(e);
      ASSERT_TRUE(scene_->processAttachedCollisionObjectMsg(att));
    }
    group_ = model_->getJointModelGroup("fr3_arm");
    ASSERT_NE(group_, nullptr);

    params_.robot = "fr3_arm_gripper";
    params_.metric = "kinetic_energy";
    params_.end_effector_link = "2f85_tcp";
    params_.attached_object.spheres = attachedSpheres();
    params_.obstacle_padding = 0.000404;
    params_.planner.collision_check_resolution = 0.002;
    params_.planner.config.grrtstar.range = 0.6;
    params_.planner.termination = "first";
    params_.planner.max_iterations = 10000;
    params_.smoothing.time_fraction = 0.3;

    // The plugin plans inside the group's bounds and the robot model's limits, here the datasheet.
    geodex_moveit::configurationBounds(*group_, lower_, upper_);
    std::vector<double> model_lower, model_upper;
    ASSERT_TRUE(geodex_moveit::modelJointBounds(params_.robot, model_lower, model_upper));
    for (std::size_t i = 0; i < lower_.size(); ++i) {
      lower_[i] = std::max({lower_[i], model_lower[i], kDatasheetLower[i]});
      upper_[i] = std::min({upper_[i], model_upper[i], kDatasheetUpper[i]});
    }
  }

  /// The plugin's validity for a request that starts at @p start, with the smoothing margin when
  /// @p with_margin is set.
  std::shared_ptr<const geodex_moveit::ValidityOracle> validity(
    const std::vector<double> & start, bool with_margin = false)
  {
    moveit::core::RobotState state(scene_->getCurrentState());
    geodex_moveit::setConfiguration(state, *group_, start.data());
    state.update();
    std::string message;
    auto v =
      geodex_moveit::buildValidity(scene_, state, *group_, params_, cache_, message, with_margin);
    EXPECT_TRUE(v) << message;
    return v;
  }

  /// Plans middle to top with @p seed and checks the returned and the densified path.
  void planMiddleToTop(std::int64_t seed, bool smooth)
  {
    params_.planner.seed = seed;
    params_.smoothing.enabled = smooth;
    const std::string tag = "seed " + std::to_string(seed) + ", smooth " + std::to_string(smooth);
    const auto backend = geodex_moveit::PlannerBackend::create(
      params_.robot, params_.metric, geodex_moveit::configurationSize(*group_));
    const auto inner = validity(kMiddle);
    ASSERT_TRUE(inner);
    ASSERT_TRUE(inner->isValid(kMiddle.data())) << tag;
    ASSERT_TRUE(inner->isValid(kTop.data())) << tag;
    const SequenceOracle oracle(inner);
    // The planner and the smoother keep the smoothing margin, as in the planning context.
    const auto planning = validity(kMiddle, true);
    ASSERT_TRUE(planning);
    ASSERT_TRUE(planning->isValid(kMiddle.data())) << tag;
    ASSERT_TRUE(planning->isValid(kTop.data())) << tag;
    geodex_moveit::PlanRequest req;
    req.planning_validity = planning.get();
    req.lower = lower_;
    req.upper = upper_;
    req.start = kMiddle;
    req.goal = kTop;
    req.time_budget = 30.0;
    const PlanResult r = backend->plan(req, oracle, geodex_moveit::plannerSettings(params_));
    ASSERT_EQ(r.status, PlanResult::Status::kSolved) << tag << ": " << r.message;
    EXPECT_EQ(r.path.front(), kMiddle) << tag;
    EXPECT_EQ(r.path.back(), kTop) << tag;
    if (!r.smoothed) EXPECT_EQ(r.path.size(), r.raw_waypoints) << tag;
    for (const auto & q : r.path) {
      for (std::size_t j = 0; j < q.size(); ++j) {
        EXPECT_GE(q[j], lower_[j]) << tag << ", joint " << j + 1;
        EXPECT_LE(q[j], upper_[j]) << tag << ", joint " << j + 1;
      }
    }

    const auto dense =
      densifiedPath(r.path, oracle.log(), params_.planner.collision_check_resolution);
    EXPECT_LE(r.path.size(), dense.size()) << tag;
    if (!smooth) EXPECT_GE(dense.size(), 10 * r.path.size()) << tag;
    // A fresh oracle agrees on every densified state.
    const auto check = validity(kMiddle);
    for (std::size_t k = 0; k < dense.size(); ++k) {
      EXPECT_TRUE(check->isValid(dense[k].data())) << tag << ", densified state " << k;
    }
  }

  moveit::core::RobotModelPtr model_;
  planning_scene::PlanningScenePtr scene_;
  const moveit::core::JointModelGroup * group_ = nullptr;
  geodex_moveit::Params params_;
  geodex_moveit::EnvironmentCache cache_;
  std::vector<double> lower_;
  std::vector<double> upper_;
};

// With the search's 10000 iterations, the planner finds a first solution for each of the four
// seeds. The returned path holds the smoother's waypoints, and every state of the densified path is
// valid.
TEST_F(Fr3SideShelf, MiddleToTopSolvesForEverySeed)
{
  for (const std::int64_t seed : {1, 2, 3, 9}) planMiddleToTop(seed, true);
}

// Without smoothing, the returned path holds the planner's waypoints, far fewer than the states of
// the densified path, and every densified state is valid.
TEST_F(Fr3SideShelf, ReturnsThePlannersWaypoints) { planMiddleToTop(1, false); }

}  // namespace
