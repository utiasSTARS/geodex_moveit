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

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "geodex_moveit/geodex_moveit_parameters.hpp"
#include "geodex_moveit/request_setup.hpp"
#include "geodex_moveit/scene_translation.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace
{

/// Every parameter of the schema, as dotted name to its declaration.
void schemaLeaves(
  const YAML::Node & node, const std::string & prefix, std::map<std::string, YAML::Node> & out)
{
  for (const auto & kv : node) {
    const std::string key = prefix + kv.first.as<std::string>();
    if (kv.second.IsMap() && kv.second["type"] && kv.second["type"].IsScalar()) {
      out[key] = kv.second;
    } else if (kv.second.IsMap()) {
      schemaLeaves(kv.second, key + ".", out);
    }
  }
}

/// Every scalar or sequence key of a configuration file, as dotted name.
void configKeys(const YAML::Node & node, const std::string & prefix, std::set<std::string> & out)
{
  for (const auto & kv : node) {
    const std::string key = prefix + kv.first.as<std::string>();
    if (kv.second.IsMap()) {
      configKeys(kv.second, key + ".", out);
    } else {
      out.insert(key);
    }
  }
}

std::map<std::string, YAML::Node> schema()
{
  std::map<std::string, YAML::Node> leaves;
  schemaLeaves(YAML::LoadFile(GEODEX_MOVEIT_SCHEMA)["geodex_moveit"], "", leaves);
  return leaves;
}

std::shared_ptr<rclcpp::Node> makeNode(const std::vector<rclcpp::Parameter> & overrides = {})
{
  static int count = 0;
  return std::make_shared<rclcpp::Node>(
    "geodex_moveit_parameter_test_" + std::to_string(count++),
    rclcpp::NodeOptions().parameter_overrides(overrides).start_parameter_services(false));
}

TEST(Parameters, DeclaredSetEqualsSchema)
{
  auto node = makeNode();
  geodex_moveit::ParamListener listener(node, "geodex");
  std::set<std::string> declared;
  for (const auto & name : node->list_parameters({"geodex"}, 0).names) {
    declared.insert(name.substr(std::string("geodex.").size()));
  }
  std::set<std::string> expected;
  for (const auto & [name, decl] : schema()) expected.insert(name);
  EXPECT_EQ(declared, expected);
}

TEST(Parameters, DefaultsComeFromTheSchema)
{
  auto node = makeNode();
  geodex_moveit::ParamListener listener(node, "geodex");
  for (const auto & [name, decl] : schema()) {
    const auto p = node->get_parameter("geodex." + name);
    const std::string type = decl["type"].as<std::string>();
    const YAML::Node d = decl["default_value"];
    if (type == "string") {
      EXPECT_EQ(p.as_string(), d.as<std::string>()) << name;
    } else if (type == "double") {
      EXPECT_EQ(p.as_double(), d.as<double>()) << name;
    } else if (type == "int") {
      EXPECT_EQ(p.as_int(), d.as<int64_t>()) << name;
    } else if (type == "bool") {
      EXPECT_EQ(p.as_bool(), d.as<bool>()) << name;
    } else if (type == "double_array") {
      EXPECT_EQ(p.as_double_array(), d.as<std::vector<double>>()) << name;
    } else {
      ADD_FAILURE() << name << " has unhandled type " << type;
    }
  }
}

/// The member defaults of the backend settings and of the translation options equal the schema's
/// defaults.
TEST(Parameters, MemberDefaultsEqualTheSchema)
{
  auto node = makeNode();
  const auto params = geodex_moveit::ParamListener(node, "geodex").get_params();
  const geodex_moveit::PlannerSettings from = geodex_moveit::plannerSettings(params);
  const geodex_moveit::PlannerSettings member;
  EXPECT_EQ(from.grrtstar_range, member.grrtstar_range);
  EXPECT_EQ(from.greedy_biasing_ratio, member.greedy_biasing_ratio);
  EXPECT_EQ(from.rewire_factor, member.rewire_factor);
  EXPECT_EQ(from.max_neighbors, member.max_neighbors);
  EXPECT_EQ(from.greedy_cost_for_tree_pruning, member.greedy_cost_for_tree_pruning);
  EXPECT_EQ(from.stop_at_first_solution, member.stop_at_first_solution);
  EXPECT_EQ(from.collision_check_resolution, member.collision_check_resolution);
  EXPECT_EQ(from.edge_motion_validator, member.edge_motion_validator);
  EXPECT_EQ(from.max_iterations, member.max_iterations);
  EXPECT_EQ(from.seed, member.seed);
  EXPECT_EQ(from.smooth, member.smooth);
  EXPECT_EQ(from.output_spacing, member.output_spacing);
  EXPECT_EQ(from.smoothing_seed, member.smoothing_seed);
  EXPECT_EQ(from.smoothing_time_fraction, member.smoothing_time_fraction);
  const geodex_moveit::TranslationOptions options;
  EXPECT_EQ(options.base_link, params.base_link);
  EXPECT_EQ(options.end_effector_link, params.end_effector_link);
  EXPECT_EQ(options.attachment_padding, params.attachment_padding);
  EXPECT_EQ(options.want_attached_box, params.exact_attached_box);
  EXPECT_EQ(options.attached_box_link, params.attached_object.link);
  EXPECT_EQ(options.attached_box_size, params.attached_object.box_size);
  EXPECT_EQ(options.attached_box_pose, params.attached_object.box_pose);
}

TEST(Parameters, InvalidValuesAreRejectedAtLoad)
{
  const std::vector<rclcpp::Parameter> bad{
    {"geodex.metric", "no_such_metric"},
    {"geodex.checker", "no_such_checker"},
    {"geodex.planner.config.grrtstar.range", -1.0},
    {"geodex.planner.config.grrtstar.greedy_biasing_ratio", 1.5},
    {"geodex.planner.termination", "sometimes"},
    {"geodex.planner.collision_check_resolution", 0.0},
    {"geodex.planner.seed", int64_t{-2}},
    {"geodex.smoothing.time_fraction", 0.95},
    {"geodex.end_effector_offset", std::vector<double>{0, 0, 0, 0, 0, 0, 0}},
    {"geodex.attached_object.box_size", std::vector<double>{0.1, 0.2, 0.3, 0.4}},
    {"geodex.attached_object.box_pose", std::vector<double>{0, 0, 0, 0, 0, 0, 0}},
    {"geodex.smoothing.seed", int64_t{-1}},
  };
  for (const auto & p : bad) {
    auto node = makeNode({p});
    EXPECT_ANY_THROW(geodex_moveit::ParamListener(node, "geodex")) << p.get_name();
  }
}

TEST(Parameters, InvalidValuesAreRejectedAtRuntime)
{
  auto node = makeNode();
  geodex_moveit::ParamListener listener(node, "geodex");
  EXPECT_FALSE(node->set_parameter({"geodex.metric", "no_such_metric"}).successful);
  EXPECT_TRUE(node->set_parameter({"geodex.robot", "panda"}).successful);
  EXPECT_TRUE(node->set_parameter({"geodex.metric", "kinetic_energy"}).successful);
  const auto params = listener.get_params();
  EXPECT_EQ(params.robot, "panda");
  EXPECT_EQ(params.metric, "kinetic_energy");
}

TEST(Parameters, DeclaredSetEqualsExampleFile)
{
  auto node = makeNode();
  geodex_moveit::ParamListener listener(node, "geodex");
  std::set<std::string> declared;
  for (const auto & name : node->list_parameters({"geodex"}, 0).names) declared.insert(name);
  std::set<std::string> example;
  configKeys(YAML::LoadFile(GEODEX_MOVEIT_EXAMPLE)["/**"]["ros__parameters"], "", example);
  EXPECT_EQ(declared, example);
}

/// Configuration files may only set MoveIt pipeline keys and parameters of the schema.
TEST(Parameters, ConfigurationFilesUseDeclaredNames)
{
  const std::set<std::string> pipeline{"planning_plugins", "request_adapters", "response_adapters"};
  const auto leaves = schema();
  const std::vector<std::string> files{GEODEX_MOVEIT_PIPELINE_CONFIG};
  for (const auto & file : files) {
    std::set<std::string> keys;
    configKeys(YAML::LoadFile(file), "", keys);
    for (const auto & key : keys) {
      const bool ok =
        pipeline.count(key) > 0 || key.rfind("totg.", 0) == 0 || leaves.count(key) > 0;
      EXPECT_TRUE(ok) << file << ": '" << key << "' is not a declared parameter";
    }
  }
}

}  // namespace
