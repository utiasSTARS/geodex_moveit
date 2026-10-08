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

// Runs with the example file passed as --params-file and checks that every value reaches Params.

#include <memory>
#include <string>
#include <vector>

#include "geodex_moveit/geodex_moveit_parameters.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace
{

void leaves(
  const YAML::Node & node, const std::string & prefix,
  std::vector<std::pair<std::string, YAML::Node>> & out)
{
  for (const auto & kv : node) {
    const std::string key = prefix + kv.first.as<std::string>();
    if (kv.second.IsMap()) {
      leaves(kv.second, key + ".", out);
    } else {
      out.emplace_back(key, kv.second);
    }
  }
}

TEST(ParameterFile, EveryValueReachesTheListener)
{
  auto node = std::make_shared<rclcpp::Node>("geodex_moveit_parameter_file_test");
  geodex_moveit::ParamListener listener(node, "geodex");
  const auto params = listener.get_params();
  std::vector<std::pair<std::string, YAML::Node>> expected;
  leaves(YAML::LoadFile(GEODEX_MOVEIT_EXAMPLE)["/**"]["ros__parameters"], "", expected);
  ASSERT_FALSE(expected.empty());
  for (const auto & [name, value] : expected) {
    const auto p = node->get_parameter(name);
    switch (p.get_type()) {
      case rclcpp::ParameterType::PARAMETER_STRING:
        EXPECT_EQ(p.as_string(), value.as<std::string>()) << name;
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        EXPECT_EQ(p.as_double(), value.as<double>()) << name;
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        EXPECT_EQ(p.as_int(), value.as<int64_t>()) << name;
        break;
      case rclcpp::ParameterType::PARAMETER_BOOL:
        EXPECT_EQ(p.as_bool(), value.as<bool>()) << name;
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
        EXPECT_EQ(p.as_double_array(), value.as<std::vector<double>>()) << name;
        break;
      default:
        ADD_FAILURE() << name << " has an unexpected type";
    }
  }
  EXPECT_EQ(params.robot, "panda");
  EXPECT_EQ(params.planner.config.grrtstar.max_neighbors, 64);
  EXPECT_EQ(params.attached_object.box_pose.size(), 6u);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
