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

// Replays recorded smoother inputs of the FR3 shelf demo through the plugin's own
// validity and the backend's smoother. Each fixture holds a planner's path, the planning scene,
// the settings and the smoother's output from the build that recorded it. The attached box's sphere
// cover is attached_spheres.csv, one sphere per row as x, y, z and radius in 2f85_tcp.
//
// Every run checks that the output keeps the endpoints, passes the smoother's own check, and is
// valid and inside the joint bounds at every waypoint. GEODEX_MOVEIT_REPLAY_EXACT=1 adds the
// value-by-value comparison with the recording, which holds only for the recording's geodex, Eigen
// version, compiler and CPU. GEODEX_MOVEIT_REPLAY_OUT names a directory for the outputs, one
// subdirectory per fixture.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geodex_moveit/backend.hpp"
#include "geodex_moveit/environment_cache.hpp"
#include "geodex_moveit/group_state.hpp"
#include "geodex_moveit/request_setup.hpp"
#include "gtest/gtest.h"
#include "moveit/planning_scene/planning_scene.hpp"
#include "moveit/robot_model/robot_model.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "srdfdom/model.h"
#include "urdf_parser/urdf_parser.h"

namespace fs = std::filesystem;

namespace
{

const fs::path kFixtures = GEODEX_MOVEIT_REPLAY_FIXTURES;

std::string slurp(const fs::path & p)
{
  std::ifstream in(p, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::map<std::string, std::string> readMeta(const fs::path & p)
{
  std::map<std::string, std::string> m;
  std::ifstream in(p);
  std::string line;
  while (std::getline(in, line)) {
    const auto c = line.find(": ");
    if (c != std::string::npos) m[line.substr(0, c)] = line.substr(c + 2);
  }
  return m;
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

void writeRows(const fs::path & p, const std::vector<std::vector<double>> & rows)
{
  std::FILE * f = std::fopen(p.c_str(), "w");
  for (const auto & r : rows) {
    for (std::size_t j = 0; j < r.size(); ++j) std::fprintf(f, j ? ",%.17g" : "%.17g", r[j]);
    std::fprintf(f, "\n");
  }
  std::fclose(f);
}

moveit_msgs::msg::PlanningScene readScene(const fs::path & p)
{
  const std::string bytes = slurp(p);
  rclcpp::SerializedMessage sm(bytes.size());
  auto & raw = sm.get_rcl_serialized_message();
  std::copy(bytes.begin(), bytes.end(), reinterpret_cast<char *>(raw.buffer));
  raw.buffer_length = bytes.size();
  moveit_msgs::msg::PlanningScene msg;
  rclcpp::Serialization<moveit_msgs::msg::PlanningScene>().deserialize_message(&sm, &msg);
  return msg;
}

/// The recorded settings in the backend's terms, every other field at its default.
geodex_moveit::PlannerSettings settingsFrom(const std::map<std::string, std::string> & m)
{
  geodex_moveit::PlannerSettings s;
  s.collision_check_resolution = std::stod(m.at("collision_check_resolution"));
  s.output_spacing = std::stod(m.at("output_spacing"));
  if (const auto it = m.find("round_corners"); it != m.end()) {
    s.round_corners = it->second == "1";
  }
  return s;
}

class SmootherReplay : public ::testing::TestWithParam<std::string>
{
};

TEST_P(SmootherReplay, OutputMatchesTheRecording)
{
  const fs::path dir = kFixtures / GetParam();
  const auto meta = readMeta(dir / "meta.yaml");

  auto urdf = urdf::parseURDF(slurp(kFixtures / "fr3_arm_gripper.urdf"));
  auto srdf = std::make_shared<srdf::Model>();
  srdf->initString(*urdf, slurp(kFixtures / "fr3_arm_gripper.srdf"));
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  scene->setPlanningSceneMsg(readScene(dir / "scene.bin"));
  const auto & group = *model->getJointModelGroup(meta.at("group"));

  geodex_moveit::Params params;
  params.robot = meta.at("robot");
  params.metric = meta.at("metric");
  params.checker = meta.at("checker");
  params.end_effector_link = "2f85_tcp";
  params.exact_attached_box = meta.at("exact_attached_box") == "1";
  params.exact_attached_box_margin = std::stod(meta.at("exact_attached_box_margin"));
  params.attachment_padding = std::stod(meta.at("attachment_padding"));
  params.obstacle_padding = std::stod(meta.at("obstacle_padding"));
  for (const auto & row : readRows(kFixtures / "attached_spheres.csv")) {
    params.attached_object.spheres.insert(
      params.attached_object.spheres.end(), row.begin(), row.end());
  }

  params.planner.collision_check_resolution = std::stod(meta.at("collision_check_resolution"));

  geodex_moveit::EnvironmentCache cache;
  std::string message;
  const auto validity =
    geodex_moveit::buildValidity(scene, scene->getCurrentState(), group, params, cache, message);
  ASSERT_TRUE(validity) << message;
  // The smoother keeps the margin of its evenly spaced waypoints, and every returned waypoint
  // passes the request's own validity.
  const auto planning = geodex_moveit::buildValidity(
    scene, scene->getCurrentState(), group, params, cache, message, true);
  ASSERT_TRUE(planning) << message;
  const auto backend = geodex_moveit::PlannerBackend::create(
    params.robot, params.metric, geodex_moveit::configurationSize(group));
  geodex_moveit::SmoothRequest request;
  geodex_moveit::configurationBounds(group, request.lower, request.upper);
  request.path = readRows(dir / "raw.csv");
  request.planning_validity = planning.get();
  const auto result = backend->smooth(request, *validity, settingsFrom(meta));

  if (const char * out = std::getenv("GEODEX_MOVEIT_REPLAY_OUT")) {
    const fs::path o = fs::path(out) / GetParam();
    fs::create_directories(o);
    writeRows(o / "path.csv", result.path);
    char length[32];
    std::snprintf(length, sizeof(length), "%.17g", result.length);
    std::ofstream(o / "result.yaml")
      << "collision_free: " << (result.collision_free ? "true" : "false") << "\n"
      << "stage: " << result.stage << "\n"
      << "length: " << length << "\n";
  }

  const auto & raw = request.path;
  ASSERT_FALSE(result.path.empty());
  EXPECT_EQ(result.path.front(), raw.front());
  EXPECT_EQ(result.path.back(), raw.back());
  EXPECT_TRUE(result.collision_free) << "stage " << result.stage;
  for (std::size_t i = 0; i < result.path.size(); ++i) {
    EXPECT_TRUE(validity->isValid(result.path[i].data())) << "waypoint " << i;
    for (std::size_t j = 0; j < result.path[i].size(); ++j) {
      EXPECT_GE(result.path[i][j], request.lower[j]) << "waypoint " << i << ", joint " << j;
      EXPECT_LE(result.path[i][j], request.upper[j]) << "waypoint " << i << ", joint " << j;
    }
  }

  const char * exact = std::getenv("GEODEX_MOVEIT_REPLAY_EXACT");
  if (exact == nullptr || std::string(exact) != "1") return;
  const auto expected = readMeta(dir / "result.yaml");
  EXPECT_EQ(result.collision_free, expected.at("collision_free") == "true");
  EXPECT_EQ(result.stage, std::stoi(expected.at("stage")));
  // The recording holds every double at round-trip precision.
  EXPECT_EQ(result.length, std::strtod(expected.at("length").c_str(), nullptr));
  EXPECT_EQ(result.path, readRows(dir / "path.csv"));
}

std::vector<std::string> fixtures()
{
  std::vector<std::string> names;
  for (const char * metric : {"euclidean", "kinetic_energy"}) {
    for (const auto & e : fs::directory_iterator(kFixtures / metric)) {
      names.push_back(std::string(metric) + "/" + e.path().filename().string());
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

INSTANTIATE_TEST_SUITE_P(
  Fr3Shelf, SmootherReplay, ::testing::ValuesIn(fixtures()), [](const auto & info) {
    std::string n = info.param;
    std::replace(n.begin(), n.end(), '/', '_');
    return n;
  });

}  // namespace
