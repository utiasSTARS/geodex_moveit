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

#include "geodex_moveit/request_setup.hpp"

#include <exception>

#include "geodex_moveit/attached_box_test.hpp"
#include "geodex_moveit/composite_validity.hpp"
#include "geodex_moveit/moveit_collision.hpp"
#include "geodex_moveit/scene_translation.hpp"
#include "rclcpp/logging.hpp"

namespace geodex_moveit
{
namespace
{

rclcpp::Logger logger() { return rclcpp::get_logger("geodex_moveit"); }

/// Pose from x, y, z, roll, pitch, yaw, the rotation applied as yaw, then pitch, then roll.
Eigen::Isometry3d poseFromXyzRpy(const std::vector<double> & p)
{
  return Eigen::Translation3d(p[0], p[1], p[2]) *
         Eigen::AngleAxisd(p[5], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(p[4], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(p[3], Eigen::Vector3d::UnitX());
}

std::string joinNames(const std::vector<std::string> & v)
{
  std::string out;
  for (const auto & s : v) out += (out.empty() ? "" : ", ") + s;
  return out;
}

}  // namespace

PlannerSettings plannerSettings(const Params & p)
{
  PlannerSettings s;
  const auto & g = p.planner.config.grrtstar;
  s.grrtstar_range = g.range;
  s.greedy_biasing_ratio = g.greedy_biasing_ratio;
  s.rewire_factor = g.rewire_factor;
  s.max_neighbors = g.max_neighbors;
  s.greedy_cost_for_tree_pruning = g.greedy_cost_for_tree_pruning;
  s.stop_at_first_solution = p.planner.termination == "first";
  s.collision_check_resolution = p.planner.collision_check_resolution;
  s.edge_motion_validator = p.planner.edge_motion_validator;
  s.max_iterations = p.planner.max_iterations;
  s.seed = p.planner.seed;
  const auto & m = p.smoothing;
  s.smooth = m.enabled;
  s.output_spacing = m.output_spacing;
  s.round_corners = m.round_corners;
  s.smoothing_seed = static_cast<std::uint64_t>(m.seed);
  s.smoothing_time_fraction = m.time_fraction;
  return s;
}

std::shared_ptr<const ValidityOracle> buildValidity(
  const planning_scene::PlanningSceneConstPtr & scene, const moveit::core::RobotState & start,
  const moveit::core::JointModelGroup & group, const Params & params, EnvironmentCache & cache,
  std::string & message, const bool with_margin)
{
  const double resolution = params.planner.collision_check_resolution;
  if (params.checker == "moveit") {
    const double margin = with_margin ? smoothingMargin(params.robot, {}, resolution) : 0.0;
    return std::make_shared<MoveItCollisionOracle>(
      scene, start, group, params.obstacle_padding + margin);
  }
  if (!hasCollisionModel(params.robot)) {
    message = "checker 'vamp' needs a geodex collision model; set 'robot' or use checker 'moveit'";
    return nullptr;
  }

  TranslationOptions options;
  options.base_link = params.base_link;
  options.end_effector_link = params.end_effector_link.empty() ? modelEndEffectorFrame(params.robot)
                                                               : params.end_effector_link;
  if (params.end_effector_offset.size() == 6) {
    options.end_effector_offset = poseFromXyzRpy(params.end_effector_offset);
  }
  options.attachment_padding = params.attachment_padding;
  options.want_attached_box = params.exact_attached_box;
  options.attached_box_link = params.attached_object.link;
  options.attached_box_size = params.attached_object.box_size;
  options.attached_box_pose = params.attached_object.box_pose;
  options.attached_spheres = params.attached_object.spheres;
  const TranslatedScene translated = translateScene(*scene, start, group, options);
  if (!translated.error.empty()) {
    message = translated.error;
    return nullptr;
  }
  if (!translated.unsupported.empty()) {
    if (!params.ignore_unsupported_shapes) {
      message = "the vamp checker cannot represent: " + joinNames(translated.unsupported) +
                "; use checker 'moveit' or set ignore_unsupported_shapes";
      return nullptr;
    }
    if (const auto fresh = cache.unseen(translated.unsupported); !fresh.empty()) {
      RCLCPP_WARN(logger(), "planning without: %s", joinNames(fresh).c_str());
    }
  }
  if (const auto fresh = cache.unseen(translated.bounded); !fresh.empty()) {
    RCLCPP_WARN(
      logger(), "checked as their oriented bounding boxes, which may block free space: %s",
      joinNames(fresh).c_str());
  }

  const double margin =
    with_margin ? smoothingMargin(params.robot, translated.scene.attached_spheres, resolution)
                : 0.0;
  CollisionScene collision_scene = translated.scene;
  collision_scene.obstacle_padding = params.obstacle_padding + margin;
  std::shared_ptr<const ValidityOracle> collision;
  try {
    collision = cache.get(params.robot, collision_scene)->makeOracle();
  } catch (const std::exception & e) {
    message = e.what();
    return nullptr;
  }
  if (!translated.attached_box) return collision;
  auto exact = std::make_shared<AttachedBoxTest>(
    start, group, *translated.attached_box, translated.model_frame_boxes,
    params.exact_attached_box_margin + margin);
  return std::make_shared<CompositeValidity>(collision, exact);
}

}  // namespace geodex_moveit
