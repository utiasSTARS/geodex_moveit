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

#include "geodex_moveit/scene_translation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include "geometric_shapes/shapes.h"

namespace geodex_moveit
{
namespace
{

const char * shapeName(shapes::ShapeType type)
{
  switch (type) {
    case shapes::BOX:
      return "box";
    case shapes::SPHERE:
      return "sphere";
    case shapes::CYLINDER:
      return "cylinder";
    case shapes::CONE:
      return "cone";
    case shapes::MESH:
      return "mesh";
    case shapes::PLANE:
      return "plane";
    case shapes::OCTREE:
      return "octree";
    default:
      return "unknown";
  }
}

/// Bounding box of a mesh or cone in its own frame, as center and half extents.
bool localBounds(const shapes::Shape & shape, Eigen::Vector3d & center, Eigen::Vector3d & half)
{
  if (shape.type == shapes::CONE) {
    const auto & c = static_cast<const shapes::Cone &>(shape);
    center.setZero();
    half = Eigen::Vector3d(c.radius, c.radius, 0.5 * c.length);
    return true;
  }
  if (shape.type == shapes::MESH) {
    const auto & m = static_cast<const shapes::Mesh &>(shape);
    if (m.vertex_count == 0) return false;
    Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d hi = -lo;
    for (unsigned int i = 0; i < m.vertex_count; ++i) {
      const Eigen::Vector3d v(m.vertices[3 * i], m.vertices[3 * i + 1], m.vertices[3 * i + 2]);
      lo = lo.cwiseMin(v);
      hi = hi.cwiseMax(v);
    }
    center = 0.5 * (lo + hi);
    half = 0.5 * (hi - lo);
    return true;
  }
  return false;
}

/// Ball with its center in some frame.
struct Ball
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double radius = 0.0;
};

/// The smallest ball centered in @p shape's bounds that contains it, in the shape's frame. Planes
/// and octrees do not have one.
std::optional<Ball> enclosingBall(const shapes::Shape & shape)
{
  switch (shape.type) {
    case shapes::SPHERE:
      return Ball{Eigen::Vector3d::Zero(), static_cast<const shapes::Sphere &>(shape).radius};
    case shapes::BOX: {
      const auto & b = static_cast<const shapes::Box &>(shape);
      return Ball{
        Eigen::Vector3d::Zero(), 0.5 * Eigen::Vector3d(b.size[0], b.size[1], b.size[2]).norm()};
    }
    case shapes::CYLINDER: {
      const auto & c = static_cast<const shapes::Cylinder &>(shape);
      return Ball{Eigen::Vector3d::Zero(), std::hypot(c.radius, 0.5 * c.length)};
    }
    case shapes::CONE: {
      const auto & c = static_cast<const shapes::Cone &>(shape);
      return Ball{Eigen::Vector3d::Zero(), std::hypot(c.radius, 0.5 * c.length)};
    }
    case shapes::MESH: {
      const auto & m = static_cast<const shapes::Mesh &>(shape);
      Eigen::Vector3d center, half;
      if (!localBounds(shape, center, half)) return std::nullopt;
      double r2 = 0.0;
      for (unsigned int i = 0; i < m.vertex_count; ++i) {
        const Eigen::Vector3d v(m.vertices[3 * i], m.vertices[3 * i + 1], m.vertices[3 * i + 2]);
        r2 = std::max(r2, (v - center).squaredNorm());
      }
      return Ball{center, std::sqrt(r2)};
    }
    default:
      return std::nullopt;
  }
}

/// The smallest ball that contains @p a and @p b.
Ball mergeBalls(const Ball & a, const Ball & b)
{
  const double d = (b.center - a.center).norm();
  if (d + b.radius <= a.radius) return a;
  if (d + a.radius <= b.radius) return b;
  const double r = 0.5 * (d + a.radius + b.radius);
  return Ball{a.center + (b.center - a.center) * ((r - a.radius) / d), r};
}

std::array<double, 3> arr(const Eigen::Vector3d & v) { return {v.x(), v.y(), v.z()}; }

std::array<double, 9> arr(const Eigen::Matrix3d & m)
{
  return {m(0, 0), m(0, 1), m(0, 2), m(1, 0), m(1, 1), m(1, 2), m(2, 0), m(2, 1), m(2, 2)};
}

SceneBox makeBox(const Eigen::Isometry3d & pose, const Eigen::Vector3d & half)
{
  return SceneBox{
    arr(Eigen::Vector3d(pose.translation())), arr(half), arr(Eigen::Matrix3d(pose.linear()))};
}

/// Largest deviation of the transform between two links over random group configurations.
double relativeMotion(
  const moveit::core::RobotState & state, const moveit::core::JointModelGroup & group,
  const std::string & from, const std::string & to)
{
  moveit::core::RobotState probe(state);
  probe.updateLinkTransforms();
  const Eigen::Isometry3d ref =
    probe.getGlobalLinkTransform(from).inverse() * probe.getGlobalLinkTransform(to);
  double err = 0.0;
  for (int i = 0; i < 4; ++i) {
    probe.setToRandomPositions(&group);
    probe.updateLinkTransforms();
    const Eigen::Isometry3d x =
      probe.getGlobalLinkTransform(from).inverse() * probe.getGlobalLinkTransform(to);
    err = std::max(err, (x.matrix() - ref.matrix()).cwiseAbs().maxCoeff());
  }
  return err;
}

}  // namespace

TranslatedScene translateScene(
  const planning_scene::PlanningScene & scene, const moveit::core::RobotState & state,
  const moveit::core::JointModelGroup & group, const TranslationOptions & options)
{
  TranslatedScene out;
  const auto & model = *state.getRobotModel();

  Eigen::Isometry3d base_from_model = Eigen::Isometry3d::Identity();
  if (!options.base_link.empty()) {
    if (!model.hasLinkModel(options.base_link)) {
      out.error = "base_link '" + options.base_link + "' is not in the robot model";
      return out;
    }
    base_from_model = state.getGlobalLinkTransform(options.base_link).inverse();
  }

  for (const auto & [id, object] : *scene.getWorld()) {
    for (std::size_t i = 0; i < object->shapes_.size(); ++i) {
      const shapes::Shape & shape = *object->shapes_[i];
      const Eigen::Isometry3d model_pose = object->pose_ * object->shape_poses_[i];
      const Eigen::Isometry3d pose = base_from_model * model_pose;
      const std::string tag = id + ":" + shapeName(shape.type);
      switch (shape.type) {
        case shapes::BOX: {
          const auto & b = static_cast<const shapes::Box &>(shape);
          const Eigen::Vector3d half(0.5 * b.size[0], 0.5 * b.size[1], 0.5 * b.size[2]);
          out.scene.boxes.push_back(makeBox(pose, half));
          out.model_frame_boxes.push_back(makeBox(model_pose, half));
          break;
        }
        case shapes::SPHERE: {
          const auto & s = static_cast<const shapes::Sphere &>(shape);
          out.scene.spheres.push_back(
            SceneSphere{arr(Eigen::Vector3d(pose.translation())), s.radius});
          break;
        }
        case shapes::CYLINDER: {
          const auto & c = static_cast<const shapes::Cylinder &>(shape);
          const SceneCylinder cylinder{
            arr(Eigen::Vector3d(pose.translation())), c.radius, c.length,
            arr(Eigen::Matrix3d(pose.linear()))};
          out.scene.cylinders.push_back(cylinder);
          break;
        }
        case shapes::MESH:
        case shapes::CONE: {
          Eigen::Vector3d c, half;
          if (!localBounds(shape, c, half)) {
            out.unsupported.push_back(tag);
            break;
          }
          const Eigen::Isometry3d local = Eigen::Isometry3d(Eigen::Translation3d(c));
          out.scene.boxes.push_back(makeBox(pose * local, half));
          out.model_frame_boxes.push_back(makeBox(model_pose * local, half));
          out.bounded.push_back(tag);
          break;
        }
        default:
          out.unsupported.push_back(tag);
          break;
      }
    }
  }

  std::vector<const moveit::core::AttachedBody *> bodies;
  state.getAttachedBodies(bodies);
  if (bodies.empty()) return out;

  const bool have_ee =
    !options.end_effector_link.empty() && model.hasLinkModel(options.end_effector_link);
  const Eigen::Isometry3d frame_from_ee = options.end_effector_offset.inverse();
  const bool given_spheres = !options.attached_spheres.empty();

  // Transform from @p link to the collision model's end-effector frame. It sets out.error and
  // returns nothing when the robot model lacks the end-effector link or @p link moves relative to
  // it.
  const auto frameFromLink =
    [&](const std::string & link, const std::string & what) -> std::optional<Eigen::Isometry3d> {
    if (!have_ee) {
      out.error = "an object is attached, but the robot model does not have the link '" +
                  options.end_effector_link +
                  "' that holds it; set end_effector_link to a link of the robot model and "
                  "end_effector_offset to the collision model's end-effector frame in it";
      return std::nullopt;
    }
    if (link == options.end_effector_link) return frame_from_ee;
    if (relativeMotion(state, group, options.end_effector_link, link) > 1e-9) {
      out.error = what + " is attached to link '" + link +
                  "', which moves relative to end_effector_link '" + options.end_effector_link +
                  "' under the planning group";
      return std::nullopt;
    }
    return frame_from_ee * state.getGlobalLinkTransform(options.end_effector_link).inverse() *
           state.getGlobalLinkTransform(link);
  };

  // Link of the attached box and spheres that parameters give, the single attached body's link when
  // they name none. Sets out.error and returns an empty name when there is no such link.
  const auto parameterLink = [&]() -> std::string {
    std::string link = options.attached_box_link;
    if (link.empty()) {
      if (bodies.size() != 1) {
        out.error = "attached_object.link must be set when more than one body is attached";
        return {};
      }
      link = bodies.front()->getAttachedLinkName();
    }
    if (!model.hasLinkModel(link)) {
      out.error = "attached_object.link '" + link + "' is not in the robot model";
      return {};
    }
    return link;
  };

  double best_volume = 0.0;
  for (const auto * body : bodies) {
    const std::string & link = body->getAttachedLinkName();
    const auto & shapes_v = body->getShapes();
    const auto & poses = body->getShapePosesInLinkFrame();
    const bool any_sphere = std::any_of(
      shapes_v.begin(), shapes_v.end(), [](const auto & sh) { return sh->type == shapes::SPHERE; });
    Eigen::Isometry3d frame_from_link = frame_from_ee;
    if (!given_spheres) {
      const auto frame = frameFromLink(link, "attached body '" + body->getName() + "'");
      if (!frame) return out;
      frame_from_link = *frame;
    }
    // Without given spheres, a body with sphere primitives is covered by them, and a body
    // without one by a single sphere that encloses its shapes.
    std::optional<Ball> enclosing;
    for (std::size_t i = 0; i < shapes_v.size(); ++i) {
      const shapes::Shape & shape = *shapes_v[i];
      const bool attached_box = shape.type == shapes::BOX && options.want_attached_box;
      if (attached_box) {
        const auto & b = static_cast<const shapes::Box &>(shape);
        const double volume = b.size[0] * b.size[1] * b.size[2];
        if (volume > best_volume && options.attached_box_size.empty()) {
          best_volume = volume;
          AttachedBox attached;
          attached.link = link;
          attached.pose_in_link = poses[i];
          attached.half_extents =
            Eigen::Vector3d(0.5 * b.size[0], 0.5 * b.size[1], 0.5 * b.size[2]);
          out.attached_box = attached;
        }
      }
      if (given_spheres) continue;
      if (any_sphere) {
        if (shape.type == shapes::SPHERE) {
          const Eigen::Vector3d c = (frame_from_link * poses[i]).translation();
          const double r = static_cast<const shapes::Sphere &>(shape).radius;
          out.scene.attached_spheres.push_back(
            {c.x(), c.y(), c.z(), r + options.attachment_padding});
        } else if (!attached_box) {
          out.unsupported.push_back(body->getName() + ":attached " + shapeName(shape.type));
        }
        continue;
      }
      const std::optional<Ball> ball = enclosingBall(shape);
      if (!ball) {
        out.unsupported.push_back(body->getName() + ":attached " + shapeName(shape.type));
        continue;
      }
      const Ball in_link{poses[i] * ball->center, ball->radius};
      enclosing = enclosing ? mergeBalls(*enclosing, in_link) : in_link;
    }
    if (enclosing) {
      const Eigen::Vector3d c = frame_from_link * enclosing->center;
      out.scene.attached_spheres.push_back(
        {c.x(), c.y(), c.z(), enclosing->radius + options.attachment_padding});
    }
  }

  if (given_spheres) {
    const auto & s = options.attached_spheres;
    if (s.size() % 4 != 0) {
      out.error = "attached_object.spheres must hold x, y, z and a radius for every sphere";
      return out;
    }
    const std::string link = parameterLink();
    if (link.empty()) return out;
    const auto frame = frameFromLink(link, "attached_object.spheres");
    if (!frame) return out;
    for (std::size_t i = 0; i < s.size(); i += 4) {
      if (!(s[i + 3] > 0.0) || !std::isfinite(s[i + 3])) {
        out.error = "attached_object.spheres has a radius that is not positive and finite";
        return out;
      }
      const Eigen::Vector3d c = *frame * Eigen::Vector3d(s[i], s[i + 1], s[i + 2]);
      out.scene.attached_spheres.push_back(
        {c.x(), c.y(), c.z(), s[i + 3] + options.attachment_padding});
    }
  }

  if (!options.attached_box_size.empty()) {
    AttachedBox attached;
    attached.link = parameterLink();
    if (attached.link.empty()) return out;
    const auto & s = options.attached_box_size;
    attached.half_extents = Eigen::Vector3d(0.5 * s[0], 0.5 * s[1], 0.5 * s[2]);
    if (!options.attached_box_pose.empty()) {
      const auto & p = options.attached_box_pose;
      attached.pose_in_link = Eigen::Translation3d(p[0], p[1], p[2]) *
                              Eigen::AngleAxisd(p[5], Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(p[4], Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(p[3], Eigen::Vector3d::UnitX());
    }
    out.attached_box = attached;
  }

  return out;
}

}  // namespace geodex_moveit
