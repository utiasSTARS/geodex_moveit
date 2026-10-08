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

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <memory>
#include <set>
#include <stdexcept>

#include "geodex_moveit/backend.hpp"
#include "geodex_moveit/group_state.hpp"
#include "geodex_moveit/scene_translation.hpp"
#include "geometric_shapes/shapes.h"
#include "gtest/gtest.h"
#include "moveit/planning_scene/planning_scene.hpp"
#include "moveit/utils/robot_model_test_utils.hpp"

namespace geodex_moveit
{
namespace
{

bool contains(const std::vector<std::string> & v, const std::string & s)
{
  return std::find(v.begin(), v.end(), s) != v.end();
}

class Translation : public ::testing::Test
{
protected:
  void SetUp() override
  {
    model_ = moveit::core::loadTestingRobotModel("panda");
    ASSERT_TRUE(model_);
    group_ = model_->getJointModelGroup("panda_arm");
    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    auto & state = scene_->getCurrentStateNonConst();
    state.setToDefaultValues(group_, "ready");
    state.update();
  }

  void addWorld(
    const std::string & id, const shapes::ShapeConstPtr & shape, const Eigen::Isometry3d & pose)
  {
    scene_->getWorldNonConst()->addToObject(id, shape, pose);
  }

  void attach(
    const std::string & id, const std::string & link,
    const std::vector<shapes::ShapeConstPtr> & shapes_v, const EigenSTL::vector_Isometry3d & poses)
  {
    auto & state = scene_->getCurrentStateNonConst();
    state.attachBody(
      id, Eigen::Isometry3d::Identity(), shapes_v, poses, std::set<std::string>{}, link);
    state.update();
  }

  moveit::core::RobotModelPtr model_;
  const moveit::core::JointModelGroup * group_ = nullptr;
  planning_scene::PlanningScenePtr scene_;
};

TEST_F(Translation, WorldShapes)
{
  const Eigen::Isometry3d at = Eigen::Translation3d(1.0, 0.0, 0.5) * Eigen::Isometry3d::Identity();
  addWorld("box", std::make_shared<shapes::Box>(0.2, 0.4, 0.6), at);
  addWorld("ball", std::make_shared<shapes::Sphere>(0.1), at);
  addWorld("can", std::make_shared<shapes::Cylinder>(0.1, 0.3), at);
  addWorld("cone", std::make_shared<shapes::Cone>(0.1, 0.4), at);
  auto mesh = std::make_shared<shapes::Mesh>(4, 1);
  const double v[12] = {0, 0, 0, 0.2, 0, 0, 0, 0.4, 0, 0, 0, 0.6};
  std::copy(v, v + 12, mesh->vertices);
  mesh->triangles[0] = 0;
  mesh->triangles[1] = 1;
  mesh->triangles[2] = 2;
  addWorld("mesh", mesh, at);
  addWorld("floor", std::make_shared<shapes::Plane>(0, 0, 1, 0), Eigen::Isometry3d::Identity());

  const TranslatedScene t =
    translateScene(*scene_, scene_->getCurrentState(), *group_, TranslationOptions{});
  EXPECT_TRUE(t.error.empty()) << t.error;
  EXPECT_EQ(t.scene.boxes.size(), 3u);
  EXPECT_EQ(t.scene.spheres.size(), 1u);
  EXPECT_EQ(t.scene.cylinders.size(), 1u);
  EXPECT_TRUE(contains(t.unsupported, "floor:plane"));
  EXPECT_TRUE(contains(t.bounded, "mesh:mesh"));
  EXPECT_TRUE(contains(t.bounded, "cone:cone"));
  EXPECT_EQ(t.model_frame_boxes.size(), 3u);

  const auto near = [](const std::array<double, 3> & a, const Eigen::Vector3d & b) {
    return (Eigen::Vector3d(a[0], a[1], a[2]) - b).norm() < 1e-12;
  };
  const auto box_it = std::find_if(t.scene.boxes.begin(), t.scene.boxes.end(), [&](const auto & b) {
    return near(b.center, Eigen::Vector3d(1.0, 0.0, 0.5));
  });
  ASSERT_NE(box_it, t.scene.boxes.end());
  EXPECT_TRUE(near(box_it->half_extents, Eigen::Vector3d(0.1, 0.2, 0.3)));
  // The mesh bound encloses its vertices, [0, 0.2] x [0, 0.4] x [0, 0.6] moved by the pose.
  const auto mesh_it = std::find_if(
    t.scene.boxes.begin(), t.scene.boxes.end(),
    [&](const auto & b) { return near(b.center, Eigen::Vector3d(1.1, 0.2, 0.8)); });
  ASSERT_NE(mesh_it, t.scene.boxes.end());
  EXPECT_TRUE(near(mesh_it->half_extents, Eigen::Vector3d(0.1, 0.2, 0.3)));
}

TEST_F(Translation, AttachedObjectSpheresAndBox)
{
  const std::vector<shapes::ShapeConstPtr> shapes_v{
    std::make_shared<shapes::Box>(0.04, 0.2, 0.1), std::make_shared<shapes::Sphere>(0.02)};
  EigenSTL::vector_Isometry3d poses{
    Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.1)),
    Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.05, 0.1))};
  attach("box", "panda_hand", shapes_v, poses);

  TranslationOptions o;
  EXPECT_FALSE(translateScene(*scene_, scene_->getCurrentState(), *group_, o).error.empty())
    << "needs end_effector_link";

  o.end_effector_link = "panda_hand";
  o.attachment_padding = 0.01;
  const TranslatedScene t = translateScene(*scene_, scene_->getCurrentState(), *group_, o);
  ASSERT_TRUE(t.error.empty()) << t.error;
  ASSERT_EQ(t.scene.attached_spheres.size(), 1u);
  EXPECT_NEAR(t.scene.attached_spheres[0][1], 0.05, 1e-12);
  EXPECT_NEAR(t.scene.attached_spheres[0][2], 0.1, 1e-12);
  EXPECT_NEAR(t.scene.attached_spheres[0][3], 0.03, 1e-12);
  ASSERT_TRUE(t.attached_box.has_value());
  EXPECT_EQ(t.attached_box->link, "panda_hand");
  EXPECT_TRUE(t.attached_box->half_extents.isApprox(Eigen::Vector3d(0.02, 0.1, 0.05)));
  EXPECT_TRUE(t.unsupported.empty());

  // Spheres are expressed in a rigidly connected end-effector frame.
  o.end_effector_link = "panda_link8";
  const TranslatedScene t8 = translateScene(*scene_, scene_->getCurrentState(), *group_, o);
  ASSERT_TRUE(t8.error.empty()) << t8.error;
  const auto & st = scene_->getCurrentState();
  const Eigen::Vector3d expect = st.getGlobalLinkTransform("panda_link8").inverse() *
                                 st.getGlobalLinkTransform("panda_hand") *
                                 Eigen::Vector3d(0.0, 0.05, 0.1);
  EXPECT_NEAR(t8.scene.attached_spheres[0][0], expect.x(), 1e-12);
  EXPECT_NEAR(t8.scene.attached_spheres[0][1], expect.y(), 1e-12);
  EXPECT_NEAR(t8.scene.attached_spheres[0][2], expect.z(), 1e-12);

  // A link that moves relative to the end effector under the group is refused.
  o.end_effector_link = "panda_link5";
  EXPECT_FALSE(translateScene(*scene_, scene_->getCurrentState(), *group_, o).error.empty());
}

// The exact test takes the box from the parameters, and runs only when a box is known.
TEST_F(Translation, ExactTestTakesTheGivenBox)
{
  attach(
    "ball", "panda_hand", {std::make_shared<shapes::Sphere>(0.03)},
    {Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.1))});
  TranslationOptions o;
  o.end_effector_link = "panda_hand";
  TranslatedScene t = translateScene(*scene_, scene_->getCurrentState(), *group_, o);
  ASSERT_TRUE(t.error.empty()) << t.error;
  EXPECT_FALSE(t.attached_box.has_value());

  o.attached_box_size = {0.06, 0.06, 0.06};
  o.attached_box_pose = {0.0, 0.0, 0.1, 0.0, 0.0, M_PI / 2};
  t = translateScene(*scene_, scene_->getCurrentState(), *group_, o);
  ASSERT_TRUE(t.error.empty()) << t.error;
  ASSERT_TRUE(t.attached_box.has_value());
  EXPECT_EQ(t.attached_box->link, "panda_hand");
  EXPECT_NEAR(t.attached_box->pose_in_link.translation().z(), 0.1, 1e-12);
}

// A body without sphere primitives becomes one sphere that encloses its shapes.
TEST_F(Translation, AttachedBodyWithoutSpheresIsEnclosedByOneSphere)
{
  attach(
    "crate", "panda_hand", {std::make_shared<shapes::Box>(0.04, 0.24, 0.16)},
    {Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.1))});
  TranslationOptions o;
  o.end_effector_link = "panda_hand";
  o.attachment_padding = 0.002;
  const TranslatedScene t = translateScene(*scene_, scene_->getCurrentState(), *group_, o);
  ASSERT_TRUE(t.error.empty()) << t.error;
  EXPECT_TRUE(t.unsupported.empty());
  ASSERT_EQ(t.scene.attached_spheres.size(), 1u);
  EXPECT_NEAR(t.scene.attached_spheres[0][0], 0.0, 1e-12);
  EXPECT_NEAR(t.scene.attached_spheres[0][2], 0.1, 1e-12);
  // Half the box diagonal, plus the attachment padding.
  EXPECT_NEAR(
    t.scene.attached_spheres[0][3],
    0.5 * std::sqrt(0.04 * 0.04 + 0.24 * 0.24 + 0.16 * 0.16) + 0.002, 1e-12);
  ASSERT_TRUE(t.attached_box.has_value());
}

// The sphere of a body with several shapes contains each of them.
TEST_F(Translation, EnclosingSphereContainsEveryShape)
{
  attach(
    "tool", "panda_hand",
    {std::make_shared<shapes::Box>(0.02, 0.02, 0.1),
     std::make_shared<shapes::Cylinder>(0.03, 0.06)},
    {Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.05)),
     Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.15))});
  TranslationOptions o;
  o.end_effector_link = "panda_hand";
  o.want_attached_box = false;
  const TranslatedScene t = translateScene(*scene_, scene_->getCurrentState(), *group_, o);
  ASSERT_TRUE(t.error.empty()) << t.error;
  ASSERT_EQ(t.scene.attached_spheres.size(), 1u);
  const Eigen::Vector3d c(
    t.scene.attached_spheres[0][0], t.scene.attached_spheres[0][1], t.scene.attached_spheres[0][2]);
  const double r = t.scene.attached_spheres[0][3];
  // The box corners and the cylinder rim lie inside the sphere.
  for (const double sx : {-1.0, 1.0}) {
    for (const double sy : {-1.0, 1.0}) {
      for (const double z : {0.0, 0.1}) {
        EXPECT_LE((Eigen::Vector3d(0.01 * sx, 0.01 * sy, z) - c).norm(), r + 1e-12);
      }
    }
  }
  for (int k = 0; k < 16; ++k) {
    const double a = 2.0 * M_PI * k / 16.0;
    for (const double z : {0.12, 0.18}) {
      EXPECT_LE((Eigen::Vector3d(0.03 * std::cos(a), 0.03 * std::sin(a), z) - c).norm(), r + 1e-12);
    }
  }
  EXPECT_NEAR(
    r, 0.5 * (std::hypot(0.01 * std::sqrt(2.0), 0.05) + 0.1 + std::hypot(0.03, 0.03)), 1e-12);
}

// A mesh and a rotated cone in one body share one sphere, and a second body gets its own.
TEST_F(Translation, EnclosingSphereOfAMeshAndARotatedCone)
{
  auto mesh = std::make_shared<shapes::Mesh>(4, 4);
  const double v[] = {0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.08, 0.0, 0.0, 0.0, 0.06};
  std::copy(std::begin(v), std::end(v), mesh->vertices);
  const unsigned int f[] = {0, 1, 2, 0, 1, 3, 0, 2, 3, 1, 2, 3};
  std::copy(std::begin(f), std::end(f), mesh->triangles);
  const Eigen::Isometry3d mesh_pose(Eigen::Translation3d(0.02, -0.03, 0.1));
  const Eigen::Isometry3d cone_pose =
    Eigen::Translation3d(0.0, 0.05, 0.15) *
    Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, 1.0, 0.0).normalized());
  attach(
    "tool", "panda_hand", {mesh, std::make_shared<shapes::Cone>(0.02, 0.08)},
    {mesh_pose, cone_pose});
  attach(
    "crate", "panda_hand", {std::make_shared<shapes::Box>(0.05, 0.05, 0.05)},
    {Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.3))});
  TranslationOptions o;
  o.end_effector_link = "panda_hand";
  o.want_attached_box = false;
  const TranslatedScene t = translateScene(*scene_, scene_->getCurrentState(), *group_, o);
  ASSERT_TRUE(t.error.empty()) << t.error;
  EXPECT_TRUE(t.unsupported.empty());
  ASSERT_EQ(t.scene.attached_spheres.size(), 2u);
  const auto & tool = std::abs(t.scene.attached_spheres[0][2] - 0.3) < 1e-9
                        ? t.scene.attached_spheres[1]
                        : t.scene.attached_spheres[0];
  const Eigen::Vector3d c(tool[0], tool[1], tool[2]);
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector3d p = mesh_pose * Eigen::Vector3d(v[3 * i], v[3 * i + 1], v[3 * i + 2]);
    EXPECT_LE((p - c).norm(), tool[3] + 1e-12) << "mesh vertex " << i;
  }
  // The cone's apex and the rim of its base, along its z axis.
  EXPECT_LE((cone_pose * Eigen::Vector3d(0.0, 0.0, 0.04) - c).norm(), tool[3] + 1e-12);
  for (int k = 0; k < 16; ++k) {
    const double a = 2.0 * M_PI * k / 16.0;
    const Eigen::Vector3d p =
      cone_pose * Eigen::Vector3d(0.02 * std::cos(a), 0.02 * std::sin(a), -0.04);
    EXPECT_LE((p - c).norm(), tool[3] + 1e-12) << "cone rim " << k;
  }
}

// Sphere primitives cover their body, and its other shapes do not add a sphere.
TEST_F(Translation, SpherePrimitivesCoverTheirBody)
{
  attach(
    "box", "panda_hand",
    {std::make_shared<shapes::Box>(0.04, 0.24, 0.16), std::make_shared<shapes::Sphere>(0.2)},
    {Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.15)),
     Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.15))});
  TranslationOptions o;
  o.end_effector_link = "panda_hand";
  const TranslatedScene t = translateScene(*scene_, scene_->getCurrentState(), *group_, o);
  ASSERT_TRUE(t.error.empty()) << t.error;
  ASSERT_EQ(t.scene.attached_spheres.size(), 1u);
  EXPECT_NEAR(t.scene.attached_spheres[0][3], 0.2, 1e-12);
}

// Spheres given by parameters replace the attached sphere primitives and cover an attached box.
TEST_F(Translation, GivenSpheresReplaceTheAttachedPrimitives)
{
  attach(
    "box", "panda_hand",
    {std::make_shared<shapes::Box>(0.04, 0.24, 0.16), std::make_shared<shapes::Sphere>(0.03)},
    {Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.15)),
     Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.1))});
  TranslationOptions o;
  o.end_effector_link = "panda_hand";
  o.attachment_padding = 0.001;
  o.attached_spheres = {0.0, 0.01, 0.12, 0.05, 0.0, -0.01, 0.18, 0.04};
  TranslatedScene t = translateScene(*scene_, scene_->getCurrentState(), *group_, o);
  ASSERT_TRUE(t.error.empty()) << t.error;
  EXPECT_TRUE(t.unsupported.empty());
  ASSERT_EQ(t.scene.attached_spheres.size(), 2u);
  EXPECT_NEAR(t.scene.attached_spheres[0][1], 0.01, 1e-12);
  EXPECT_NEAR(t.scene.attached_spheres[0][2], 0.12, 1e-12);
  EXPECT_NEAR(t.scene.attached_spheres[0][3], 0.051, 1e-12);
  EXPECT_NEAR(t.scene.attached_spheres[1][2], 0.18, 1e-12);
  ASSERT_TRUE(t.attached_box.has_value());
  EXPECT_NEAR(t.attached_box->pose_in_link.translation().z(), 0.15, 1e-12);

  // The spheres are given in panda_hand, and the collision model's frame lies 0.105 m along its
  // z axis.
  o.end_effector_offset = Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.105));
  t = translateScene(*scene_, scene_->getCurrentState(), *group_, o);
  ASSERT_TRUE(t.error.empty()) << t.error;
  EXPECT_NEAR(t.scene.attached_spheres[0][2], 0.015, 1e-12);

  o.attached_spheres = {0.0, 0.0, 0.1};
  EXPECT_FALSE(translateScene(*scene_, scene_->getCurrentState(), *group_, o).error.empty());
  o.attached_spheres = {0.0, 0.0, 0.1, 0.0};
  EXPECT_FALSE(translateScene(*scene_, scene_->getCurrentState(), *group_, o).error.empty());
  o.attached_spheres = {0.0, 0.0, 0.1, 0.05};
  o.attached_box_link = "no_such_link";
  EXPECT_FALSE(translateScene(*scene_, scene_->getCurrentState(), *group_, o).error.empty());
}

// Every primitive type the translation produces reaches the collision checker.
TEST_F(Translation, CollisionEnvironmentSeesEveryPrimitive)
{
  const auto q = configuration(scene_->getCurrentState(), *group_);
  const Eigen::Vector3d h =
    scene_->getCurrentState().getGlobalLinkTransform("panda_hand").translation();
  const std::array<double, 3> hand{h.x(), h.y(), h.z()};
  const auto check = [&](const CollisionScene & s) {
    return CollisionEnvironment::create("panda", s)->makeOracle()->isValid(q.data());
  };
  EXPECT_TRUE(check(CollisionScene{}));
  CollisionScene with_box;
  with_box.boxes.push_back(SceneBox{hand, {0.1, 0.1, 0.1}});
  EXPECT_FALSE(check(with_box));
  CollisionScene with_sphere;
  with_sphere.spheres.push_back(SceneSphere{hand, 0.1});
  EXPECT_FALSE(check(with_sphere));
  CollisionScene with_cylinder;
  with_cylinder.cylinders.push_back(SceneCylinder{hand, 0.1, 0.3});
  EXPECT_FALSE(check(with_cylinder)) << "cylinder obstacles must be checked";
}

// A box obstacle reaches the collision checker at its full size. For some axis direction from the
// hand, a box of half extent 0.14 centered 0.25 m away misses the robot, and one of half extent
// 0.28 at the same center covers the hand.
TEST_F(Translation, BoxObstaclesKeepTheirSize)
{
  const auto q = configuration(scene_->getCurrentState(), *group_);
  const Eigen::Vector3d h =
    scene_->getCurrentState().getGlobalLinkTransform("panda_hand").translation();
  const auto valid_with_box = [&](const Eigen::Vector3d & center, double half) {
    CollisionScene s;
    s.boxes.push_back(SceneBox{{center.x(), center.y(), center.z()}, {half, half, half}});
    return CollisionEnvironment::create("panda", s)->makeOracle()->isValid(q.data());
  };
  int probes = 0;
  for (int axis = 0; axis < 3; ++axis) {
    for (const double sign : {-1.0, 1.0}) {
      const Eigen::Vector3d center = h + sign * 0.25 * Eigen::Vector3d::Unit(axis);
      if (!valid_with_box(center, 0.14)) continue;
      ++probes;
      EXPECT_FALSE(valid_with_box(center, 0.28)) << "axis " << axis << ", sign " << sign;
    }
  }
  EXPECT_GT(probes, 0) << "no direction leaves the small box clear of the robot";
}

// The obstacle padding inflates every obstacle when the collision environment is built. A box
// of half extent 0.14 that clears the robot blocks it once padded by 0.14, as the box of half
// extent 0.28 above does. A negative padding is refused.
TEST_F(Translation, ObstaclePaddingInflatesEveryPrimitive)
{
  const auto q = configuration(scene_->getCurrentState(), *group_);
  const Eigen::Vector3d h =
    scene_->getCurrentState().getGlobalLinkTransform("panda_hand").translation();
  const auto valid = [&](CollisionScene s, double padding) {
    s.obstacle_padding = padding;
    return CollisionEnvironment::create("panda", s)->makeOracle()->isValid(q.data());
  };
  int probes = 0;
  for (int axis = 0; axis < 3; ++axis) {
    for (const double sign : {-1.0, 1.0}) {
      const Eigen::Vector3d c = h + sign * 0.25 * Eigen::Vector3d::Unit(axis);
      const std::array<double, 3> center{c.x(), c.y(), c.z()};
      CollisionScene box;
      box.boxes.push_back(SceneBox{center, {0.14, 0.14, 0.14}});
      CollisionScene sphere;
      sphere.spheres.push_back(SceneSphere{center, 0.14});
      CollisionScene cylinder;
      cylinder.cylinders.push_back(SceneCylinder{center, 0.14, 0.28});
      if (!valid(box, 0.0) || !valid(sphere, 0.0) || !valid(cylinder, 0.0)) continue;
      ++probes;
      EXPECT_FALSE(valid(box, 0.14)) << "box, axis " << axis << ", sign " << sign;
      EXPECT_FALSE(valid(sphere, 0.14)) << "sphere, axis " << axis << ", sign " << sign;
      EXPECT_FALSE(valid(cylinder, 0.14)) << "cylinder, axis " << axis << ", sign " << sign;
    }
  }
  EXPECT_GT(probes, 0) << "no direction leaves the unpadded obstacles clear of the robot";
  CollisionScene s;
  s.obstacle_padding = -0.001;
  EXPECT_THROW(CollisionEnvironment::create("panda", s), std::invalid_argument);
}

// Attached objects come from the state handed in, the request's start state, and not from the
// scene.
TEST_F(Translation, AttachedBodiesComeFromTheGivenState)
{
  moveit::core::RobotState start(scene_->getCurrentState());
  start.attachBody(
    "ball", Eigen::Isometry3d::Identity(), {std::make_shared<shapes::Sphere>(0.03)},
    {Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.1))}, std::set<std::string>{},
    "panda_hand");
  start.update();
  TranslationOptions o;
  o.end_effector_link = "panda_hand";
  o.want_attached_box = false;
  EXPECT_TRUE(
    translateScene(*scene_, scene_->getCurrentState(), *group_, o).scene.attached_spheres.empty());
  const TranslatedScene t = translateScene(*scene_, start, *group_, o);
  ASSERT_TRUE(t.error.empty()) << t.error;
  ASSERT_EQ(t.scene.attached_spheres.size(), 1u);
  EXPECT_NEAR(t.scene.attached_spheres[0][2], 0.1, 1e-12);
}

// Attached-object spheres are expressed in the collision model's end-effector frame, which sits at
// end_effector_offset in end_effector_link.
TEST_F(Translation, EndEffectorOffsetMovesTheSpheres)
{
  attach(
    "ball", "panda_hand", {std::make_shared<shapes::Sphere>(0.03)},
    {Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.25))});
  TranslationOptions o;
  o.end_effector_link = "panda_hand";
  o.end_effector_offset = Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.105));
  o.want_attached_box = false;
  const TranslatedScene t = translateScene(*scene_, scene_->getCurrentState(), *group_, o);
  ASSERT_TRUE(t.error.empty()) << t.error;
  ASSERT_EQ(t.scene.attached_spheres.size(), 1u);
  EXPECT_NEAR(t.scene.attached_spheres[0][2], 0.145, 1e-12);
  o.end_effector_link = "panda_grasptarget";
  EXPECT_FALSE(translateScene(*scene_, scene_->getCurrentState(), *group_, o).error.empty())
    << "the link is not in this robot model";
}

}  // namespace
}  // namespace geodex_moveit
