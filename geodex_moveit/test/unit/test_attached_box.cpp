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

#include <cmath>
#include <vector>

#include "geodex_moveit/attached_box_test.hpp"
#include "geodex_moveit/group_state.hpp"
#include "gtest/gtest.h"
#include "moveit/robot_state/robot_state.hpp"
#include "moveit/utils/robot_model_test_utils.hpp"

namespace geodex_moveit
{
namespace
{

const Eigen::Matrix3d kI = Eigen::Matrix3d::Identity();

TEST(AttachedBoxTest, SeparatingAxisOnAlignedBoxes)
{
  const Eigen::Vector3d h(0.5, 0.5, 0.5);
  EXPECT_TRUE(AttachedBoxTest::disjoint({0, 0, 0}, kI, h, {1.01, 0, 0}, kI, h));
  EXPECT_FALSE(AttachedBoxTest::disjoint({0, 0, 0}, kI, h, {0.99, 0, 0}, kI, h));
  EXPECT_FALSE(AttachedBoxTest::disjoint({0, 0, 0}, kI, h, {0, 0, 0}, kI, 0.1 * h));
}

TEST(AttachedBoxTest, SeparatingAxisNeedsEdgeAxes)
{
  // Two thin skewed sticks, one along x and one along (0, 1, 1). Only the cross product of their
  // long axes separates them.
  const Eigen::Vector3d ha(1.0, 0.05, 0.05);
  const Eigen::Vector3d hb(0.05, 1.0, 0.05);
  const Eigen::Vector3d u = Eigen::Vector3d(0.0, 1.0, 1.0).normalized();
  const Eigen::Vector3d n = Eigen::Vector3d(0.0, -1.0, 1.0).normalized();
  const Eigen::Vector3d x = Eigen::Vector3d::UnitX();
  const double c = std::cos(M_PI / 6), s = std::sin(M_PI / 6);
  Eigen::Matrix3d rb;
  rb.col(0) = c * x - s * n;
  rb.col(1) = u;
  rb.col(2) = c * n + s * x;
  EXPECT_TRUE(AttachedBoxTest::disjoint({0, 0, 0}, kI, ha, 0.3 * n, rb, hb));
  EXPECT_FALSE(AttachedBoxTest::disjoint({0, 0, 0}, kI, ha, 0.1 * n, rb, hb));
}

class PandaBox : public ::testing::Test
{
protected:
  void SetUp() override
  {
    model_ = moveit::core::loadTestingRobotModel("panda");
    ASSERT_TRUE(model_);
    group_ = model_->getJointModelGroup("panda_arm");
    ASSERT_NE(group_, nullptr);
    state_ = std::make_unique<moveit::core::RobotState>(model_);
    state_->setToDefaultValues();
    state_->update();
    box_.link = "panda_hand";
    box_.pose_in_link = Eigen::Translation3d(0.0, 0.0, 0.15) * Eigen::Isometry3d::Identity();
    box_.half_extents = Eigen::Vector3d(0.02, 0.12, 0.08);
  }

  moveit::core::RobotModelPtr model_;
  const moveit::core::JointModelGroup * group_ = nullptr;
  std::unique_ptr<moveit::core::RobotState> state_;
  AttachedBox box_;
};

TEST_F(PandaBox, ChainKinematicsMatchesRobotState)
{
  const AttachedBoxTest test(*state_, *group_, box_, {}, 0.0);
  EXPECT_TRUE(test.usesChainKinematics());
  moveit::core::RobotState probe(*state_);
  for (int i = 0; i < 20; ++i) {
    probe.setToRandomPositions(group_);
    probe.update();
    const auto q = configuration(probe, *group_);
    const Eigen::Isometry3d expected =
      probe.getGlobalLinkTransform("panda_hand") * box_.pose_in_link;
    EXPECT_LT((test.boxPose(q.data()).matrix() - expected.matrix()).cwiseAbs().maxCoeff(), 1e-12);
  }
}

TEST_F(PandaBox, DetectsTheAttachedBoxInsideAWorldBox)
{
  const auto q = configuration(*state_, *group_);
  const Eigen::Isometry3d pose = state_->getGlobalLinkTransform("panda_hand") * box_.pose_in_link;

  const auto box = [](const Eigen::Vector3d & c, const Eigen::Matrix3d & r) {
    SceneBox b;
    b.center = {c.x(), c.y(), c.z()};
    b.half_extents = {0.05, 0.05, 0.05};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) b.rotation[static_cast<std::size_t>(3 * i + j)] = r(i, j);
    }
    return b;
  };
  const SceneBox around = box(pose.translation(), Eigen::Matrix3d::Identity());
  const SceneBox far =
    box(pose.translation() + Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Matrix3d::Identity());

  EXPECT_FALSE(AttachedBoxTest(*state_, *group_, box_, {around}, 0.0).isValid(q.data()));
  EXPECT_TRUE(AttachedBoxTest(*state_, *group_, box_, {far}, 0.0).isValid(q.data()));

  // A margin grows the attached box. A world box just past its face collides only with the margin.
  const Eigen::Vector3d up = pose.linear().col(2);
  const SceneBox near =
    box(pose.translation() + up * (box_.half_extents.z() + 0.05 + 0.005), pose.linear());
  EXPECT_TRUE(AttachedBoxTest(*state_, *group_, box_, {near}, 0.0).isValid(q.data()));
  EXPECT_FALSE(AttachedBoxTest(*state_, *group_, box_, {near}, 0.01).isValid(q.data()));
}

TEST_F(PandaBox, RejectsAnUnknownLink)
{
  AttachedBox bad = box_;
  bad.link = "no_such_link";
  EXPECT_THROW(AttachedBoxTest(*state_, *group_, bad, {}, 0.0), std::invalid_argument);
}

}  // namespace
}  // namespace geodex_moveit
