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

#include "geodex_moveit/attached_box_test.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

#include "geodex_moveit/group_state.hpp"

namespace geodex_moveit
{

AttachedBoxTest::AttachedBoxTest(
  const moveit::core::RobotState & state, const moveit::core::JointModelGroup & group,
  const AttachedBox & box, std::vector<SceneBox> obstacles, double margin)
: state_(state),
  group_(&group),
  box_in_link_(box.pose_in_link),
  half_(box.half_extents + Eigen::Vector3d::Constant(margin)),
  dimension_(configurationSize(group))
{
  link_ = state_.getRobotModel()->getLinkModel(box.link);
  if (link_ == nullptr) {
    throw std::invalid_argument("attached box link '" + box.link + "' is not in the robot model");
  }
  obstacles_.reserve(obstacles.size());
  for (const auto & o : obstacles) {
    const Eigen::Matrix3d rot =
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(o.rotation.data());
    const Obb obb{
      Eigen::Vector3d(o.center[0], o.center[1], o.center[2]),
      Eigen::Vector3d(o.half_extents[0], o.half_extents[1], o.half_extents[2]), rot};
    obstacles_.push_back(obb);
  }

  // Precompute the chain from the root to the attach link. A check evaluates this short chain of
  // joint transforms instead of a full state update.
  std::vector<const moveit::core::LinkModel *> links;
  for (const auto * l = link_; l != nullptr; l = l->getParentLinkModel()) links.push_back(l);
  std::reverse(links.begin(), links.end());

  // Map each robot-state variable index to its configuration index.
  std::unordered_map<int, int> var_to_q;
  int k = 0;
  for (const auto * joint : group_->getActiveJointModels()) {
    for (std::size_t v = 0; v < joint->getVariableCount(); ++v) {
      var_to_q[joint->getFirstVariableIndex() + static_cast<int>(v)] = k++;
    }
  }

  bool buildable = true;
  for (const auto * link : links) {
    const auto * joint = link->getParentJointModel();
    ChainNode node;
    node.joint = joint;
    node.origin = link->getJointOriginTransform();
    node.origin_identity = link->jointOriginTransformIsIdentity();
    if (joint != nullptr && joint->getVariableCount() > 0) {
      const auto it = var_to_q.find(joint->getFirstVariableIndex());
      if (it == var_to_q.end()) {
        node.constant = state_.getJointTransform(joint);
        node.constant_identity = false;
      } else if (joint->getVariableCount() == 1) {
        node.q_index = it->second;
      } else {
        buildable = false;
      }
    }
    chain_.push_back(node);
  }

  // Use the chain only when it reproduces RobotState's transform exactly.
  if (buildable) {
    moveit::core::RobotState probe(state_);
    double max_err = 0.0;
    std::vector<double> vals;
    for (int trial = 0; trial < 16; ++trial) {
      if (trial == 0) {
        probe = state_;
      } else {
        probe.setToRandomPositions(group_);
      }
      vals = configuration(probe, *group_);
      probe.updateLinkTransforms();
      const Eigen::Isometry3d ref = probe.getGlobalLinkTransform(link_);
      chain_valid_ = true;
      const Eigen::Isometry3d fast = linkPose(vals.data());
      max_err = std::max(max_err, (ref.matrix() - fast.matrix()).cwiseAbs().maxCoeff());
    }
    chain_valid_ = max_err <= 1e-12;
  }
}

Eigen::Isometry3d AttachedBoxTest::linkPose(const double * q) const
{
  if (!chain_valid_) {
    setConfiguration(state_, *group_, q);
    state_.updateLinkTransforms();
    return state_.getGlobalLinkTransform(link_);
  }
  Eigen::Isometry3d x = Eigen::Isometry3d::Identity();
  for (const auto & node : chain_) {
    if (!node.origin_identity) x = x * node.origin;
    if (node.q_index >= 0) {
      Eigen::Isometry3d jt;
      node.joint->computeTransform(q + node.q_index, jt);
      x = x * jt;
    } else if (!node.constant_identity) {
      x = x * node.constant;
    }
  }
  return x;
}

Eigen::Isometry3d AttachedBoxTest::boxPose(const double * q) const
{
  return linkPose(q) * box_in_link_;
}

bool AttachedBoxTest::disjoint(
  const Eigen::Vector3d & ca, const Eigen::Matrix3d & ra, const Eigen::Vector3d & ha,
  const Eigen::Vector3d & cb, const Eigen::Matrix3d & rb, const Eigen::Vector3d & hb)
{
  const Eigen::Vector3d t = ca - cb;
  bool separated = false;
  auto test = [&](const Eigen::Vector3d & axis) {
    if (separated) return;
    if (axis.squaredNorm() < 1e-12) return;  // Skip a degenerate cross-product axis.
    const double d = std::abs(t.dot(axis));
    const double a = ha[0] * std::abs(ra.col(0).dot(axis)) + ha[1] * std::abs(ra.col(1).dot(axis)) +
                     ha[2] * std::abs(ra.col(2).dot(axis));
    const double b = hb[0] * std::abs(rb.col(0).dot(axis)) + hb[1] * std::abs(rb.col(1).dot(axis)) +
                     hb[2] * std::abs(rb.col(2).dot(axis));
    if (d > a + b) separated = true;
  };
  // Test the face axes of b, the face axes of a, then the nine edge cross products.
  for (int i = 0; i < 3 && !separated; ++i) test(rb.col(i));
  for (int i = 0; i < 3 && !separated; ++i) test(ra.col(i));
  for (int i = 0; i < 3 && !separated; ++i) {
    for (int j = 0; j < 3 && !separated; ++j) test(rb.col(i).cross(ra.col(j)));
  }
  return separated;
}

bool AttachedBoxTest::isValid(const double * q) const
{
  const Eigen::Isometry3d x = boxPose(q);
  const Eigen::Matrix3d r = x.linear();
  const Eigen::Vector3d c = x.translation();
  for (const auto & o : obstacles_) {
    if (!disjoint(c, r, half_, o.center, o.rot, o.half)) return false;
  }
  return true;
}

}  // namespace geodex_moveit
