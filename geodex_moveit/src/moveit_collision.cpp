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

#include "geodex_moveit/moveit_collision.hpp"

#include <utility>

#include "geodex_moveit/group_state.hpp"

namespace geodex_moveit
{

MoveItCollisionOracle::MoveItCollisionOracle(
  planning_scene::PlanningSceneConstPtr scene, const moveit::core::RobotState & state,
  const moveit::core::JointModelGroup & group, double padding)
: scene_(std::move(scene)), group_(&group), state_(state), dimension_(configurationSize(group))
{
  if (padding > 0.0) {
    padded_ = scene_->diff();
    padded_->getCollisionEnvNonConst()->setPadding(padding);
  }
}

bool MoveItCollisionOracle::isValid(const double * q) const
{
  setConfiguration(state_, *group_, q);
  state_.updateCollisionBodyTransforms();
  if (!padded_) return !scene_->isStateColliding(state_);
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  const auto & acm = scene_->getAllowedCollisionMatrix();
  scene_->getCollisionEnv()->checkSelfCollision(req, res, state_, acm);
  if (res.collision) return false;
  padded_->getCollisionEnv()->checkRobotCollision(req, res, state_, acm);
  return !res.collision;
}

}  // namespace geodex_moveit
