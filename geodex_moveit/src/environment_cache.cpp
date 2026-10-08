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

#include "geodex_moveit/environment_cache.hpp"

namespace geodex_moveit
{

std::shared_ptr<const CollisionEnvironment> EnvironmentCache::get(
  const std::string & robot, const CollisionScene & scene)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (environment_ && robot == robot_ && scene == scene_) return environment_;
  environment_ = CollisionEnvironment::create(robot, scene);
  robot_ = robot;
  scene_ = scene;
  ++builds_;
  return environment_;
}

int EnvironmentCache::builds() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return builds_;
}

std::vector<std::string> EnvironmentCache::unseen(const std::vector<std::string> & tags)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> out;
  for (const auto & t : tags) {
    if (seen_.insert(t).second) out.push_back(t);
  }
  return out;
}

}  // namespace geodex_moveit
