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

/// @file environment_cache.hpp
/// @brief Cache of the most recently built collision environment.

#pragma once

#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "geodex_moveit/backend.hpp"
#include "geodex_moveit/scene_translation.hpp"

namespace geodex_moveit
{

/// @brief Holds the most recently built collision environment and the scene it was built from.
///
/// A lookup returns the cached environment only when the robot and the full scene compare equal.
/// The environment hands every request its own validity oracle. The cache is thread-safe.
class EnvironmentCache
{
public:
  /// @brief Environment for @p robot and @p scene, built on a miss.
  std::shared_ptr<const CollisionEnvironment> get(
    const std::string & robot, const CollisionScene & scene);

  /// @brief Number of environments built so far.
  int builds() const;

  /// @brief Entries of @p tags that earlier calls did not pass.
  std::vector<std::string> unseen(const std::vector<std::string> & tags);

private:
  mutable std::mutex mutex_;
  std::set<std::string> seen_;
  std::string robot_;
  CollisionScene scene_;
  std::shared_ptr<const CollisionEnvironment> environment_;
  int builds_ = 0;
};

}  // namespace geodex_moveit
