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

/// @file composite_validity.hpp
/// @brief Conjunction of a collision oracle and an exact test on attached objects.

#pragma once

#include <memory>
#include <utility>

#include "geodex_moveit/validity.hpp"

namespace geodex_moveit
{

/// @brief Evaluates the collision oracle first and the exact test only on configurations that
/// pass it, in blocks of the collision oracle's batch width.
class CompositeValidity : public ValidityOracle
{
public:
  CompositeValidity(
    std::shared_ptr<const ValidityOracle> collision, std::shared_ptr<const ValidityOracle> exact)
  : collision_(std::move(collision)), exact_(std::move(exact))
  {
  }

  bool isValid(const double * q) const override;
  int batchWidth() const override { return collision_->batchWidth(); }
  bool allValid(const double * q, int count) const override;
  int dimension() const override { return collision_->dimension(); }
  const ValidityOracle & collisionOracle() const override { return collision_->collisionOracle(); }

private:
  std::shared_ptr<const ValidityOracle> collision_;
  std::shared_ptr<const ValidityOracle> exact_;
};

}  // namespace geodex_moveit
