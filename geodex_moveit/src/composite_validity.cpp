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

#include "geodex_moveit/composite_validity.hpp"

#include <cstddef>

namespace geodex_moveit
{

bool CompositeValidity::isValid(const double * q) const
{
  if (!collision_->isValid(q)) return false;
  return !exact_ || exact_->isValid(q);
}

bool CompositeValidity::allValid(const double * q, int count) const
{
  if (!collision_->allValid(q, count)) return false;
  if (!exact_) return true;
  const int dim = dimension();
  for (int i = 0; i < count; ++i) {
    if (!exact_->isValid(q + static_cast<std::ptrdiff_t>(i) * dim)) return false;
  }
  return true;
}

}  // namespace geodex_moveit
