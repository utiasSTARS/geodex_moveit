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

/// @file validity.hpp
/// @brief Validity oracle for joint configurations, in the planning group's variable order.

#pragma once

#if defined(_WIN32)
#define GEODEX_MOVEIT_BACKEND_API
#else
/// Marks the symbols that the backend library exports. The library hides every other symbol.
#define GEODEX_MOVEIT_BACKEND_API __attribute__((visibility("default")))
#endif

namespace geodex_moveit
{

/// @brief Answers whether joint configurations of one planning group are valid.
///
/// One instance serves one planning request on one thread. Configurations are plain arrays of
/// the group's active variables.
class GEODEX_MOVEIT_BACKEND_API ValidityOracle
{
public:
  virtual ~ValidityOracle();

  /// @brief True when the configuration @p q is valid.
  virtual bool isValid(const double * q) const = 0;

  /// @brief Configurations the oracle evaluates per call, 1 when it does not batch.
  virtual int batchWidth() const;

  /// @brief True when every one of @p count row-major configurations is valid.
  virtual bool allValid(const double * q, int count) const;

  /// @brief Number of variables per configuration.
  virtual int dimension() const = 0;

  /// @brief The oracle that checks the robot's collision model, this oracle by default. A
  /// composite oracle returns the oracle it consults first.
  virtual const ValidityOracle & collisionOracle() const;
};

}  // namespace geodex_moveit
