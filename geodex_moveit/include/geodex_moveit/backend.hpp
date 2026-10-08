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

/// @file backend.hpp
/// @brief The plugin's only boundary to geodex and OMPL.
///
/// The geodex_moveit_backend shared library implements this header. It compiles against geodex's
/// own Eigen, links the OMPL fork statically and hides every symbol outside this API. The API
/// passes only plain data, and no Eigen, OMPL or MoveIt type crosses it.

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geodex_moveit/validity.hpp"

namespace geodex_moveit
{

/// @brief Planner identifiers the backend accepts, the default first.
GEODEX_MOVEIT_BACKEND_API const std::vector<std::string> & plannerIds();

/// @brief True when geodex has a model named @p robot, for the metric or the collision checker.
GEODEX_MOVEIT_BACKEND_API bool isRegisteredRobot(const std::string & robot);

/// @brief True when geodex has a mass matrix for @p robot, which the kinetic-energy metric needs.
GEODEX_MOVEIT_BACKEND_API bool hasMassMatrix(const std::string & robot);

/// @brief True when geodex has a collision model for @p robot.
GEODEX_MOVEIT_BACKEND_API bool hasCollisionModel(const std::string & robot);

/// @brief True when geodex's model of @p robot has a planar mobile base ahead of its arm joints.
GEODEX_MOVEIT_BACKEND_API bool hasMobileBase(const std::string & robot);

/// @brief Configuration dimension of a registered robot, or -1 for an unknown name.
///
/// A robot with a mobile base counts its whole-body configuration, the base pose followed by the
/// arm joints.
GEODEX_MOVEIT_BACKEND_API int robotDimension(const std::string & robot);

/// @brief Names of the robots geodex has models for.
GEODEX_MOVEIT_BACKEND_API std::vector<std::string> registeredRobots();

/// @brief Joint names of geodex's collision model of @p robot, in configuration order.
///
/// @return an empty list when geodex does not have a collision model for @p robot.
GEODEX_MOVEIT_BACKEND_API std::vector<std::string> modelJointNames(const std::string & robot);

/// @brief Frame of the attached-object spheres in geodex's collision model of @p robot.
///
/// @return an empty string when geodex does not have a collision model for @p robot.
GEODEX_MOVEIT_BACKEND_API std::string modelEndEffectorFrame(const std::string & robot);

/// @brief Margin in meters that the smoother's evenly spaced waypoints need. It is the motion of
/// the collision model's spheres and of @p attached_spheres, given in the model's end-effector
/// frame, over half of @p resolution plus the smoother's corner tolerance. It is 0 for a robot
/// without a collision model.
GEODEX_MOVEIT_BACKEND_API double smoothingMargin(
  const std::string & robot, const std::vector<std::array<double, 4>> & attached_spheres,
  double resolution);

/// @brief Joint limits of geodex's model of @p robot, in its joint order.
///
/// @return false when geodex does not keep limits for @p robot.
GEODEX_MOVEIT_BACKEND_API bool modelJointBounds(
  const std::string & robot, std::vector<double> & lower, std::vector<double> & upper);

/// @brief Receives the planner library's console output, level 0 debug to 3 error.
using LogSink = std::function<void(int level, const std::string & message)>;

/// @brief Route the planner library's console output to @p sink while the returned handle lives.
///
/// The most recently added live sink receives the output. Release the handle before the code
/// that defined the sink is unloaded. The function is thread-safe.
GEODEX_MOVEIT_BACKEND_API std::shared_ptr<void> addLogSink(LogSink sink);

/// @brief Planner and smoother settings, copied from the plugin parameters that reach geodex.
///
/// The member defaults equal the defaults of the parameter schema.
struct PlannerSettings
{
  // Search.
  double grrtstar_range = 1.2;
  double greedy_biasing_ratio = 0.9;
  double rewire_factor = 1.1;
  std::int64_t max_neighbors = 0;
  bool greedy_cost_for_tree_pruning = true;
  bool stop_at_first_solution = false;
  /// Joint-space distance between edge checks, positive.
  double collision_check_resolution = 0.01;
  bool edge_motion_validator = true;
  std::int64_t max_iterations = 0;  ///< Search iteration budget, 0 for none.
  std::int64_t seed = -1;           ///< Seed of the search, negative to seed from the system.
  // Smoothing.
  bool smooth = true;
  double output_spacing = 0.01;
  bool round_corners = true;  ///< Round the corners of the smoothed path into C2 curves.
  std::uint64_t smoothing_seed = 42;
  /// Share of the time budget an anytime search leaves to smoothing.
  double smoothing_time_fraction = 0.2;
};

/// @brief One planning problem, in the planning group's variable order.
struct PlanRequest
{
  std::vector<double> lower;  ///< Per-variable lower bound.
  std::vector<double> upper;  ///< Per-variable upper bound.
  std::vector<double> start;  ///< Start configuration.
  std::vector<double> goal;   ///< Goal configuration.
  /// Seconds for the whole call, search and smoothing. An anytime search stops early enough to
  /// leave PlannerSettings::smoothing_time_fraction of it to smoothing.
  double time_budget = 0.0;
  std::string planner_id;  ///< One of plannerIds(), empty for the default.
  /// Another thread sets this flag to stop the search or the smoothing.
  const std::atomic_bool * cancel = nullptr;
  /// Stricter check of the planner and the smoother, null for plan()'s validity. plan() checks
  /// the returned states with its validity, and the margin between the two gives the smoother's
  /// evenly spaced waypoints room.
  const ValidityOracle * planning_validity = nullptr;
};

/// @brief Outcome of one planning problem.
struct PlanResult
{
  enum class Status
  {
    kSolved,
    kNoExactSolution,
    kCanceled,
    kInvalidInput,
    kInvalidPath
  };
  Status status = Status::kNoExactSolution;
  std::string message;     ///< Reason, when not solved.
  std::string planner_id;  ///< Planner that ran.
  /// Waypoints from start to goal when solved, the planner's or the smoother's. plan() checks every
  /// state of the path densified at PlannerSettings::collision_check_resolution.
  std::vector<std::vector<double>> path;
  double reported_time_s = 0.0;  ///< Search, densification and smoothing wall clock.
  double search_time_s = 0.0;    ///< Search wall clock.
  /// Search wall clock to the first exact solution, -1 without one.
  double first_solution_time_s = -1.0;
  /// Search iterations at the first exact solution, -1 without one or without a count.
  std::int64_t first_solution_iterations = -1;
  double smooth_time_s = 0.0;     ///< Smoother wall clock, 0 when it did not run.
  bool smoothed = false;          ///< True when the returned path is the smoother's.
  int smoothing_stage = -1;       ///< SmoothResult::stage, -1 when the smoother did not run.
  std::size_t raw_waypoints = 0;  ///< Waypoints of the search's solution.
  std::int64_t iterations = -1;   ///< Search iterations, -1 when the planner does not keep a count.
  /// True when smoothing stopped on the budget or on a cancel and plan() returned the planner's
  /// path.
  bool smoothing_stopped = false;
};

/// @brief A path to smooth, with the bounds of the problem it solves.
struct SmoothRequest
{
  std::vector<double> lower;              ///< Per-variable lower bound.
  std::vector<double> upper;              ///< Per-variable upper bound.
  std::vector<std::vector<double>> path;  ///< Waypoints from start to goal.
  /// Stricter check of the smoother, null for smooth()'s validity, as in PlanRequest.
  const ValidityOracle * planning_validity = nullptr;
};

/// @brief Output of the smoother on one path.
struct SmoothResult
{
  /// True when @p path passed the smoother's check of every waypoint and every edge.
  bool collision_free = false;
  /// Stage the smoother returned. 0 is the smoothed path, 1 the optimized waypoints before
  /// resampling, 2 the first shortcut round, 3 the input.
  int stage = 3;
  double length = 0.0;                    ///< Metric length of @p path.
  std::vector<std::vector<double>> path;  ///< Returned path, endpoints kept.
};

/// @brief Planner for one robot model, metric and planning-group dimension.
///
/// The backend is immutable after creation, and plan() builds all per-request state itself.
/// Concurrent calls from different planning contexts are safe.
class GEODEX_MOVEIT_BACKEND_API PlannerBackend
{
public:
  virtual ~PlannerBackend() = default;

  /// @brief Create the backend for @p robot, or for a plain Euclidean space of @p dimension
  /// when @p robot is empty.
  ///
  /// @throw std::invalid_argument for an unknown robot or metric, a robot with a mobile base, a
  /// metric that needs a mass matrix the robot does not have, or a dimension that differs from
  /// the robot model's.
  static std::shared_ptr<const PlannerBackend> create(
    const std::string & robot, const std::string & metric, int dimension);

  /// @brief Configuration dimension this backend plans in.
  virtual int dimension() const = 0;

  /// @brief Short description for logs, naming the metric and the heuristic.
  virtual std::string describe() const = 0;

  /// @brief Solve @p request under @p validity with @p settings.
  virtual PlanResult plan(
    const PlanRequest & request, const ValidityOracle & validity,
    const PlannerSettings & settings) const = 0;

  /// @brief Run the smoother that plan() runs on @p request's path, with the same validity, edge
  /// checks and settings.
  virtual SmoothResult smooth(
    const SmoothRequest & request, const ValidityOracle & validity,
    const PlannerSettings & settings) const = 0;
};

/// @brief Oriented box obstacle, half extents along its own axes. The rotation is world from box,
/// row-major.
struct SceneBox
{
  std::array<double, 3> center{};
  std::array<double, 3> half_extents{};
  std::array<double, 9> rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
  bool operator==(const SceneBox &) const = default;
};

/// @brief Sphere obstacle.
struct SceneSphere
{
  std::array<double, 3> center{};
  double radius = 0.0;
  bool operator==(const SceneSphere &) const = default;
};

/// @brief Oriented cylinder obstacle along its local z axis. The rotation is row-major.
struct SceneCylinder
{
  std::array<double, 3> center{};
  double radius = 0.0;
  double height = 0.0;
  std::array<double, 9> rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
  bool operator==(const SceneCylinder &) const = default;
};

/// @brief Obstacles and attached-object spheres in the collision model's base frame.
///
/// Two scenes that compare equal produce the same collision environment, and the environment
/// cache keys on that equality.
struct CollisionScene
{
  std::vector<SceneBox> boxes;
  std::vector<SceneSphere> spheres;
  std::vector<SceneCylinder> cylinders;
  /// Attached-object spheres as {x, y, z, r} in the collision model's end-effector frame,
  /// modelEndEffectorFrame(). Each radius includes the attachment padding.
  std::vector<std::array<double, 4>> attached_spheres;
  /// Meters by which every obstacle is inflated when the collision environment is built.
  double obstacle_padding = 0.0;
  bool operator==(const CollisionScene &) const = default;
};

/// @brief Collision environment built from a CollisionScene, shareable across requests.
class GEODEX_MOVEIT_BACKEND_API CollisionEnvironment
{
public:
  virtual ~CollisionEnvironment() = default;

  /// @brief Build the environment for geodex's collision model of @p robot, every obstacle
  /// inflated by the scene's obstacle padding.
  ///
  /// @throw std::invalid_argument when geodex does not have a collision model for @p robot, or the
  /// padding is negative or not finite.
  static std::shared_ptr<const CollisionEnvironment> create(
    const std::string & robot, const CollisionScene & scene);

  /// @brief A validity oracle over this environment, for one request and one thread.
  ///
  /// Oracles made from one environment may run concurrently. With an attached object, each check
  /// poses the attachment inside the collision scene, and each oracle gets its own copy of that
  /// scene.
  virtual std::shared_ptr<const ValidityOracle> makeOracle() const = 0;
};

}  // namespace geodex_moveit
