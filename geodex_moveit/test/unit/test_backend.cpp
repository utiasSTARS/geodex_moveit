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
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geodex_moveit/backend.hpp"
#include "gtest/gtest.h"

namespace geodex_moveit
{
namespace
{

/// Validity from a predicate over the configuration.
class FnOracle : public ValidityOracle
{
public:
  FnOracle(int dim, std::function<bool(const double *)> fn) : dim_(dim), fn_(std::move(fn)) {}
  bool isValid(const double * q) const override { return fn_(q); }
  int dimension() const override { return dim_; }

private:
  int dim_;
  std::function<bool(const double *)> fn_;
};

PlanRequest box(
  int n, double lo, double hi, std::vector<double> start, std::vector<double> goal, double budget)
{
  PlanRequest r;
  r.lower.assign(n, lo);
  r.upper.assign(n, hi);
  r.start = std::move(start);
  r.goal = std::move(goal);
  r.time_budget = budget;
  return r;
}

TEST(Backend, CreateChecksRobotMetricAndDimension)
{
  EXPECT_NO_THROW(PlannerBackend::create("panda", "euclidean", 7));
  EXPECT_NO_THROW(PlannerBackend::create("panda", "kinetic_energy", 7));
  EXPECT_NO_THROW(PlannerBackend::create("", "euclidean", 3));
  EXPECT_THROW(PlannerBackend::create("panda", "euclidean", 1), std::invalid_argument);
  EXPECT_THROW(PlannerBackend::create("no_such_robot", "euclidean", 7), std::invalid_argument);
  EXPECT_THROW(PlannerBackend::create("", "kinetic_energy", 7), std::invalid_argument);
  EXPECT_THROW(PlannerBackend::create("panda", "no_such_metric", 7), std::invalid_argument);
  EXPECT_EQ(robotDimension("panda"), 7);
  EXPECT_EQ(robotDimension("no_such_robot"), -1);
  EXPECT_TRUE(isRegisteredRobot("panda"));
  EXPECT_FALSE(isRegisteredRobot(""));
}

// Every robot geodex registers is served or refused with a reason. The plugin does not keep its
// own list of robots.
TEST(Backend, RegisteredRobotsAreHandledGenerically)
{
  ASSERT_FALSE(registeredRobots().empty());
  for (const auto & name : registeredRobots()) {
    const int n = robotDimension(name);
    ASSERT_GT(n, 0) << name;
    if (hasMobileBase(name)) {
      try {
        PlannerBackend::create(name, "euclidean", n);
        ADD_FAILURE() << name << " has a mobile base and must be refused";
      } catch (const std::invalid_argument & e) {
        EXPECT_NE(std::string(e.what()).find("mobile base"), std::string::npos) << e.what();
      }
      continue;
    }
    EXPECT_NO_THROW(PlannerBackend::create(name, "euclidean", n)) << name;
    if (hasMassMatrix(name)) {
      EXPECT_NO_THROW(PlannerBackend::create(name, "kinetic_energy", n)) << name;
    }
  }
}

TEST(Backend, SolvesAndKeepsEndpointsExact)
{
  const auto backend = PlannerBackend::create("", "euclidean", 3);
  const FnOracle free(3, [](const double *) { return true; });
  PlannerSettings params;
  params.stop_at_first_solution = true;
  const auto req = box(3, -1.0, 1.0, {-0.5, -0.5, 0.0}, {0.5, 0.5, 0.1}, 1.0);
  const PlanResult r = backend->plan(req, free, params);
  ASSERT_EQ(r.status, PlanResult::Status::kSolved) << r.message;
  ASSERT_GE(r.path.size(), 2u);
  EXPECT_EQ(r.path.front(), req.start);
  EXPECT_EQ(r.path.back(), req.goal);
  EXPECT_GT(r.iterations, 0) << "G-RRT* reports its iteration count";
}

// The search reports the time and the iteration count of its first exact solution. The time is
// positive and at most the search time. A search that stops at its first solution stops in that
// iteration.
TEST(Backend, ReportsTheFirstSolution)
{
  const auto backend = PlannerBackend::create("", "euclidean", 2);
  const FnOracle oracle(2, [](const double * q) { return std::abs(q[0]) > 0.07 || q[1] > 0.62; });
  for (const bool first : {false, true}) {
    PlannerSettings settings;
    settings.smooth = false;
    settings.seed = 5;
    settings.max_iterations = 500;
    settings.stop_at_first_solution = first;
    const auto req = box(2, -1.0, 1.0, {-0.5, 0.0}, {0.5, 0.0}, 5.0);
    const PlanResult r = backend->plan(req, oracle, settings);
    ASSERT_EQ(r.status, PlanResult::Status::kSolved) << r.message;
    EXPECT_GT(r.first_solution_time_s, 0.0) << "first " << first;
    EXPECT_LE(r.first_solution_time_s, r.search_time_s) << "first " << first;
    EXPECT_GE(r.first_solution_iterations, 1) << "first " << first;
    EXPECT_LE(r.first_solution_iterations, r.iterations) << "first " << first;
    if (first) {
      EXPECT_EQ(r.first_solution_iterations, r.iterations);
    } else {
      EXPECT_EQ(r.iterations, 500);
    }
  }
}

TEST(Backend, RoutesAroundAnObstacle)
{
  const auto backend = PlannerBackend::create("", "euclidean", 2);
  // A wall at x = 0 has a gap above y = 0.6. The planner sees the wall inflated by more than half
  // the collision resolution. No chord between checked samples reaches the true wall.
  const auto valid = [](const double * q) { return std::abs(q[0]) > 0.05 || q[1] > 0.6; };
  const FnOracle oracle(2, [](const double * q) { return std::abs(q[0]) > 0.07 || q[1] > 0.62; });
  PlannerSettings params;
  params.collision_check_resolution = 0.01;
  for (const auto & id : plannerIds()) {
    auto req = box(2, -1.0, 1.0, {-0.5, 0.0}, {0.5, 0.0}, 0.5);
    req.planner_id = id;
    const PlanResult r = backend->plan(req, oracle, params);
    ASSERT_EQ(r.status, PlanResult::Status::kSolved) << id << ": " << r.message;
    for (std::size_t i = 0; i + 1 < r.path.size(); ++i) {
      for (int k = 0; k <= 50; ++k) {
        const double t = k / 50.0;
        const double q[2] = {
          r.path[i][0] + t * (r.path[i + 1][0] - r.path[i][0]),
          r.path[i][1] + t * (r.path[i + 1][1] - r.path[i][1])};
        ASSERT_TRUE(valid(q)) << id << " edge " << i;
      }
    }
  }
}

// Records every configuration it is asked about.
class RecordingOracle : public ValidityOracle
{
public:
  explicit RecordingOracle(int dim) : dim_(dim) {}
  bool isValid(const double * q) const override
  {
    checked_.insert(std::vector<double>(q, q + dim_));
    return std::abs(q[0]) > 0.07 || q[1] > 0.62;
  }
  int batchWidth() const override { return 8; }
  int dimension() const override { return dim_; }
  bool wasChecked(const std::vector<double> & q) const { return checked_.count(q) > 0; }

private:
  int dim_;
  mutable std::set<std::vector<double>> checked_;
};

// Every returned state is one that the motion validator checked, value for value.
TEST(Backend, ReturnedStatesWereChecked)
{
  const auto backend = PlannerBackend::create("", "euclidean", 2);
  for (const bool edge_validator : {true, false}) {
    PlannerSettings settings;
    settings.smooth = false;
    settings.collision_check_resolution = 0.03;
    settings.edge_motion_validator = edge_validator;
    const RecordingOracle oracle(2);
    auto req = box(2, -1.0, 1.0, {-0.5, 0.0}, {0.5, 0.0}, 0.3);
    const PlanResult r = backend->plan(req, oracle, settings);
    ASSERT_EQ(r.status, PlanResult::Status::kSolved) << r.message;
    ASSERT_GT(r.path.size(), 2u);
    for (std::size_t i = 0; i < r.path.size(); ++i) {
      EXPECT_TRUE(oracle.wasChecked(r.path[i]))
        << "edge validator " << edge_validator << ", waypoint " << i;
    }
  }
}

/// Logs the configuration of every point check in call order. Batched checks answer without the
/// log.
class SequenceOracle : public ValidityOracle
{
public:
  SequenceOracle(int dim, std::function<bool(const double *)> fn) : dim_(dim), fn_(std::move(fn)) {}
  bool isValid(const double * q) const override
  {
    log_.emplace_back(q, q + dim_);
    return fn_(q);
  }
  int batchWidth() const override { return 8; }
  bool allValid(const double * q, int count) const override
  {
    for (int i = 0; i < count; ++i) {
      if (!fn_(q + static_cast<std::ptrdiff_t>(i) * dim_)) return false;
    }
    return true;
  }
  int dimension() const override { return dim_; }
  const std::vector<std::vector<double>> & log() const { return log_; }

private:
  int dim_;
  std::function<bool(const double *)> fn_;
  mutable std::vector<std::vector<double>> log_;
};

/// The densified path that plan() checked last, the point checks from the last check of the start
/// onwards. Checks that it runs from @p path's start to its goal through every waypoint of @p path
/// in order, along the chords between them, with at most @p resolution between neighbors.
std::vector<std::vector<double>> densifiedPath(
  const std::vector<std::vector<double>> & path, const std::vector<std::vector<double>> & log,
  double resolution)
{
  std::vector<std::vector<double>> dense;
  const auto start = std::find(log.rbegin(), log.rend(), path.front());
  if (start == log.rend()) {
    ADD_FAILURE() << "the start was never checked";
    return dense;
  }
  dense.assign(start.base() - 1, log.end());
  EXPECT_EQ(dense.back(), path.back());
  std::size_t edge = 0;
  for (std::size_t k = 0; k < dense.size(); ++k) {
    if (edge + 1 < path.size() && dense[k] == path[edge + 1]) ++edge;
    if (k > 0) {
      double d2 = 0.0;
      for (std::size_t j = 0; j < dense[k].size(); ++j) {
        d2 += (dense[k][j] - dense[k - 1][j]) * (dense[k][j] - dense[k - 1][j]);
      }
      EXPECT_LE(std::sqrt(d2), resolution * (1.0 + 1e-9)) << "densified state " << k;
    }
    if (edge + 1 < path.size() && path[edge] != path[edge + 1]) {
      // The state lies on the chord from waypoint edge to waypoint edge + 1.
      const auto & a = path[edge];
      const auto & b = path[edge + 1];
      double ab2 = 0.0;
      double t = 0.0;
      for (std::size_t j = 0; j < a.size(); ++j) {
        ab2 += (b[j] - a[j]) * (b[j] - a[j]);
        t += (dense[k][j] - a[j]) * (b[j] - a[j]);
      }
      t /= ab2;
      double off2 = 0.0;
      for (std::size_t j = 0; j < a.size(); ++j) {
        const double e = dense[k][j] - (a[j] + t * (b[j] - a[j]));
        off2 += e * e;
      }
      EXPECT_LE(std::sqrt(off2), 1e-9) << "densified state " << k << " off edge " << edge;
    }
  }
  EXPECT_EQ(edge + 1, path.size()) << "the densified path skips a waypoint";
  return dense;
}

// The plan returns the planner's waypoints, without smoothing, or the smoother's. It checked every
// state of the path densified on the motion validator's grid, which has several times more states,
// and each of them clears the true wall. The smoother spaces its waypoints 0.05 apart.
TEST(Backend, ReturnsWaypointsAndChecksTheDensifiedPath)
{
  const auto backend = PlannerBackend::create("", "euclidean", 2);
  const auto valid = [](const double * q) { return std::abs(q[0]) > 0.05 || q[1] > 0.6; };
  for (const bool smooth : {false, true}) {
    const SequenceOracle oracle(
      2, [](const double * q) { return std::abs(q[0]) > 0.07 || q[1] > 0.62; });
    PlannerSettings settings;
    settings.smooth = smooth;
    settings.round_corners = false;
    settings.output_spacing = 0.05;
    settings.collision_check_resolution = 0.01;
    settings.stop_at_first_solution = true;
    settings.seed = 4;
    const auto req = box(2, -1.0, 1.0, {-0.5, 0.0}, {0.5, 0.0}, 5.0);
    const PlanResult r = backend->plan(req, oracle, settings);
    ASSERT_EQ(r.status, PlanResult::Status::kSolved) << r.message;
    ASSERT_EQ(r.smoothed, smooth) << "stage " << r.smoothing_stage;
    if (!smooth) EXPECT_EQ(r.path.size(), r.raw_waypoints);
    const auto dense = densifiedPath(r.path, oracle.log(), settings.collision_check_resolution);
    EXPECT_GE(dense.size(), (smooth ? 3 : 10) * r.path.size()) << "smooth " << smooth;
    for (std::size_t k = 0; k < dense.size(); ++k) {
      EXPECT_TRUE(valid(dense[k].data())) << "smooth " << smooth << ", densified state " << k;
    }
  }
}

// The smoother checks against the planner's validity, bounds and motion validator. The planner
// and the smoother keep the margin of the evenly spaced waypoints, half the check spacing plus
// the corner tolerance. The plan returns the smoothed path, and every densified edge of it stays
// clear of the true wall.
TEST(Backend, SmoothedPathIsCertified)
{
  const auto backend = PlannerBackend::create("", "euclidean", 2);
  const auto valid = [](const double * q) { return std::abs(q[0]) > 0.05 || q[1] > 0.6; };
  const FnOracle oracle(2, [](const double * q) { return std::abs(q[0]) > 0.07 || q[1] > 0.62; });
  constexpr double kMargin = 0.5 * 0.01 + 1e-4;
  const FnOracle planning(
    2, [](const double * q) { return std::abs(q[0]) > 0.07 + kMargin || q[1] > 0.62 + kMargin; });
  for (const bool edge_validator : {true, false}) {
    PlannerSettings settings;
    settings.collision_check_resolution = 0.01;
    settings.edge_motion_validator = edge_validator;
    auto req = box(2, -1.0, 1.0, {-0.5, 0.0}, {0.5, 0.0}, 0.5);
    req.planning_validity = &planning;
    const PlanResult r = backend->plan(req, oracle, settings);
    ASSERT_EQ(r.status, PlanResult::Status::kSolved) << r.message;
    EXPECT_TRUE(r.smoothed) << "stage " << r.smoothing_stage;
    EXPECT_GE(r.smoothing_stage, 0);
    EXPECT_LT(r.smoothing_stage, 3);
    EXPECT_EQ(r.path.front(), req.start);
    EXPECT_EQ(r.path.back(), req.goal);
    for (std::size_t i = 0; i + 1 < r.path.size(); ++i) {
      for (int k = 0; k <= 20; ++k) {
        const double t = k / 20.0;
        const double q[2] = {
          r.path[i][0] + t * (r.path[i + 1][0] - r.path[i][0]),
          r.path[i][1] + t * (r.path[i + 1][1] - r.path[i][1])};
        ASSERT_TRUE(valid(q)) << "edge validator " << edge_validator << ", edge " << i;
        ASSERT_LE(std::abs(q[0]), 1.0);
        ASSERT_LE(std::abs(q[1]), 1.0);
      }
    }
  }
}

// A waypoint outside the request's bounds fails the smoother's check, whatever the collision
// oracle says.
TEST(Backend, SmoothingChecksTheBounds)
{
  const auto backend = PlannerBackend::create("", "euclidean", 2);
  const FnOracle free(2, [](const double *) { return true; });
  SmoothRequest req;
  req.lower = {-1.0, -1.0};
  req.upper = {1.0, 1.0};
  req.path = {{-0.5, 0.0}, {0.0, 0.5}, {0.5, 0.0}};
  const SmoothResult inside = backend->smooth(req, free, PlannerSettings{});
  EXPECT_TRUE(inside.collision_free);
  EXPECT_EQ(inside.path.front(), req.path.front());
  EXPECT_EQ(inside.path.back(), req.path.back());
  req.path.back() = {1.5, 0.0};
  EXPECT_FALSE(backend->smooth(req, free, PlannerSettings{}).collision_free);
}

// The corner rounding setting reaches the smoother. Around a disc, the smoother returns a
// different path with rounded corners than without, and rounding adds at most 0.1 percent to the
// length.
TEST(Backend, RoundCornersSettingReachesTheSmoother)
{
  const auto backend = PlannerBackend::create("", "euclidean", 2);
  const FnOracle oracle(2, [](const double * q) { return q[0] * q[0] + q[1] * q[1] > 0.09; });
  SmoothRequest req;
  req.lower = {-1.0, -1.0};
  req.upper = {1.0, 1.0};
  req.path = {{-0.6, 0.0}, {-0.45, 0.45}, {0.0, 0.6}, {0.45, 0.45}, {0.6, 0.0}};
  PlannerSettings rounded;
  rounded.collision_check_resolution = 0.005;
  PlannerSettings cornered = rounded;
  cornered.round_corners = false;
  const SmoothResult a = backend->smooth(req, oracle, rounded);
  const SmoothResult b = backend->smooth(req, oracle, cornered);
  ASSERT_TRUE(a.collision_free);
  ASSERT_TRUE(b.collision_free);
  EXPECT_EQ(a.path.front(), req.path.front());
  EXPECT_EQ(a.path.back(), req.path.back());
  EXPECT_NE(a.path, b.path);
  EXPECT_LE(a.length, b.length * (1.0 + 1e-3));
}

TEST(Backend, NoExactSolutionIsNotSolved)
{
  const auto backend = PlannerBackend::create("", "euclidean", 2);
  // Only small neighborhoods of the endpoints are valid. No exact path exists.
  const FnOracle oracle(2, [](const double * q) {
    return std::hypot(q[0] + 0.5, q[1]) < 0.1 || std::hypot(q[0] - 0.5, q[1]) < 0.1;
  });
  for (const auto & id : plannerIds()) {
    auto req = box(2, -1.0, 1.0, {-0.5, 0.0}, {0.5, 0.0}, 0.2);
    req.planner_id = id;
    const PlanResult r = backend->plan(req, oracle, PlannerSettings{});
    EXPECT_EQ(r.status, PlanResult::Status::kNoExactSolution) << id;
    EXPECT_TRUE(r.path.empty()) << id;
    EXPECT_EQ(r.first_solution_time_s, -1.0) << id;
  }
}

TEST(Backend, CancelStopsTheSearch)
{
  const auto backend = PlannerBackend::create("", "euclidean", 2);
  const FnOracle oracle(2, [](const double * q) {
    return std::hypot(q[0] + 0.5, q[1]) < 0.1 || std::hypot(q[0] - 0.5, q[1]) < 0.1;
  });
  std::atomic_bool cancel{true};
  auto req = box(2, -1.0, 1.0, {-0.5, 0.0}, {0.5, 0.0}, 30.0);
  req.cancel = &cancel;
  const auto t0 = std::chrono::steady_clock::now();
  const PlanResult r = backend->plan(req, oracle, PlannerSettings{});
  EXPECT_EQ(r.status, PlanResult::Status::kCanceled);
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), 5.0);
}

TEST(Backend, RejectsUnknownPlannerAndWrongDimension)
{
  EXPECT_EQ(plannerIds(), std::vector<std::string>{"GreedyRRTstar"});
  const auto backend = PlannerBackend::create("", "euclidean", 2);
  const FnOracle free(2, [](const double *) { return true; });
  auto req = box(2, -1.0, 1.0, {0.0, 0.0}, {0.5, 0.5}, 0.1);
  for (const std::string id : {"NoSuchPlanner", "RRTConnect"}) {
    req.planner_id = id;
    EXPECT_EQ(backend->plan(req, free, PlannerSettings{}).status, PlanResult::Status::kInvalidInput)
      << id;
  }
  auto bad = box(3, -1.0, 1.0, {0.0, 0.0, 0.0}, {0.5, 0.5, 0.5}, 0.1);
  EXPECT_EQ(backend->plan(bad, free, PlannerSettings{}).status, PlanResult::Status::kInvalidInput);
}

TEST(Backend, ConcurrentPlansOnOneBackend)
{
  const auto backend = PlannerBackend::create("panda", "kinetic_energy", 7);
  const FnOracle free(7, [](const double *) { return true; });
  PlannerSettings params;
  params.stop_at_first_solution = true;
  const std::vector<double> lo{-2.8, -1.7, -2.8, -3.0, -2.8, 0.0, -2.8};
  const std::vector<double> hi{2.8, 1.7, 2.8, -0.1, 2.8, 3.7, 2.8};
  std::vector<PlanResult> results(4);
  std::vector<std::thread> threads;
  for (int t = 0; t < 4; ++t) {
    threads.emplace_back([&, t] {
      PlanRequest req;
      req.lower = lo;
      req.upper = hi;
      req.start = {0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.0};
      req.goal = {0.2 * (t + 1), 0.3, -0.2, -1.5, 0.1, 2.0, 0.5};
      req.time_budget = 2.0;
      results[static_cast<std::size_t>(t)] = backend->plan(req, free, params);
    });
  }
  for (auto & th : threads) th.join();
  for (int t = 0; t < 4; ++t) {
    const auto & r = results[static_cast<std::size_t>(t)];
    ASSERT_EQ(r.status, PlanResult::Status::kSolved) << r.message;
    EXPECT_DOUBLE_EQ(r.path.back()[0], 0.2 * (t + 1));
  }
}

const std::vector<double> kPandaLower{-2.8, -1.7, -2.8, -3.0, -2.8, 0.0, -2.8};
const std::vector<double> kPandaUpper{2.8, 1.7, 2.8, -0.1, 2.8, 3.7, 2.8};

std::vector<std::vector<double>> pandaSamples(int n, unsigned seed)
{
  std::mt19937 rng(seed);
  std::vector<std::vector<double>> qs;
  for (int i = 0; i < n; ++i) {
    std::vector<double> q(7);
    for (std::size_t j = 0; j < 7; ++j) {
      q[j] = std::uniform_real_distribution<double>(kPandaLower[j], kPandaUpper[j])(rng);
    }
    qs.push_back(q);
  }
  return qs;
}

// Concurrent requests share one cached environment. With an attached object, every check poses the
// attachment, and each oracle holds its own copy of the scene. Four threads checking one
// environment give the single-threaded answers.
TEST(Backend, SharedEnvironmentWithAttachedObjectIsThreadSafe)
{
  CollisionScene s;
  s.boxes.push_back(SceneBox{{0.5, 0.0, 0.3}, {0.05, 0.4, 0.3}});
  s.boxes.push_back(SceneBox{{-0.4, 0.4, 0.5}, {0.2, 0.05, 0.3}});
  s.boxes.push_back(SceneBox{{0.0, -0.5, 0.2}, {0.3, 0.05, 0.2}});
  s.attached_spheres = {{0.0, 0.0, 0.15, 0.12}, {0.0, 0.1, 0.25, 0.12}, {0.0, -0.1, 0.25, 0.12}};
  const auto shared = CollisionEnvironment::create("panda", s);
  const auto qs = pandaSamples(4000, 7);
  std::vector<char> truth;
  {
    const auto oracle = shared->makeOracle();
    for (const auto & q : qs) truth.push_back(oracle->isValid(q.data()) ? 1 : 0);
  }
  const auto valid = std::count(truth.begin(), truth.end(), 1);
  ASSERT_GT(valid, 400);
  ASSERT_LT(valid, 3600) << "the samples must mix valid and invalid configurations";
  std::atomic<int> mismatches{0};
  std::atomic<bool> go{false};
  std::vector<std::thread> pool;
  for (int t = 0; t < 4; ++t) {
    pool.emplace_back([&, t] {
      const auto oracle = shared->makeOracle();
      while (!go.load()) {
      }
      for (int rep = 0; rep < 5; ++rep) {
        for (std::size_t k = 0; k < qs.size(); ++k) {
          const std::size_t i = (k * 7919 + static_cast<std::size_t>(t) * 104729) % qs.size();
          if (oracle->isValid(qs[i].data()) != static_cast<bool>(truth[i])) ++mismatches;
        }
      }
    });
  }
  go.store(true);
  for (auto & th : pool) th.join();
  EXPECT_EQ(mismatches.load(), 0);
}

// A wrist-only motion has to cross an invalid band of joint 7. The default resolution bounds the
// joint-space distance between checks for every metric, and no metric reports a path. A
// resolution of 0 is refused.
TEST(Backend, DefaultResolutionHoldsUnderEachMetric)
{
  const FnOracle band(7, [](const double * q) { return std::abs(q[6]) > 0.1; });
  for (const std::string metric : {"euclidean", "kinetic_energy"}) {
    const auto backend = PlannerBackend::create("panda", metric, 7);
    PlanRequest req;
    req.lower = kPandaLower;
    req.upper = kPandaUpper;
    req.start = {0.0, -0.5, 0.0, -2.0, 0.0, 1.5, -1.2};
    req.goal = {0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 1.3};
    req.time_budget = 1.0;
    PlannerSettings settings;
    settings.stop_at_first_solution = true;
    EXPECT_EQ(backend->plan(req, band, settings).status, PlanResult::Status::kNoExactSolution)
      << metric;
    settings.collision_check_resolution = 0.0;
    EXPECT_EQ(backend->plan(req, band, settings).status, PlanResult::Status::kInvalidInput)
      << metric;
  }
}

/// Counts the checked configurations outside a box, point checks and batched checks alike.
class BoundsRecorder : public ValidityOracle
{
public:
  BoundsRecorder(
    std::vector<double> lower, std::vector<double> upper, std::function<bool(const double *)> fn)
  : lower_(std::move(lower)), upper_(std::move(upper)), fn_(std::move(fn))
  {
  }
  bool isValid(const double * q) const override
  {
    ++checked_;
    for (std::size_t j = 0; j < lower_.size(); ++j) {
      if (!(q[j] >= lower_[j] && q[j] <= upper_[j])) {
        ++outside_;
        break;
      }
    }
    return fn_(q);
  }
  int batchWidth() const override { return 8; }
  int dimension() const override { return static_cast<int>(lower_.size()); }
  std::int64_t checked() const { return checked_; }
  std::int64_t outside() const { return outside_; }

private:
  std::vector<double> lower_;
  std::vector<double> upper_;
  std::function<bool(const double *)> fn_;
  mutable std::int64_t checked_ = 0;
  mutable std::int64_t outside_ = 0;
};

// The FR3 datasheet bounds of joints 4 and 6 lie outside [-1, 1]. Both endpoints hold joint 4 at
// -2.8 and joint 6 at 3.9. A wall across joint 1 leaves one gap, at joint 6 above 4.1 and joint 4
// below -2.5. The robot backends of both metrics and the backend without a robot sample inside the
// request's bounds, reach the gap and do not check any configuration outside the bounds.
TEST(Backend, SamplesInsideTheRequestBounds)
{
  const std::vector<double> lower{-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159};
  const std::vector<double> upper{2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159};
  const auto wall = [](const double * q) {
    return std::abs(q[0]) > 0.2 || (q[5] > 4.1 && q[3] < -2.5);
  };
  const std::vector<std::pair<std::string, std::string>> backends{
    {"fr3_arm_gripper", "euclidean"}, {"fr3_arm_gripper", "kinetic_energy"}, {"", "euclidean"}};
  for (const auto & [robot, metric] : backends) {
    const auto backend = PlannerBackend::create(robot, metric, 7);
    for (const std::int64_t seed : {1, 2, 3}) {
      const std::string tag = "robot '" + robot + "', " + metric + ", seed " + std::to_string(seed);
      const BoundsRecorder oracle(lower, upper, wall);
      PlanRequest req;
      req.lower = lower;
      req.upper = upper;
      req.start = {-1.2, 0.0, 0.0, -2.8, 0.0, 3.9, 0.0};
      req.goal = {1.2, 0.0, 0.0, -2.8, 0.0, 3.9, 0.0};
      req.time_budget = 10.0;
      PlannerSettings settings;
      settings.smooth = false;
      settings.stop_at_first_solution = true;
      settings.seed = seed;
      const PlanResult r = backend->plan(req, oracle, settings);
      ASSERT_EQ(r.status, PlanResult::Status::kSolved) << tag << ": " << r.message;
      EXPECT_GT(oracle.checked(), 0) << tag;
      EXPECT_EQ(oracle.outside(), 0) << tag;
      double top = 0.0;
      for (const auto & q : r.path) {
        top = std::max(top, q[5]);
        for (std::size_t j = 0; j < q.size(); ++j) {
          EXPECT_GE(q[j], lower[j]) << tag << ", joint " << j + 1;
          EXPECT_LE(q[j], upper[j]) << tag << ", joint " << j + 1;
        }
      }
      EXPECT_GT(top, 4.1) << tag << ": the path passes the gap";
    }
  }
}

// An oracle whose batched answer disagrees with its point answer lets the planner's edges cross
// a wall without a gap. The point check of every densified state refuses the path.
TEST(Backend, EveryReturnedStateIsVerified)
{
  class Inconsistent : public ValidityOracle
  {
  public:
    bool isValid(const double * q) const override { return std::abs(q[0]) > 0.05; }
    bool allValid(const double *, int) const override { return true; }
    int batchWidth() const override { return 8; }
    int dimension() const override { return 2; }
  };
  const auto backend = PlannerBackend::create("", "euclidean", 2);
  PlannerSettings settings;
  settings.smooth = false;
  settings.collision_check_resolution = 0.01;
  settings.stop_at_first_solution = true;
  const auto req = box(2, -1.0, 1.0, {-0.5, 0.0}, {0.5, 0.0}, 0.5);
  const PlanResult r = backend->plan(req, Inconsistent{}, settings);
  EXPECT_EQ(r.status, PlanResult::Status::kInvalidPath) << r.message;
  EXPECT_TRUE(r.path.empty());
}

/// Waits 100 microseconds. A sleep this short lasts much longer on macOS.
void spin100us()
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::microseconds(100);
  while (std::chrono::steady_clock::now() < end) {
    std::this_thread::yield();
  }
}

// A seeded request is timed once with room to spare, then run again with a budget that ends
// during smoothing. The second call ends at the budget with the planner's path. A cancel during
// smoothing ends the call at once. The slow oracle makes smoothing long compared with scheduling
// noise.
TEST(Backend, SmoothingKeepsTheBudgetAndStopsOnCancel)
{
  const auto backend = PlannerBackend::create("", "euclidean", 2);
  const FnOracle slow(2, [](const double * q) {
    spin100us();
    return std::abs(q[0]) > 0.07 || q[1] > 0.3;
  });
  // The planner and the smoother keep the margin of the evenly spaced waypoints.
  constexpr double kMargin = 0.5 * 0.005 + 1e-4;
  const FnOracle slow_planning(2, [](const double * q) {
    spin100us();
    return std::abs(q[0]) > 0.07 + kMargin || q[1] > 0.3 + kMargin;
  });
  PlannerSettings settings;
  settings.stop_at_first_solution = true;
  settings.collision_check_resolution = 0.005;
  settings.seed = 3;
  auto req = box(2, -1.0, 1.0, {-0.5, 0.0}, {0.5, 0.0}, 60.0);
  req.planning_validity = &slow_planning;
  const PlanResult free_run = backend->plan(req, slow, settings);
  ASSERT_EQ(free_run.status, PlanResult::Status::kSolved) << free_run.message;
  ASSERT_TRUE(free_run.smoothed);
  const double search = free_run.search_time_s;
  const double smoothing = free_run.smooth_time_s;
  ASSERT_GT(smoothing, 0.2) << "smoothing must take long against scheduling noise";

  req.time_budget = search + 0.3 * smoothing;
  const PlanResult cut = backend->plan(req, slow, settings);
  ASSERT_EQ(cut.status, PlanResult::Status::kSolved) << cut.message;
  EXPECT_TRUE(cut.smoothing_stopped);
  EXPECT_FALSE(cut.smoothed);
  EXPECT_LT(cut.reported_time_s, req.time_budget + 0.1);

  std::atomic_bool cancel{false};
  req.time_budget = 60.0;
  req.cancel = &cancel;
  std::thread stopper([&] {
    std::this_thread::sleep_for(std::chrono::duration<double>(search + 0.3 * smoothing));
    cancel.store(true);
  });
  const auto t0 = std::chrono::steady_clock::now();
  const PlanResult c = backend->plan(req, slow, settings);
  stopper.join();
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_LT(wall, search + 0.3 * smoothing + 0.2);
  EXPECT_FALSE(c.smoothed);
}

// With a seed and an iteration budget, two requests return the same path.
TEST(Backend, SeedAndIterationBudgetReplay)
{
  const FnOracle wall(7, [](const double * q) { return std::abs(q[0]) > 0.2 || q[3] > -1.0; });
  for (const std::string metric : {"euclidean", "kinetic_energy"}) {
    const auto backend = PlannerBackend::create("panda", metric, 7);
    PlanRequest req;
    req.lower = kPandaLower;
    req.upper = kPandaUpper;
    req.start = {-0.8, -0.5, 0.0, -2.0, 0.0, 1.5, 0.3};
    req.goal = {0.8, -0.5, 0.0, -2.0, 0.0, 1.5, 0.3};
    req.time_budget = 30.0;
    PlannerSettings settings;
    settings.seed = 11;
    settings.max_iterations = 300;
    const PlanResult a = backend->plan(req, wall, settings);
    const PlanResult b = backend->plan(req, wall, settings);
    ASSERT_EQ(a.status, PlanResult::Status::kSolved) << metric << ": " << a.message;
    EXPECT_EQ(a.iterations, 300) << metric;
    EXPECT_EQ(a.path, b.path) << metric;
  }
}

// The model queries that the plugin checks a planning group and an attached object against.
TEST(Backend, ModelJointsFrameAndBounds)
{
  const std::vector<std::string> panda{"panda_joint1", "panda_joint2", "panda_joint3",
                                       "panda_joint4", "panda_joint5", "panda_joint6",
                                       "panda_joint7"};
  EXPECT_EQ(modelJointNames("panda"), panda);
  EXPECT_EQ(modelEndEffectorFrame("panda"), "panda_grasptarget");
  EXPECT_TRUE(modelJointNames("no_such_robot").empty());
  EXPECT_TRUE(modelEndEffectorFrame("").empty());
  std::vector<double> lo, hi;
  ASSERT_TRUE(modelJointBounds("panda", lo, hi));
  ASSERT_EQ(lo.size(), 7u);
  for (std::size_t i = 0; i < 7; ++i) EXPECT_LT(lo[i], hi[i]);
  for (const auto & name : registeredRobots()) {
    if (hasCollisionModel(name)) {
      EXPECT_EQ(static_cast<int>(modelJointNames(name).size()), robotDimension(name)) << name;
      EXPECT_FALSE(modelEndEffectorFrame(name).empty()) << name;
    }
  }
}

}  // namespace
}  // namespace geodex_moveit
