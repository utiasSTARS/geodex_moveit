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

// This library holds every geodex and OMPL call of the plugin. It sees only geodex's Eigen and
// does not include any ROS or MoveIt header.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "Eigen/Core"
#include "geodex/algorithm/path_smoothing.hpp"
#include "geodex/geodex.hpp"
#include "geodex/integration/ompl/geodex_optimization_objective.hpp"
#include "geodex/integration/ompl/geodex_state_space.hpp"
#include "geodex/integration/ompl/validity_checker.hpp"
#include "geodex/integration/vamp/registry.hpp"
#include "geodex/robots/mass_lower_bound.hpp"
#include "geodex/robots/mass_matrix.hpp"
#include "geodex_moveit/backend.hpp"
#include "ompl/base/MotionValidator.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/ScopedState.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/geometric/planners/rrt/GreedyRRTstar.h"
#include "ompl/util/Console.h"
#include "ompl/util/RandomNumbers.h"

namespace geodex_moveit
{
namespace
{

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace gio = geodex::integration::ompl;
namespace giv = geodex::integration::vamp;
namespace gh = geodex::heuristics;
namespace gr = geodex::robots;

constexpr const char * kGreedyRRTstar = "GreedyRRTstar";

/// Forwards OMPL's console output to the newest live sink. Errors and warnings keep their level,
/// and every other message goes out at debug level.
class SinkOutputHandler : public ompl::msg::OutputHandler
{
public:
  void log(
    const std::string & text, ompl::msg::LogLevel level, const char * /*filename*/,
    int /*line*/) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (sinks_.empty()) return;
    // OMPL reports the deliberate reseed of a seeded request as an error. Log it at debug level.
    const bool reseed = text.rfind("Random number generation already started", 0) == 0;
    sinks_.back().second(
      reseed                          ? 0
      : level == ompl::msg::LOG_ERROR ? 3
      : level == ompl::msg::LOG_WARN  ? 2
                                      : 0,
      text);
  }

  std::shared_ptr<void> add(LogSink sink)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::uint64_t id = next_++;
    sinks_.emplace_back(id, std::move(sink));
    return std::shared_ptr<void>(nullptr, [this, id](void *) { remove(id); });
  }

private:
  void remove(std::uint64_t id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::erase_if(sinks_, [id](const auto & s) { return s.first == id; });
  }

  std::mutex mutex_;
  std::uint64_t next_ = 0;
  std::vector<std::pair<std::uint64_t, LogSink>> sinks_;
};

SinkOutputHandler & outputHandler()
{
  static SinkOutputHandler handler;
  static std::once_flag once;
  std::call_once(once, [] { ompl::msg::useOutputHandler(&handler); });
  return handler;
}

bool vampHas(const std::string & robot)
{
  const auto names = giv::registered_robots();
  return std::find(names.begin(), names.end(), robot) != names.end();
}

Eigen::Vector3d vec(const std::array<double, 3> & a) { return {a[0], a[1], a[2]}; }

Eigen::Matrix3d mat(const std::array<double, 9> & a)
{
  Eigen::Matrix3d m;
  m << a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8];
  return m;
}

/// A copy of @p m whose base manifold samples inside [@p lower, @p upper]. The uniform samples of
/// the planner and of the informed sampler come from that box.
template <typename Base, typename Metric>
geodex::ConfigurationSpace<Base, Metric> withSamplingBounds(
  const geodex::ConfigurationSpace<Base, Metric> & m, const std::vector<double> & lower,
  const std::vector<double> & upper)
{
  Base base = m.base();
  const auto n = static_cast<Eigen::Index>(lower.size());
  base.set_sampling_bounds(
    Eigen::Map<const Eigen::VectorXd>(lower.data(), n),
    Eigen::Map<const Eigen::VectorXd>(upper.data(), n));
  return geodex::ConfigurationSpace<Base, Metric>(std::move(base), m.metric());
}

/// Validates an edge in blocks of the oracle's batch width.
///
/// The samples sit at fractions j / n of the edge for the state space's valid segment count n,
/// the same states that PathGeometric::interpolate() creates. Block b takes every nb-th sample
/// from sample n - b down, and each block spans the whole edge.
class EdgeMotionValidator : public ob::MotionValidator
{
public:
  EdgeMotionValidator(const ob::SpaceInformationPtr & si, const ValidityOracle & validity, int dim)
  : ob::MotionValidator(si),
    validity_(validity),
    dim_(dim),
    width_(std::max(1, validity.batchWidth()))
  {
    for (int i = 0; i < width_; ++i) scratch_.push_back(si_->allocState());
    buf_.assign(static_cast<std::size_t>(width_) * static_cast<std::size_t>(dim_), 0.0);
  }

  ~EdgeMotionValidator() override
  {
    for (auto * st : scratch_) si_->freeState(st);
  }

  bool checkMotion(const ob::State * s1, const ob::State * s2) const override
  {
    const auto * space = si_->getStateSpace().get();
    const unsigned int n = std::max(1u, space->validSegmentCount(s1, s2));
    const unsigned int nb = (n + static_cast<unsigned int>(width_) - 1) / width_;
    for (unsigned int b = 0; b < nb; ++b) {
      // Samples j = n - b, n - b - nb, ... down to 1. Sample n is s2, and s1 is valid on entry.
      int count = 0;
      unsigned int j = n - b;
      while (j >= 1 && count < width_) {
        auto * st = scratch_[static_cast<std::size_t>(count)];
        if (j == n) {
          si_->copyState(st, s2);
        } else {
          space->interpolate(s1, s2, static_cast<double>(j) / static_cast<double>(n), st);
        }
        space->copyToReals(tmp_, st);
        const std::size_t d = std::min(tmp_.size(), static_cast<std::size_t>(dim_));
        std::copy_n(tmp_.begin(), d, buf_.begin() + static_cast<std::ptrdiff_t>(count) * dim_);
        ++count;
        j = j > nb ? j - nb : 0;
      }
      if (count > 0 && !validity_.allValid(buf_.data(), count)) return false;
    }
    return true;
  }

  bool checkMotion(
    const ob::State * s1, const ob::State * s2,
    std::pair<ob::State *, double> & /*last_valid*/) const override
  {
    return checkMotion(s1, s2);
  }

private:
  const ValidityOracle & validity_;
  int dim_;
  int width_;
  std::vector<ob::State *> scratch_;
  mutable std::vector<double> buf_;
  mutable std::vector<double> tmp_;
};

/// The end of a request's time budget and its cancel flag.
struct Deadline
{
  std::chrono::steady_clock::time_point at = std::chrono::steady_clock::time_point::max();
  const std::atomic_bool * cancel = nullptr;

  bool passed() const
  {
    return (cancel != nullptr && cancel->load()) || std::chrono::steady_clock::now() >= at;
  }
};

/// The smoother's validity functor, for single points and for blocks of the batch width. A
/// configuration outside the request's bounds is invalid. After the deadline, every configuration
/// is invalid, and the smoother's result fails its check.
struct SmootherValidity
{
  const ValidityOracle * oracle = nullptr;
  const std::vector<double> * lower = nullptr;
  const std::vector<double> * upper = nullptr;
  int dim = 0;
  int width = 1;
  const Deadline * deadline = nullptr;
  mutable std::vector<double> block;

  bool inBounds(const double * q) const
  {
    for (int i = 0; i < dim; ++i) {
      const auto k = static_cast<std::size_t>(i);
      if (!(q[i] >= (*lower)[k] && q[i] <= (*upper)[k])) return false;
    }
    return true;
  }

  bool stopped() const { return deadline != nullptr && deadline->passed(); }

  template <typename PointT>
  bool operator()(const PointT & q) const
  {
    if (stopped()) return false;
    const Eigen::Ref<const Eigen::VectorXd> r(q);
    return inBounds(r.data()) && oracle->isValid(r.data());
  }

  std::size_t batch_size() const { return static_cast<std::size_t>(std::max(width, 1)); }

  template <typename PointT>
  bool batch(const PointT * pts, std::size_t n) const
  {
    if (stopped()) return false;
    const std::size_t w = batch_size();
    const auto d = static_cast<std::size_t>(dim);
    block.resize(w * d);
    for (std::size_t base = 0; base < n; base += w) {
      const std::size_t cnt = std::min(w, n - base);
      for (std::size_t i = 0; i < cnt; ++i) {
        for (std::size_t j = 0; j < d; ++j) {
          block[i * d + j] = pts[base + i][static_cast<Eigen::Index>(j)];
        }
        if (!inBounds(block.data() + i * d)) return false;
      }
      if (!oracle->allValid(block.data(), static_cast<int>(cnt))) return false;
    }
    return true;
  }
};

/// Collision checks with a robot's VAMP kernel. The planner's search and the smoother read its
/// robot and environment for the kernel's edge check and the sphere travel along an edge.
class VampOracle : public ValidityOracle
{
public:
  VampOracle(std::string robot, giv::EnvHandle env, int dim)
  : robot_(std::move(robot)),
    env_(std::move(env)),
    checker_(giv::make_vamp_checker(robot_, env_)),
    dim_(dim),
    width_(checker_->batch_width())
  {
  }

  bool isValid(const double * q) const override { return checker_->is_valid(q, dim_); }
  int batchWidth() const override { return width_; }
  bool allValid(const double * q, int count) const override
  {
    return checker_->all_valid(q, dim_, count);
  }
  int dimension() const override { return dim_; }

  const std::string & robot() const { return robot_; }
  const giv::EnvHandle & env() const { return env_; }

private:
  std::string robot_;
  giv::EnvHandle env_;
  std::unique_ptr<giv::CollisionChecker> checker_;
  int dim_;
  int width_;
};

template <typename ManifoldT, typename HeuristicT>
class Backend : public PlannerBackend
{
  using Point = typename ManifoldT::Point;

  /// The OMPL side of one request, the state space over the request's bounds with the planner's
  /// validity checker and motion validator.
  struct Problem
  {
    std::shared_ptr<gio::GeodexStateSpace<ManifoldT>> space;
    ob::SpaceInformationPtr si;
    std::vector<double> lower;
    std::vector<double> upper;
    double resolution = 0.0;  ///< The planner's collision resolution.
  };

public:
  Backend(ManifoldT manifold, HeuristicT heuristic, int dimension, std::string description)
  : manifold_(std::move(manifold)),
    heuristic_(std::move(heuristic)),
    dimension_(dimension),
    description_(std::move(description))
  {
  }

  int dimension() const override { return dimension_; }
  std::string describe() const override { return description_; }

  SmoothResult smooth(
    const SmoothRequest & req, const ValidityOracle & validity,
    const PlannerSettings & settings) const override
  {
    const int n = dimension_;
    if (
      static_cast<int>(req.lower.size()) != n || static_cast<int>(req.upper.size()) != n ||
      validity.dimension() != n ||
      (req.planning_validity && req.planning_validity->dimension() != n)) {
      throw std::invalid_argument(
        "smoothing request dimension does not match the planner's " + std::to_string(n));
    }
    if (!(settings.collision_check_resolution > 0.0)) {
      throw std::invalid_argument("collision_check_resolution must be positive");
    }
    std::vector<Point> input;
    input.reserve(req.path.size());
    for (const auto & q : req.path) {
      if (static_cast<int>(q.size()) != n) {
        throw std::invalid_argument("path waypoint dimension does not match the planner's");
      }
      input.push_back(Eigen::Map<const Point>(q.data(), static_cast<Eigen::Index>(q.size())));
    }
    const ValidityOracle & check = req.planning_validity ? *req.planning_validity : validity;
    const ManifoldT manifold = withSamplingBounds(manifold_, req.lower, req.upper);
    const Problem problem = makeProblem(manifold, req.lower, req.upper, check, settings);
    const auto result = smoothPoints(manifold, problem, input, check, settings, nullptr);
    SmoothResult out;
    out.collision_free = result.collision_free;
    out.stage = result.profile.fallback;
    out.length = result.length;
    out.path.reserve(result.path.size());
    for (const auto & p : result.path) out.path.emplace_back(p.data(), p.data() + p.size());
    return out;
  }

  PlanResult plan(
    const PlanRequest & req, const ValidityOracle & validity,
    const PlannerSettings & settings) const override
  {
    using Clock = std::chrono::steady_clock;
    outputHandler();
    PlanResult out;
    const int n = dimension_;
    const auto sized = [n](const std::vector<double> & v) {
      return static_cast<int>(v.size()) == n;
    };
    if (
      !sized(req.start) || !sized(req.goal) || !sized(req.lower) || !sized(req.upper) ||
      validity.dimension() != n ||
      (req.planning_validity && req.planning_validity->dimension() != n)) {
      out.status = PlanResult::Status::kInvalidInput;
      out.message = "request dimension does not match the planner's " + std::to_string(n);
      return out;
    }
    const std::string id = req.planner_id.empty() ? kGreedyRRTstar : req.planner_id;
    if (id != kGreedyRRTstar) {
      out.status = PlanResult::Status::kInvalidInput;
      out.message = "unknown planner_id '" + id + "'";
      return out;
    }
    if (!(settings.collision_check_resolution > 0.0)) {
      out.status = PlanResult::Status::kInvalidInput;
      out.message = "collision_check_resolution must be positive";
      return out;
    }
    out.planner_id = id;
    // Every OMPL generator takes its seed from one process-wide generator when it is created.
    // Requests create their generators under geodex's lock for that process-wide generator.
    std::unique_lock rng_lock(gio::detail::ompl_rng_mutex());
    if (settings.seed >= 0) {
      ompl::RNG::setSeed(static_cast<std::uint_fast32_t>(settings.seed) + 1);
    }
    // Each request plans on a private copy of the manifold. Metrics such as the kinetic-energy
    // metric hold scratch buffers. The copy samples inside the request's bounds.
    // The planner and the smoother use the stricter check, and the returned states pass validity.
    const ValidityOracle & check = req.planning_validity ? *req.planning_validity : validity;
    const ManifoldT manifold = withSamplingBounds(manifold_, req.lower, req.upper);
    const Problem problem = makeProblem(manifold, req.lower, req.upper, check, settings);
    const auto & space = problem.space;
    const ob::SpaceInformationPtr & si = problem.si;
    if (settings.seed >= 0) space->setSamplerSeed(static_cast<std::uint64_t>(settings.seed));
    og::SimpleSetup ss(si);

    ob::ScopedState<> goal(space);
    ob::ScopedState<> start(space);
    auto * g = goal.get()->as<gio::GeodexState<ManifoldT>>();
    auto * s = start.get()->as<gio::GeodexState<ManifoldT>>();
    for (int i = 0; i < n; ++i) {
      g->values[i] = req.goal[static_cast<std::size_t>(i)];
      s->values[i] = req.start[static_cast<std::size_t>(i)];
    }
    ss.setGoalState(goal);
    ss.setStartState(start);

    std::vector<double> goal_coords(req.goal);
    auto objective = std::make_shared<gio::GeodexOptimizationObjective<ManifoldT, HeuristicT>>(
      si, Eigen::Map<Eigen::VectorXd>(goal_coords.data(), n), heuristic_);
    ss.setOptimizationObjective(objective);
    objective->setGreedyBiasingRatio(settings.greedy_biasing_ratio);
    if (settings.stop_at_first_solution) {
      // A threshold of infinity makes the first finite-cost solution satisfy the objective.
      objective->setCostThreshold(ob::Cost(std::numeric_limits<double>::infinity()));
    }

    // An anytime search leaves smoothing its share of the budget. A search that stops at its first
    // solution may use the whole budget.
    const bool reserve = settings.smooth && !settings.stop_at_first_solution;
    const double search_budget =
      reserve ? req.time_budget * (1.0 - settings.smoothing_time_fraction) : req.time_budget;
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(search_budget);
    auto grrtstar = makeGreedyRRTstar(si, settings);
    if (settings.max_iterations > 0) {
      const auto cap = static_cast<unsigned int>(settings.max_iterations);
      og::GreedyRRTstar * raw = grrtstar.get();
      ptc = ob::plannerOrTerminationCondition(
        ptc, ob::PlannerTerminationCondition([raw, cap] { return raw->numIterations() >= cap; }));
    }
    ss.setPlanner(grrtstar);
    if (req.cancel != nullptr) {
      const std::atomic_bool * cancel = req.cancel;
      ptc = ob::plannerOrTerminationCondition(
        ptc, ob::PlannerTerminationCondition([cancel] { return cancel->load(); }));
    }

    // Record the time and the iteration count of the first exact solution. The informed sampler
    // chains its own callback onto this one when the planner allocates it.
    std::optional<Clock::time_point> first_at;
    std::int64_t first_iterations = -1;
    ss.getProblemDefinition()->setIntermediateSolutionCallback(
      [&first_at, &first_iterations](
        const ob::Planner * planner, const std::vector<const ob::State *> & /*path*/,
        const ob::Cost /*cost*/) {
        if (first_at) return;
        first_at = Clock::now();
        const auto & props = planner->getPlannerProgressProperties();
        if (const auto it = props.find("iterations INTEGER"); it != props.end()) {
          first_iterations = std::stoll(it->second());
        }
      });

    ss.setup();
    rng_lock.unlock();

    const auto t0 = Clock::now();
    const Deadline deadline{
      t0 +
        std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(req.time_budget)),
      req.cancel};
    ss.solve(ptc);
    out.search_time_s = std::chrono::duration<double>(Clock::now() - t0).count();
    const auto & progress = ss.getPlanner()->getPlannerProgressProperties();
    if (const auto it = progress.find("iterations INTEGER"); it != progress.end()) {
      out.iterations = std::stoll(it->second());
    }
    const bool canceled = req.cancel != nullptr && req.cancel->load();

    if (!ss.haveExactSolutionPath()) {
      out.status = canceled ? PlanResult::Status::kCanceled : PlanResult::Status::kNoExactSolution;
      out.message = canceled ? "canceled before an exact solution was found"
                             : "no exact solution within the allowed planning time";
      out.reported_time_s = std::chrono::duration<double>(Clock::now() - t0).count();
      return out;
    }

    if (first_at) {
      out.first_solution_time_s = std::chrono::duration<double>(*first_at - t0).count();
      out.first_solution_iterations = first_iterations;
    }

    const og::PathGeometric raw = ss.getSolutionPath();
    out.raw_waypoints = raw.getStateCount();
    og::PathGeometric path(raw);
    if (settings.smooth && !canceled) {
      if (deadline.passed()) {
        out.smoothing_stopped = true;
      } else {
        const auto ts = Clock::now();
        out.smoothing_stage = smooth(manifold, problem, path, check, settings, deadline);
        out.smoothed = out.smoothing_stage >= 0 && out.smoothing_stage < 3;
        out.smooth_time_s = std::chrono::duration<double>(Clock::now() - ts).count();
        out.smoothing_stopped = !out.smoothed && deadline.passed();
      }
    }
    // The returned path keeps the planner's or the smoother's waypoints. Its densified copy holds
    // a state every collision_check_resolution on the same chords.
    og::PathGeometric dense(path);
    dense.interpolate();
    if (out.smoothed && !allValid(*space, dense, validity)) {
      // A smoothed path that fails here falls back to the planner's path, which passes the same
      // check below or fails the request.
      path = raw;
      dense = raw;
      dense.interpolate();
      out.smoothed = false;
    }
    out.reported_time_s = std::chrono::duration<double>(Clock::now() - t0).count();

    std::vector<double> reals;
    // Check every densified state again before reporting success.
    for (std::size_t i = 0; i < dense.getStateCount(); ++i) {
      space->copyToReals(reals, dense.getState(static_cast<unsigned int>(i)));
      if (!validity.isValid(reals.data())) {
        out.status = PlanResult::Status::kInvalidPath;
        out.message = "state " + std::to_string(i) + " of the densified solution failed validation";
        return out;
      }
    }
    out.path.reserve(path.getStateCount());
    for (const auto * st : path.getStates()) {
      space->copyToReals(reals, st);
      out.path.emplace_back(reals.begin(), reals.begin() + n);
    }
    out.status = PlanResult::Status::kSolved;
    return out;
  }

private:
  Problem makeProblem(
    const ManifoldT & manifold, const std::vector<double> & lower,
    const std::vector<double> & upper, const ValidityOracle & validity,
    const PlannerSettings & settings) const
  {
    const int n = dimension_;
    Problem p;
    p.lower = lower;
    p.upper = upper;
    ob::RealVectorBounds bounds(static_cast<unsigned int>(n));
    for (int i = 0; i < n; ++i) {
      const auto k = static_cast<std::size_t>(i);
      bounds.setLow(static_cast<unsigned int>(i), lower[k]);
      bounds.setHigh(static_cast<unsigned int>(i), upper[k]);
    }
    p.space = std::make_shared<gio::GeodexStateSpace<ManifoldT>>(manifold, bounds);
    // Edges are straight chords in the configuration space, the curve that MoveIt and the
    // controllers execute between waypoints. The metric enters through the edge cost.
    p.space->setInterpolationMode(gio::InterpolationMode::BaseGeodesic);
    p.space->setCollisionResolution(settings.collision_check_resolution);
    p.resolution = settings.collision_check_resolution;
    p.si = std::make_shared<ob::SpaceInformation>(p.space);
    p.si->setStateValidityChecker(
      gio::make_validity_checker<ManifoldT>(p.si, [&validity](const auto & q) {
        const Eigen::Ref<const Eigen::VectorXd> r(q);
        return validity.isValid(r.data());
      }));
    if (settings.edge_motion_validator) {
      if (const auto * vamp = dynamic_cast<const VampOracle *>(&validity.collisionOracle())) {
        // A robot with a VAMP kernel searches with the kernel's own edge check, as
        // geodex::robots::plan does. The smoother checks the path the search returns.
        const std::shared_ptr<ob::MotionValidator> edge_validator(
          giv::make_vamp_motion_validator(vamp->robot(), p.si, vamp->env()));
        p.si->setMotionValidator(edge_validator);
      } else {
        p.si->setMotionValidator(std::make_shared<EdgeMotionValidator>(p.si, validity, n));
      }
    }
    p.si->setup();
    return p;
  }

  /// The planner's motion validator as the smoother's edge test, failing once @p deadline has
  /// passed.
  static geodex::algorithm::EdgePredicate edgeValidator(
    const ob::SpaceInformationPtr & si, const Deadline * deadline)
  {
    struct Endpoints
    {
      explicit Endpoints(ob::SpaceInformationPtr s)
      : si(std::move(s)), a(si->allocState()), b(si->allocState())
      {
      }
      ~Endpoints()
      {
        si->freeState(a);
        si->freeState(b);
      }
      Endpoints(const Endpoints &) = delete;
      Endpoints & operator=(const Endpoints &) = delete;
      ob::SpaceInformationPtr si;
      ob::State * a;
      ob::State * b;
    };
    auto e = std::make_shared<Endpoints>(si);
    return
      [e, deadline](
        const Eigen::Ref<const Eigen::VectorXd> & x, const Eigen::Ref<const Eigen::VectorXd> & y) {
        if (deadline != nullptr && deadline->passed()) return false;
        auto * sa = e->a->template as<gio::GeodexState<ManifoldT>>();
        auto * sb = e->b->template as<gio::GeodexState<ManifoldT>>();
        for (Eigen::Index i = 0; i < x.size(); ++i) {
          sa->values[i] = x[i];
          sb->values[i] = y[i];
        }
        return e->si->checkMotion(e->a, e->b);
      };
  }

  std::shared_ptr<og::GreedyRRTstar> makeGreedyRRTstar(
    const ob::SpaceInformationPtr & si, const PlannerSettings & c) const
  {
    auto p = std::make_shared<og::GreedyRRTstar>(si);
    p->setRange(c.grrtstar_range);
    // The sampler applies the greedy ratio.
    p->setGreedyBiasingRatio(0.0);
    p->setRewireFactor(c.rewire_factor);
    p->setMaxNeighbors(static_cast<unsigned int>(std::max<std::int64_t>(0, c.max_neighbors)));
    // Pruning to the greedy bound belongs to greedy sampling. A greedy ratio of 0 prunes to the
    // informed set of the solution cost.
    p->setGreedyCostForTreePruning(c.greedy_cost_for_tree_pruning && c.greedy_biasing_ratio > 0.0);
    // Nearest-neighbor queries use the chord length of the joint coordinates.
    const int n = dimension_;
    p->setNNDistanceFunction([n](const ob::State * a, const ob::State * b) {
      const auto * sa = a->as<gio::GeodexState<ManifoldT>>();
      const auto * sb = b->as<gio::GeodexState<ManifoldT>>();
      double d2 = 0.0;
      for (int i = 0; i < n; ++i) {
        const double d = sa->values[i] - sb->values[i];
        d2 += d * d;
      }
      return std::sqrt(d2);
    });
    return p;
  }

  /// Calls geodex's smoother for plan() and smooth(). The smoother checks waypoints against
  /// @p validity and the bounds. With a VAMP kernel, it checks each edge at configurations
  /// spaced by the sphere travel along the edge, as geodex::robots::plan does. Otherwise the
  /// planner's motion validator checks the edges.
  auto smoothPoints(
    const ManifoldT & manifold, const Problem & problem, const std::vector<Point> & input,
    const ValidityOracle & validity, const PlannerSettings & settings,
    const Deadline * deadline) const
  {
    SmootherValidity fn;
    fn.oracle = &validity;
    fn.lower = &problem.lower;
    fn.upper = &problem.upper;
    fn.dim = dimension_;
    fn.width = validity.batchWidth();
    fn.deadline = deadline;
    geodex::algorithm::PathSmoothingSettings s;
    s.output_spacing = settings.output_spacing;
    s.round_corners = settings.round_corners;
    s.seed = settings.smoothing_seed;
    if (const auto * vamp = dynamic_cast<const VampOracle *>(&validity.collisionOracle())) {
      // The spacing is the sphere travel of the resolution at the robot's largest sphere speed,
      // the spacing that the smoothing margin covers.
      s.collision_check_resolution =
        giv::sphere_speed(vamp->robot(), vamp->env()) * problem.resolution;
      s.edge_travel = [travel = giv::make_sphere_travel(vamp->robot(), vamp->env())](
                        const Eigen::Ref<const Eigen::VectorXd> & a,
                        const Eigen::Ref<const Eigen::VectorXd> & b) {
        return travel(Eigen::VectorXd(b - a));
      };
    } else {
      s.collision_check_resolution = problem.resolution;
      s.edge_validator = edgeValidator(problem.si, deadline);
    }
    return geodex::algorithm::smooth_path(manifold, fn, input, s);
  }

  /// Replaces @p path with the smoother's path when that path passes the check and differs from
  /// the input. Returns the smoother's stage.
  int smooth(
    const ManifoldT & manifold, const Problem & problem, og::PathGeometric & path,
    const ValidityOracle & validity, const PlannerSettings & settings,
    const Deadline & deadline) const
  {
    std::vector<Point> input;
    input.reserve(path.getStateCount());
    std::vector<double> reals;
    for (const auto * st : path.getStates()) {
      problem.space->copyToReals(reals, st);
      input.push_back(
        Eigen::Map<const Point>(reals.data(), static_cast<Eigen::Index>(reals.size())));
    }
    const auto result = smoothPoints(manifold, problem, input, validity, settings, &deadline);
    const int stage = result.profile.fallback;
    if (!result.collision_free || result.path.empty() || stage >= 3) return stage;
    path.clear();
    ob::State * scratch = problem.si->allocState();
    for (const auto & p : result.path) {
      problem.space->copyFromReals(scratch, std::vector<double>(p.data(), p.data() + p.size()));
      path.append(scratch);
    }
    problem.si->freeState(scratch);
    return stage;
  }

  /// True when every state of @p path passes @p validity.
  static bool allValid(
    const gio::GeodexStateSpace<ManifoldT> & space, const og::PathGeometric & path,
    const ValidityOracle & validity)
  {
    std::vector<double> reals;
    for (std::size_t i = 0; i < path.getStateCount(); ++i) {
      space.copyToReals(reals, path.getState(static_cast<unsigned int>(i)));
      if (!validity.isValid(reals.data())) return false;
    }
    return true;
  }

  ManifoldT manifold_;
  HeuristicT heuristic_;
  int dimension_;
  std::string description_;
};

template <typename ManifoldT, typename HeuristicT>
std::shared_ptr<const PlannerBackend> makeBackend(
  ManifoldT m, HeuristicT h, int dim, std::string description)
{
  return std::make_shared<Backend<ManifoldT, HeuristicT>>(
    std::move(m), std::move(h), dim, std::move(description));
}

class VampEnvironment : public CollisionEnvironment
{
public:
  VampEnvironment(std::string robot, CollisionScene scene)
  : robot_(std::move(robot)), dim_(giv::robot_dimension(robot_)), scene_(std::move(scene))
  {
    // Without an attached object, checks only read the environment, and every oracle shares it.
    if (scene_.attached_spheres.empty()) shared_ = build();
  }

  std::shared_ptr<const ValidityOracle> makeOracle() const override
  {
    // VAMP poses an attached object inside the environment on every check. With an attachment, each
    // oracle gets a freshly built environment.
    const giv::EnvHandle env = scene_.attached_spheres.empty() ? shared_ : build();
    return std::make_shared<VampOracle>(robot_, env, dim_);
  }

private:
  giv::EnvHandle build() const
  {
    const double p = scene_.obstacle_padding;
    auto builder = giv::make_scene_builder();
    for (const auto & b : scene_.boxes) {
      const Eigen::Vector3d size = 2.0 * (vec(b.half_extents) + Eigen::Vector3d::Constant(p));
      giv::scene_add_box(builder, vec(b.center), size, mat(b.rotation));
    }
    for (const auto & s : scene_.spheres) {
      giv::scene_add_sphere(builder, vec(s.center), s.radius + p);
    }
    for (const auto & c : scene_.cylinders) {
      giv::scene_add_cylinder(
        builder, vec(c.center), c.radius + p, c.height + 2.0 * p, mat(c.rotation));
    }
    giv::EnvHandle env = giv::build_scene(builder);
    if (!scene_.attached_spheres.empty()) giv::attach_spheres(env, scene_.attached_spheres);
    return env;
  }

  std::string robot_;
  int dim_;
  CollisionScene scene_;
  giv::EnvHandle shared_;
};

}  // namespace

const std::vector<std::string> & plannerIds()
{
  static const std::vector<std::string> ids{kGreedyRRTstar};
  return ids;
}

bool hasMassMatrix(const std::string & robot) { return gr::robot_from_name(robot).has_value(); }

bool hasCollisionModel(const std::string & robot) { return !robot.empty() && vampHas(robot); }

double smoothingMargin(
  const std::string & robot, const std::vector<std::array<double, 4>> & attached_spheres,
  const double resolution)
{
  if (!hasCollisionModel(robot)) return 0.0;
  giv::EnvHandle env = giv::build_scene(giv::make_scene_builder());
  if (!attached_spheres.empty()) giv::attach_spheres(env, attached_spheres);
  const double tolerance = geodex::algorithm::PathSmoothingSettings{}.corner_tolerance;
  return giv::sphere_speed(robot, env) * (0.5 * resolution + tolerance);
}

bool isRegisteredRobot(const std::string & robot)
{
  return hasMassMatrix(robot) || hasCollisionModel(robot);
}

bool hasMobileBase(const std::string & robot)
{
  const auto r = gr::robot_from_name(robot);
  return r && gr::visit(*r, []<gr::Robot R>() { return gr::has_planar_base<R>; });
}

int robotDimension(const std::string & robot)
{
  if (hasCollisionModel(robot)) return giv::robot_dimension(robot);
  if (const auto r = gr::robot_from_name(robot)) {
    // The mass matrix covers the arm joints, and a mobile base adds its planar pose.
    return gr::visit(*r, []<gr::Robot R>() {
      return gr::detail::RobotTraits<R>::Nq + (gr::has_planar_base<R> ? 3 : 0);
    });
  }
  return -1;
}

std::vector<std::string> modelJointNames(const std::string & robot)
{
  if (!hasCollisionModel(robot)) return {};
  return giv::robot_joint_names(robot);
}

std::string modelEndEffectorFrame(const std::string & robot)
{
  if (!hasCollisionModel(robot)) return {};
  return giv::robot_end_effector(robot);
}

bool modelJointBounds(
  const std::string & robot, std::vector<double> & lower, std::vector<double> & upper)
{
  const auto r = gr::robot_from_name(robot);
  if (!r) return false;
  return gr::visit(*r, [&]<gr::Robot R>() {
    using T = gr::detail::RobotTraits<R>;
    if constexpr (gr::has_planar_base<R>) {
      return false;
    } else {
      lower.assign(T::lower_limit, T::lower_limit + T::Nq);
      upper.assign(T::upper_limit, T::upper_limit + T::Nq);
      return true;
    }
  });
}

std::vector<std::string> registeredRobots()
{
  std::set<std::string> names;
  for (const auto r : gr::registered_robots()) names.emplace(gr::name(r));
  for (const auto & n : giv::registered_robots()) names.insert(n);
  return {names.begin(), names.end()};
}

std::shared_ptr<void> addLogSink(LogSink sink) { return outputHandler().add(std::move(sink)); }

std::shared_ptr<const PlannerBackend> PlannerBackend::create(
  const std::string & robot, const std::string & metric, int dimension)
{
  if (metric != "euclidean" && metric != "kinetic_energy") {
    throw std::invalid_argument("unknown metric '" + metric + "'");
  }
  if (dimension <= 0) throw std::invalid_argument("the planning group has no active variables");
  if (!robot.empty() && !isRegisteredRobot(robot)) {
    throw std::invalid_argument("geodex has no robot model named '" + robot + "'");
  }
  if (hasMobileBase(robot)) {
    throw std::invalid_argument(
      "robot '" + robot + "' has a mobile base, and this plugin plans fixed-base arms only");
  }
  if (!robot.empty() && robotDimension(robot) != dimension) {
    throw std::invalid_argument(
      "the planning group has " + std::to_string(dimension) + " active variables, robot model '" +
      robot + "' has " + std::to_string(robotDimension(robot)));
  }
  const auto r = gr::robot_from_name(robot);
  if (!r) {
    if (metric != "euclidean") {
      throw std::invalid_argument(
        "metric '" + metric + "' needs a geodex mass matrix; " +
        (robot.empty() ? std::string("set 'robot'") : "robot '" + robot + "' has none"));
    }
    using CS = geodex::ConfigurationSpace<geodex::Euclidean<>, geodex::EuclideanStandardMetric<>>;
    return makeBackend(
      CS{geodex::Euclidean<>{dimension}, {}}, gh::Euclidean{}, dimension,
      "euclidean metric, euclidean heuristic, dimension " + std::to_string(dimension));
  }
  return gr::visit(*r, [&]<gr::Robot R>() -> std::shared_ptr<const PlannerBackend> {
    constexpr int Nq = gr::detail::RobotTraits<R>::Nq;
    if constexpr (gr::has_planar_base<R>) {
      throw std::logic_error("mobile-base robots are refused above");
    } else {
      if (metric == "euclidean") {
        using CS =
          geodex::ConfigurationSpace<geodex::Euclidean<Nq>, geodex::EuclideanStandardMetric<Nq>>;
        return makeBackend(
          CS{geodex::Euclidean<Nq>{}, {}}, gh::Euclidean{}, Nq,
          "robot '" + robot + "', euclidean metric, euclidean heuristic");
      }
      using MM = gr::MassMatrix<R>;
      using CS = geodex::ConfigurationSpace<geodex::Euclidean<Nq>, geodex::KineticEnergyMetric<MM>>;
      std::ostringstream d;
      d << "robot '" << robot << "', kinetic_energy metric, mass-matrix lower-bound heuristic"
        << " (certificate " << gr::MassLowerBound<R>::certificate
        << (gr::MassLowerBound<R>::converged ? ")" : ", not converged)");
      return makeBackend(
        CS{geodex::Euclidean<Nq>{}, geodex::KineticEnergyMetric<MM>{MM{}}},
        gh::MatrixLowerBound<Nq>{gr::MassLowerBound<R>::matrix()}, Nq, d.str());
    }
  });
}

std::shared_ptr<const CollisionEnvironment> CollisionEnvironment::create(
  const std::string & robot, const CollisionScene & scene)
{
  if (!hasCollisionModel(robot)) {
    throw std::invalid_argument("geodex has no collision model named '" + robot + "'");
  }
  if (!(scene.obstacle_padding >= 0.0) || !std::isfinite(scene.obstacle_padding)) {
    throw std::invalid_argument("obstacle padding must be a finite, non-negative distance");
  }
  return std::make_shared<VampEnvironment>(robot, scene);
}

}  // namespace geodex_moveit
