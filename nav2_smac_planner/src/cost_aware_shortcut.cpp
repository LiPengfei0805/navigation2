// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0.

#include "nav2_smac_planner/cost_aware_shortcut.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace nav2_smac_planner
{
namespace
{
using Clock = std::chrono::steady_clock;
using Point = geometry_msgs::msg::Point;
struct BudgetExpired {};
constexpr size_t MAX_OUTPUT_POINTS = 1000000;

class Evaluator
{
public:
  Evaluator(
    const nav2_costmap_2d::Costmap2D & map, double weight, bool allow_unknown,
    const std::function<bool()> & cancel, Clock::time_point deadline)
  : map_(map), weight_(weight), allow_unknown_(allow_unknown),
    cancel_(cancel), deadline_(deadline) {}

  void check() const
  {
    checkCancellation();
    if (Clock::now() >= deadline_) {
      throw BudgetExpired{};
    }
  }

  void checkCancellation() const
  {
    if (cancel_ && cancel_()) {
      throw nav2_core::PlannerCancelled("Smac LOS shortcut processing canceled");
    }
  }

  // Match GridCollisionChecker::inCollision(index, traverse_unknown).
  bool traversable(unsigned char cost) const
  {
    return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
           (cost == nav2_costmap_2d::NO_INFORMATION && allow_unknown_);
  }

  // Amanatides/Woo-style grid traversal with exact in-cell line lengths.
  // Walk only the cells actually crossed; do not scan a bounding box or allocate
  // all crossings before discovering a wall. Boundary-only contact follows the
  // same floor/worldToMap convention as costmap center-cell checks.
  double segment(const Point & a, const Point & b, bool check_los) const
  {
    check();
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length = std::hypot(dx, dy);
    unsigned int ax, ay, bx, by;
    if (!std::isfinite(length) || !std::isfinite(a.z) || !std::isfinite(b.z) ||
      !map_.worldToMap(a.x, a.y, ax, ay) || !map_.worldToMap(b.x, b.y, bx, by))
    {
      return std::numeric_limits<double>::infinity();
    }
    if (check_los &&
      (!traversable(map_.getCost(ax, ay)) || !traversable(map_.getCost(bx, by))))
    {
      return std::numeric_limits<double>::infinity();
    }
    if (length == 0.0) {
      return 0.0;
    }
    const double res = map_.getResolution();
    const double infinity = std::numeric_limits<double>::infinity();
    const int sx = (dx > 0.0) - (dx < 0.0), sy = (dy > 0.0) - (dy < 0.0);
    int x = static_cast<int>(ax), y = static_cast<int>(ay);
    const double dt_x = sx ? res / std::abs(dx) : infinity;
    const double dt_y = sy ? res / std::abs(dy) : infinity;
    double next_x = sx ? (map_.getOriginX() + (x + (sx > 0)) * res - a.x) / dx : infinity;
    double next_y = sy ? (map_.getOriginY() + (y + (sy > 0)) * res - a.y) / dy : infinity;
    double t = 0.0, total = 0.0;
    while (t < 1.0) {
      check();
      if (x < 0 || y < 0 || x >= static_cast<int>(map_.getSizeInCellsX()) ||
        y >= static_cast<int>(map_.getSizeInCellsY()))
      {
        return infinity;
      }
      const double end = std::max(t, std::min({1.0, next_x, next_y}));
      if (end > t) {
        const auto cost = map_.getCost(x, y);
        if (check_los && !traversable(cost)) {
          return infinity;
        }
        const double normalized = std::min(static_cast<double>(cost), 252.0) / 252.0;
        total += length * (end - t) * (1.0 + weight_ * normalized * normalized);
      }
      if (end >= 1.0) {
        break;
      }
      const bool cross_x = next_x <= next_y, cross_y = next_y <= next_x;
      if (cross_x) {
        x += sx;
        next_x += dt_x;
      }
      if (cross_y) {
        y += sy;
        next_y += dt_y;
      }
      t = end;
    }
    return total;
  }

private:
  const nav2_costmap_2d::Costmap2D & map_;
  double weight_;
  bool allow_unknown_;
  const std::function<bool()> & cancel_;
  Clock::time_point deadline_;
};

// Interpolate a candidate into a temporary buffer before committing it. If the
// budget expires here, earlier accepted edges remain valid and are retained.
bool interpolate(
  const geometry_msgs::msg::PoseStamped & a, const geometry_msgs::msg::PoseStamped & b,
  double spacing, size_t existing_size, const Evaluator & evaluator,
  std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  const auto & start = a.pose.position;
  const auto & end = b.pose.position;
  const double length = std::hypot(end.x - start.x, end.y - start.y);
  const double count = std::max(1.0, std::ceil(length / spacing));
  if (!std::isfinite(count) || count > MAX_OUTPUT_POINTS - existing_size) {
    return false;
  }
  const auto steps = static_cast<size_t>(count);
  poses.reserve(steps);
  for (size_t j = 1; j <= steps; ++j) {
    evaluator.check();
    auto pose = b;
    if (j < steps) {
      const double t = static_cast<double>(j) / steps;
      pose.pose.position.x = start.x + t * (end.x - start.x);
      pose.pose.position.y = start.y + t * (end.y - start.y);
      pose.pose.position.z = start.z + t * (end.z - start.z);
    }
    poses.push_back(pose);
  }
  return true;
}
}  // namespace

CostAwareShortcut::CostAwareShortcut(const ShortcutOptions & options)
: options_(options)
{
  if (!std::isfinite(options.cost_weight) || options.cost_weight < 0.0 ||
    !std::isfinite(options.min_cost_improvement) || options.min_cost_improvement < 0.0 ||
    !std::isfinite(options.resample_spacing) || options.resample_spacing <= 0.0 ||
    !std::isfinite(options.max_processing_time) || options.max_processing_time <= 0.0 ||
    options.max_processing_time > 60.0 || options.max_candidate_checks <= 0)
  {
    throw std::invalid_argument("Invalid Smac shortcut parameters");
  }
}

ShortcutResult CostAwareShortcut::process(
  const nav_msgs::msg::Path & baseline, const nav2_costmap_2d::Costmap2D & costmap,
  bool allow_unknown, const std::function<bool()> & cancel_checker,
  Clock::time_point planning_deadline) const
{
  const auto deadline = std::min(
    planning_deadline, Clock::now() + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(options_.max_processing_time)));
  auto result = processPass(baseline, costmap, allow_unknown, cancel_checker, deadline);
  if (!options_.enabled || result.timed_out || result.path.poses.size() < 3 ||
    !result.completed)
  {
    return result;
  }

  // A forward-only farthest-visible choice can leave a redundant bend well
  // past a corner. Pull back from the goal on the resulting geometry, with
  // the SAME deadline, to remove that directional bias.
  auto reverse = result.path;
  std::reverse(reverse.poses.begin(), reverse.poses.end());
  auto refined = processPass(reverse, costmap, allow_unknown, cancel_checker, deadline);
  result.candidate_checks += refined.candidate_checks;
  result.processed_anchors += refined.processed_anchors;
  result.collision_rejections += refined.collision_rejections;
  result.cost_rejections += refined.cost_rejections;
  result.timed_out = refined.timed_out;
  result.completed = result.completed && refined.completed;
  if (refined.accepted_shortcuts > 0 &&
    refined.final_cost + options_.min_cost_improvement < result.final_cost)
  {
    result.accepted_shortcuts += refined.accepted_shortcuts;
    result.final_cost = refined.final_cost;
    std::reverse(refined.path.poses.begin(), refined.path.poses.end());
    for (size_t k = 0; k + 1 < refined.path.poses.size(); ++k) {
      if (cancel_checker && cancel_checker()) {
        throw nav2_core::PlannerCancelled("Smac LOS shortcut processing canceled");
      }
      const auto & a = refined.path.poses[k].pose.position;
      const auto & b = refined.path.poses[k + 1].pose.position;
      if (a.x != b.x || a.y != b.y) {
        const double yaw = std::atan2(b.y - a.y, b.x - a.x);
        auto & q = refined.path.poses[k].pose.orientation;
        q.x = q.y = 0.0;
        q.z = std::sin(yaw / 2.0);
        q.w = std::cos(yaw / 2.0);
      }
    }
    refined.path.poses.back().pose.orientation = baseline.poses.back().pose.orientation;
    result.path = std::move(refined.path);
  }
  return result;
}

ShortcutResult CostAwareShortcut::processPass(
  const nav_msgs::msg::Path & baseline, const nav2_costmap_2d::Costmap2D & costmap,
  bool allow_unknown, const std::function<bool()> & cancel_checker,
  Clock::time_point planning_deadline) const
{
  ShortcutResult result;
  result.path = baseline;
  if (!options_.enabled) {
    return result;
  }
  if (baseline.poses.size() < 3) {
    if (cancel_checker && cancel_checker()) {
      throw nav2_core::PlannerCancelled("Smac LOS shortcut processing canceled");
    }
    result.completed = true;
    return result;
  }
  if (costmap.getSizeInCellsX() == 0 || costmap.getSizeInCellsY() == 0 ||
    !std::isfinite(costmap.getResolution()) || costmap.getResolution() <= 0.0)
  {
    return result;
  }
  const auto deadline = std::min(
    planning_deadline, Clock::now() + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(options_.max_processing_time)));
  const Evaluator evaluator(costmap, options_.cost_weight, allow_unknown, cancel_checker, deadline);
  std::vector<double> prefix(baseline.poses.size(), 0.0);
  try {
    // Evaluate the existing path's score only. Preserve the planner's behavior:
    // no new footprint/whole-path collision rejection is imposed on the baseline.
    for (size_t k = 1; k < baseline.poses.size(); ++k) {
      const double score = evaluator.segment(
        baseline.poses[k - 1].pose.position, baseline.poses[k].pose.position, false);
      if (!std::isfinite(score)) {
        return result;
      }
      prefix[k] = prefix[k - 1] + score;
    }
  } catch (const BudgetExpired &) {
    result.timed_out = true;
    return result;
  }
  result.baseline_cost = result.final_cost = prefix.back();
  nav_msgs::msg::Path output;
  output.header = baseline.header;
  output.poses.push_back(baseline.poses.front());
  const size_t last = baseline.poses.size() - 1;
  size_t i = 0;
  try {
    while (i < last) {
      evaluator.check();
      ++result.processed_anchors;
      int checks_here = 0;
      size_t best = i + 1;
      double best_score = prefix[best] - prefix[i];
      auto probe = [&](size_t j) {
          ++checks_here;
          ++result.candidate_checks;
          const double score = evaluator.segment(
            baseline.poses[i].pose.position, baseline.poses[j].pose.position, true);
          if (!std::isfinite(score)) {
            ++result.collision_rejections;
            return false;
          }
          const double old_score = prefix[j] - prefix[i];
          if (score + options_.min_cost_improvement < old_score) {
            if (j > best) {
              best = j;
              best_score = score;
            }
          } else if (score > old_score + 1e-9) {
            ++result.cost_rejections;
          }
          return true;
        };

      size_t reachable = i + 1;
      if (last > i + 1) {
        const bool goal_visible = probe(last);
        if (best != last) {
          // Expand from the local segment (2,4,8,... points) instead of spending
          // a shared budget rejecting thousands of far-away endpoints first.
          size_t stride = 2, blocked = last;
          while (reachable < last && checks_here < options_.max_candidate_checks) {
            const size_t j = i + std::min(stride, last - i);
            const bool visible = j == last ? goal_visible : probe(j);
            if (!visible) {
              blocked = j;
              break;
            }
            reachable = j;
            if (j == last) {
              break;
            }
            stride = std::min(last - i, stride * 2);
          }
          // Refine the last visible frontier. Visibility need not be globally
          // monotone; this is a bounded local greedy search, not an optimality claim.
          while (blocked > reachable + 1 &&
            checks_here < options_.max_candidate_checks)
          {
            const size_t j = reachable + (blocked - reachable) / 2;
            if (probe(j)) {
              reachable = j;
            } else {
              blocked = j;
            }
          }
        }
      }

      if (best > i + 1) {
        std::vector<geometry_msgs::msg::PoseStamped> replacement;
        if (output.poses.size() < MAX_OUTPUT_POINTS && interpolate(
            baseline.poses[i], baseline.poses[best], options_.resample_spacing,
            output.poses.size(), evaluator, replacement))
        {
          output.poses.insert(output.poses.end(), replacement.begin(), replacement.end());
          result.final_cost -= (prefix[best] - prefix[i]) - best_score;
          ++result.accepted_shortcuts;
          i = best;
          continue;
        }
      }
      // No improving shortcut: retain these existing poses exactly. Advance over
      // the examined visible span so dense collinear paths do not waste the budget.
      const size_t next = std::max(i + 1, reachable);
      output.poses.insert(output.poses.end(), baseline.poses.begin() + i + 1,
        baseline.poses.begin() + next + 1);
      i = next;
    }
    result.completed = true;
  } catch (const BudgetExpired &) {
    result.timed_out = true;
  }
  if (result.accepted_shortcuts == 0) {
    return result;
  }
  // Commit previous improvements even if later work ran out of time. All newly
  // inserted edges were checked before insertion; this suffix is the original path.
  output.poses.insert(output.poses.end(), baseline.poses.begin() + i + 1, baseline.poses.end());
  for (size_t k = 0; k < output.poses.size(); ++k) {
    evaluator.checkCancellation();
    output.poses[k].header = output.header;
    if (k + 1 < output.poses.size()) {
      const auto & a = output.poses[k].pose.position;
      const auto & b = output.poses[k + 1].pose.position;
      if (a.x != b.x || a.y != b.y) {
        const double yaw = std::atan2(b.y - a.y, b.x - a.x);
        auto & q = output.poses[k].pose.orientation;
        q.x = q.y = 0.0;
        q.z = std::sin(yaw / 2.0);
        q.w = std::cos(yaw / 2.0);
      }
    }
  }
  result.path = std::move(output);
  return result;
}
}  // namespace nav2_smac_planner
