// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0.

#ifndef NAV2_SMAC_PLANNER__COST_AWARE_SHORTCUT_HPP_
#define NAV2_SMAC_PLANNER__COST_AWARE_SHORTCUT_HPP_

#include <chrono>
#include <functional>

#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_smac_planner
{

struct ShortcutOptions
{
  bool enabled{false};
  double cost_weight{10.0};
  double min_cost_improvement{0.001};
  double resample_spacing{0.05};
  double max_processing_time{0.2};
  // Per anchor, not a shared limit for all points of a long path.
  int max_candidate_checks{200};
};

struct ShortcutResult
{
  nav_msgs::msg::Path path;
  double baseline_cost{0.0};
  double final_cost{0.0};
  int candidate_checks{0};
  int accepted_shortcuts{0};
  int processed_anchors{0};
  int collision_rejections{0};
  int cost_rejections{0};
  bool timed_out{false};
  bool completed{false};
};

/**
 * Optional LOS postprocessing of the existing Smac 2D output.
 * Caller holds the costmap mutex. New segments use the Node2D cell policy:
 * costs >= 253 are blocked, except unknown when the search allows unknown.
 * No footprint, radius, inflation or additional baseline collision model is used.
 * On budget exhaustion retain committed shortcuts and the original unprocessed
 * suffix. Cancellation propagates. Disabled processing is an exact passthrough.
 */
class CostAwareShortcut
{
public:
  explicit CostAwareShortcut(const ShortcutOptions & options);

  ShortcutResult process(
    const nav_msgs::msg::Path & baseline,
    const nav2_costmap_2d::Costmap2D & costmap,
    bool allow_unknown,
    const std::function<bool()> & cancel_checker,
    std::chrono::steady_clock::time_point planning_deadline) const;

private:
  ShortcutResult processPass(
    const nav_msgs::msg::Path & baseline,
    const nav2_costmap_2d::Costmap2D & costmap,
    bool allow_unknown,
    const std::function<bool()> & cancel_checker,
    std::chrono::steady_clock::time_point deadline) const;

  ShortcutOptions options_;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__COST_AWARE_SHORTCUT_HPP_
