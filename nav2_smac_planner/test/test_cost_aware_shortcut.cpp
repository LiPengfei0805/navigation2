// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <cmath>
#include <limits>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_smac_planner/cost_aware_shortcut.hpp"

namespace
{
using nav2_smac_planner::CostAwareShortcut;
using nav2_smac_planner::ShortcutOptions;
using Clock = std::chrono::steady_clock;

nav_msgs::msg::Path path(std::initializer_list<std::pair<double, double>> points)
{
  nav_msgs::msg::Path p;
  p.header.frame_id = "map";
  for (const auto & xy : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = p.header;
    pose.pose.position.x = xy.first;
    pose.pose.position.y = xy.second;
    pose.pose.orientation.w = 1.0;
    p.poses.push_back(pose);
  }
  return p;
}

nav_msgs::msg::Path densify(const nav_msgs::msg::Path & p, double spacing)
{
  auto result = p;
  result.poses.clear();
  result.poses.push_back(p.poses.front());
  for (size_t i = 1; i < p.poses.size(); ++i) {
    const auto & a = p.poses[i - 1].pose.position;
    const auto & b = p.poses[i].pose.position;
    const int steps = std::max(1,
        static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / spacing)));
    for (int j = 1; j <= steps; ++j) {
      auto pose = p.poses[i];
      const double t = static_cast<double>(j) / steps;
      pose.pose.position.x = a.x + t * (b.x - a.x);
      pose.pose.position.y = a.y + t * (b.y - a.y);
      result.poses.push_back(pose);
    }
  }
  return result;
}

double length(const nav_msgs::msg::Path & p)
{
  double result = 0.0;
  for (size_t i = 1; i < p.poses.size(); ++i) {
    const auto & a = p.poses[i - 1].pose.position;
    const auto & b = p.poses[i].pose.position;
    result += std::hypot(b.x - a.x, b.y - a.y);
  }
  return result;
}

class ShortcutTest : public ::testing::Test
{
protected:
  nav2_costmap_2d::Costmap2D map{120, 100, 0.1, 0.0, 0.0, 0};
  ShortcutOptions options;

  void SetUp() override
  {
    options.enabled = true;
    options.max_processing_time = 3.0;
  }

  nav2_smac_planner::ShortcutResult run(const nav_msgs::msg::Path & p, bool unknown = false)
  {
    return CostAwareShortcut(options).process(
      p, map, unknown, []() {return false;}, Clock::now() + std::chrono::seconds(5));
  }

  void block(unsigned char cost)
  {
    for (unsigned int x = 40; x < 45; ++x) {
      for (unsigned int y = 0; y < 50; ++y) {
        map.setCost(x, y, cost);
      }
    }
  }

  void expectTraversable(const nav_msgs::msg::Path & p)
  {
    for (const auto & pose : densify(p, 0.005).poses) {
      unsigned int mx, my;
      ASSERT_TRUE(map.worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my));
      EXPECT_LT(map.getCost(mx, my), 253);
    }
  }

  nav_msgs::msg::Path longPath()
  {
    map.resizeMap(400, 400, 0.1, 0.0, 0.0);
    for (unsigned int y = 0; y < 400; ++y) {
      if (y < 250) {
        map.setCost(100, y, 254);
        map.setCost(300, y, 254);
      }
      if (y >= 150) {
        map.setCost(200, y, 254);
      }
    }
    return densify(path({{2, 2}, {8, 27}, {12, 27}, {18, 12}, {22, 12},
               {28, 27}, {32, 27}, {32, 30}, {35, 30}, {38, 35}}), 0.025);
  }
};

TEST_F(ShortcutTest, FreeSpacePreservesEndpointsAndSetsTangentOrientation)
{
  auto p = path({{1, 1}, {1, 4}, {8, 4}, {8, 6}});
  p.poses.back().pose.orientation.z = 1.0;
  p.poses.back().pose.orientation.w = 0.0;
  const auto r = run(p);
  EXPECT_EQ(r.accepted_shortcuts, 1);
  EXPECT_NEAR(r.final_cost, std::hypot(7.0, 5.0), 1e-9);
  EXPECT_EQ(r.path.poses.front().pose.position, p.poses.front().pose.position);
  EXPECT_EQ(r.path.poses.back().pose, p.poses.back().pose);
  EXPECT_LT(r.final_cost, r.baseline_cost);
  for (size_t i = 0; i + 1 < r.path.poses.size(); ++i) {
    const auto & a = r.path.poses[i];
    const auto & b = r.path.poses[i + 1];
    EXPECT_EQ(a.header, p.header);
    EXPECT_LE(std::hypot(b.pose.position.x - a.pose.position.x,
      b.pose.position.y - a.pose.position.y), options.resample_spacing + 1e-9);
    EXPECT_NEAR(2.0 * std::atan2(a.pose.orientation.z, a.pose.orientation.w),
      std::atan2(5.0, 7.0), 1e-9);
  }
}

TEST_F(ShortcutTest, HighCostRegionDoesNotBecomeAShortcut)
{
  block(252);
  options.cost_weight = 30.0;
  const auto r = run(path({{1, 2}, {1, 6}, {8, 6}, {8, 2}}));
  EXPECT_GT(r.final_cost, 7.0);
  EXPECT_LE(r.final_cost, r.baseline_cost);
  EXPECT_GT(r.cost_rejections, 0);
}

TEST_F(ShortcutTest, LosUsesExistingInscribedAndLethalCellThreshold)
{
  for (unsigned char value : {253, 254}) {
    block(value);
    const auto r = run(path({{1, 2}, {1, 6}, {8, 6}, {8, 2}}));
    expectTraversable(r.path);
    EXPECT_GT(r.collision_rejections, 0);
  }
}

TEST_F(ShortcutTest, UnknownPolicyComesFromSmac)
{
  block(255);
  options.cost_weight = 0.0;
  const auto p = path({{1, 2}, {1, 6}, {8, 6}, {8, 2}});
  EXPECT_NEAR(run(p, true).final_cost, 7.0, 1e-9);
  const auto forbidden = run(p, false);
  EXPECT_GT(forbidden.final_cost, 7.0);
  expectTraversable(forbidden.path);
}

TEST_F(ShortcutTest, NoAdditionalFootprintOrBoundaryClearanceIsImposed)
{
  // The center line is valid in the original costmap but a 0.28 m disk would fail.
  map.setCost(40, 22, 254);
  const auto r = run(path({{1, 2.05}, {2, 1.9}, {6, 1.9}, {8, 2.05}}));
  EXPECT_NEAR(r.final_cost, 7.0, 1e-9);
  EXPECT_EQ(r.accepted_shortcuts, 1);
  EXPECT_NO_THROW(run(path({{0.01, 1}, {0.01, 2}, {0.01, 3}})));
}

TEST_F(ShortcutTest, BaselineIsNotRejectedByANewCollisionValidator)
{
  block(254);
  const auto p = path({{4.05, 1}, {4.05, 2}, {4.05, 3}});
  EXPECT_EQ(run(p).path, p);
  const auto outside = path({{-1, 1}, {1, 1}, {2, 1}});
  EXPECT_EQ(run(outside).path, outside);
}

TEST_F(ShortcutTest, ScoreDoesNotDependOnWaypointDensityOrDirection)
{
  for (unsigned int x = 30; x < 50; ++x) {
    for (unsigned int y = 0; y < 100; ++y) {
      map.setCost(x, y, 150);
    }
  }
  options.min_cost_improvement = 1000.0;
  const auto p = path({{1.01, 1.02}, {4.52, 3.53}, {8.03, 6.04}});
  const auto sparse = run(p);
  EXPECT_NEAR(sparse.baseline_cost, run(densify(p, 0.013)).baseline_cost, 1e-8);
  auto reverse = p;
  std::reverse(reverse.poses.begin(), reverse.poses.end());
  EXPECT_NEAR(sparse.baseline_cost, run(reverse).baseline_cost, 1e-8);
}

TEST_F(ShortcutTest, GridBoundaryAlignedSegmentsTerminateAndKeepExactLengths)
{
  options.cost_weight = 0.0;
  for (const auto & p : {path({{1, 1}, {1, 2}, {1, 8}}),
      path({{8, 1}, {4, 1}, {1, 1}}), path({{1, 1}, {4, 4}, {8, 8}})})
  {
    const auto r = run(p);
    EXPECT_NEAR(r.baseline_cost, length(p), 1e-9);
    EXPECT_TRUE(r.completed);
  }
}

TEST_F(ShortcutTest, DisabledIsExactPassthrough)
{
  options.enabled = false;
  const auto p = path({{1, 2}, {8, 2}});
  EXPECT_EQ(run(p).path, p);
}

TEST_F(ShortcutTest, ShortPathsAndDuplicates)
{
  for (const auto & p : {path({}), path({{1, 1}}), path({{1, 1}, {2, 2}})}) {
    EXPECT_EQ(run(p).path, p);
  }
  EXPECT_NO_THROW(run(path({{1, 1}, {1, 1}, {4, 4}, {4, 4}})));
}

TEST_F(ShortcutTest, LongMultiCornerPathStillStraightensTheTail)
{
  const auto p = longPath();
  ASSERT_GT(p.poses.size(), 2000u);
  options.max_candidate_checks = 12;
  const auto r = run(p);
  EXPECT_TRUE(r.completed);
  EXPECT_FALSE(r.timed_out);
  EXPECT_GT(r.accepted_shortcuts, 3);
  EXPECT_GT(r.candidate_checks, options.max_candidate_checks);
  EXPECT_LE(r.candidate_checks, r.processed_anchors * options.max_candidate_checks);
  EXPECT_LT(length(r.path), length(p));
  expectTraversable(r.path);
  auto tail = r.path;
  tail.poses.clear();
  for (const auto & pose : r.path.poses) {
    if (pose.pose.position.x >= 32.0) {
      tail.poses.push_back(pose);
    }
  }
  ASSERT_GT(tail.poses.size(), 2u);
  const auto & a = tail.poses.front().pose.position;
  const auto & b = tail.poses.back().pose.position;
  EXPECT_NEAR(length(tail), std::hypot(b.x - a.x, b.y - a.y), 1e-6);
}

TEST_F(ShortcutTest, BudgetIsRenewedAtEveryAnchor)
{
  const auto p = longPath();
  options.max_candidate_checks = 4;
  const auto r = run(p);
  EXPECT_TRUE(r.completed);
  EXPECT_GT(r.accepted_shortcuts, 2);
  EXPECT_LE(r.candidate_checks, 4 * r.processed_anchors);
}

TEST_F(ShortcutTest, ExpiredBudgetReturnsOriginalPlan)
{
  const auto p = path({{1, 1}, {1, 4}, {8, 6}});
  const auto r = CostAwareShortcut(options).process(
    p, map, false, []() {return false;}, Clock::now() - std::chrono::seconds(1));
  EXPECT_TRUE(r.timed_out);
  EXPECT_EQ(r.path, p);
}

TEST_F(ShortcutTest, TimeoutRetainsAlreadyCommittedShortcuts)
{
  const auto p = longPath();
  int calls = 0;
  const auto normal = CostAwareShortcut(options).process(
    p, map, false, [&]() {++calls; return false;}, Clock::now() + std::chrono::seconds(5));
  ASSERT_GT(normal.accepted_shortcuts, 2);
  // Expire midway through the two-direction search, after some independently
  // validated edges have been committed. Call counts are deterministic here.
  const int trigger = calls / 2;
  ASSERT_GT(trigger, 0);
  calls = 0;
  options.max_processing_time = 0.1;
  auto cancel = [&]() {
      if (++calls == trigger) {
        std::this_thread::sleep_for(std::chrono::milliseconds(110));
      }
      return false;
    };
  const auto r = CostAwareShortcut(options).process(
    p, map, false, cancel, Clock::now() + std::chrono::seconds(5));
  EXPECT_TRUE(r.timed_out);
  EXPECT_GT(r.accepted_shortcuts, 0);
  EXPECT_LT(r.final_cost, r.baseline_cost);
  EXPECT_EQ(r.path.poses.front().pose.position, p.poses.front().pose.position);
  EXPECT_EQ(r.path.poses.back().pose.position, p.poses.back().pose.position);
  expectTraversable(r.path);
}

TEST_F(ShortcutTest, CancellationPropagatesInsteadOfReturningAPartialPlan)
{
  const auto p = longPath();
  int checks = 0;
  EXPECT_THROW(CostAwareShortcut(options).process(
      p, map, false, [&]() {return ++checks > 100;},
      Clock::now() + std::chrono::seconds(5)), nav2_core::PlannerCancelled);
}

TEST_F(ShortcutTest, MapResizeAndOriginChangesDoNotUseStaleGeometry)
{
  const auto original = run(path({{1, 1}, {1, 4}, {8, 6}}));
  map.resizeMap(200, 200, 0.05, -5.0, -5.0);
  const auto shifted = run(path({{-4, -4}, {-4, -1}, {3, 1}}));
  EXPECT_NEAR(shifted.final_cost, original.final_cost, 1e-9);
}

TEST_F(ShortcutTest, PathologicalResamplingDoesNotAllocateUnboundedMemory)
{
  options.resample_spacing = 1e-300;
  const auto p = path({{1, 1}, {1, 4}, {8, 6}});
  EXPECT_EQ(run(p).path, p);
}

TEST_F(ShortcutTest, InvalidConfigurationRejected)
{
  options.resample_spacing = 0.0;
  EXPECT_THROW(CostAwareShortcut{options}, std::invalid_argument);
  options.resample_spacing = 0.05;
  options.cost_weight = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(CostAwareShortcut{options}, std::invalid_argument);
  options.cost_weight = 1.0;
  options.max_candidate_checks = 0;
  EXPECT_THROW(CostAwareShortcut{options}, std::invalid_argument);
}
}  // namespace
