#include <gtest/gtest.h>
#include <nav2_core/progress_checker.hpp>
#include <nav2_core/goal_checker.hpp>
#include <pluginlib/class_loader.hpp>
TEST(PassageCheckers, InstalledPluginsAreDiscoverable) {
  pluginlib::ClassLoader<nav2_core::ProgressChecker> progress("nav2_core", "nav2_core::ProgressChecker");
  pluginlib::ClassLoader<nav2_core::GoalChecker> goal("nav2_core", "nav2_core::GoalChecker");
  EXPECT_NO_THROW(progress.createSharedInstance("lc_passage_checkers/PassageProgressChecker"));
  EXPECT_NO_THROW(goal.createSharedInstance("lc_passage_checkers/PassageGoalChecker"));
}
