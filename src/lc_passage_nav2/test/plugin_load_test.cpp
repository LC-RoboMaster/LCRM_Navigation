#include <algorithm>
#include <gtest/gtest.h>
#include <nav2_core/global_planner.hpp>
#include <pluginlib/class_loader.hpp>

TEST(PassagePlugin, DiscoverAndConstruct) {
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader(
      "nav2_core", "nav2_core::GlobalPlanner");
  const auto classes = loader.getDeclaredClasses();
  ASSERT_NE(std::find(classes.begin(), classes.end(),
                      "lc_passage_nav2/TdtPassagePlanner"),
            classes.end());
  EXPECT_NO_THROW({
    auto plugin =
        loader.createSharedInstance("lc_passage_nav2/TdtPassagePlanner");
    ASSERT_TRUE(plugin);
  });
}
