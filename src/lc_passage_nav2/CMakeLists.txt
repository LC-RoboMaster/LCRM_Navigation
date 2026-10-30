cmake_minimum_required(VERSION 3.18)
project(lc_passage_nav2 VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
# Bundled solver projects use older CMake policy versions.
if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0")
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()

find_package(ament_cmake REQUIRED)
set(ros_dependencies rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util
  nav_msgs geometry_msgs std_msgs visualization_msgs tf2_ros pluginlib lc_passage_msgs lc_passage_common)
foreach(dependency IN LISTS ros_dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

# The vendor project disables its solver tests via BUILD_TESTING; preserve the
# outer ROS package option before configuring it, then restore it afterwards.
option(BUILD_TESTING "Build regression and plugin-loading tests" ON)
set(lc_passage_build_testing ${BUILD_TESTING})
set(TDT_BUILD_APPS OFF CACHE BOOL "No standalone apps in ROS build" FORCE)
set(TDT_BUILD_TESTS ${lc_passage_build_testing} CACHE BOOL "Core regressions" FORCE)
add_subdirectory(vendor/tdt_passage)
set(BUILD_TESTING ${lc_passage_build_testing} CACHE BOOL "Build tests" FORCE)

add_library(lc_passage_nav2_plugin SHARED src/tdt_passage_planner.cpp)
target_include_directories(lc_passage_nav2_plugin PRIVATE include)
target_link_libraries(lc_passage_nav2_plugin tdt_passage)
ament_target_dependencies(lc_passage_nav2_plugin ${ros_dependencies})
target_compile_options(lc_passage_nav2_plugin PRIVATE -Wall -Wextra -Wpedantic)
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

add_executable(passage_cli vendor/tdt_passage/cpp/passage_cli.cpp)
target_link_libraries(passage_cli PRIVATE tdt_passage)
add_executable(passage_export_nav2_map src/export_nav2_map.cpp)
target_link_libraries(passage_export_nav2_map PRIVATE tdt_passage)
install(TARGETS lc_passage_nav2_plugin
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS passage_cli passage_export_nav2_map RUNTIME DESTINATION lib/${PROJECT_NAME})
# Runtime parameters are installed by pb2025_nav_bringup. Git does not retain
# the unused empty config/ directory, so only install the tracked documentation.
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE THIRD_PARTY_NOTICES.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(passage_simulation_map_test test/simulation_map_test.cpp)
  target_link_libraries(passage_simulation_map_test PRIVATE tdt_passage)
  add_test(NAME passage_simulation_map COMMAND passage_simulation_map_test
    ${CMAKE_CURRENT_SOURCE_DIR}/test/data/simulation_project.json)
  add_executable(costmap_snapshot_tests test/costmap_snapshot_tests.cpp)
  target_include_directories(costmap_snapshot_tests PRIVATE include)
  target_link_libraries(costmap_snapshot_tests PRIVATE tdt_passage)
  add_test(NAME costmap_snapshot COMMAND costmap_snapshot_tests)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(plugin_load_test test/plugin_load_test.cpp)
  ament_target_dependencies(plugin_load_test nav2_core pluginlib)
  ament_add_gtest(planner_ros_test test/ros_integration_test.cpp TIMEOUT 60)
  target_include_directories(planner_ros_test PRIVATE include)
  target_link_libraries(planner_ros_test lc_passage_nav2_plugin tdt_passage)
  ament_target_dependencies(planner_ros_test ${ros_dependencies})
endif()
ament_package()
