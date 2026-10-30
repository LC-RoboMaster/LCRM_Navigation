# Standalone native regressions for hosts without ROS. Does not pretend to
# compile or load the ROS plugin; the ROS package's plugin_load_test does that.
cmake_minimum_required(VERSION 3.18)
project(lc_passage_native_checks LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(TDT_BUILD_APPS OFF CACHE BOOL "" FORCE)
set(TDT_BUILD_TESTS ON CACHE BOOL "" FORCE)
if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0")
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()
enable_testing()
find_package(nlohmann_json REQUIRED)
add_subdirectory(../vendor/tdt_passage vendor-build)
add_executable(costmap_snapshot_tests costmap_snapshot_tests.cpp)
target_include_directories(costmap_snapshot_tests PRIVATE ../include)
target_link_libraries(costmap_snapshot_tests PRIVATE tdt_passage)
add_test(NAME costmap_snapshot COMMAND costmap_snapshot_tests)
add_executable(passage_export_nav2_map ../src/export_nav2_map.cpp)
target_link_libraries(passage_export_nav2_map PRIVATE tdt_passage)


add_executable(passage_simulation_map_test simulation_map_test.cpp)
target_link_libraries(passage_simulation_map_test PRIVATE tdt_passage)
add_test(NAME passage_simulation_map COMMAND passage_simulation_map_test
  ${CMAKE_CURRENT_SOURCE_DIR}/data/simulation_project.json)
