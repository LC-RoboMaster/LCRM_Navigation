cmake_minimum_required(VERSION 3.10)
project(pb_omni_pid_pursuit_controller)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/omni_pid_pursuit_controller.cpp
  src/pid.cpp
  src/passage_controller.cpp
)

option(PASSAGE_BUILD_TESTING "Build passage execution and ROS adapter regressions" ${BUILD_TESTING})
if(PASSAGE_BUILD_TESTING)
  enable_testing()
  add_executable(passage_execution_tests test/execution_tests.cpp)
  target_compile_features(passage_execution_tests PRIVATE cxx_std_17)
  target_include_directories(passage_execution_tests PRIVATE include)
  ament_target_dependencies(passage_execution_tests lc_passage_common)
  add_test(NAME passage_execution COMMAND passage_execution_tests)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(passage_controller_ros_test test/ros_integration_test.cpp TIMEOUT 60)
  target_link_libraries(passage_controller_ros_test ${PROJECT_NAME})
  ament_target_dependencies(passage_controller_ros_test rclcpp rclcpp_lifecycle lc_passage_msgs lc_passage_common)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(nav2_core pb_omni_pid_pursuit_controller.xml)

ament_auto_package()
