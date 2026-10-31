cmake_minimum_required(VERSION 3.10)
project(pcd_map_saver)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_executable(${PROJECT_NAME}_node src/pcd_map_saver.cpp)
target_include_directories(${PROJECT_NAME}_node SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node ${PCL_LIBRARIES})

#############
## Install ##
#############

ament_auto_package()
