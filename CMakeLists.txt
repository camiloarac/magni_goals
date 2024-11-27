cmake_minimum_required(VERSION 3.0.2)
project(magni_goals)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  move_base_msgs
  roscpp
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS
#  LIBRARIES magni_goals
#  CATKIN_DEPENDS actionlib actionlib_msgs move_base_msgs roscpp
 DEPENDS 
  actionlib
  actionlib_msgs
  move_base_msgs
  roscpp
)

###########
## Build ##
###########

add_library(goals_lib src/magni_goals.cpp src/read_csv.cpp)
target_include_directories(goals_lib
  PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(goals_lib
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} src/main.cpp)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  goals_lib
)
