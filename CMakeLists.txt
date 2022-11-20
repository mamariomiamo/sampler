cmake_minimum_required(VERSION 3.0.2)
project(sampler)
add_compile_options(-std=c++17) # for structural binding
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES sampler
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
  DEPENDS Eigen3
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include/sampler
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(sampler
  src/main.cpp
)

target_link_libraries(sampler
  ${catkin_LIBRARIES}
)

