cmake_minimum_required(VERSION 3.0.2)
project(OBVP_Planner)


set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

if(POLICY CMP0094)
  cmake_policy(SET CMP0094 NEW)
endif()
set(Python3_FIND_VIRTUALENV FIRST)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    include
)

catkin_package()
add_executable(obvp_planner_test src/obvp_planner_test.cpp)
target_link_libraries(obvp_planner_test
    ${catkin_LIBRARIES}
)