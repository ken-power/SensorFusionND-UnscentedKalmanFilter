cmake_minimum_required(VERSION 3.17 FATAL_ERROR)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(unscented-kalman-filter)

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable(ukf_highway main.cpp ukf/ukf.cpp ukf/tools.cpp render/render.cpp)
target_link_libraries(ukf_highway ${PCL_LIBRARIES})
