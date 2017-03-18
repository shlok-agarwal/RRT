project(RRT)
cmake_minimum_required(VERSION 2.8)
add_executable(RRT src/RRT.cpp)
find_package(OpenRAVE REQUIRED)
include_directories(include ${OpenRAVE_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS})
set_target_properties(RRT PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(RRT ${OpenRAVE_LIBRARIES})


