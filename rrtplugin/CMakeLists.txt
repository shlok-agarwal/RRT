cmake_minimum_required (VERSION 2.6.0)
project(rrtplugin)
find_package(OpenRAVE 0.9 REQUIRED)
set(CMAKE_CXX_FLAGS "-std=c++11")

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

include_directories(include ${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_library(rrtplugin SHARED src/rrtplugin.cpp src/RRTNode.cpp src/NodeTree.cpp src/RRT.cpp)
set_target_properties(rrtplugin PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(rrtplugin ${OpenRAVE_LIBRARIES})
install(TARGETS rrtplugin DESTINATION .)
