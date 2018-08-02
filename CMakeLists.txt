cmake_minimum_required(VERSION 3.0.0)
project(chinese_postman_problem)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(EXECUTABLE_OUTPUT_PATH "../bin")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})


include_directories(src)

#MESSAGE( STATUS "variable is: " ${EIGEN3_INCLUDE_DIR} )

add_executable(create_graph
  src/test/create_graph.cpp
  src/graph/graph.cpp
  src/graph/graph_utils.cpp
  src/opencv_utilities.cpp
  src/paths/shortest_paths.cpp
)

target_link_libraries(create_graph
  ${OpenCV_LIBS}
)

add_executable(routing
  src/test/routing_test.cpp
  src/graph/graph.cpp
  src/graph/graph_utils.cpp
  src/routing/routing_problem.cpp
  src/routing/eulerian_extension.cpp
  src/routing/branch_bound.cpp
  src/paths/network_flow.cpp
  src/paths/shortest_paths.cpp
  src/utilities.cpp
  src/opencv_utilities.cpp
)


target_link_libraries(routing
  ${OpenCV_LIBS}
)





