cmake_minimum_required(VERSION 3.0.0)
project(chinese_postman_problem)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(EXECUTABLE_OUTPUT_PATH "../bin")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package( OpenCV )
if (OpenCV_FOUND)
  add_definitions ( -DOPENCV_LIBRARY_FOUND=12 )
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()


include_directories(src)

#MESSAGE( STATUS "variable is: " ${EIGEN3_INCLUDE_DIR} )

set(CREATE_GRAPH_SRC 
  src/graph/graph.cpp
  src/graph/graph_utils.cpp
  src/graph/graph_factory.cpp
  src/paths/shortest_paths.cpp
)

if (OpenCV_FOUND)
  set (CREATE_GRAPH_SRC ${CREATE_GRAPH_SRC}
    src/opencv_utilities.cpp
  )
endif()

add_executable(create_graph
  src/test/create_graph.cpp
  ${CREATE_GRAPH_SRC}
)

if (OpenCV_FOUND)
  target_link_libraries(create_graph ${OpenCV_LIBS})
endif()

set (ROUTING_SRC ${CREATE_GRAPH_SRC}
  src/routing/routing_problem.cpp
  src/routing/eulerian_extension.cpp
  src/routing/branch_bound.cpp
  src/paths/network_flow.cpp
  src/utilities.cpp
)

add_executable(routing
  src/test/routing_test.cpp
  ${ROUTING_SRC}
)


if (OpenCV_FOUND)
  target_link_libraries(routing ${OpenCV_LIBS})
endif()


add_executable(benchmark
  src/test/benchmark.cpp
  ${ROUTING_SRC}
)

if (OpenCV_FOUND)
  target_link_libraries(benchmark ${OpenCV_LIBS})
endif()