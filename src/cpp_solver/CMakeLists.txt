

set(GRAPH_SRC
  graph/graph.cpp
  graph/graph_utils.cpp
  graph/graph_factory.cpp
)

set(ROUTING_SRC
  routing/routing_problem.cpp
  routing/eulerian_extension.cpp
  routing/branch_bound.cpp
)

set(PATHS_ALGORITHMS_SRC
  paths/shortest_paths.cpp
  paths/network_flow.cpp
)


set(LIBRARY_SRC
  ${GRAPH_SRC}
  ${ROUTING_SRC}
  ${PATHS_ALGORITHMS_SRC}
  utilities.cpp
)

if (OpenCV_FOUND)
  set (LIBRARY_SRC ${LIBRARY_SRC}
    opencv_utilities.cpp
  )
endif()


add_library(cpp_solver_library SHARED
  ${LIBRARY_SRC}
)



