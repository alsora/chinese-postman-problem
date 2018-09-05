

set (TARGET_LIBRARIES
 cpp_solver_library
)

if (OpenCV_FOUND)
  set(TARGET_LIBRARIES
    ${TARGET_LIBRARIES} 
    ${OpenCV_LIBS}
  )
endif()


add_executable(create_graph
  create_graph.cpp
)

target_link_libraries(create_graph
  ${TARGET_LIBRARIES} 
)



add_executable(routing
  routing_test.cpp
)


target_link_libraries(routing
  ${TARGET_LIBRARIES} 
)



add_executable(benchmark
  benchmark.cpp
)

target_link_libraries(benchmark
  ${TARGET_LIBRARIES} 
)
