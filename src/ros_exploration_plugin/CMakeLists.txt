project(cpp_solver)

  message(STATUS "dir='${PROJECT_NAME}'")




###########
## Build ##
###########

include_directories(
  src/
  ${catkin_INCLUDE_DIRS}
)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

add_library(ChinesePostmanProblemPlugin
  src/ChinesePostmanPlanner.cpp
  src/cpp_exploration_plugin.cpp
  )


  message(STATUS "catkin_EXPORTED_TARGETS='${catkin_EXPORTED_TARGETS}'")

add_dependencies(ChinesePostmanProblemPlugin ${catkin_EXPORTED_TARGETS})

target_link_libraries(ChinesePostmanProblemPlugin cpp_solver_library ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS ChinesePostmanProblemPlugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
  cpp_exploration.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cpp_exploration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
