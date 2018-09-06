
###########
## Build ##
###########

include_directories(
  include/${PROJECT_NAME}/
  ${catkin_INCLUDE_DIRS}
)
add_library(CppExplorationPlugins
  src/ChinesePostmanPlanner.cpp
  src/exploration_plugins.cpp)

add_dependencies(CppExplorationPlugins ${catkin_EXPORTED_TARGETS})

target_link_libraries(CppExplorationPlugins cpp_solver_library ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS CppExplorationPlugins
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
  exploration.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nav2d_exploration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
