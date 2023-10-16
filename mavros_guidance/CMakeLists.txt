cmake_minimum_required(VERSION 3.0.2)
project(mavros_guidance)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros_base
  message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Waypoint.msg
  WaypointSequence.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  mavros_msgs
)

catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp mavros_base
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  mavros_base_INCLUDE_DIRS
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/GuidanceBase.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(mvguide_waypoint_node 
  planners/waypoints.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Link the executable to the library just created.
target_link_libraries(mvguide_waypoint_node
  ${PROJECT_NAME}
)

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)