cmake_minimum_required(VERSION 3.0.2)
project(mavros_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  mavros_base
)

catkin_package(
 INCLUDE_DIRS include/${PROJECT_NAME}
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp mavros_base
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  mavros_base_INCLUDE_DIRS
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/ControllerBase.cpp
)

## Declare a C++ executable
add_executable(mvcon_basic_flight_node
  examples/basic_flight.cpp
  # src/offb_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Link the executable to the library just created.
target_link_libraries(mvcon_basic_flight_node
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
  # PATTERN ".svn" EXCLUDE
)