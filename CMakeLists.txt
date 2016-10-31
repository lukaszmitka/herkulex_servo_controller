cmake_minimum_required(VERSION 3.1.0)
project(herkulex_servo_controller)

set(THREADS_PREFER_PTHREAD_FLAG ON)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package (Threads REQUIRED)
#set (CMAKE_THREAD_PREFER_PTHREAD)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES herkulex_servo_controller
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}

)

## Declare a C++ library
# add_library(herkulex_servo_controller
#   src/${PROJECT_NAME}/herkulex_servo_controller.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(herkulex_servo_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(herkulex_servo_controller_node src/herkulex_servo_controller_node.cpp src/ComPortDriver.cpp src/rs232.c src/HerkulexController.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(herkulex_servo_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(herkulex_servo_controller_node
   ${catkin_LIBRARIES}
 )