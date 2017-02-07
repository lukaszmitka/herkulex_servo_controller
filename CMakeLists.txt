cmake_minimum_required(VERSION 3.1.0)
project(herkulex_servo_controller)

set(THREADS_PREFER_PTHREAD_FLAG ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Threads REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        serial
        )

catkin_package(
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)


add_executable(servo_waggle src/waggle_node.cpp)

add_executable(servo_controller src/herkulex_servo_controller_node.cpp src/HerkulexController.cpp)


target_link_libraries(servo_controller
        ${catkin_LIBRARIES}
        )


target_link_libraries(servo_waggle
        ${catkin_LIBRARIES}
        )