set(PROJECT_NAME robotranYarpPlugin)
project(${PROJECT_NAME})
cmake_minimum_required(VERSION 2.6)
find_package(YARP REQUIRED)


include_directories(${YARP_INCLUDE_DIRS})

set(ROBOTRAN_PLUGINS_SRC    yarp_files.h
                            yarp_finish.cpp
                            yarp_init.cpp
                            yarp_update.cpp)


set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${YARP_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
