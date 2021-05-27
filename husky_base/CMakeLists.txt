cmake_minimum_required(VERSION 2.8.3)
project(husky_base)

find_package(catkin REQUIRED COMPONENTS controller_manager hardware_interface husky_msgs
  diagnostic_updater roslaunch roslint roscpp sensor_msgs)
find_package(Boost REQUIRED COMPONENTS chrono)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS diagnostic_updater hardware_interface husky_msgs roscpp sensor_msgs
  DEPENDS Boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} /usr/include/python2.7/ /usr/include/arm-linux-gnueabihf/python2.7/)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;")

# add_custom_command(
#      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/src/cython-thunderborg/src/cython_thunderborg.cxx
#      DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/src/cython-thunderborg/src/cython_thunderborg.cxx
# )

# CYTHON MODULE
set(cython_thunderborg_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/src/cython-thunderborg)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${cython_thunderborg_ROOT}/cmake)
include(UseCython)
add_custom_target(ReplicatePythonSourceTree ALL ${CMAKE_COMMAND} -P
  ${cython_thunderborg_ROOT}/cmake/ReplicatePythonSourceTree.cmake
  ${CMAKE_CURRENT_BINARY_DIR}
  WORKING_DIRECTORY ${cython_thunderborg_ROOT})

# # if not set, cython_catkin_example will build as standalone project
set(cython_thunderborg_ROS TRUE)
set_source_files_properties(${cython_thunderborg_ROOT}/src/ThunderBorgCythonMock.pyx PROPERTIES CYTHON_IS_CXX TRUE)
add_subdirectory(${cython_thunderborg_ROOT}/src)

# include Python.h in cython_thunderborg.h  - compiling with cython does not do this for us, for some reason.

#FILE(APPEND ${CMAKE_CURRENT_BINARY_DIR}/src/cython-thunderborg/src/cython_thunderborg.h "/n #include <Python.h>")

# set(IN_FILE ${CMAKE_CURRENT_BINARY_DIR}/src/cython-thunderborg/src/cython_thunderborg.h)
# set(OUT_FILE ${CMAKE_CURRENT_BINARY_DIR}/src/cython-thunderborg/src/thunderborg.h)

#Create static library thunderborg

#configure_file(${CMAKE_CURRENT_BINARY_DIR}/src/cython-thunderborg/src/cython_thunderborg.h ${CMAKE_CURRENT_BINARY_DIR}/src/thunderborg/src/thunderborg.h)
#configure_file(${CMAKE_CURRENT_BINARY_DIR}/src/cython-thunderborg/src/cython_thunderborg.cxx ${CMAKE_CURRENT_BINARY_DIR}/src/thunderborg/src/thunderborg.cpp)

#set(TB_HEADER ${CMAKE_CURRENT_BINARY_DIR}/src/thunderborg/src/thunderborg.h)

#file (READ $TB_HEADER CONTENT)
#file(WRITE $TB_HEADER "#include <Python.h>/n")
#file(APPEND $TB_HEADER $CONTENT)

# #file(APPEND ${IN_FILE} "${LINES}")
# foreach(LINE IN LINES)
#   #  string(REGEX REPLACE ${pattern} "" STRIPPED "${LINE}")
#     file(APPEND ${IN_FILE} "${LINE}\n")
#   endforeach()

#set(CMD  "cat <(echo "#include <Python.h>\") cython_thunderborg.h > thunderborg.h" )
# set(args "-i.old '1s;^;#include <Python.h>;/n; cython_thunderborg.h")


# add_custom_command(
#      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/src/cython-thunderborg/src/thunderborg.h
#      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/src/cython-thunderborg/src/
#      COMMAND bash -c sed "${args}" VERBATIM
#      DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/src/cython-thunderborg/src/cython_thunderborg.h
#  )


# add_library(thunderborg
#   ${CMAKE_CURRENT_BINARY_DIR}/src/cython-thunderborg/src/cython_thunderborg.cxx
#   # ${CMAKE_CURRENT_BINARY_DIR}/src/cython-thunderborg/src/thunderborg.h
# )

# set(THUNDERBORG_INCLUDES ${CMAKE_CURRENT_BINARY_DIR}/src/cython-thunderborg/src)
# include_directories(${THUNDERBORG_INCLUDES})

# target_include_directories (thunderborg PUBLIC ${THUNDERBORG_INCLUDES})
 
# Build nemesis_node
add_executable(husky_node
  src/husky_base.cpp
  src/husky_hardware.cpp
#  src/husky_diagnostics.cpp
  src/horizon_legacy_wrapper.cpp)


#target_link_libraries(husky_node thunderborg ${catkin_LIBRARIES} ${Boost_LIBRARIES})

target_link_libraries(husky_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# roslint_cpp(
#   src/husky_base.cpp
#   src/husky_hardware.cpp
#   src/husky_diagnostics.cpp
#   include/husky_base/husky_diagnostics.h
#   include/husky_base/husky_hardware.h
# )

roslaunch_add_file_check(launch)

install(TARGETS husky_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
