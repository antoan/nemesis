cmake_minimum_required(VERSION 2.8.3)

project(nemesis_base)

find_package(catkin REQUIRED COMPONENTS controller_manager hardware_interface nemesis_msgs
   diagnostic_updater roslaunch roslint roscpp sensor_msgs)
 find_package(Boost REQUIRED COMPONENTS chrono)

find_package(PythonLibs REQUIRED)

set(cython_thunderborg_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/src/cython-thunderborg)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${cython_thunderborg_ROOT}/cmake)

find_package(Cython)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS diagnostic_updater hardware_interface nemesis_msgs roscpp sensor_msgs
   DEPENDS Boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;")
include(UseCython)

add_subdirectory(${cython_thunderborg_ROOT}/src)

# Build nemesis_node
add_executable(nemesis_node
   src/nemesis_base.cpp
   src/nemesis_hardware.cpp
   src/nemesis_diagnostics.cpp
)

add_definitions(-include Python.h)
target_link_libraries(nemesis_node cython_thunderborg ${catkin_LIBRARIES} ${Boost_LIBRARIES})

roslint_cpp(
  src/nemesis_base.cpp
  src/nemesis_hardware.cpp
  src/nemesis_diagnostics.cpp
  include/nemesis_base/nemesis_diagnostics.h
  include/nemesis_base/nemesis_hardware.h
)

roslaunch_add_file_check(launch)

install(TARGETS nemesis_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
