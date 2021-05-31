cmake_minimum_required(VERSION 2.8.3)

project(husky_base)

find_package(catkin REQUIRED COMPONENTS controller_manager hardware_interface husky_msgs
   diagnostic_updater roslaunch roslint roscpp sensor_msgs)
 find_package(Boost REQUIRED COMPONENTS chrono)

find_package(PythonLibs REQUIRED)

set(cython_thunderborg_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/src/cython-thunderborg)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${cython_thunderborg_ROOT}/cmake)

find_package(Cython)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS diagnostic_updater hardware_interface husky_msgs roscpp sensor_msgs
   DEPENDS Boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;")
include(UseCython)

add_subdirectory(${cython_thunderborg_ROOT}/src)

# CYTHON MODULE
# set_source_files_properties(${cython_thunderborg_ROOT}/src/ThunderBorgCythonMock.pyx PROPERTIES CYTHON_IS_CXX TRUE)
#
#   # Note: In this case, either one of these arguments may be omitted; their
#   # value would have been inferred from that of the other.

# add_custom_target(ReplicatePythonSourceTree ALL ${CMAKE_COMMAND} -P
#   ${cython_thunderborg_ROOT}/cmake/ReplicatePythonSourceTree.cmake
#   ${CMAKE_CURRENT_BINARY_DIR}
#   WORKING_DIRECTORY ${cython_thunderborg_ROOT})

#add_cython_target(cy_code ${CMAKE_CURRENT_BINARY_DIR}/src/cython-thunderborg/src/ThunderBorgCythonMock.pyx)
#add_dependencies(cy_code ReplicatePythonSourceTree)
#add_library(cy_code MODULE ${cy_code})

# target_link_libraries(cy_code ...)

#target_include_directories(cy_code PUBLIC  ${CMAKE_CURRENT_BINARY_DIR}/src/)


#target_link_libraries(thunderborg ${PYTHON_LIBRARIES})
 
# # Build nemesis_node
# add_executable(husky_node
#    src/husky_base.cpp
#    src/husky_hardware.cpp
#  #  src/husky_diagnostics.cpp
#    src/horizon_legacy_wrapper.cpp
# # # ${CMAKE_CURRENT_BINARY_DIR}/src/cython-thunderborg/src/cython_thunderborg.cxx
# # # ${CMAKE_CURRENT_BINARY_DIR}/src/cython-thunderborg/src/cython_thunderborg.h
# )

# add_definitions(-include Python.h)
# target_link_libraries(husky_node thunderborg ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# # target_link_libraries(husky_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# # # roslint_cpp(
# # #   src/husky_base.cpp
# # #   src/husky_hardware.cpp
# # #   src/husky_diagnostics.cpp
# # #   include/husky_base/husky_diagnostics.h
# # #   include/husky_base/husky_hardware.h
# # # )

# roslaunch_add_file_check(launch)

# install(TARGETS husky_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# install(DIRECTORY launch config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
