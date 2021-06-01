project(cython_thunderborg_src)
# If the pyx file is a C++ file, we should specify that here.
set_source_files_properties(
  ${CMAKE_CURRENT_SOURCE_DIR}/ThunderBorg.pyx
  PROPERTIES CYTHON_IS_CXX TRUE )
  
# add_custom_target(ReplicatePythonSourceTree ALL ${CMAKE_COMMAND} -P
#    ${cython_thunderborg_ROOT}/cmake/ReplicatePythonSourceTree.cmake
#    ${CMAKE_CURRENT_BINARY_DIR}
#    WORKING_DIRECTORY ${cython_thunderborg_ROOT})

# Multi-file cython modules do not appear to be working at the moment.
cython_add_module(cython_thunderborg ThunderBorg.pyx)

 

#add_cython_target(cy_code ${CMAKE_CURRENT_BINARY_DIR}/src/cython-thunderborg/src/ThunderBorgCythonMock.pyx)
#add_library(cy_code MODULE ${cy_code})

# 
#add_library(thunderborg STATIC ${generated_file})

#add_dependencies(thunderborg cython_thunderborg)

# target_link_libraries(cy_code ...)

#target_include_directories(thunderborg PUBLIC  ${CMAKE_CURRENT_BINARY_DIR}/src/)


#target_link_libraries(thunderborg ${PYTHON_LIBRARIES})
 
#target_link_libraries(cython_catkin_example ${PCL_LIBRARIES})

# if(${cython_thunderborg_ROS})
	# set_target_properties(cython_thunderborg PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
	# install(TARGETS cython_thunderborg
	#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
	#   LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
	# )
# endif(${cython_thunderborg_ROS})
