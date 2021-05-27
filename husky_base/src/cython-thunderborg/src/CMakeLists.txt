# If the pyx file is a C++ file, we should specify that here.
set_source_files_properties(
  ${CMAKE_CURRENT_SOURCE_DIR}/ThunderBorgCythonMock.pyx
  PROPERTIES CYTHON_IS_CXX TRUE )
  

# Multi-file cython modules do not appear to be working at the moment.
cython_add_module(cython_thunderborg ThunderBorgCythonMock.pyx)

#target_link_libraries(cython_catkin_example ${PCL_LIBRARIES})

if(${cython_thunderborg_ROS})
	set_target_properties(cython_thunderborg PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
	install(TARGETS cython_thunderborg
	  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
	  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
	)
endif(${cython_catkin_example_ROS})
