project(cython_thunderborg_src)

set_source_files_properties(
  ${CMAKE_CURRENT_SOURCE_DIR}/ThunderBorg.pyx
  PROPERTIES CYTHON_IS_CXX TRUE )
  
cython_add_module(cython_thunderborg ThunderBorg.pyx)

