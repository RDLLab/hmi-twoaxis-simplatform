#----------------------------------------------------------------
# Generated CMake target import file for configuration "RELWITHDEBINFO".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "oppt" for configuration "RELWITHDEBINFO"
set_property(TARGET oppt APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(oppt PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/liboppt.so"
  IMPORTED_SONAME_RELWITHDEBINFO "liboppt.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS oppt )
list(APPEND _IMPORT_CHECK_FILES_FOR_oppt "${_IMPORT_PREFIX}/lib/liboppt.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
