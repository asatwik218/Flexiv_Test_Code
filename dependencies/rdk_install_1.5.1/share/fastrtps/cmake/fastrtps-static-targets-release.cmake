#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "fastrtps" for configuration "Release"
set_property(TARGET fastrtps APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(fastrtps PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX;RC"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libfastrtps-2.6.lib"
  )

list(APPEND _cmake_import_check_targets fastrtps )
list(APPEND _cmake_import_check_files_for_fastrtps "${_IMPORT_PREFIX}/lib/libfastrtps-2.6.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
