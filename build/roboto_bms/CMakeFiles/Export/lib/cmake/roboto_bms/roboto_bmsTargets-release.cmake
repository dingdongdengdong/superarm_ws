#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "roboto_bms::bms" for configuration "Release"
set_property(TARGET roboto_bms::bms APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(roboto_bms::bms PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libbms.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS roboto_bms::bms )
list(APPEND _IMPORT_CHECK_FILES_FOR_roboto_bms::bms "${_IMPORT_PREFIX}/lib/libbms.a" )

# Import target "roboto_bms::tws_bms" for configuration "Release"
set_property(TARGET roboto_bms::tws_bms APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(roboto_bms::tws_bms PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libtws_bms.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS roboto_bms::tws_bms )
list(APPEND _IMPORT_CHECK_FILES_FOR_roboto_bms::tws_bms "${_IMPORT_PREFIX}/lib/libtws_bms.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
