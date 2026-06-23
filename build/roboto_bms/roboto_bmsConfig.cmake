
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was roboto_bmsConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

include(CMakeFindDependencyMacro)
find_dependency(fmt)
find_dependency(spdlog)

include("${CMAKE_CURRENT_LIST_DIR}/roboto_bmsTargets.cmake")

if(NOT TARGET roboto_bms::roboto_bms)
    add_library(roboto_bms::roboto_bms INTERFACE IMPORTED)
    target_link_libraries(roboto_bms::roboto_bms INTERFACE 
        roboto_bms::bms
        roboto_bms::tws_bms
    )
endif()

check_required_components(roboto_bms)
