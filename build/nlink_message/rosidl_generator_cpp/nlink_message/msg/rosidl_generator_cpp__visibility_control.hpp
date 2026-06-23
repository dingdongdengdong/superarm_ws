// generated from rosidl_generator_cpp/resource/rosidl_generator_cpp__visibility_control.hpp.in
// generated code does not contain a copyright notice

#ifndef NLINK_MESSAGE__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
#define NLINK_MESSAGE__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_CPP_EXPORT_nlink_message __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_CPP_IMPORT_nlink_message __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_CPP_EXPORT_nlink_message __declspec(dllexport)
    #define ROSIDL_GENERATOR_CPP_IMPORT_nlink_message __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_CPP_BUILDING_DLL_nlink_message
    #define ROSIDL_GENERATOR_CPP_PUBLIC_nlink_message ROSIDL_GENERATOR_CPP_EXPORT_nlink_message
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_nlink_message ROSIDL_GENERATOR_CPP_IMPORT_nlink_message
  #endif
#else
  #define ROSIDL_GENERATOR_CPP_EXPORT_nlink_message __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_CPP_IMPORT_nlink_message
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_CPP_PUBLIC_nlink_message __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_nlink_message
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // NLINK_MESSAGE__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
