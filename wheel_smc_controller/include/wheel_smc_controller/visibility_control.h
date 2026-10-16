#ifndef WHEEL_SMC_CONTROLLER__VISIBILITY_CONTROL_H_
#define WHEEL_SMC_CONTROLLER__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define WHEEL_SMC_CONTROLLER_EXPORT __attribute__((dllexport))
    #define WHEEL_SMC_CONTROLLER_IMPORT __attribute__((dllimport))
  #else
    #define WHEEL_SMC_CONTROLLER_EXPORT __declspec(dllexport)
    #define WHEEL_SMC_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef WHEEL_SMC_CONTROLLER_BUILDING_DLL
    #define WHEEL_SMC_CONTROLLER_PUBLIC WHEEL_SMC_CONTROLLER_EXPORT
  #else
    #define WHEEL_SMC_CONTROLLER_PUBLIC WHEEL_SMC_CONTROLLER_IMPORT
  #endif
  #define WHEEL_SMC_CONTROLLER_PUBLIC_TYPE WHEEL_SMC_CONTROLLER_PUBLIC
  #define WHEEL_SMC_CONTROLLER_LOCAL
#else
  #define WHEEL_SMC_CONTROLLER_EXPORT __attribute__((visibility("default")))
  #define WHEEL_SMC_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define WHEEL_SMC_CONTROLLER_PUBLIC __attribute__((visibility("default")))
    #define WHEEL_SMC_CONTROLLER_LOCAL  __attribute__((visibility("hidden")))
  #else
    #define WHEEL_SMC_CONTROLLER_PUBLIC
    #define WHEEL_SMC_CONTROLLER_LOCAL
  #endif
  #define WHEEL_SMC_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // WHEEL_SMC_CONTROLLER__VISIBILITY_CONTROL_H_