#ifndef LROBOT_CONTROLLER_VISIBILITY_CONTROL_H
#define LROBOT_CONTROLLER_VISIBILITY_CONTROL_H


// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define LROBOT_CONTROLLER_EXPORT __attribute__((dllexport))
#define LROBOT_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define LROBOT_CONTROLLER_EXPORT __declspec(dllexport)
#define LROBOT_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef LROBOT_CONTROLLER_BUILDING_DLL
#define LROBOT_CONTROLLER_PUBLIC LROBOT_CONTROLLER_EXPORT
#else
#define LROBOT_CONTROLLER_PUBLIC LROBOT_CONTROLLER_IMPORT
#endif
#define LROBOT_CONTROLLER_PUBLIC_TYPE LROBOT_CONTROLLER_PUBLIC
#define LROBOT_CONTROLLER_LOCAL
#else
#define LROBOT_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define LROBOT_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define LROBOT_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define LROBOT_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define LROBOT_CONTROLLER_PUBLIC
#define LROBOT_CONTROLLER_LOCAL
#endif
#define LROBOT_CONTROLLER_PUBLIC_TYPE
#endif

#endif /* LROBOT_CONTROLLER_VISIBILITY_CONTROL_H */

