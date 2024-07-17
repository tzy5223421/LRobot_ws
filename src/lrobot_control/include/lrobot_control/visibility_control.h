#ifndef LROBOT_CONTROL__VISIBILITY_CONTORL_H_
#define LROBOT_CONTROL__VISIBILITY_CONTORL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define LROBOT_CONTROL_EXPORT __attribute__((dllexport))
#define LROBOT_CONTROL_IMPORT __attribute__((dllimport))
#else
#define LROBOT_CONTROL_EXPORT __declspec(dllexport)
#define LROBOT_CONTROL_IMPORT __declspec(dllimport)
#endif
#ifdef LROBOT_CONTROL_BUILDING_DLL
#define LROBOT_CONTROL_PUBLIC LROBOT_CONTROL_EXPORT
#else
#define LROBOT_CONTROL_PUBLIC LROBOT_CONTROL_IMPORT
#endif
#define LROBOT_CONTROL_PUBLIC_TYPE LROBOT_CONTROL_PUBLIC
#define LROBOT_CONTROL_LOCAL
#else
#define LROBOT_CONTROL_EXPORT __attribute__((visibility("default")))
#define LROBOT_CONTROL_IMPORT
#if __GNUC__ >= 4
#define LROBOT_CONTROL_PUBLIC __attribute__((visibility("default")))
#define LROBOT_CONTROL_LOCAL __attribute__((visibility("hidden")))
#else
#define LROBOT_CONTROL_PUBLIC
#define LROBOT_CONTROL_LOCAL
#endif
#define LROBOT_CONTROL_PUBLIC_TYPE
#endif
#endif