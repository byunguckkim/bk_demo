#pragma once

// clang-format off
#if defined _WIN32 || defined __CYGWIN__
  #ifdef BUILDING_DLL
    #ifdef __GNUC__
      #define APPLIED_INTERNAL_API __attribute__((dllexport))
    #else
      // Note: actually gcc seems to also supports this syntax.
      #define APPLIED_INTERNAL_API __declspec(dllexport)
    #endif
  #else
    #ifdef __GNUC__
      #define APPLIED_INTERNAL_API __attribute__((dllimport))
    #else
      // Note: actually gcc seems to also supports this syntax.
      #define APPLIED_INTERNAL_API __declspec(dllimport)
    #endif
  #endif
#else
  #if __GNUC__ >= 4
    #define APPLIED_INTERNAL_API __attribute__((visibility("default")))
  #else
    #error "Applied Intuition requires a GCC version newer than 4"
  #endif
#endif

#define APPLIED_PUBLIC_API APPLIED_INTERNAL_API

// clang-format on
