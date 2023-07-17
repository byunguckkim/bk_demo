#pragma once

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-warning-option"
#pragma clang diagnostic ignored "-Wshadow"
#pragma clang diagnostic ignored "-Wmisleading-indentation"
#pragma clang diagnostic ignored "-Wint-in-bool-context"
#pragma clang diagnostic ignored "-Wrange-loop-construct"
#pragma clang diagnostic ignored "-Wdeprecated-copy"
#endif

#if defined(_MSC_VER)
// C5054
//   operator '*': deprecated between enumerations of different types
//   operator '+': deprecated between enumerations of different types
#pragma warning(push)
#pragma warning(disable : 5054)
#endif  // defined(_MSC_VER)

#include "Eigen/Core"
#include "Eigen/Geometry"

#if defined(_MSC_VER)
#pragma warning(pop)
#endif  // defined(_MSC_VER)

#ifdef __clang__
#pragma clang diagnostic pop
#endif
