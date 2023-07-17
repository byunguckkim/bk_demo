// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt
#pragma once

// TODO split this into smaller pieces, it is turning into a black hole

#include <sstream>

// Make Linux-ish code work on Windows

// usleep

#ifdef __linux__

#include <unistd.h>

#else
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#undef RGB
#undef ERROR

typedef unsigned int useconds_t;

inline int usleep(useconds_t useconds) {
  Sleep(useconds / 1000);
  return 0;
}

#endif

// err, errx

#ifdef __linux__

#include <err.h>
#include <cstdlib>

#else

#define EXIT_FAILURE 1

#define errx(exit_code, fmt, ...) \
  while (true) {                  \
    printf((fmt), (__VA_ARGS__)); \
    exit((exit_code));            \
  }

#define err(exit_code, fmt)                                \
  while (true) {                                           \
    printf("%s (no strerror plumbing on Windows)", (fmt)); \
    exit((exit_code));                                     \
  }

#endif
