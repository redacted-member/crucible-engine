/* SPDX-License-Identifier: MIT */
/* CRUCIBLE_API: the export decoration shared by crucible.h and crucible_module.h. */
#ifndef CRUCIBLE_EXPORT_H
#define CRUCIBLE_EXPORT_H

/* Symbol export. CRUCIBLE_API marks every function below. Linking the static library: define CRUCIBLE_STATIC (the CMake
 * target crucible_engine does it for you). Building a shared library from the engine sources: define CRUCIBLE_BUILD_SHARED
 * (functions are exported: dllexport on Windows, default visibility elsewhere). Using a shared library: define nothing
 * (dllimport on Windows). */
#if defined(CRUCIBLE_STATIC)
#define CRUCIBLE_API
#elif defined(_WIN32) || defined(__CYGWIN__)
#if defined(CRUCIBLE_BUILD_SHARED)
#define CRUCIBLE_API __declspec(dllexport)
#else
#define CRUCIBLE_API __declspec(dllimport)
#endif
#elif defined(__GNUC__) || defined(__clang__)
#define CRUCIBLE_API __attribute__((visibility("default")))
#else
#define CRUCIBLE_API
#endif

#endif
