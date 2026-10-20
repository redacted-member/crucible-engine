# SPDX-License-Identifier: MIT
# Standalone build of the fuzz / soak harness (not part of the engine's own build):
#   cmake -S tools/fuzz -B build-fuzz -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo "-DCRUCIBLE_ENGINE_SANITIZE=address;undefined"
#   cmake --build build-fuzz
#   build-fuzz/crucible-fuzz --root . --seeds 1:100 --stats
# The engine is consumed with add_subdirectory, so the harness sees only what an embedding host sees. The sanitizer
# flags are applied here too (the engine's CMakeLists applies them to its own directory only).
cmake_minimum_required(VERSION 3.22)
project(CrucibleFuzz LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CRUCIBLE_ENGINE_SANITIZE "" CACHE STRING "Sanitizers to enable: address, undefined, thread (semicolon separated)")
if(CRUCIBLE_ENGINE_SANITIZE)
    string(REPLACE ";" "," CRUCIBLE_FUZZ_SANITIZE_FLAGS "${CRUCIBLE_ENGINE_SANITIZE}")
    add_compile_options(-fsanitize=${CRUCIBLE_FUZZ_SANITIZE_FLAGS} -fno-omit-frame-pointer -fno-sanitize-recover=all)
    add_link_options(-fsanitize=${CRUCIBLE_FUZZ_SANITIZE_FLAGS})
endif()

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../.." crucible-engine EXCLUDE_FROM_ALL)

add_executable(crucible-fuzz crucible_fuzz.cpp)
target_link_libraries(crucible-fuzz PRIVATE crucible::engine)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # The replaced operator new / delete (allocation counting without AddressSanitizer) pair malloc with free on purpose.
    target_compile_options(crucible-fuzz PRIVATE -Wall -Wextra -Wno-mismatched-new-delete)
endif()
