#!/bin/sh
# SPDX-License-Identifier: MIT
# Installs the engine from a build tree into a scratch prefix, then configures, builds and runs examples/consumer
# against it with find_package(CrucibleEngine). Usage: tools/test-consumer.sh <build-dir> [scratch-dir]
set -eu
build=${1:?usage: test-consumer.sh <build-dir> [scratch-dir]}
here=$(cd "$(dirname "$0")/.." && pwd)
scratch=${2:-$(mktemp -d)}
build=$(cd "$build" && pwd)
rm -rf "$scratch/prefix" "$scratch/consumer"
cmake --install "$build" --prefix "$scratch/prefix" > /dev/null
# A sanitized engine build needs the consumer linked with the same runtime: CRUCIBLE_ENGINE_SANITIZE=address;undefined
# (or thread) is read from the build tree's cache and turned into flags for the consumer.
sanitize=$(sed -n 's/^CRUCIBLE_ENGINE_SANITIZE:[A-Z]*=//p' "$build/CMakeCache.txt")
san_flags=""
[ -n "$sanitize" ] && san_flags="-fsanitize=$(echo "$sanitize" | tr ';' ',') -fno-omit-frame-pointer"
cmake -S "$here/examples/consumer" -B "$scratch/consumer" -DCMAKE_PREFIX_PATH="$scratch/prefix" \
    -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}" \
    "-DCMAKE_CXX_FLAGS=$san_flags" "-DCMAKE_EXE_LINKER_FLAGS=$san_flags" > /dev/null
cmake --build "$scratch/consumer" --config "${CMAKE_BUILD_TYPE:-Release}"
version=$(sed -n 's/.*CRUCIBLE_VERSION_STRING "\(.*\)".*/\1/p' "$scratch/prefix/include/crucible/Version.h")
exe=$(find "$scratch/consumer" -name consumer -type f -o -name consumer.exe -type f | head -n 1)
"$exe" "$version"
echo "consumer ok (installed to $scratch/prefix)"
