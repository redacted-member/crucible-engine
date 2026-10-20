# SPDX-License-Identifier: MIT
# A minimal project that uses an installed Crucible Engine through find_package(). tools/test-consumer.sh installs the
# engine into a scratch prefix and builds this project against it.
cmake_minimum_required(VERSION 3.22)
project(CrucibleConsumer LANGUAGES CXX)

find_package(CrucibleEngine 0.1 REQUIRED)

add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE crucible::engine)
