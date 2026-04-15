# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(armv7m_branch_it)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common common)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
