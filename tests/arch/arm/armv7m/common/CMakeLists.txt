# SPDX-License-Identifier: Apache-2.0
# Add common harness sources to the parent 'app' target so they compile
# after Zephyr's generated headers (offsets.h) are available.

target_sources(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/armv7m_test.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/armv7m_test_asm.S
)
target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
