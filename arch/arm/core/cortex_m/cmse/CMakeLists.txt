# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(arm_core_cmse.c)

# Build-time check: no __attribute__((cmse_nonsecure_entry)) function may
# use variadic arguments.  ARM DDI 0553B §C1.4.5.
if(CONFIG_ARM_SECURE_FIRMWARE)
  # Collect all C/H files in the application and Zephyr arch directories.
  # We build the file list at configure time; a full recursive scan is
  # acceptable since this executes only once per CMake configure invocation.
  file(GLOB_RECURSE _cmse_check_sources
    "${ZEPHYR_BASE}/arch/arm/*.c"
    "${ZEPHYR_BASE}/arch/arm/*.h"
    "${APPLICATION_SOURCE_DIR}/*.c"
    "${APPLICATION_SOURCE_DIR}/*.h"
  )

  # Write paths to a response file to avoid command-line length limits.
  set(_cmse_rsp "${CMAKE_BINARY_DIR}/cmse_check_sources.rsp")
  file(WRITE "${_cmse_rsp}" "")
  foreach(f ${_cmse_check_sources})
    file(APPEND "${_cmse_rsp}" "${f}\n")
  endforeach()

  add_custom_target(check_cmse_entry_no_variadic ALL
    COMMAND ${PYTHON_EXECUTABLE}
            ${ZEPHYR_BASE}/scripts/check_cmse_entry_no_variadic.py
            @${_cmse_rsp}
    COMMENT "Checking for variadic cmse_nonsecure_entry functions (DDI 0553B §C1.4.5)"
    VERBATIM
  )
endif()
