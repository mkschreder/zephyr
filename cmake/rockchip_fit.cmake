# Copyright (c) 2026 Martin Schröder <info@swedishembedded.com>
# SPDX-License-Identifier: Apache-2.0
#
# rockchip_fit.cmake
#
# Post-build helper: wraps zephyr.bin into a Rockchip-compatible FIT image
# (zephyr.itb) that the Rockchip miniloader can load and execute.
#
# Usage (from board.cmake or board's CMakeLists.txt):
#
#   include(${ZEPHYR_BASE}/cmake/rockchip_fit.cmake)
#   rockchip_fit_image(
#     LOAD_ADDR  0x00200000
#     ENTRY_ADDR 0x00200000
#   )
#
# The function adds a post-build command on the "zephyr_final" target that:
#   1. Writes zephyr.its next to the build output.
#   2. Runs "mkimage -f zephyr.its -E zephyr.itb".
#
# LOAD_ADDR / ENTRY_ADDR default to 0x00200000 (Rockchip miniloader convention
# for the Luckfox Pico Plus DDR alias window).

function(rockchip_fit_image)
  cmake_parse_arguments(FIT "" "LOAD_ADDR;ENTRY_ADDR" "" ${ARGN})

  if(NOT DEFINED FIT_LOAD_ADDR)
    set(FIT_LOAD_ADDR "0x00200000")
  endif()
  if(NOT DEFINED FIT_ENTRY_ADDR)
    set(FIT_ENTRY_ADDR "0x00200000")
  endif()

  set(fit_its  ${PROJECT_BINARY_DIR}/zephyr/zephyr.its)
  set(fit_itb  ${PROJECT_BINARY_DIR}/zephyr/zephyr.itb)
  set(fit_bin  ${PROJECT_BINARY_DIR}/zephyr/zephyr.bin)

  # Write the ITS template to a file at configure time so the content is
  # visible to mkimage at build time.
  file(WRITE ${fit_its}
"/dts-v1/;\n\
/ {\n\
    description = \"Zephyr FIT for Rockchip miniloader\";\n\
    #address-cells = <1>;\n\
    images {\n\
        zephyr {\n\
            description = \"Zephyr Cortex-A7\";\n\
            data = /incbin/(\"${fit_bin}\");\n\
            type = \"standalone\";\n\
            arch = \"arm\";\n\
            os = \"u-boot\";\n\
            compression = \"none\";\n\
            load = <${FIT_LOAD_ADDR}>;\n\
            entry = <${FIT_ENTRY_ADDR}>;\n\
            hash-1 { algo = \"sha256\"; };\n\
        };\n\
    };\n\
    configurations {\n\
        default = \"conf\";\n\
        conf {\n\
            description = \"Zephyr only\";\n\
            loadables = \"zephyr\";\n\
        };\n\
    };\n\
};\n"
  )

  find_program(MKIMAGE mkimage)
  if(NOT MKIMAGE)
    message(WARNING "mkimage not found; zephyr.itb will not be generated. "
                    "Install u-boot-tools (apt install u-boot-tools).")
    return()
  endif()

  add_custom_command(
    TARGET zephyr_final POST_BUILD
    COMMAND ${MKIMAGE} -f ${fit_its} -E ${fit_itb}
    COMMENT "Generating Rockchip FIT image: zephyr.itb"
    VERBATIM
  )

  message(STATUS "rockchip_fit: will generate ${fit_itb} after zephyr_final")
endfunction()
