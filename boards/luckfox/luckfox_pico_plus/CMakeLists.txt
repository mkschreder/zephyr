# Copyright (c) 2026 Martin Schröder <info@swedishembedded.com>
# SPDX-License-Identifier: Apache-2.0

# Generate the Rockchip FIT image (zephyr.itb) as a post-build step.
# The Rockchip miniloader loads the FIT image from NAND LBA 0x400
# and jumps to the entry point at 0x00200000 (DDR alias).

find_program(MKIMAGE mkimage)

if(MKIMAGE)
  set(fit_bin  ${CMAKE_BINARY_DIR}/zephyr/${CONFIG_KERNEL_BIN_NAME}.bin)
  set(fit_its  ${CMAKE_BINARY_DIR}/zephyr/zephyr.its)
  set(fit_itb  ${CMAKE_BINARY_DIR}/zephyr/zephyr.itb)

  file(GENERATE OUTPUT ${fit_its} CONTENT
"/dts-v1/;
/ {
    description = \"Zephyr FIT for Rockchip miniloader\";
    #address-cells = <1>;
    images {
        zephyr {
            description = \"Zephyr Cortex-A7\";
            data = /incbin/(\"${fit_bin}\");
            type = \"standalone\";
            arch = \"arm\";
            os = \"u-boot\";
            compression = \"none\";
            load = <0x00200000>;
            entry = <0x00200000>;
            hash-1 { algo = \"sha256\"; };
        };
    };
    configurations {
        default = \"conf\";
        conf {
            description = \"Zephyr only\";
            loadables = \"zephyr\";
        };
    };
};
")

  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND ${MKIMAGE} -f ${fit_its} -E ${fit_itb}
  )
  message(STATUS "rockchip_fit: will generate zephyr.itb after build (mkimage=${MKIMAGE})")
else()
  message(WARNING "mkimage not found; zephyr.itb will not be generated. "
                  "Install u-boot-tools: apt install u-boot-tools")
endif()
