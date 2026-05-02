# Copyright (c) 2026 Martin Schröder <info@swedishembedded.com>
# SPDX-License-Identifier: Apache-2.0

# Post-build: generate zephyr.itb (FIT for NAND/USB flash) and
#             sdcard.img (raw SD card image for recovery).
#
# Rockchip vendor SPL (U-Boot 2017.09) FIT layout requirements
# ─────────────────────────────────────────────────────────────
# The vendor SPL expects the FIT to mirror the stock uboot.img structure:
#
#   images/uboot   – standalone, os = "U-Boot", load = 0x00200000
#   images/fdt     – flat_dt stub (SPL requires this to complete boot)
#
#   configurations/conf – loadables = "uboot"; fdt = "fdt"
#
# Without the fdt image in the configuration the SPL considers the FIT boot
# attempt incomplete and falls through to the next boot device even after
# successfully verifying the main image.
#
# SD card image layout (sdcard.img)
# ─────────────────────────────────
# Derived from the official Luckfox MicroSD sd_update.txt mmc-write commands:
#
#   LBA 0x0000 (    0 KB) – env-sd.img   (32K,   0x40 sectors) – ENVF partition
#   LBA 0x0040 (   32 KB) – idblock-sd   (184K,  0x170 sectors) – DDR+SPL
#   LBA 0x0440 (  544 KB) – zephyr.itb   (≤256K, 0x200 sectors) – FIT image
#
# The SPL reads the ENVF partition at LBA 0x0 to get the partition table, then
# uses it to locate the uboot/FIT image at LBA 0x440.  Without a valid ENVF the
# SPL aborts with "partition error" and falls back to NAND.
#
# Recovery boot: with a valid SD card inserted, the SPL tries MMC2 (SD) BEFORE
# MTD1 (NAND), so inserting this card will load Zephyr from SD even if NAND is
# broken.
#
# Flash to SD card:
#   sudo dd if=build/.../zephyr/sdcard.img of=/dev/sdX bs=4M conv=fsync
#   (replace /dev/sdX with your actual SD device node)

find_program(MKIMAGE mkimage)
find_program(DTC dtc)

if(MKIMAGE AND DTC)
  # ── paths ──────────────────────────────────────────────────────────────
  set(fit_bin    ${CMAKE_BINARY_DIR}/zephyr/${CONFIG_KERNEL_BIN_NAME}.bin)
  set(fit_dts    ${CMAKE_BINARY_DIR}/zephyr/rv1106-fdt.dts)
  set(fit_dtb    ${CMAKE_BINARY_DIR}/zephyr/rv1106-fdt.dtb)
  set(fit_its    ${CMAKE_BINARY_DIR}/zephyr/zephyr.its)
  set(fit_itb    ${CMAKE_BINARY_DIR}/zephyr/zephyr.itb)
  set(sd_img     ${CMAKE_BINARY_DIR}/zephyr/sdcard.img)
  set(sd_env     ${BOARD_DIR}/../../../soc/rockchip/rv1106/blobs/env-sd.img)
  set(sd_idblock ${BOARD_DIR}/../../../soc/rockchip/rv1106/blobs/idblock-sd.img)

  # ── minimal FDT stub ───────────────────────────────────────────────────
  # SPL loads & hashes this; Zephyr itself ignores it at runtime.
  file(GENERATE OUTPUT ${fit_dts} CONTENT
"/dts-v1/;
/ {
    compatible = \"rockchip,rv1106\";
    model = \"Luckfox Pico Plus (Zephyr)\";
    #address-cells = <1>;
    #size-cells = <1>;
    memory@200000 {
        device_type = \"memory\";
        reg = <0x00200000 0x04000000>;
    };
};
")

  # ── FIT image source ───────────────────────────────────────────────────
  file(GENERATE OUTPUT ${fit_its} CONTENT
"/dts-v1/;
/ {
    description = \"Zephyr FIT for Rockchip miniloader\";
    #address-cells = <1>;
    images {
        uboot {
            description = \"Zephyr (Cortex-A7)\";
            data = /incbin/(\"${fit_bin}\");
            type = \"standalone\";
            arch = \"arm\";
            os = \"U-Boot\";
            compression = \"none\";
            load = <0x00200000>;
            hash-1 { algo = \"sha256\"; };
        };
        fdt {
            description = \"Minimal stub FDT (required by vendor SPL)\";
            data = /incbin/(\"${fit_dtb}\");
            type = \"flat_dt\";
            arch = \"arm\";
            compression = \"none\";
            hash-1 { algo = \"sha256\"; };
        };
    };
    configurations {
        default = \"conf\";
        conf {
            description = \"Zephyr for RV1106\";
            loadables = \"uboot\";
            fdt = \"fdt\";
        };
    };
};
")

  # ── post-build commands ────────────────────────────────────────────────
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    # 1. Compile the minimal FDT stub
    COMMAND ${DTC} -O dtb -o ${fit_dtb} ${fit_dts}
    # 2. Pack the flat FIT (no -E: data embedded inline for 2017.09 SPL)
    COMMAND ${MKIMAGE} -f ${fit_its} ${fit_itb}
    # 3. Build the raw SD card image (4 MB, all zeros)
    COMMAND dd if=/dev/zero of=${sd_img} bs=512 count=8192 status=none
    # 4. env at LBA 0x000 (    0 KB) — ENVF partition table (SPL reads this first)
    COMMAND dd if=${sd_env} of=${sd_img} bs=512 seek=0 conv=notrunc status=none
    # 5. idblock at LBA 0x040 (   32 KB) — DDR-init + SPL (BootROM SD search offset)
    COMMAND dd if=${sd_idblock} of=${sd_img} bs=512 seek=64 conv=notrunc status=none
    # 6. FIT at LBA 0x440 (  544 KB) — next-stage image (per sd_update.txt mmc write)
    COMMAND dd if=${fit_itb} of=${sd_img} bs=512 seek=1088 conv=notrunc status=none
  )

  message(STATUS
    "rockchip_fit: post-build will produce zephyr.itb and sdcard.img")

elseif(NOT MKIMAGE)
  message(WARNING
    "mkimage not found – install u-boot-tools (apt install u-boot-tools); "
    "zephyr.itb and sdcard.img will NOT be generated")
elseif(NOT DTC)
  message(WARNING
    "dtc not found – install device-tree-compiler; "
    "zephyr.itb and sdcard.img will NOT be generated")
endif()
