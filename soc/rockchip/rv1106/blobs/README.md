# RV1106 Firmware Blobs

## idblock.bin

**Source:** Extracted from the official Luckfox Pico Plus factory image
`Luckfox_Pico_Plus_Flash_250429/update.img`, partition `idblock`
(RKAF nand\_addr=`0x200`, size=188416 bytes).

**Purpose:** The Rockchip BootROM loads this blob from SPI NAND LBA `0x200`.
It contains the official Rockchip DDR-init code and miniloader (SPL).
The miniloader initialises LPDDR4, then loads the FIT image from NAND LBA
`0x400` (the `uboot` partition slot) and jumps to `0x00200000`.

**Interim status:** This is a prebuilt binary captured from the stock firmware.
The long-term goal is to rebuild it from the rkbin repository
(`https://github.com/rockchip-linux/rkbin`) using the `boot_merger` tool and
the appropriate DDR blob (`rv1106_ddr_*.bin`), which removes the dependency on
this prebuilt blob.  A west submanifest for rkbin is included in
`applications/zephyr/submanifests/rockchip-rkbin.yaml` to enable that future
workflow.

**NAND layout for `west flash`:**

| NAND LBA | Contents          |
|----------|-------------------|
| `0x200`  | idblock.bin       |
| `0x400`  | zephyr.itb (FIT)  |

The runner writes both blobs in MaskROM → Loader mode via `rockutil`.
All other NAND partitions (boot, rootfs, oem, etc.) are left untouched.
