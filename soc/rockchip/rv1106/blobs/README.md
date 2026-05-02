# RV1106 Firmware Blobs

Two distinct binary blobs are stored here.  They serve completely different
purposes and must not be confused:

---

## MiniLoaderAll.bin  (RKBOOT / `LDR ` magic)

**Source:** `download.bin` from the official Luckfox Pico Plus SDK image
`Luckfox_Pico_Plus_Flash_250429/download.bin`.

**Format:** Rockchip RKBOOT container (`LDR ` magic, verified with
`rockutil PRINT`).  Contains:
- e471 entries: `UsbHead` (RKNS header stub) + `rv1106_ddr_924MHz_v1`
  DDR-init code sent via USB control code 0x471.
- e472 entries: `rv1106_usbplug_v1` USB plug binary sent via USB control
  code 0x472.

**Purpose:** Used exclusively for the **MaskROM → Loader USB handshake**:

```
rockutil UL MiniLoaderAll.bin
```

This puts the device into Loader mode (re-enumerates as PID 0x110D) from
which NAND writes are possible.  **This file is never written to NAND.**

---

## idblock.bin  (RKNS magic)

**Source:** Partition `idblock` extracted from the official Luckfox Pico Plus
factory image `Luckfox_Pico_Plus_Flash_250429/update.img`
(nand_addr=`0x200`, size=188416 bytes).

**Format:** Rockchip NAND boot image (`RKNS` magic).  Contains the DDR-init
code and miniloader that the BootROM loads from SPI NAND during a normal
(non-MaskROM) power-on.  Segment 0 is the UsbHead stub; subsequent segments
are the DDR-init and miniloader body, loaded to SRAM and then DDR.

**Purpose:** Written to **NAND LBA 0x200** by the runner:

```
rockutil WL 0x200 idblock.bin
```

After reboot the BootROM finds this image on NAND, loads it, and execution
proceeds: DDR init → miniloader → loads `zephyr.itb` from LBA 0x400 →
Zephyr starts at 0x00200000.

---

## NAND layout written by `west flash`

| NAND LBA | File           | Purpose                          |
|----------|----------------|----------------------------------|
| `0x200`  | idblock.bin    | DDR-init + miniloader (BootROM)  |
| `0x400`  | zephyr.itb     | Zephyr FIT image (miniloader)    |

All other NAND partitions (boot, rootfs, oem, etc.) are left untouched.

---

## Interim status

Both blobs are prebuilt binaries from the stock Luckfox SDK.  The long-term
goal is to rebuild them from the rkbin repository
(`https://github.com/rockchip-linux/rkbin`) using the `boot_merger` tool and
the appropriate DDR blob (`rv1106_ddr_*.bin`).  A west submanifest for rkbin
is included in `submanifests/rockchip-rkbin.yaml`.
