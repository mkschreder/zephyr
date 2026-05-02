# Copyright (c) 2026 Martin Schröder <info@swedishembedded.com>
# SPDX-License-Identifier: Apache-2.0

board_runner_args(rockutil
  # RKBOOT-format loader (LDR/BOOT magic): sent over USB via 'rockutil UL'
  # to perform the MaskROM handshake (DDR-init + usbplug).  NOT written to NAND.
  "--loader=${BOARD_DIR}/../../../soc/rockchip/rv1106/blobs/MiniLoaderAll.bin"
  # RKNS-format NAND idblock: written to NAND LBA 0x200 via 'rockutil WL'.
  "--idblock=${BOARD_DIR}/../../../soc/rockchip/rv1106/blobs/idblock.bin"
)

include(${ZEPHYR_BASE}/boards/common/rockutil.board.cmake)
