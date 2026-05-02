# Copyright (c) 2026 Martin Schröder <info@swedishembedded.com>
# SPDX-License-Identifier: Apache-2.0

board_runner_args(rockutil
  "--idblock=${BOARD_DIR}/../../../soc/rockchip/rv1106/blobs/idblock.bin"
)

include(${ZEPHYR_BASE}/boards/common/rockutil.board.cmake)
