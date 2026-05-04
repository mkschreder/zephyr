# Copyright (c) 2026 Martin Schröder <info@swedishembedded.com>
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

zephyr_compile_definitions_ifdef(CONFIG_SOC_VEMU_CORTEX_M33 SOC_VEMU_CORTEX_M33)

zephyr_include_directories(.)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
