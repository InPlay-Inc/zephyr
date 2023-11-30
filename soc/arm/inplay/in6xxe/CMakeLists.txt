# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
  soc.c
  )

zephyr_library_include_directories(
  ${ZEPHYR_BASE}/kernel/include
  ${ZEPHYR_BASE}/arch/arm/include
  ${ZEPHYR_BASE}/soc/arm/common/cortex_m
  )

set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")
