# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=IN6XXE" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)

