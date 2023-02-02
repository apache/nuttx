# Copyright (c) 2021 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

sdk_compile_definitions_ifdef(BOARD_LCD_WIDTH "-DBOARD_LCD_WIDTH=${BOARD_LCD_WIDTH}")
sdk_compile_definitions_ifdef(BOARD_LCD_HEIGHT "-DBOARD_LCD_HEIGHT=${BOARD_LCD_HEIGHT}")
sdk_compile_definitions_ifdef(BUILD_FOR_SECONDARY_CORE BOARD_RUNNING_CORE=HPM_CORE1)

sdk_inc(.)
sdk_src(pinmux.c)
sdk_src(board.c)
