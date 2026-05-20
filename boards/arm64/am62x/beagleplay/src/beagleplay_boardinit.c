/****************************************************************************
 * boards/arm64/am62x/beagleplay/src/beagleplay_boardinit.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <nuttx/board.h>
#include "beagleplay.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am62x_memory_initialize
 *
 * Description:
 *   Called from the very early boot path (before .bss is zeroed or .data
 *   is initialised) to perform any board-level memory bring-up such as
 *   DRAM training.
 *
 *   On BeaglePlay, U-Boot trains and initialises the 512 MB DDR4
 *   before loading NuttX, so there is nothing left to do here.
 *
 *   IMPORTANT: This function must not use any global variables because
 *   they are not yet initialised when it is called.
 *
 ****************************************************************************/

void am62x_memory_initialize(void)
{
  /* DRAM has been initialised by U-Boot / SYSFW before NuttX is loaded. */
}

/****************************************************************************
 * Name: am62x_board_initialize
 *
 * Description:
 *   Called from am62x_boot.c (arm64_chip_boot) after the MMU is enabled
 *   and the C runtime is ready, but before any device drivers are
 *   registered.  Use this hook to configure on-board resources that must
 *   be ready before up_initialize() runs.
 *
 ****************************************************************************/

void am62x_board_initialize(void)
{
#ifdef CONFIG_ARCH_LEDS
  /* Initialise the user LED GPIO outputs.  LEDs are left OFF. */

  beagleplay_led_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   When CONFIG_BOARD_LATE_INITIALIZE is selected this function is called
 *   immediately after up_initialize() completes and just before the initial
 *   application task is created.  All device drivers are available at this
 *   point so we use it to bring up higher-level board subsystems (procfs,
 *   I2C bus registration, etc.).
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  beagleplay_bringup();
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
