/****************************************************************************
 * boards/arm/stm32l5/stm32l562e-dk/src/stm32_boot.c
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

#include <debug.h>

#include <nuttx/board.h>

#include "arm_internal.h"
#include "stm32l562e-dk.h"
#include "stm32l5_pwr.h"

#include <arch/board/board.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l5_board_initialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after all memory
 *   has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32l5_board_initialize(void)
{
  /* On the STM32L562E-DK Vddio2 is supplied by Vdd_mcu.  Thus, when the MCU
   * is running Vddio2 is guaranteed to be valid.  LED LD10 is driven by
   * PG12, whose power is supplied by Vddio2.  Thus, its important to report
   * Vddio2 to be valid here.
   */

  stm32l5_pwr_vddio2_valid(true);

#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  board_autoled_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called immediately after up_initialize() is called and just before
 *   the initial application is started.  This additional initialization
 *   phase may be used, for example, to initialize board-specific device
 *   drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform board-specific initialization here if so configured */

  stm32_bringup();
}
#endif
