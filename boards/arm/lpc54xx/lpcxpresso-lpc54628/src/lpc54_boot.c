/****************************************************************************
 * boards/arm/lpc54xx/lpcxpresso-lpc54628/src/lpc54_boot.c
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

#include <nuttx/board.h>

#include "lpcxpresso-lpc54628.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_board_initialize
 *
 * Description:
 *   All LPC54xx architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after
 *   clocking and memory have been configured but before caches have been
 *   enabled and before any devices have been initialized.
 *
 ****************************************************************************/

void lpc54_board_initialize(void)
{
#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  board_autoled_initialize();
#endif

  /* Configure the LCD GPIOs if LCD support has been selected. This is done
   * unconditionally because even if the LCD is not configured, we will
   * still want to turn the backlight off.
   */

  lpc54_lcd_initialize();

#ifdef CONFIG_LPC54_EMC
  /* Initialize SDRAM */

  lpc54_sdram_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize(). board_late_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform board-specific initialization */

  lpc54_bringup();
}
#endif
