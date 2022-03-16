/****************************************************************************
 * boards/arm/lpc17xx_40xx/lpc4088-devkit/src/lpc17_40_boardinitialize.c
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
#include <arch/board/board.h>

#include "arm_internal.h"
#include "lpc17_40_emc.h"

#include "lpc4088-devkit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_boardinitialize
 *
 * Description:
 *   All LPC17xx/LPC40xx architectures must provide the following entry
 *   point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void lpc17_40_boardinitialize(void)
{
  /* Initialize the EMC, SDRAM, NOR FLASH, and NAND FLASH */

#ifdef CONFIG_LPC17_40_EMC
  lpc17_40_emcinitialize();

#ifdef CONFIG_LPC17_40_EXTDRAM
  lpc4088_devkit_sdram_initialize();
#endif

#ifdef CONFIG_LPC17_40_EXTNOR
  lpc4088_devkit_nor_initialize();
#endif

#ifdef CONFIG_LPC17_40_EXTNAND
  lpc4088_devkit_nand_initialize();
#endif
#endif

#if defined(CONFIG_LPC17_40_SSP0) || defined(CONFIG_LPC17_40_SSP1) || \
    defined(CONFIG_LPC17_40_SSP2)
  /* Configure SSP chip selects if 1) at least one SSP is enabled, and 2)
   * the weak function lpc4088_devkit_sspdev_initialize() has been brought
   * into the link.
   */

  if (lpc4088_devkit_sspdev_initialize)
    {
      lpc4088_devkit_sspdev_initialize();
    }
#endif

#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  board_autoled_initialize();
#endif

#ifdef CONFIG_LPC17_40_LCD
  /* Configure the LCD GPIOs if LCD support has been selected. */

  lpc4088_devkit_lcd_initialize();
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

  lpc4088_devkit_bringup();
}
#endif
