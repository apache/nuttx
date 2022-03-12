/****************************************************************************
 * boards/arm/lpc17xx_40xx/u-blox-c027/src/lpc17_40_boot.c
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

#include "arm_arch.h"
#include "arm_internal.h"

#include "lpc17_40_gpio.h"
#include "lpc17_40_ssp.h"
#include "u-blox-c027.h"

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
  /* Configure SSP chip selects if 1) at least one SSP is enabled, and 2)
   * the weak function c027_sspdev_initialize() has been brought into the
   * link.
   */

#if defined(CONFIG_LPC17_40_SSP0) || defined(CONFIG_LPC17_40_SSP1)
  if (c027_sspdev_initialize)
    {
      c027_sspdev_initialize();
    }
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
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
#ifdef CONFIG_MODEM_U_BLOX
  lpc17_40_ubxmdm_init(false);
#endif

#if 0
  lpc17_40_configgpio(C027_MDMEN     | GPIO_VALUE_ZERO); /* Modem disabled */
  lpc17_40_configgpio(C027_MDMRST    | GPIO_VALUE_ONE);  /* Modem reset on */
  lpc17_40_configgpio(C027_MDMPWR    | GPIO_VALUE_ONE);  /* Modem power off */
  lpc17_40_configgpio(C027_GPSEN     | GPIO_VALUE_ZERO); /* GPS disabled */
  lpc17_40_configgpio(C027_GPSRST    | GPIO_VALUE_ONE);  /* GPS reset on */
  lpc17_40_configgpio(C027_MDMLVLOE  | GPIO_VALUE_ONE);  /* UART shifter disabled */
  lpc17_40_configgpio(C027_MDMILVLOE | GPIO_VALUE_ZERO); /* I2C shifter disabled */
  lpc17_40_configgpio(C027_MDMUSBDET | GPIO_VALUE_ZERO); /* USB sense off */
#endif
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
