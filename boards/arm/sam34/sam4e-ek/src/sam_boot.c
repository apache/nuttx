/****************************************************************************
 * boards/arm/sam34/sam4e-ek/src/sam_boot.c
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
#include "sam4e-ek.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_config_usart1
 *
 * Description:
 *   USART1: To avoid any electrical conflict, the RS232 and RS485
 *   transceiver are isolated from the receiving line PA21.
 *
 *   - Chose RS485 channel: Close 1-2 pins on JP11 and set PA23 to high level
 *   - Chose RS232 channel: Close 2-3 pins on JP11 and set PA23 to low level
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_USART1
static inline void board_config_usart1(void)
{
#if defined(CONFIG_USART1_RS485)
  sam_configgpio(GPIO_RS485_ENABLE);
#else /* if defined(CONFIG_USART1_SERIALDRIVER) */
  sam_configgpio(GPIO_RS232_ENABLE);
#endif
}
#else
#  define board_config_usart1()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_boardinitialize
 *
 * Description:
 *   All SAM3U architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void sam_boardinitialize(void)
{
  /* Configure USART1 for RS-232/RS-485 operation */

  board_config_usart1();

  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak
   * function sam_spidev_initialize() has been brought into the link.
   */

#ifdef CONFIG_SAM34_SPI0
  if (sam_spidev_initialize)
    {
      sam_spidev_initialize();
    }
#endif

  /* Configure board resources to support networkingif the 1) networking is
   * enabled, 2) the EMAC module is enabled, and 2) the weak function
   * sam_netinitialize() has been brought into the build.
   */

#ifdef HAVE_NETWORK
  if (sam_netinitialize)
    {
      sam_netinitialize();
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
  /* Perform NSH initialization here instead of from the NSH.
   * This alternative NSH initialization is necessary when NSH is ran in
   * user-space but the initialization function must run in kernel space.
   */

#if defined(CONFIG_NSH_LIBRARY) && !defined(CONFIG_BOARDCTL)
  board_app_initialize(0);
#endif
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
