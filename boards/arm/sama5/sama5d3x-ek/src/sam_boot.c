/****************************************************************************
 * boards/arm/sama5/sama5d3x-ek/src/sam_boot.c
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

#include "sam_sckc.h"
#include "sama5d3x-ek.h"

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
 * Name: sam_boardinitialize
 *
 * Description:
 *   All SAMA5 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void sam_boardinitialize(void)
{
#ifdef CONFIG_SAMA5D3XEK_SLOWCLOCK
  /* Enable the external slow clock */

  sam_sckc_enable(true);
#endif

  /* Configure SPI chip selects if 1) SPI is enable, and 2) the weak function
   * sam_spidev_initialize() has been brought into the link.
   */

#if defined(CONFIG_SAMA5_SPI0) || defined(CONFIG_SAMA5_SPI1)
  if (sam_spidev_initialize)
    {
      sam_spidev_initialize();
    }
#endif

#if defined(CONFIG_SAMA5_DDRCS) && !defined(CONFIG_SAMA5_BOOT_SDRAM)

  /* Configure SDRAM if
   * (1) SDRAM has been enabled in the NuttX configuration and
   * (2) if we are not currently running out of SDRAM.
   * If we are now running out of SDRAM then we have to assume that some
   * second level bootloader has properly configured SDRAM for our use.
   */

  sam_sdram_config();

#endif

  /* Initialize USB if the
   * 1) the HS host or device controller is in the configuration and
   * 2) the weak function sam_usbinitialize() has been brought
   * into the build.
   * Presumeably either CONFIG_USBDEV or CONFIG_USBHOST is also selected.
   */

#if defined(CONFIG_SAMA5_UHPHS) || defined(CONFIG_SAMA5_UDPHS)
  if (sam_usbinitialize)
    {
      sam_usbinitialize();
    }
#endif

  /* Configure board resources to support networking if the
   * 1) networking is enabled,
   * 2) the EMAC or GMAC module is enabled, and
   * 2) the weak function
   * sam_netinitialize() has been brought into the build.
   */

#ifdef HAVE_NETWORK
  if (sam_netinitialize)
    {
      sam_netinitialize();
    }
#endif

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
