/************************************************************************************
 * configs/sama5d3-xplained/src/sam_boot.c
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/board.h>

#include "sam_sckc.h"
#include "sama5d3-xplained.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_boardinitialize
 *
 * Description:
 *   All SAMA5 architectures must provide the following entry point.  This entry
 *   point is called early in the intitialization -- after all memory has been
 *   configured and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void sam_boardinitialize(void)
{
#ifdef CONFIG_SAMA5D3XPLAINED_SLOWCLOCK
  /* Enable the external slow clock */

  sam_sckc_enable(true);
#endif

  /* Configure SPI chip selects if 1) SPI is enable, and 2) the weak function
   * sam_spiinitialize() has been brought into the link.
   */

#if defined(CONFIG_SAMA5_SPI0) || defined(CONFIG_SAMA5_SPI1)
  if (sam_spiinitialize)
    {
      sam_spiinitialize();
    }
#endif

#if defined(CONFIG_SAMA5_DDRCS) && !defined(CONFIG_SAMA5_BOOT_SDRAM)

  /* Configure SDRAM if (1) SDRAM has been enalbled in the NuttX configuration and
   * (2) if we are not currently running out of SDRAM.  If we are now running out
   * of SDRAM then we have to assume that some second level bootloader has properly
   * configured SDRAM for our use.
   */

  sam_sdram_config();

#endif

  /* Initialize USB if the 1) the HS host or device controller is in the
   * configuration and 2) the weak function sam_usbinitialize() has been brought
   * into the build. Presumeably either CONFIG_USBDEV or CONFIG_USBHOST is also
   * selected.
   */

#if defined(CONFIG_SAMA5_UHPHS) || defined(CONFIG_SAMA5_UDPHS)
  if (sam_usbinitialize)
    {
      sam_usbinitialize();
    }
#endif

  /* Configure board resources to support networkingif the 1) networking is enabled,
   * 2) the EMAC or GMAC module is enabled, and 2) the weak function
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

  board_led_initialize();
#endif
}

/****************************************************************************
 * Name: board_initialize
 *
 * Description:
 *   If CONFIG_BOARD_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_initialize().  board_initialize() will be
 *   called immediately after up_intiialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_INITIALIZE
void board_initialize(void)
{
  /* Perform NSH initialization here instead of from the NSH.  This
   * alternative NSH initialization is necessary when NSH is ran in user-space
   * but the initialization function must run in kernel space.
   */

#if defined(CONFIG_NSH_LIBRARY) && !defined(CONFIG_NSH_ARCHINIT)
  (void)board_app_initialize();
#endif
}
#endif /* CONFIG_BOARD_INITIALIZE */
