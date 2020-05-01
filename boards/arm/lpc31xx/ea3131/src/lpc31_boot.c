/****************************************************************************
 * boards/arm/lpc31xx/ea3131/src/lpc31_boot.c
 *
 *   Copyright (C) 2009, 2012, 2015 Gregory Nutt. All rights reserved.
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
#include "lpc31.h"
#include "ea3131.h"

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
 * Name: lpc31_boardinitialize
 *
 * Description:
 *   All LPC31XX architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void lpc31_boardinitialize(void)
{
  /* Initialize configured, external memory resources */

#ifdef CONFIG_LPC31_EXTDRAM
  lpc31_meminitialize();
#endif

  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak
   * function lpc31_spidev_initialize() has been brought into the link.
   */

#if defined(CONFIG_LPC31_SPI)
  if (lpc31_spidev_initialize)
    {
      lpc31_spidev_initialize();
    }
#endif

  /* Initialize USB is 1) USBDEV is selected, 2) the USB controller is not
   * disabled, and 3) the weak function lpc31_usbdev_initialize() has been
   * brought into the build.
   */

#if defined(CONFIG_USBDEV) && defined(CONFIG_LPC31_USBOTG)
  if (lpc31_usbdev_initialize)
    {
      lpc31_usbdev_initialize();
    }
#endif

  /* Initialize USB if the 1) the HS host or device controller is in the
   * configuration and 2) the weak function lpc31_usbhost_bootinitialize() has
   * been brought into the build. Presumably either CONFIG_USBDEV or
   * CONFIG_USBHOST is also selected.
   */

#if defined(CONFIG_SAMA5_UHPHS) || defined(CONFIG_SAMA5_UDPHS)
  if (lpc31_usbhost_bootinitialize)
    {
      lpc31_usbhost_bootinitialize();
    }
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif

  /* Set up mass storage device to support on demand paging */

#if defined(CONFIG_PAGING)
  if (lpc31_pginitialize)
    {
      lpc31_pginitialize();
    }
#endif
}
