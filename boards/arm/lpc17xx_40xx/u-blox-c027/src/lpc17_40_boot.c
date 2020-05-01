/****************************************************************************
 * boards/arm/lpc17xx_40xx/u-blox-c027/src/lpc17_40_boot.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
 *   All LPC17xx/LPC40xx architectures must provide the following entry point.
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
  lpc17_40_ubxmdm_init(false);

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
