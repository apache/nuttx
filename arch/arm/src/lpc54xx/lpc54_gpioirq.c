/****************************************************************************
 * arch/arm/src/lpc54/lpc54_gpioirq.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include "up_arch.h"

#include "chip/lpc54_syscon.h"
#include "lpc54_gpio.h"

#ifdef CONFIG_LPC54_GPIOIRQ

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_gpio_irqinitialize
 *
 * Description:
 *   Initialize logic to support interrupting GPIO pins.  This function is
 *   called by the OS inialization logic and is not a user interface.
 *
 ****************************************************************************/

void lpc54_gpio_irqinitialize(void)
{
#ifdef CONFIG_LPC54_GPIOIRQ_GROUPS
  /* Enable the Input Mux, PINT, and GINT modules */

  putreg32(SYSCON_AHBCLKCTRL0_INPUTMUX | SYSCON_AHBCLKCTRL0_PINT |
           SYSCON_AHBCLKCTRL0_GINT, LPC54_SYSCON_AHBCLKCTRLSET0);
#else
  /* Enable the Input Mux and PINT modules */

  putreg32(SYSCON_AHBCLKCTRL0_INPUTMUX | SYSCON_AHBCLKCTRL0_PINT,
           LPC54_SYSCON_AHBCLKCTRLSET0);
#endif

#warning Missing logic
}

/************************************************************************************
 * Name: lpc54_gpio_interrupt
 *
 * Description:
 *   Configure a GPIO interrupt pin based on bit-encoded description of the pin.
 *   This function is called by lpc54_gpio_config to setup interrupting pins.  It is
 *   not a user interface.
 *
 ************************************************************************************/

void lpc54_gpio_interrupt(lpc54_pinset_t pinset)
{
#warning Missing logic
}


#endif /* CONFIG_LPC54_GPIOIRQ */
