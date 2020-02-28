/****************************************************************************
 * boards/arm/lpc54xx/lpcxpresso-lpc54628/src/lpc54_autoleds.c
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

/* The LPCXpress-LPC54628 has three user LEDs: D9, D11, and D12.  These
 * LEDs are for application use. They are illuminated when the driving
 * signal from the LPC546xx is low. The LEDs are driven by ports P2-2 (D9),
 * P3-3 (D11) and P3-14 (D12).
 *
 * These LEDs are not used by the NuttX port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/lpc54_autoleds.c.  The LEDs are used to encode
 * OS-related events as follows:
 *                                  D9     D11    D12
 * LED_STARTED                0     OFF    OFF    OFF
 * LED_HEAPALLOCATE           1     ON     OFF    OFF
 * LED_IRQSENABLED            2     OFF    ON     OFF
 * LED_STACKCREATED           3     OFF    OFF    OFF
 *
 * LED_INIRQ                  4     NC     NC     ON  (momentary)
 * LED_SIGNAL                 4     NC     NC     ON  (momentary)
 * LED_ASSERTION              4     NC     NC     ON  (momentary)
 * LED_PANIC                  4     NC     NC     ON  (2Hz flashing)
 * LED_IDLE                         Sleep mode indication not supported
 *
 * After booting, LEDs D9 and D11 are available for use by the user.  If the
 * system booted properly, D9 and D11 should be OFF and D12 should be glowing
 * to indicate that interrupts are occurring.  If D12 is flash at 2Hz, then
 * the system has crashed.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "lpc54_gpio.h"
#include "lpcxpresso-lpc54628.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED GPIOs for output */

  lpc54_gpio_config(GPIO_LED_D9);
  lpc54_gpio_config(GPIO_LED_D11);
  lpc54_gpio_config(GPIO_LED_D12);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  /* D9 and D11 are only changed during boot up states */

  if ((unsigned int)led <= 3)
    {
      bool d9off  = (led != 1);
      bool d11off = (led != 2);

      lpc54_gpio_write(GPIO_LED_D9,  d9off);   /* Low illuminates */
      lpc54_gpio_write(GPIO_LED_D11, d11off);  /* Low illuminates */
      lpc54_gpio_write(GPIO_LED_D12, true);    /* Low illuminates */
    }
  else
    {
      lpc54_gpio_write(GPIO_LED_D12, false);    /* Low illuminates */
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led == 4)
    {
      lpc54_gpio_write(GPIO_LED_D12, true);    /* Low illuminates */
    }
}

#endif /* CONFIG_ARCH_LEDS */
