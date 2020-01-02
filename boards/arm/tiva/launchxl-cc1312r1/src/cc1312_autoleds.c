/****************************************************************************
 * boards/arm/tiva/launchxl-cc1312r1/src/cc1312_autoleds.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>

#include "tiva_gpio.h"
#include "launchxl-cc1312r1.h"

#include <arch/board/board.h>

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  tiva_configgpio(&g_gpio_gled);
  tiva_configgpio(&g_gpio_rled);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  bool gled_change = true;   /* True: Change GLED */
  bool gled_on     = false;  /* High output illuminates */
  bool rled_on     = false;

  /* SYMBOL           VAL MEANING                 GLED RLED
   * ---------------- --- ----------------------- ---- -----
   * LED_STARTED       0  NuttX has been started  OFF  OFF
   * LED_HEAPALLOCATE  1  Heap has been allocated OFF  ON
   * LED_IRQSENABLED   1  Interrupts enabled      OFF  ON
   * LED_STACKCREATED  2  Idle stack created      ON   OFF
   * LED_INIRQ         3  In an interrupt         N/C  GLOW
   * LED_SIGNAL        3  In a signal handler     N/C  GLOW
   * LED_ASSERTION     3  An assertion failed     N/C  GLOW
   * LED_PANIC         4  The system has crashed  OFF  BLINK
   */

  switch (led)
    {
      case 0:  /* GLED=OFF RLED=OFF */
        break;

      case 3:  /* GLED=N/C RLED=ON */
        gled_change = false;

        /* Fall through */

      case 1:  /* GLED=OFF RLED=ON */
      case 4:  /* GLED=OFF RLED=ON */
        rled_on = true;
        break;

      case 2:  /* GLED=ON  RLED=OFF */
        gled_on = true;
        break;

      default:
        return;
    }

  /* Set the new state of the GLED (unless is is N/C) */

  if (gled_change)
    {
      tiva_gpiowrite(&g_gpio_gled, gled_on);
    }

  /* Set the new state of the RLED */

  tiva_gpiowrite(&g_gpio_rled, rled_on);
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  /* SYMBOL           VAL MEANING                 GLED RLED
   * ---------------- --- ----------------------- ---- -----
   * LED_STARTED       0  NuttX has been started  OFF  OFF
   * LED_HEAPALLOCATE  1  Heap has been allocated OFF  ON
   * LED_IRQSENABLED   1  Interrupts enabled      OFF  ON
   * LED_STACKCREATED  2  Idle stack created      ON   OFF
   * LED_INIRQ         3  In an interrupt         N/C  GLOW
   * LED_SIGNAL        3  In a signal handler     N/C  GLOW
   * LED_ASSERTION     3  An assertion failed     N/C  GLOW
   * LED_PANIC         4  The system has crashed  OFF  BLINK
   */

  switch (led)
    {
      case 4:  /* GLED=OFF RLED=OFF */
        tiva_gpiowrite(&g_gpio_gled, false);

        /* Fall through */

      case 3:  /* GLED=N/C RLED=OFF */
        tiva_gpiowrite(&g_gpio_rled, false);
        break;

      case 0:  /* Should not happen */
      case 1:  /* Should not happen */
      case 2:  /* Should not happen */
      default:
        return;
    }
}

#endif /* CONFIG_ARCH_LEDS */
