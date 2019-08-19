/****************************************************************************
 * boards/arm/stm32f7/stm32f769i-disco/src/stm32_autoleds.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>

#include "stm32_gpio.h"
#include "stm32f769i-disco.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure the LD1 GPIO for output. Initial state is OFF */

  stm32_configgpio(GPIO_LD3);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  bool ledstate = false;

  switch (led)
    {
    case 0:                   /* LED_STARTED:      NuttX has been started  STATUS LED=OFF */
                              /* LED_HEAPALLOCATE: Heap has been allocated STATUS LED=OFF */
                              /* LED_IRQSENABLED:  Interrupts enabled      STATUS LED=OFF */
      break;                  /* Leave ledstate == true to turn OFF */

    default:
    case 2:                   /* LED_INIRQ:        In an interrupt         STATUS LED=N/C */
                              /* LED_SIGNAL:       In a signal handler     STATUS LED=N/C */
                              /* LED_ASSERTION:    An assertion failed     STATUS LED=N/C */
      return;                 /* Return to leave STATUS LED unchanged */

    case 3:                   /* LED_PANIC:        The system has crashed  STATUS LED=FLASH */
    case 1:                   /* LED_STACKCREATED: Idle stack created      STATUS LED=ON */
      ledstate = true;       /* Set ledstate == false to turn ON */
      break;
    }

   stm32_gpiowrite(GPIO_LD3, ledstate);
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
    /* These should not happen and are ignored */

    default:
    case 0:                   /* LED_STARTED:      NuttX has been started  STATUS LED=OFF */
                              /* LED_HEAPALLOCATE: Heap has been allocated STATUS LED=OFF */
                              /* LED_IRQSENABLED:  Interrupts enabled      STATUS LED=OFF */
    case 1:                   /* LED_STACKCREATED: Idle stack created      STATUS LED=ON */

    /* These result in no-change */

    case 2:                   /* LED_INIRQ:        In an interrupt         STATUS LED=N/C */
                              /* LED_SIGNAL:       In a signal handler     STATUS LED=N/C */
                              /* LED_ASSERTION:    An assertion failed     STATUS LED=N/C */
      return;                 /* Return to leave STATUS LED unchanged */

    /* Turn STATUS LED off set driving the output high */

    case 3:                   /* LED_PANIC:        The system has crashed  STATUS LED=FLASH */
      stm32_gpiowrite(GPIO_LD3, false);
      break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
