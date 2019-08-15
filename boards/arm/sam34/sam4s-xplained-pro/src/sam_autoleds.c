/****************************************************************************
 * boards/arm/sam34/sam4s-xplained-pro/src/sam_autoleds.c
 *
 *   Copyright (C) 2014, 2015 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Bob Doiron
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
#include <arch/board/board.h>

#include "chip.h"
#include "sam_gpio.h"
#include "sam4s-xplained-pro.h"

#ifdef CONFIG_ARCH_LEDS

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on
 * board the SAM4S Xplained Pro.  The following definitions describe how NuttX
 * controls the LEDs:
 *
 *   SYMBOL                Meaning                     LED state
 *                                                   D9     D10
 *   -------------------  -----------------------  -------- --------
 *   LED_STARTED          NuttX has been started     OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF
 *   LED_STACKCREATED     Idle stack created         ON
 *   LED_INIRQ            In an interrupt            No change
 *   LED_SIGNAL           In a signal handler        No change
 *   LED_ASSERTION        An assertion failed        No change
 *   LED_PANIC            The system has crashed     OFF
 *   LED_IDLE             MCU is is sleep mode       Not used
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED GPIO for output */

  sam_configgpio(GPIO_D301);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      case 0:  /* LED_STARTED, LED_HEAPALLOCATE, LED_IRQSENABLED - off while initializing */
        sam_gpiowrite(GPIO_D301, LED_D301_OFF);
        break;

      case 1:  /* LED_STACKCREATED - turn on when ready */
        sam_gpiowrite(GPIO_D301, LED_D301_ON);
        break;

      case 2:  /* LED_INIRQ, LED_SIGNAL - turn off inside irqs/signal processing */
        sam_gpiowrite(GPIO_D301, LED_D301_OFF);
        return;

      case 3:  /* LED_PANIC - flash */
        sam_gpiowrite(GPIO_D301, LED_D301_ON);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
  {
      default:
        break;

      case 2:  /* LED_INIRQ, LED_SIGNAL - return to on after irq/signal processing */
        sam_gpiowrite(GPIO_D301, LED_D301_ON);
        return;

      case 3:  /* LED_PANIC - flashes */
        sam_gpiowrite(GPIO_D301, LED_D301_OFF);
        break;
  }
}

#endif /* CONFIG_ARCH_LEDS */
