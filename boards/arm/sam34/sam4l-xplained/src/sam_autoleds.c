/****************************************************************************
 * boards/arm/sam34/sam4l-xplained/src/sam_autoleds.c
 *
 *   Copyright (C) 201, 2015 Gregory Nutt. All rights reserved.
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
/* There are three LEDs on board the SAM4L Xplained Pro board:  The EDBG
 * controls two of the LEDs, a power LED and a status LED.  There is only
 * one user controllable LED, a yellow LED labeled LED0 near the SAM4L USB
 * connector.
 *
 * This LED is controlled by PC07 and LED0 can be activated by driving the
 * PC07 to GND.
 *
 * When CONFIG_ARCH_LEDS is defined in the NuttX configuration, NuttX will
 * control LED0 as follows:
 *
 *   SYMBOL              Meaning                 LED0
 *   ------------------- ----------------------- ------
 *   LED_STARTED         NuttX has been started  OFF
 *   LED_HEAPALLOCATE    Heap has been allocated OFF
 *   LED_IRQSENABLED     Interrupts enabled      OFF
 *   LED_STACKCREATED    Idle stack created      ON
 *   LED_INIRQ           In an interrupt**       N/C
 *   LED_SIGNAL          In a signal handler***  N/C
 *   LED_ASSERTION       An assertion failed     N/C
 *   LED_PANIC           The system has crashed  FLASH
 *
 * Thus is LED0 is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If LED0 is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
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

#include "sam_gpio.h"
#include "sam4l-xplained.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  sam_configgpio(GPIO_LED0);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  bool ledstate = true;

  switch (led)
    {
    case 0:                   /* LED_STARTED:      NuttX has been started  LED0=OFF */
                              /* LED_HEAPALLOCATE: Heap has been allocated LED0=OFF */
                              /* LED_IRQSENABLED:  Interrupts enabled      LED0=OFF */
      break;                  /* Leave ledstate == true to turn OFF */

    default:
    case 2:                   /* LED_INIRQ:        In an interrupt         LED0=N/C */
                              /* LED_SIGNAL:       In a signal handler     LED0=N/C */
                              /* LED_ASSERTION:    An assertion failed     LED0=N/C */
      return;                 /* Return to leave LED0 unchanged */

    case 3:                   /* LED_PANIC:        The system has crashed  LED0=FLASH */
    case 1:                   /* LED_STACKCREATED: Idle stack created      LED0=ON */
      ledstate = false;       /* Set ledstate == false to turn ON */
      break;
    }

  sam_gpiowrite(GPIO_LED0, ledstate);
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
    case 0:                   /* LED_STARTED:      NuttX has been started  LED0=OFF */
                              /* LED_HEAPALLOCATE: Heap has been allocated LED0=OFF */
                              /* LED_IRQSENABLED:  Interrupts enabled      LED0=OFF */
    case 1:                   /* LED_STACKCREATED: Idle stack created      LED0=ON */

    /* These result in no-change */

    case 2:                   /* LED_INIRQ:        In an interrupt         LED0=N/C */
                              /* LED_SIGNAL:       In a signal handler     LED0=N/C */
                              /* LED_ASSERTION:    An assertion failed     LED0=N/C */
      return;                 /* Return to leave LED0 unchanged */

    /* Turn LED0 off set driving the output high */

    case 3:                   /* LED_PANIC:        The system has crashed  LED0=FLASH */
      sam_gpiowrite(GPIO_LED0, true);
      break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
