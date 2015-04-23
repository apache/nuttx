/****************************************************************************
 * configs/arduino-due/src/sam_autoleds.c
 *
 *   Copyright (C) 2013, 2015 Gregory Nutt. All rights reserved.
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
#include <arch/board/board.h>

#include "chip.h"
#include "sam_gpio.h"
#include "arduino-due.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/*  There are three user-controllable LEDs on board the Arduino Due board:
 *
 *     LED              GPIO
 *     ---------------- -----
 *     L   Amber LED    PB27
 *     TX  Yellow LED   PA21
 *     RX  Yellow LED   PC30
 *
 * LED L is connected to ground and can be illuminated by driving the PB27
 * output high. The TX and RX LEDs are pulled high and can be illuminated by
 * driving the corresponding
 * GPIO output to low.
 *
 * These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *   SYMBOL                MEANING                         LED STATE
 *                                                   L         TX       RX
 *   -------------------  -----------------------  -------- -------- --------
 *   LED_STARTED          NuttX has been started     OFF      OFF      OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF      OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF      OFF      OFF
 *   LED_STACKCREATED     Idle stack created         ON       OFF      OFF
 *   LED_INIRQ            In an interrupt            N/C      GLOW     OFF
 *   LED_SIGNAL           In a signal handler        N/C      GLOW     OFF
 *   LED_ASSERTION        An assertion failed        N/C      GLOW     OFF
 *   LED_PANIC            The system has crashed     N/C      N/C      Blinking
 *   LED_IDLE             MCU is is sleep mode       ------ Not used --------
 *
 * Thus if LED L is statically on, NuttX has successfully booted and is,
 * apparently, running normmally.  If LED RX is glowing, then NuttX is
 * handling interrupts (and also signals and assertions).  If TX is flashing
 * at approximately 2Hz, then a fatal error has been detected and the system
 */

/* CONFIG_DEBUG_LEDS enables debug output from this file (needs CONFIG_DEBUG
 * with CONFIG_DEBUG_VERBOSE too)
 */

#ifdef CONFIG_DEBUG_LEDS
#  define leddbg  lldbg
#  define ledvdbg llvdbg
#else
#  define leddbg(x...)
#  define ledvdbg(x...)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_led_initialize
 ****************************************************************************/

void board_led_initialize(void)
{
  /* Configure RX and TX LED GPIOs for output */

  sam_configgpio(GPIO_LED_L);
  sam_configgpio(GPIO_LED_RX);
  sam_configgpio(GPIO_LED_TX);
}

/****************************************************************************
 * Name: board_led_on
 ****************************************************************************/

void board_led_on(int led)
{
  switch (led)
    {
      /* 0: LED_STARTED, LED_HEAPALLOCATE, LED_IRQSENABLED: L=OFF TX=OFF
       *    RX=OFF
       *
       * Since the LEDs were initially all OFF and since this state only
       * occurs one time, nothing need be done.
       */

      default:
      case 0:
        break;

      /* 1: LED_STACKCREATED: L=ON TX=OFF RX=OFF
       *
       * This case will also occur only once.  Note that unlike the other
       * LEDs, LED L is active high.
       */

      case 1:
        sam_gpiowrite(GPIO_LED_L, true);
        break;

      /* 2: LED_INIRQ, LED_SIGNAL, LED_ASSERTION: L=N/C TX=ON RX=N/C
       *
       * This case will occur many times.  LED TX is active low.
       */

     case 2:
        sam_gpiowrite(GPIO_LED_TX, false);
        break;

      /* 3: LED_PANIC: L=N/X TX=N/C RX=ON
       *
       * This case will also occur many times. LED RX is active low.
       */

      case 3:
        sam_gpiowrite(GPIO_LED_RX, false);
        break;
    }
}

/****************************************************************************
 * Name: board_led_off
 ****************************************************************************/

void board_led_off(int led)
{
  switch (led)
    {
      /* 0: LED_STARTED, LED_HEAPALLOCATE, LED_IRQSENABLED: L=OFF TX=OFF
       *    RX=OFF
       * 1: LED_STACKCREATED: L=ON TX=OFF RX=OFF
       *
       * These cases should never happen.
       */

      default:
      case 1:
      case 0:
        break;

      /* 2: LED_INIRQ, LED_SIGNAL, LED_ASSERTION: L=N/C TX=OFF RX=N/C
       *
       * This case will occur many times.  LED TX is active low.
       */

     case 2:
        sam_gpiowrite(GPIO_LED_TX, true);
        break;

      /* 3: LED_PANIC: L=N/X TX=N/C RX=OFF
       *
       * This case will also occur many times. LED RX is active low.
       */

      case 3:
        sam_gpiowrite(GPIO_LED_RX, true);
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
