/****************************************************************************
 * configs/sama5d3-xplained/src/sam_autoleds.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
/* There are two LEDs on the SAMA5D3 series-CM board that can be controlled
 * by software.  A  blue LED is controlled via PIO pins.  A red LED normally
 * provides an indication that power is supplied to the board but can also
 * be controlled via software.
 *
 *   PE25.  This blue LED is pulled high and is illuminated by pulling PE25
 *   low.
 *
 *   PE24.  The red LED is also pulled high but is driven by a transistor so
 *   that it is illuminated when power is applied even if PE24 is not
 *   configured as an output.  If PE24 is configured as an output, then the
 *   LCD is illuminated by a low output.
 *
 * These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *   SYMBOL            Val Meaning                     LED state
 *                                                    Blue     Red
 *   ----------------- --- -----------------------  -------- --------
 *   LED_STARTED        0  NuttX has been started     OFF      OFF
 *   LED_HEAPALLOCATE   0  Heap has been allocated    OFF      OFF
 *   LED_IRQSENABLED    0  Interrupts enabled         OFF      OFF
 *   LED_STACKCREATED   1  Idle stack created         ON       OFF
 *   LED_INIRQ          2  In an interrupt              No change
 *   LED_SIGNAL         2  In a signal handler          No change
 *   LED_ASSERTION      2  An assertion failed          No change
 *   LED_PANIC          3  The system has crashed     OFF      Blinking
 *   LED_IDLE          N/A MCU is is sleep mode         Not used
 *
 * Thus if the blue LED is statically on, NuttX has successfully booted and
 * is, apparently, running normally.  If the red is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "sam_pio.h"
#include "sama5d3-xplained.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
  /* Configure LED PIOs for output */

  sam_configpio(PIO_BLUE);
  sam_configpio(PIO_RED);
}

/****************************************************************************
 * Name: board_led_on
 ****************************************************************************/

void board_led_on(int led)
{
  bool blueoff = true;  /* Low illuminates */
  bool redon   = false; /* High illuminates */

  switch (led)
    {
      case 0:  /* LED_STARTED, LED_HEAPALLOCATE, LED_IRQSENABLED */
        break;

      case 1:  /* LED_STACKCREATED */
        blueoff = false;
        break;

      default:
      case 2:  /* LED_INIRQ, LED_SIGNAL, LED_ASSERTION */
        return;

      case 3:  /* LED_PANIC */
        redon = true;
        break;
    }

  sam_piowrite(PIO_BLUE, blueoff);
  sam_piowrite(PIO_RED, redon);
}

/****************************************************************************
 * Name: board_led_off
 ****************************************************************************/

void board_led_off(int led)
{
  if (led != 2)
    {
      sam_piowrite(PIO_BLUE, true);  /* Low illuminates */
      sam_piowrite(PIO_RED, false);  /* High illuminates */
    }
}

#endif /* CONFIG_ARCH_LEDS */
