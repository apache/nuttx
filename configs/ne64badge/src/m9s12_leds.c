/****************************************************************************
 * configs/ne64badge/src/m9s12_leds.c
 *
 *   Copyright (C) 2011, 2013, 2015 Gregory Nutt. All rights reserved.
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

#include "ne64badge_internal.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_DEBUG_LEDS enables debug output from this file (needs CONFIG_DEBUG
 * with CONFIG_DEBUG_VERBOSE too)
 */

#ifdef CONFIG_DEBUG_LEDS
#  define leddbg  lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define ledvdbg lldbg
#  else
#    define ledvdbg(x...)
#  endif
#else
#  define leddbg(x...)
#  define ledvdbg(x...)
#endif

/* Dump GPIO registers */

#if defined(CONFIG_DEBUG_VERBOSE) && defined(CONFIG_DEBUG_LEDS)
#  define led_dumpgpio(m) m9s12_dumpgpio(m)
#else
#  define led_dumpgpio(m)
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
 *
 * Description:
 *   Configure and initialize on-board LEDs
 *
 ****************************************************************************/

void board_led_initialize(void)
{
  /* Configure all LED GPIO lines */

  led_dumpgpio("board_led_initialize() Entry)");

  hcs12_configgpio(NE64BADGE_LED1);
  hcs12_configgpio(NE64BADGE_LED2);

  led_dumpgpio("board_led_initialize() Exit");
}

/****************************************************************************
 * Name: board_led_on
 ****************************************************************************/

void board_led_on(int led)
{
  switch (led)
    {
       default:
       case 0 : /* STARTED, HEAPALLOCATE, IRQSENABLED */
        hcs12_gpiowrite(NE64BADGE_LED1, true);
        hcs12_gpiowrite(NE64BADGE_LED2, true);
        break;

       case 1 : /* STACKCREATED */
        hcs12_gpiowrite(NE64BADGE_LED1, false);
        hcs12_gpiowrite(NE64BADGE_LED2, true);
        break;

       case 2 : /* INIRQ, SIGNAL, ASSERTION, PANIC */
        hcs12_gpiowrite(NE64BADGE_LED2, false);
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
       default:
       case 0 : /* STARTED, HEAPALLOCATE, IRQSENABLED */
       case 1 : /* STACKCREATED */
        hcs12_gpiowrite(NE64BADGE_LED1, true);
       case 2 : /* INIRQ, SIGNAL, ASSERTION, PANIC */
        hcs12_gpiowrite(NE64BADGE_LED2, true);
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
