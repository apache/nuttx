/****************************************************************************
 * boards/arm/lpc17xx_40xx/olimex-lpc1766stk/src/lpc17_40_leds.c
 *
 *   Copyright (C) 2010-2011, 2013, 2015 Gregory Nutt. All rights reserved.
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
#include "arm_arch.h"
#include "arm_internal.h"

#include "lpc17_40_gpio.h"

#include "lpc1766stk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Dump GPIO registers */

#ifdef CONFIG_DEBUG_LEDS_INFO
#  define led_dumpgpio(m) lpc17_40_dumpgpio(LPC1766STK_LED1, m)
#else
#  define led_dumpgpio(m)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
static bool g_uninitialized = true;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize/board_autoled_initialize
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
uint32_t board_userled_initialize(void) /* Name when invoked externally */
#else
void board_autoled_initialize(void) /* Name when invoked via lpc17_40_boot.c */
#endif
{
  /* Configure all LED GPIO lines */

  led_dumpgpio("board_*led_initialize() Entry)");

  lpc17_40_configgpio(LPC1766STK_LED1);
  lpc17_40_configgpio(LPC1766STK_LED2);

  led_dumpgpio("board_*led_initialize() Exit");
#ifndef CONFIG_ARCH_LEDS
  return BOARD_NLEDS;
#endif
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void board_userled(int led, bool ledon)
{
  if (led == BOARD_LED1)
    {
      lpc17_40_gpiowrite(LPC1766STK_LED1, !ledon);
    }
  else if (led == BOARD_LED2)
    {
      lpc17_40_gpiowrite(LPC1766STK_LED2, !ledon);
    }
}
#endif

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void board_userled_all(uint32_t ledset)
{
  lpc17_40_gpiowrite(LPC1766STK_LED1, (ledset & BOARD_LED1_BIT) == 0);
  lpc17_40_gpiowrite(LPC1766STK_LED2, (ledset & BOARD_LED2_BIT) == 0);
}
#endif

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_on(int led)
{
  switch (led)
    {
       default:
       case 0 : /* STARTED, HEAPALLOCATE, IRQSENABLED */
        lpc17_40_gpiowrite(LPC1766STK_LED1, true);
        lpc17_40_gpiowrite(LPC1766STK_LED2, true);
        break;

       case 1 : /* STACKCREATED */
        lpc17_40_gpiowrite(LPC1766STK_LED1, false);
        lpc17_40_gpiowrite(LPC1766STK_LED2, true);
        g_uninitialized = false;
        break;

       case 2 : /* INIRQ, SIGNAL, ASSERTION, PANIC */
        lpc17_40_gpiowrite(LPC1766STK_LED2, false);
        break;

       case 3 : /* IDLE */
        lpc17_40_gpiowrite(LPC1766STK_LED1, true);
        break;
    }
}
#endif

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_off(int led)
{
  switch (led)
    {
       default:
       case 0 : /* STARTED, HEAPALLOCATE, IRQSENABLED */
       case 1 : /* STACKCREATED */
        lpc17_40_gpiowrite(LPC1766STK_LED1, true);

       case 2 : /* INIRQ, SIGNAL, ASSERTION, PANIC */
        lpc17_40_gpiowrite(LPC1766STK_LED2, true);
        break;

       case 3 : /* IDLE */
        lpc17_40_gpiowrite(LPC1766STK_LED1, g_uninitialized);
        break;
    }
}
#endif
