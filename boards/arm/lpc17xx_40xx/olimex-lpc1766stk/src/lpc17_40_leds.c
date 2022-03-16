/****************************************************************************
 * boards/arm/lpc17xx_40xx/olimex-lpc1766stk/src/lpc17_40_leds.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
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
