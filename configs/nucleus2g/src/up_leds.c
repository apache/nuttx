/****************************************************************************
 * configs/nucleus2g/src/up_leds.c
 * arch/arm/src/board/up_leds.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "lpc17_internal.h"

#include "nucleus2g_internal.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define LED_OFF        0
#define LED_ON         1
#define LED_GREEN      2
#define LED_PLUSGREEN  3
#define LED_MINUSGREEN 4
#define LED_RED        5
#define LED_PLUSRED    6
#define LED_MINUSRED   7
#define LED_NC         8
#define LED_PREV       9

/* Enables debug output from this file (needs CONFIG_DEBUG with
 * CONFIG_DEBUG_VERBOSE too)
 */

#undef LED_DEBUG  /* Define to enable debug */

#ifdef LED_DEBUG
#  define leddbg  lldbg
#  define ledvdbg llvdbg
#else
#  define leddbg(x...)
#  define ledvdbg(x...)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The Nucleus2G has 3 LEDs... two on the Babel CAN board and a "heartbeat" LED."
 * The LEDs on the Babel CAN board are capabl of OFF/GREEN/RED/AMBER status.
 * In normal usage, the two LEDs on the Babel CAN board would show CAN status, but if
 * CONFIG_ARCH_LEDS is defined, these LEDs will be controlled as follows for NuttX
 * debug functionality (where NC means "No Change").
 * 
 *                      LED1   LED2   HEARTBEAT
 *                    +------- ------ -----------------------
 *   LED_STARTED      | OFF    OFF    OFF
 *   LED_HEAPALLOCATE | GREEN  OFF    OFF
 *   LED_IRQSENABLED  | OFF    GREEN  OFF
 *   LED_STACKCREATED | GREEN  GREEN  OFF
 *   LED_INIRQ        | NC     NC     ON 
 *   LED_SIGNAL       | NC     RED    NC 
 *   LED_ASSERTION    | RED    NC     NC
 *   LED_PANIC        | RED    RED    NC  (1Hz flashing)
 */

static const uint8_t g_led1on[8] =
{
  LED_OFF, LED_GREEN,    LED_OFF,      LED_GREEN,
  LED_NC,  LED_NC,       LED_PLUSRED,  LED_PLUSRED
};

static const uint8_t g_led1off[8] =
{
  LED_OFF, LED_OFF,      LED_GREEN,    LED_OFF,
  LED_NC,  LED_NC,       LED_MINUSRED, LED_PREV
};

static const uint8_t g_led2on[8] =
{
  LED_OFF, LED_OFF,      LED_GREEN,    LED_GREEN,
  LED_NC,  LED_PLUSRED,  LED_NC,       LED_PLUSRED
};

static const uint8_t g_led2off[8] =
{
  LED_OFF, LED_OFF,      LED_OFF,      LED_GREEN,
  LED_NC,  LED_MINUSRED, LED_NC,       LED_PREV
};

static const uint8_t g_ledhbon[8] =
{
  LED_OFF, LED_OFF,      LED_OFF,      LED_OFF,
  LED_ON,  LED_NC,       LED_NC,       LED_NC
};

static const uint8_t g_ledhboff[8] =
{
  LED_OFF, LED_OFF,      LED_OFF,      LED_OFF,
  LED_OFF, LED_NC,       LED_NC,       LED_NC
};

static bool g_prevled1a;
static bool g_currled1a;
static bool g_prevled1b;
static bool g_currled1b;
static bool g_prevled2a;
static bool g_currled2a;
static bool g_prevled2b;
static bool g_currled2b;
static bool g_prevledhb;
static bool g_currledhb;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_led1
 ****************************************************************************/

void up_led1(uint8_t newstate)
{
  bool led1a = false;
  bool led1b = false;

  switch (newstate)
    {
      default:
      case LED_OFF:
      case LED_ON:
        break;

     case LED_GREEN:
        led1b = true;
        break;

     case LED_PLUSGREEN:
        led1b = true;
     case LED_MINUSGREEN:
        led1a = g_currled1a;
        break;

     case LED_RED:
        break;

     case LED_PLUSRED:
        led1a = true;
     case LED_MINUSRED:
        led1b = g_currled1b;
        break;

     case LED_NC:
        led1a = g_currled1a;
        led1b = g_currled1b;
        break;

     case LED_PREV:
        led1a = g_prevled1a;
        led1b = g_prevled1b;
        break;
    }
        
  lpc17_gpiowrite(NUCLEUS2G_LED1_A, led1a);
  lpc17_gpiowrite(NUCLEUS2G_LED1_B, led1b);
  g_prevled1a = g_currled1a;
  g_currled1a = led1a;
  g_prevled1b = g_currled1b;
  g_currled1b = led1b;
}

/****************************************************************************
 * Name: up_led2
 ****************************************************************************/

void up_led2(uint8_t newstate)
{
  bool led2a = false;
  bool led2b = false;

  switch (newstate)
    {
      default:
      case LED_OFF:
      case LED_ON:
        break;

     case LED_GREEN:
        led2b = true;
        break;

     case LED_PLUSGREEN:
        led2b = true;
     case LED_MINUSGREEN:
        led2a = g_currled2a;
        break;

     case LED_RED:
        break;

     case LED_PLUSRED:
        led2a = true;
     case LED_MINUSRED:
        led2b = g_currled2b;
        break;

     case LED_NC:
        led2a = g_currled2a;
        led2b = g_currled2b;
        break;

     case LED_PREV:
        led2a = g_prevled2a;
        led2b = g_prevled2b;
        break;
    }
        
  lpc17_gpiowrite(NUCLEUS2G_LED2_A, led2a);
  lpc17_gpiowrite(NUCLEUS2G_LED2_B, led2b);
  g_prevled2a = g_currled2a;
  g_currled2a = led2a;
  g_prevled2b = g_currled2b;
  g_currled2b = led2b;
}

/****************************************************************************
 * Name: up_led2
 ****************************************************************************/

void up_ledhb(uint8_t newstate)
{
  bool ledhb = false;

  switch (newstate)
    {
      default:
      case LED_OFF:
        break;

      case LED_ON:
        ledhb = true;
        break;
    }
  lpc17_gpiowrite(NUCLEUS2G_HEARTBEAT, ledhb);
  g_prevledhb = g_currledhb;
  g_currledhb = newstate;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_ledinit
 ****************************************************************************/

void up_ledinit(void)
{
  /* Configure all LED GPIO lines */

  lpc17_configgpio(NUCLEUS2G_LED1_A);
  lpc17_configgpio(NUCLEUS2G_LED1_B);
  lpc17_configgpio(NUCLEUS2G_LED2_A);
  lpc17_configgpio(NUCLEUS2G_LED2_B);
  lpc17_configgpio(NUCLEUS2G_HEARTBEAT);
  lpc17_configgpio(NUCLEUS2G_EXTRA_LED);
}

/****************************************************************************
 * Name: up_ledon
 ****************************************************************************/

void up_ledon(int led)
{
  up_led1(g_led1on[led]);
  up_led2(g_led2on[led]);
  up_ledhb(g_ledhbon[led]);
}

/****************************************************************************
 * Name: up_ledoff
 ****************************************************************************/

void up_ledoff(int led)
{
  up_led1(g_led1off[led]);
  up_led2(g_led2off[led]);
  up_ledhb(g_ledhboff[led]);
}

#endif /* CONFIG_ARCH_LEDS */
