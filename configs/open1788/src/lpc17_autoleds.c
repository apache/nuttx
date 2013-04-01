/****************************************************************************
 * configs/open1788/src/lpc17_autoleds.c
 * arch/arm/src/board/lpc17_autoleds.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "lpc17_gpio.h"
#include "open1788.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 *
 * LED1 -- Connected to P1[14]
 * LED2 -- Connected to P0[16]
 * LED3 -- Connected to P1[13]
 * LED4 -- Connected to P4[27]
 *
 * These LEDs are connecte to ground so a high output value will illuminate them.
 *
 * If CONFIG_ARCH_LEDs is defined, then NuttX will control the four LEDs
 * on the WaveShare Open1788K.  The following definitions describe how NuttX
 * controls the LEDs:
 *
 *                                 LED1 LED2 LED3 LED4
 *   LED_STARTED                0  OFF  OFF  OFF  OFF
 *   LED_HEAPALLOCATE           1  ON   OFF  OFF  OFF
 *   LED_IRQSENABLED            2  OFF   ON  OFF  OFF
 *   LED_STACKCREATED           3  ON    ON  OFF  OFF
 *   LED_INIRQ                  4  LED3 glows, on while in interupt
 *   LED_SIGNAL                 4  LED3 glows, on while in signal handler
 *   LED_ASSERTION              4  LED3 glows, on while in assertion
 *   LED_PANIC                  4  LED3 Flashes at 2Hz
 *   LED_IDLE                   5  LED glows, ON while sleeping
 */

/* The following definitions map the encoded LED setting to GPIO settings */

#define OPEN1788_LED1     (1 << 0)
#define OPEN1788_LED2     (1 << 1)
#define OPEN1788_LED3     (1 << 2)
#define OPEN1788_LED4     (1 << 3)

#define ON_SETBITS_SHIFT  (0)
#define ON_CLRBITS_SHIFT  (4)
#define OFF_SETBITS_SHIFT (8)
#define OFF_CLRBITS_SHIFT (12)

#define ON_BITS(v)        ((v) & 0xff)
#define OFF_BITS(v)       (((v) >> 8) & 0x0ff)
#define SETBITS(b)        ((b) & 0x0f)
#define CLRBITS(b)        (((b) >> 4) & 0x0f)

#define ON_SETBITS(v)     (SETBITS(ON_BITS(v))
#define ON_CLRBITS(v)     (CLRBITS(ON_BITS(v))
#define OFF_SETBITS(v)    (SETBITS(OFF_BITS(v))
#define OFF_CLRBITS(v)    (CLRBITS(OFF_BITS(v))

#define LED_STARTED_ON_SETBITS       ((0) << ON_SETBITS_SHIFT)
#define LED_STARTED_ON_CLRBITS       ((OPEN1788_LED1|OPEN1788_LED2|OPEN1788_LED3|OPEN1788_LED4) << ON_CLRBITS_SHIFT)
#define LED_STARTED_OFF_SETBITS      (0 << OFF_SETBITS_SHIFT)
#define LED_STARTED_OFF_CLRBITS      ((OPEN1788_LED1|OPEN1788_LED2|OPEN1788_LED3|OPEN1788_LED4) << OFF_CLRBITS_SHIFT)

#define LED_HEAPALLOCATE_ON_SETBITS  ((OPEN1788_LED1) << ON_SETBITS_SHIFT)
#define LED_HEAPALLOCATE_ON_CLRBITS  ((OPEN1788_LED2|OPEN1788_LED3|OPEN1788_LED4) << ON_CLRBITS_SHIFT)
#define LED_HEAPALLOCATE_OFF_SETBITS (0 << OFF_SETBITS_SHIFT)
#define LED_HEAPALLOCATE_OFF_CLRBITS ((OPEN1788_LED1|OPEN1788_LED2|OPEN1788_LED3|OPEN1788_LED4) << OFF_CLRBITS_SHIFT)

#define LED_IRQSENABLED_ON_SETBITS   ((OPEN1788_LED2) << ON_SETBITS_SHIFT)
#define LED_IRQSENABLED_ON_CLRBITS   ((OPEN1788_LED1|OPEN1788_LED3|OPEN1788_LED4) << ON_CLRBITS_SHIFT)
#define LED_IRQSENABLED_OFF_SETBITS  ((OPEN1788_LED1) << OFF_SETBITS_SHIFT)
#define LED_IRQSENABLED_OFF_CLRBITS  ((OPEN1788_LED2|OPEN1788_LED3|OPEN1788_LED4) << OFF_CLRBITS_SHIFT)

#define LED_STACKCREATED_ON_SETBITS  ((OPEN1788_LED1|OPEN1788_LED2) << ON_SETBITS_SHIFT)
#define LED_STACKCREATED_ON_CLRBITS  ((OPEN1788_LED3|OPEN1788_LED4) << ON_CLRBITS_SHIFT)
#define LED_STACKCREATED_OFF_SETBITS ((OPEN1788_LED2) << OFF_SETBITS_SHIFT)
#define LED_STACKCREATED_OFF_CLRBITS ((OPEN1788_LED1|OPEN1788_LED3|OPEN1788_LED4) << OFF_CLRBITS_SHIFT)

#define LED_EVENT_ON_SETBITS         ((OPEN1788_LED3) << ON_SETBITS_SHIFT)
#define LED_EVENT_ON_CLRBITS         ((0) << ON_CLRBITS_SHIFT)
#define LED_EVENT_OFF_SETBITS        ((0) << OFF_SETBITS_SHIFT)
#define LED_EVENT_OFF_CLRBITS        ((OPEN1788_LED3) << OFF_CLRBITS_SHIFT)

#define LED_IDLE_ON_SETBITS          ((0) << ON_SETBITS_SHIFT)
#define LED_IDLE_ON_CLRBITS          ((OPEN1788_LED4) << ON_CLRBITS_SHIFT)
#define LED_IDLE_OFF_SETBITS         ((OPEN1788_LED4) << OFF_SETBITS_SHIFT)
#define LED_IDLE_OFF_CLRBITS         ((0) << OFF_CLRBITS_SHIFT)

/* CONFIG_DEBUG_LEDS enables debug output from this file (needs CONFIG_DEBUG
 * with CONFIG_DEBUG_VERBOSE too)
 */

#ifdef CONFIG_DEBUG_LEDS
#  define leddbg lldbg
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
#  define led_dumpgpio(m) lpc17_dumpgpio(???, m)
#else
#  define led_dumpgpio(m)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint16_t g_ledbits[8] =
{
  (LED_STARTED_ON_SETBITS       | LED_STARTED_ON_CLRBITS |
   LED_STARTED_OFF_SETBITS      | LED_STARTED_OFF_CLRBITS),

  (LED_HEAPALLOCATE_ON_SETBITS  | LED_HEAPALLOCATE_ON_CLRBITS |
   LED_HEAPALLOCATE_OFF_SETBITS | LED_HEAPALLOCATE_OFF_CLRBITS),

  (LED_IRQSENABLED_ON_SETBITS   | LED_IRQSENABLED_ON_CLRBITS |
   LED_IRQSENABLED_OFF_SETBITS  | LED_IRQSENABLED_OFF_CLRBITS),

  (LED_STACKCREATED_ON_SETBITS  | LED_STACKCREATED_ON_CLRBITS |
   LED_STACKCREATED_OFF_SETBITS | LED_STACKCREATED_OFF_CLRBITS),

  (LED_EVENT_ON_SETBITS         | LED_EVENT_ON_CLRBITS |
   LED_EVENT_OFF_SETBITS        | LED_EVENT_OFF_CLRBITS),

  (LED_IDLE_ON_SETBITS          | LED_IDLE_ON_CLRBITS |
   LED_IDLE_OFF_SETBITS         | LED_IDLE_OFF_CLRBITS)
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void led_clrbits(unsigned int clrbits)
{
  if ((clrbits & OPEN1788_LED1) != 0)
    {
      lpc17_gpiowrite(GPIO_LED1, false);
    }

  if ((clrbits & OPEN1788_LED2) != 0)
    {
      lpc17_gpiowrite(GPIO_LED2, false);
    }

  if ((clrbits & OPEN1788_LED3) != 0)
    {
      lpc17_gpiowrite(GPIO_LED3, false);
    }

  if ((clrbits & OPEN1788_LED4) != 0)
    {
      lpc17_gpiowrite(GPIO_LED4, false);
    }
}

static inline void led_setbits(unsigned int setbits)
{
  if ((setbits & OPEN1788_LED1) != 0)
    {
      lpc17_gpiowrite(GPIO_LED1, true);
    }

  if ((setbits & OPEN1788_LED2) != 0)
    {
      lpc17_gpiowrite(GPIO_LED2, true);
    }

  if ((setbits & OPEN1788_LED3) != 0)
    {
      lpc17_gpiowrite(GPIO_LED3, true);
    }

  if ((setbits & OPEN1788_LED4) != 0)
    {
      lpc17_gpiowrite(GPIO_LED4, true);
    }
}

static void led_setonoff(unsigned int bits)
{
  led_clrbits(CLRBITS(bits));
  led_setbits(SETBITS(bits));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_ledinit
 ****************************************************************************/

void up_ledinit(void)
{
  /* Configure LED1-4 GPIOs for output */

  lpc17_configgpio(GPIO_LED1);
  lpc17_configgpio(GPIO_LED2);
  lpc17_configgpio(GPIO_LED3);
  lpc17_configgpio(GPIO_LED4);
}

/****************************************************************************
 * Name: up_ledon
 ****************************************************************************/

void up_ledon(int led)
{
  led_setonoff(ON_BITS(g_ledbits[led]));
}

/****************************************************************************
 * Name: up_ledoff
 ****************************************************************************/

void up_ledoff(int led)
{
  led_setonoff(OFF_BITS(g_ledbits[led]));
}
