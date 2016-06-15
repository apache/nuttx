/****************************************************************************
 * configs/stm32l476vg-disco/src/stm32_autoleds.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: dev@ziggurat29.com
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
#include "up_arch.h"
#include "up_internal.h"
#include "stm32l4.h"
#include "stm32l476vg-disco.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LD4,5 GPIO for output */

  stm32l4_configgpio(GPIO_LED_RED);
  stm32l4_configgpio(GPIO_LED_GRN);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      /* 0: LED_STARTED, LED_HEAPALLOCATE, LED_IRQSENABLED
       *
       * Since the LEDs were initially all OFF and since this state only
       * occurs one time, nothing need be done.
       */

      default:
      case LED_STARTED:
      case LED_HEAPALLOCATE:
      case LED_IRQSENABLED:
        break;

      /* 1: LED_STACKCREATED
       *
       * This case will also occur only once.
       */

      case LED_STACKCREATED:
        break;

      /* 2: LED_INIRQ, LED_SIGNAL, LED_ASSERTION
       *
       * This case will occur many times.
       */

      case LED_INIRQ:
      case LED_SIGNAL:
      case LED_ASSERTION:
        stm32l4_gpiowrite(GPIO_LED_RED, true);
        break;

      /* 3: LED_PANIC: GPIO_LED_GRN=OFF RX=ON
       *
       * This case will also occur many times.
       */

      case LED_PANIC:
        stm32l4_gpiowrite(GPIO_LED_GRN, false);
        stm32l4_gpiowrite(GPIO_LED_RED, true);
        break;

      case LED_IDLE:
        stm32l4_gpiowrite(GPIO_LED_GRN, true);
        stm32l4_gpiowrite(GPIO_LED_RED, false);
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
      /* 0: LED_STARTED, LED_HEAPALLOCATE, LED_IRQSENABLED:
       * 1: LED_STACKCREATED:
       *
       * These cases should never happen.
       */

      default:
      case LED_STARTED:
      case LED_HEAPALLOCATE:
      case LED_IRQSENABLED:
      case LED_STACKCREATED:
        break;

      /* 2: LED_INIRQ, LED_SIGNAL, LED_ASSERTION:
       *
       * This case will occur many times.
       */

      case LED_INIRQ:
      case LED_SIGNAL:
      case LED_ASSERTION:
        stm32l4_gpiowrite(GPIO_LED_RED, false);
        break;

      /* 3: LED_PANIC: GPIO_LED_GRN=OFF RX=OFF
       *
       * This case will also occur many times.
       */

      case LED_PANIC:
        stm32l4_gpiowrite(GPIO_LED_GRN, false);
        stm32l4_gpiowrite(GPIO_LED_RED, false);
        break;

      case LED_IDLE:
        stm32l4_gpiowrite(GPIO_LED_GRN, false);
        stm32l4_gpiowrite(GPIO_LED_RED, false);
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
