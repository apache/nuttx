/****************************************************************************
 * boards/arm/efm32/efm32-g8xx-stk/src/efm32_autoleds.c
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
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

#include "efm32_gpio.h"
#include "efm32-g8xx-stk.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The following definitions map the encoded LED setting to GPIO settings */

#define EFM32_LED0        (1 << 0)
#define EFM32_LED1        (1 << 1)
#define EFM32_LED2        (1 << 2)
#define EFM32_LED3        (1 << 3)

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

#define LED_STARTED_ON_SETBITS       ((EFM32_LED0) << ON_SETBITS_SHIFT)
#define LED_STARTED_ON_CLRBITS       ((EFM32_LED1|EFM32_LED2|EFM32_LED3) << ON_CLRBITS_SHIFT)
#define LED_STARTED_OFF_SETBITS      (0 << OFF_SETBITS_SHIFT)
#define LED_STARTED_OFF_CLRBITS      ((EFM32_LED0|EFM32_LED1|EFM32_LED2|EFM32_LED3) << OFF_CLRBITS_SHIFT)

#define LED_HEAPALLOCATE_ON_SETBITS  ((EFM32_LED1) << ON_SETBITS_SHIFT)
#define LED_HEAPALLOCATE_ON_CLRBITS  ((EFM32_LED0|EFM32_LED2|EFM32_LED3) << ON_CLRBITS_SHIFT)
#define LED_HEAPALLOCATE_OFF_SETBITS ((EFM32_LED0) << OFF_SETBITS_SHIFT)
#define LED_HEAPALLOCATE_OFF_CLRBITS ((EFM32_LED1|EFM32_LED2|EFM32_LED3) << OFF_CLRBITS_SHIFT)

#define LED_IRQSENABLED_ON_SETBITS   ((EFM32_LED0|EFM32_LED1) << ON_SETBITS_SHIFT)
#define LED_IRQSENABLED_ON_CLRBITS   ((EFM32_LED2|EFM32_LED3) << ON_CLRBITS_SHIFT)
#define LED_IRQSENABLED_OFF_SETBITS  ((EFM32_LED1) << OFF_SETBITS_SHIFT)
#define LED_IRQSENABLED_OFF_CLRBITS  ((EFM32_LED0|EFM32_LED2|EFM32_LED3) << OFF_CLRBITS_SHIFT)

#define LED_STACKCREATED_ON_SETBITS  ((EFM32_LED2) << ON_SETBITS_SHIFT)
#define LED_STACKCREATED_ON_CLRBITS  ((EFM32_LED0|EFM32_LED1|EFM32_LED3) << ON_CLRBITS_SHIFT)
#define LED_STACKCREATED_OFF_SETBITS ((EFM32_LED0|EFM32_LED1) << OFF_SETBITS_SHIFT)
#define LED_STACKCREATED_OFF_CLRBITS ((EFM32_LED2|EFM32_LED3) << OFF_CLRBITS_SHIFT)

#define LED_INIRQ_ON_SETBITS         ((EFM32_LED0) << ON_SETBITS_SHIFT)
#define LED_INIRQ_ON_CLRBITS         ((0) << ON_CLRBITS_SHIFT)
#define LED_INIRQ_OFF_SETBITS        ((0) << OFF_SETBITS_SHIFT)
#define LED_INIRQ_OFF_CLRBITS        ((EFM32_LED0) << OFF_CLRBITS_SHIFT)

#define LED_SIGNAL_ON_SETBITS        ((EFM32_LED1) << ON_SETBITS_SHIFT)
#define LED_SIGNAL_ON_CLRBITS        ((0) << ON_CLRBITS_SHIFT)
#define LED_SIGNAL_OFF_SETBITS       ((0) << OFF_SETBITS_SHIFT)
#define LED_SIGNAL_OFF_CLRBITS       ((EFM32_LED1) << OFF_CLRBITS_SHIFT)

#define LED_ASSERTION_ON_SETBITS     ((EFM32_LED3) << ON_SETBITS_SHIFT)
#define LED_ASSERTION_ON_CLRBITS     ((0) << ON_CLRBITS_SHIFT)
#define LED_ASSERTION_OFF_SETBITS    ((0) << OFF_SETBITS_SHIFT)
#define LED_ASSERTION_OFF_CLRBITS    ((EFM32_LED3) << OFF_CLRBITS_SHIFT)

#define LED_PANIC_ON_SETBITS         ((EFM32_LED3) << ON_SETBITS_SHIFT)
#define LED_PANIC_ON_CLRBITS         ((0) << ON_CLRBITS_SHIFT)
#define LED_PANIC_OFF_SETBITS        ((0) << OFF_SETBITS_SHIFT)
#define LED_PANIC_OFF_CLRBITS        ((EFM32_LED3) << OFF_CLRBITS_SHIFT)

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

  (LED_INIRQ_ON_SETBITS         | LED_INIRQ_ON_CLRBITS |
   LED_INIRQ_OFF_SETBITS        | LED_INIRQ_OFF_CLRBITS),

  (LED_SIGNAL_ON_SETBITS        | LED_SIGNAL_ON_CLRBITS |
   LED_SIGNAL_OFF_SETBITS       | LED_SIGNAL_OFF_CLRBITS),

  (LED_ASSERTION_ON_SETBITS     | LED_ASSERTION_ON_CLRBITS |
   LED_ASSERTION_OFF_SETBITS    | LED_ASSERTION_OFF_CLRBITS),

  (LED_PANIC_ON_SETBITS         | LED_PANIC_ON_CLRBITS |
   LED_PANIC_OFF_SETBITS        | LED_PANIC_OFF_CLRBITS)
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void led_clrbits(unsigned int clrbits)
{
  if ((clrbits & EFM32_LED0) != 0)
    {
      efm32_gpiowrite(GPIO_LED0, false);
    }

  if ((clrbits & EFM32_LED1) != 0)
    {
      efm32_gpiowrite(GPIO_LED1, false);
    }

  if ((clrbits & EFM32_LED2) != 0)
    {
      efm32_gpiowrite(GPIO_LED2, false);
    }

  if ((clrbits & EFM32_LED3) != 0)
    {
      efm32_gpiowrite(GPIO_LED3, false);
    }
}

static inline void led_setbits(unsigned int setbits)
{
  if ((setbits & EFM32_LED0) != 0)
    {
      efm32_gpiowrite(GPIO_LED0, true);
    }

  if ((setbits & EFM32_LED1) != 0)
    {
      efm32_gpiowrite(GPIO_LED1, true);
    }

  if ((setbits & EFM32_LED2) != 0)
    {
      efm32_gpiowrite(GPIO_LED2, true);
    }

  if ((setbits & EFM32_LED3) != 0)
    {
      efm32_gpiowrite(GPIO_LED3, true);
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
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED0-4 GPIOs for output */

  efm32_configgpio(GPIO_LED0);
  efm32_configgpio(GPIO_LED1);
  efm32_configgpio(GPIO_LED2);
  efm32_configgpio(GPIO_LED3);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  led_setonoff(ON_BITS(g_ledbits[led]));
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  led_setonoff(OFF_BITS(g_ledbits[led]));
}

#endif /* CONFIG_ARCH_LEDS */
