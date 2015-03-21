/****************************************************************************
 * configs/spark/src/stm32_autoleds.c
 *
 *   Copyright (C) 2012-2013, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           David Sidrane <david_s5@nscdg.com>
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
#include "stm32.h"
#include "spark.h"

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

/* The following definitions map the encoded LED setting to bit sets used to
 * manipulate the LEDs. All terms are in true logic, the led polarity is dealt
 * with in the  phy_xxx operations
 */

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

#define LED_STARTED_ON_SETBITS       ((BOARD_RED_LED_BIT) << ON_SETBITS_SHIFT)
#define LED_STARTED_ON_CLRBITS       ((BOARD_GREEN_LED_BIT|BOARD_BLUE_LED_BIT|BOARD_USR_LED_BIT) << ON_CLRBITS_SHIFT)
#define LED_STARTED_OFF_SETBITS      (0 << OFF_SETBITS_SHIFT)
#define LED_STARTED_OFF_CLRBITS      ((BOARD_RED_LED_BIT|BOARD_GREEN_LED_BIT|BOARD_BLUE_LED_BIT|BOARD_USR_LED_BIT) << OFF_CLRBITS_SHIFT)

#define LED_HEAPALLOCATE_ON_SETBITS  ((BOARD_BLUE_LED_BIT) << ON_SETBITS_SHIFT)
#define LED_HEAPALLOCATE_ON_CLRBITS  ((BOARD_RED_LED_BIT|BOARD_GREEN_LED_BIT|BOARD_USR_LED_BIT) << ON_CLRBITS_SHIFT)
#define LED_HEAPALLOCATE_OFF_SETBITS ((BOARD_RED_LED_BIT) << OFF_SETBITS_SHIFT)
#define LED_HEAPALLOCATE_OFF_CLRBITS ((BOARD_GREEN_LED_BIT|BOARD_BLUE_LED_BIT|BOARD_USR_LED_BIT) << OFF_CLRBITS_SHIFT)

#define LED_IRQSENABLED_ON_SETBITS   ((BOARD_RED_LED_BIT|BOARD_GREEN_LED_BIT) << ON_SETBITS_SHIFT)
#define LED_IRQSENABLED_ON_CLRBITS   ((BOARD_BLUE_LED_BIT|BOARD_USR_LED_BIT) << ON_CLRBITS_SHIFT)
#define LED_IRQSENABLED_OFF_SETBITS  ((BOARD_BLUE_LED_BIT) << OFF_SETBITS_SHIFT)
#define LED_IRQSENABLED_OFF_CLRBITS  ((BOARD_RED_LED_BIT|BOARD_GREEN_LED_BIT|BOARD_USR_LED_BIT) << OFF_CLRBITS_SHIFT)

#define LED_STACKCREATED_ON_SETBITS  ((BOARD_GREEN_LED_BIT) << ON_SETBITS_SHIFT)
#define LED_STACKCREATED_ON_CLRBITS  ((BOARD_RED_LED_BIT|BOARD_BLUE_LED_BIT|BOARD_USR_LED_BIT) << ON_CLRBITS_SHIFT)
#define LED_STACKCREATED_OFF_SETBITS ((BOARD_RED_LED_BIT|BOARD_GREEN_LED_BIT) << OFF_SETBITS_SHIFT)
#define LED_STACKCREATED_OFF_CLRBITS ((BOARD_BLUE_LED_BIT|BOARD_USR_LED_BIT) << OFF_CLRBITS_SHIFT)

#define LED_INIRQ_ON_SETBITS         ((BOARD_RED_LED_BIT) << ON_SETBITS_SHIFT)
#define LED_INIRQ_ON_CLRBITS         ((0) << ON_CLRBITS_SHIFT)
#define LED_INIRQ_OFF_SETBITS        ((0) << OFF_SETBITS_SHIFT)
#define LED_INIRQ_OFF_CLRBITS        ((BOARD_RED_LED_BIT) << OFF_CLRBITS_SHIFT)

#define LED_SIGNAL_ON_SETBITS        ((BOARD_BLUE_LED_BIT) << ON_SETBITS_SHIFT)
#define LED_SIGNAL_ON_CLRBITS        ((0) << ON_CLRBITS_SHIFT)
#define LED_SIGNAL_OFF_SETBITS       ((0) << OFF_SETBITS_SHIFT)
#define LED_SIGNAL_OFF_CLRBITS       ((BOARD_BLUE_LED_BIT) << OFF_CLRBITS_SHIFT)

#define LED_ASSERTION_ON_SETBITS     ((BOARD_RED_LED_BIT|BOARD_GREEN_LED_BIT|BOARD_BLUE_LED_BIT) << ON_SETBITS_SHIFT)
#define LED_ASSERTION_ON_CLRBITS     ((0) << ON_CLRBITS_SHIFT)
#define LED_ASSERTION_OFF_SETBITS    ((0) << OFF_SETBITS_SHIFT)
#define LED_ASSERTION_OFF_CLRBITS    ((BOARD_RED_LED_BIT|BOARD_GREEN_LED_BIT|BOARD_BLUE_LED_BIT) << OFF_CLRBITS_SHIFT)

#define LED_PANIC_ON_SETBITS         ((BOARD_RED_LED_BIT) << ON_SETBITS_SHIFT)
#define LED_PANIC_ON_CLRBITS         ((0) << ON_CLRBITS_SHIFT)
#define LED_PANIC_OFF_SETBITS        ((0) << OFF_SETBITS_SHIFT)
#define LED_PANIC_OFF_CLRBITS        ((BOARD_RED_LED_BIT) << OFF_CLRBITS_SHIFT)

/**************************************************************************************
 * Private Function Prototypes
 **************************************************************************************/

/* LED State Controls */

static inline void phy_led_on(unsigned int on_bitset);
static inline void phy_led_off(unsigned int off_bitset);
static void led_setonoff(unsigned int bitset);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint16_t g_ledbits[LED_NUM_CODES] =
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

/****************************************************************************
 * Name: phy_led_on
 *
 * Description:
 *   Performs the physical IO to turn On LEDs to the bit encoded state
 *
 ****************************************************************************/

static inline void phy_led_on(unsigned int on_bitset)
{
  /* All RGB LEDs are pulled up and, hence, active low */

  if ((on_bitset & BOARD_RED_LED_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED2, false);
    }

  if ((on_bitset & BOARD_GREEN_LED_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED4, false);
    }

  if ((on_bitset & BOARD_BLUE_LED_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED3, false);
    }

  /* USR LED is pulled down and, hence, active high */

  if ((on_bitset & BOARD_USR_LED_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED1, true);
    }
}

/****************************************************************************
 * Name: phy_led_off
 *
 * Description:
 *   Performs the physical IO to turn Off LEDs to the bit encoded state
 *
 ****************************************************************************/

static inline void phy_led_off(unsigned int off_bitset)
{
  /* All RGB LEDs are pulled up and, hence, active low */

  if ((off_bitset & BOARD_RED_LED_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED2, true);
    }

  if ((off_bitset & BOARD_GREEN_LED_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED4, true);
    }

  if ((off_bitset & BOARD_BLUE_LED_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED3, true);
    }

  /* USR LED is pulled down and, hence, active high */

  if ((off_bitset & BOARD_USR_LED_BIT) != 0)
    {
      stm32_gpiowrite(GPIO_LED1, false);
    }
}

/****************************************************************************
 * Name: led_setonoff
 *
 * Description:
 *   Set/clear all LEDs to the bit encoded state
 *
 ****************************************************************************/

static void led_setonoff(unsigned int bits)
{
  phy_led_off(CLRBITS(bits));
  phy_led_on(SETBITS(bits));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_led_initialize
 ****************************************************************************/

void board_led_initialize(void)
{
   /* Configure LED1-4 GPIOs for output */

   stm32_configgpio(GPIO_LED1);
   stm32_configgpio(GPIO_LED2);
   stm32_configgpio(GPIO_LED3);
   stm32_configgpio(GPIO_LED4);
}

/****************************************************************************
 * Name: board_led_on
 ****************************************************************************/

void board_led_on(int led)
{
  led_setonoff(ON_BITS(g_ledbits[led]));
}

/****************************************************************************
 * Name: board_led_off
 ****************************************************************************/

void board_led_off(int led)
{
  led_setonoff(OFF_BITS(g_ledbits[led]));
}

#endif /* CONFIG_ARCH_LEDS */
