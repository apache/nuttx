/****************************************************************************
 * boards/arm/lpc17xx_40xx/lx_cpu/src/lpc17_40_autoleds.c
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
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "lpc17_40_gpio.h"
#include "lx_cpu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 *
 * LED1 -- Connected to P1[29]
 * LED2 -- Connected to P0[16]
 *
 * These LEDs are connected to ground so a high output value will illuminate
 * them.
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
 *   LED_INIRQ                  4  LED3 glows, on while in interrupt
 *   LED_SIGNAL                 4  LED3 glows, on while in signal handler
 *   LED_ASSERTION              4  LED3 glows, on while in assertion
 *   LED_PANIC                  4  LED3 Flashes at 2Hz
 *   LED_IDLE                   5  LED glows, ON while sleeping
 */

/* The following definitions map the encoded LED setting to GPIO settings */

#define LX_CPU_LED1     (1 << 0)
#define LX_CPU_LED2     (1 << 1)

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
#define LED_STARTED_ON_CLRBITS       ((LX_CPU_LED1|LX_CPU_LED2) << ON_CLRBITS_SHIFT)
#define LED_STARTED_OFF_SETBITS      (0 << OFF_SETBITS_SHIFT)
#define LED_STARTED_OFF_CLRBITS      ((LX_CPU_LED1|LX_CPU_LED2) << OFF_CLRBITS_SHIFT)

#define LED_HEAPALLOCATE_ON_SETBITS  ((LX_CPU_LED1) << ON_SETBITS_SHIFT)
#define LED_HEAPALLOCATE_ON_CLRBITS  ((LX_CPU_LED2) << ON_CLRBITS_SHIFT)
#define LED_HEAPALLOCATE_OFF_SETBITS (0 << OFF_SETBITS_SHIFT)
#define LED_HEAPALLOCATE_OFF_CLRBITS ((LX_CPU_LED1|LX_CPU_LED2) << OFF_CLRBITS_SHIFT)

#define LED_IRQSENABLED_ON_SETBITS   ((LX_CPU_LED2) << ON_SETBITS_SHIFT)
#define LED_IRQSENABLED_ON_CLRBITS   ((LX_CPU_LED1) << ON_CLRBITS_SHIFT)
#define LED_IRQSENABLED_OFF_SETBITS  ((LX_CPU_LED1) << OFF_SETBITS_SHIFT)
#define LED_IRQSENABLED_OFF_CLRBITS  ((LX_CPU_LED2) << OFF_CLRBITS_SHIFT)

#define LED_STACKCREATED_ON_SETBITS  ((LX_CPU_LED1|LX_CPU_LED2) << ON_SETBITS_SHIFT)
#define LED_STACKCREATED_ON_CLRBITS  ((0) << ON_CLRBITS_SHIFT)
#define LED_STACKCREATED_OFF_SETBITS ((LX_CPU_LED2) << OFF_SETBITS_SHIFT)
#define LED_STACKCREATED_OFF_CLRBITS ((LX_CPU_LED1) << OFF_CLRBITS_SHIFT)

#define LED_EVENT_ON_SETBITS         ((0) << ON_SETBITS_SHIFT)
#define LED_EVENT_ON_CLRBITS         ((0) << ON_CLRBITS_SHIFT)
#define LED_EVENT_OFF_SETBITS        ((0) << OFF_SETBITS_SHIFT)
#define LED_EVENT_OFF_CLRBITS        ((0) << OFF_CLRBITS_SHIFT)

#define LED_IDLE_ON_SETBITS          ((0) << ON_SETBITS_SHIFT)
#define LED_IDLE_ON_CLRBITS          ((0) << ON_CLRBITS_SHIFT)
#define LED_IDLE_OFF_SETBITS         ((0) << OFF_SETBITS_SHIFT)
#define LED_IDLE_OFF_CLRBITS         ((0) << OFF_CLRBITS_SHIFT)

/* Dump GPIO registers */

#ifdef CONFIG_DEBUG_LEDS_INFO
#  define led_dumpgpio(m) lpc17_40_dumpgpio(???, m)
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
  if ((clrbits & LX_CPU_LED1) != 0)
    {
      lpc17_40_gpiowrite(GPIO_LED1, false);
    }

  if ((clrbits & LX_CPU_LED2) != 0)
    {
      lpc17_40_gpiowrite(GPIO_LED2, false);
    }
}

static inline void led_setbits(unsigned int setbits)
{
  if ((setbits & LX_CPU_LED1) != 0)
    {
      lpc17_40_gpiowrite(GPIO_LED1, true);
    }

  if ((setbits & LX_CPU_LED2) != 0)
    {
      lpc17_40_gpiowrite(GPIO_LED2, true);
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
  /* Configure LED1-4 GPIOs for output */

  lpc17_40_configgpio(GPIO_LED1);
  lpc17_40_configgpio(GPIO_LED2);
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
