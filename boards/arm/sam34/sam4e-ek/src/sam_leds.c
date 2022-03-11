/****************************************************************************
 * boards/arm/sam34/sam4e-ek/src/sam_leds.c
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
#include "sam_gpio.h"
#include "sam4e-ek.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The SAM4E-EK board has three, user-controllable LEDs labelled D2 (blue),
 * D3 (amber), and D4 (green) on the board.  Usage of these LEDs is defined
 * in include/board.h and src/up_leds.c. They are encoded as follows:
 *
 *   SYMBOL              Meaning                 D3*     D2      D4
 *   ------------------- ----------------------- ------- ------- -------
 *   LED_STARTED         NuttX has been started  OFF     OFF     OFF
 *   LED_HEAPALLOCATE    Heap has been allocated OFF     OFF     ON
 *   LED_IRQSENABLED     Interrupts enabled      OFF     ON      OFF
 *   LED_STACKCREATED    Idle stack created      OFF     ON      ON
 *   LED_INIRQ           In an interrupt**       N/C     FLASH   N/C
 *   LED_SIGNAL          In a signal handler***  N/C     N/C     FLASH
 *   LED_ASSERTION       An assertion failed     FLASH   N/C     N/C
 *   LED_PANIC           The system has crashed  FLASH   N/C     N/C
 *
 *   * If D2 and D4 are statically on, then NuttX probably failed to boot
 *     and these LEDs will give you some indication of where the failure was
 *  ** The normal state is D3=OFF, D4=ON and D2 faintly glowing.  This faint
 *     glow is because of timer interrupts that result in the LED being
 *     illuminated on a small proportion of the time.
 * *** D4 may also flicker normally if signals are processed.
 */

#define LED_OFF        0
#define LED_ON         1
#define LED_NOCHANGE   2
#define LED_MASK       3

#define D3_SHIFT       0
#define D3_OFF         (LED_OFF << D3_SHIFT)
#define D3_ON          (LED_ON << D3_SHIFT)
#define D3_NOCHANGE    (LED_NOCHANGE << D3_SHIFT)
#define D2_SHIFT       2
#define D2_OFF         (LED_OFF << D2_SHIFT)
#define D2_ON          (LED_ON << D2_SHIFT)
#define D2_NOCHANGE    (LED_NOCHANGE << D2_SHIFT)
#define D4_SHIFT       4
#define D4_OFF         (LED_OFF << D4_SHIFT)
#define D4_ON          (LED_ON << D4_SHIFT)
#define D4_NOCHANGE    (LED_NOCHANGE << D4_SHIFT)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_ledon[8] =
{
  (D3_OFF      | D2_OFF      | D4_OFF),      /* LED_STARTED  */
  (D3_ON       | D2_OFF      | D4_ON),       /* LED_HEAPALLOCATE */
  (D3_OFF      | D2_ON       | D4_OFF),      /* LED_IRQSENABLED  */
  (D3_ON       | D2_ON       | D4_ON),       /* LED_STACKCREATED  */

  (D3_NOCHANGE | D2_OFF      | D4_NOCHANGE), /* LED_INIRQ  */
  (D3_NOCHANGE | D2_NOCHANGE | D4_OFF),      /* LED_SIGNAL  */
  (D3_ON       | D2_NOCHANGE | D4_NOCHANGE), /* LED_ASSERTION  */
  (D3_ON       | D2_NOCHANGE | D4_NOCHANGE)  /* LED_PANIC */
};

static const uint8_t g_ledoff[8] =
{
  (D3_OFF      | D2_OFF      | D4_OFF),      /* LED_STARTED (does not happen) */
  (D3_ON       | D2_OFF      | D4_ON),       /* LED_HEAPALLOCATE (does not happen) */
  (D3_OFF      | D2_ON       | D4_OFF),      /* LED_IRQSENABLED (does not happen) */
  (D3_ON       | D2_ON       | D4_ON),       /* LED_STACKCREATED (does not happen) */

  (D3_NOCHANGE | D2_ON       | D4_NOCHANGE), /* LED_INIRQ  */
  (D3_NOCHANGE | D2_NOCHANGE | D4_ON),       /* LED_SIGNAL */
  (D3_OFF      | D2_NOCHANGE | D4_NOCHANGE), /* LED_ASSERTION */
  (D3_OFF      | D2_NOCHANGE | D4_NOCHANGE)  /* LED_PANIC */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_setled
 ****************************************************************************/

static void sam_setled(gpio_pinset_t pinset, uint8_t state)
{
  /* Assume active high.  Initial state == 0 means active high */

  bool polarity = ((pinset & GPIO_OUTPUT_SET) == 0);
  switch (state)
    {
      case LED_OFF:
        polarity = !polarity;

      case LED_ON:
        break;

      case LED_NOCHANGE:
      default:
        return;
    }

  sam_gpiowrite(pinset, polarity);
}

/****************************************************************************
 * Name: sam_setleds
 ****************************************************************************/

static void sam_setleds(uint8_t state)
{
  sam_setled(GPIO_D3, (state >> D3_SHIFT) & LED_MASK);
  sam_setled(GPIO_D2, (state >> D2_SHIFT) & LED_MASK);
  sam_setled(GPIO_D4, (state >> D4_SHIFT) & LED_MASK);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  sam_configgpio(GPIO_D3);
  sam_configgpio(GPIO_D2);
  sam_configgpio(GPIO_D4);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  sam_setleds(g_ledon[led & 7]);
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  sam_setleds(g_ledoff[led & 7]);
}

#endif /* CONFIG_ARCH_LEDS */
