/****************************************************************************
 * boards/arm/stm32/clicker2-stm32/src/stm32_autoleds.c
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

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 2 LEDs on
 * board the Clicker2 for STM32.  The following definitions describe how
 * NuttX controls the LEDs:
 *
 *   SYMBOL               Meaning                      LED state
 *                                                   LED1     LED2
 *   -------------------  -----------------------  -------- --------
 *   LED_STARTED          NuttX has been started     OFF      OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF      OFF
 *   LED_STACKCREATED     Idle stack created         ON       OFF
 *   LED_INIRQ            In an interrupt            N/C      ON
 *   LED_SIGNAL           In a signal handler          No change
 *   LED_ASSERTION        An assertion failed          No change
 *   LED_PANIC            The system has crashed     OFF      Blinking
 *   LED_IDLE             STM32 is is sleep mode       Not used
 *
 *   VALUE
 *   --------------------------------------------  -------- --------
 *   0                                               OFF      OFF
 *   1                                               ON       OFF
 *   2                                               N/C      ON
 *   3                                               N/C      N/C
 *   4                                               OFF      ON
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "stm32.h"
#include "clicker2-stm32.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void board_led1_on(int led)
{
  bool ledon = false;

  switch (led)
    {
      case 0:           /* LED1=OFF */
      case 4:           /* LED1=OFF */
        break;

      case 1:           /* LED1=ON */
        ledon  = true;
        break;

      case 2:           /* LED1=N/C */
      case 3:           /* LED1=N/C */
      default:
        return;
    }

  stm32_gpiowrite(GPIO_LED1, ledon);
}

static void board_led2_on(int led)
{
  bool ledon = false;

  switch (led)
    {
      case 0:           /* LED2=OFF */
      case 1:           /* LED2=OFF */
        break;

      case 2:           /* LED2=ON */
      case 4:           /* LED2=ON */
        ledon  = true;
        break;

      case 3:           /* LED2=N/C */
      default:
        return;
    }

  stm32_gpiowrite(GPIO_LED2, ledon);
}

static void board_led1_off(int led)
{
  switch (led)
    {
      case 0:           /* LED1=OFF */
      case 1:           /* LED1=OFF */
      case 4:           /* LED1=OFF */
        break;

      case 2:           /* LED1=N/C */
      case 3:           /* LED1=N/C */
      default:
        return;
    }

  stm32_gpiowrite(GPIO_LED1, false);
}

static void board_led2_off(int led)
{
  switch (led)
    {
      case 0:           /* LED2=OFF */
      case 1:           /* LED2=OFF */
      case 2:           /* LED2=OFF */
      case 4:           /* LED2=OFF */
        break;

      case 3:           /* LED2=N/C */
      default:
        return;
    }

  stm32_gpiowrite(GPIO_LED2, false);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED1-2 GPIOs for output */

  stm32_configgpio(GPIO_LED1);
  stm32_configgpio(GPIO_LED2);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  board_led1_on(led);
  board_led2_on(led);
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  board_led1_off(led);
  board_led2_off(led);
}

#endif /* CONFIG_ARCH_LEDS */
