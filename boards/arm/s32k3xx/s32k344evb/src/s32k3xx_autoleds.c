/****************************************************************************
 * boards/arm/s32k3xx/s32k344evb/src/s32k3xx_autoleds.c
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

/* Copyright 2022 NXP */

/* The S32K344EVB has two RGB LEDs:
 *
 *   RedLED0    PTA29  (EMIOS1 CH12 / EMIOS2 CH12)
 *   GreenLED0  PTA30  (EMIOS1 CH13 / EMIOS2 CH13)
 *   BlueLED0   PTA31  (EMIOS1 CH14 / FXIO D0)
 *
 *   RedLED1    PTB18  (EMIOS1 CH15 / EMIOS2 CH14 / FXIO D1)
 *   GreenLED1  PTB25  (EMIOS1 CH21 / EMIOS2 CH21 / FXIO D6)
 *   BlueLED1   PTE12  (EMIOS1 CH5  / FXIO D8)
 *
 * An output of '1' illuminates the LED.
 *
 * If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the S32K344EVB.  The following definitions describe how NuttX controls the
 * LEDs:
 *
 *   SYMBOL            Meaning                    LED state
 *                                                RED    GREEN  BLUE
 *   ----------------  ------------------------  --------------------
 *   LED_STARTED       NuttX has been started     OFF    OFF    OFF
 *   LED_HEAPALLOCATE  Heap has been allocated    OFF    OFF    ON
 *   LED_IRQSENABLED   Interrupts enabled         OFF    OFF    ON
 *   LED_STACKCREATED  Idle stack created         OFF    ON     OFF
 *   LED_INIRQ         In an interrupt           (No change)
 *   LED_SIGNAL        In a signal handler       (No change)
 *   LED_ASSERTION     An assertion failed       (No change)
 *   LED_PANIC         The system has crashed     FLASH  OFF    OFF
 *   LED_IDLE          S32K344 is in sleep mode  (Optional, not used)
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/board.h>

#include "s32k3xx_pin.h"

#include "s32k344evb.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Summary of all possible settings */

#define LED_NOCHANGE     0 /* LED_IRQSENABLED, LED_INIRQ, LED_SIGNAL, LED_ASSERTION */
#define LED_OFF_OFF_OFF  1 /* LED_STARTED */
#define LED_OFF_OFF_ON   2 /* LED_HEAPALLOCATE */
#define LED_OFF_ON_OFF   3 /* LED_STACKCREATED */
#define LED_ON_OFF_OFF   4 /* LED_PANIC */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED GPIOs for output */

  s32k3xx_pinconfig(GPIO_LED0_R);
  s32k3xx_pinconfig(GPIO_LED0_G);
  s32k3xx_pinconfig(GPIO_LED0_B);

  s32k3xx_pinconfig(GPIO_LED1_R);
  s32k3xx_pinconfig(GPIO_LED1_G);
  s32k3xx_pinconfig(GPIO_LED1_B);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if (led != LED_NOCHANGE)
    {
      bool redon   = false;
      bool greenon = false;
      bool blueon  = false;

      switch (led)
        {
          default:
          case LED_OFF_OFF_OFF:
            break;

          case LED_OFF_OFF_ON:
            blueon = true;
            break;

          case LED_OFF_ON_OFF:
            greenon = true;
            break;

          case LED_ON_OFF_OFF:
            redon = true;
            break;
        }

      /* An output of '1' illuminates the LED */

      s32k3xx_gpiowrite(GPIO_LED0_R, redon);
      s32k3xx_gpiowrite(GPIO_LED0_G, greenon);
      s32k3xx_gpiowrite(GPIO_LED0_B, blueon);

      s32k3xx_gpiowrite(GPIO_LED1_R, redon);
      s32k3xx_gpiowrite(GPIO_LED1_G, greenon);
      s32k3xx_gpiowrite(GPIO_LED1_B, blueon);
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led == LED_ON_OFF_OFF)
    {
      /* An output of '1' illuminates the LED */

      s32k3xx_gpiowrite(GPIO_LED0_R, true);
      s32k3xx_gpiowrite(GPIO_LED0_G, false);
      s32k3xx_gpiowrite(GPIO_LED0_B, false);

      s32k3xx_gpiowrite(GPIO_LED1_R, true);
      s32k3xx_gpiowrite(GPIO_LED1_G, false);
      s32k3xx_gpiowrite(GPIO_LED1_B, false);
    }
}

#endif /* CONFIG_ARCH_LEDS */
