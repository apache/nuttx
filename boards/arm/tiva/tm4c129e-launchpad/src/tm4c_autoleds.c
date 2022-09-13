/****************************************************************************
 * boards/arm/tiva/tm4c129e-launchpad/src/tm4c_autoleds.c
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

#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "tiva_gpio.h"
#include "tm4c129e-launchpad.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tm4c_led_initialize
 *
 * Description:
 *   Called to initialize the on-board LEDs.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void tm4c_led_initialize(void)
{
  /* Configure LED PIOs for output */

  tiva_configgpio(GPIO_LED_D1);
  tiva_configgpio(GPIO_LED_D2);
  tiva_configgpio(GPIO_LED_D3);
  tiva_configgpio(GPIO_LED_D4);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
/* --------------- ------- ----- ----- ----- ----- ----------
 * STATE           VALUE   LED1  LED2  LED3  LED4
 * --------------- ------- ----- ----- ----- ----- ----------
 * LED_STARTED       1     ON    OFF   NC    NC
 * LED_HEAPALLOCATE  0     NC    NC    NC    NC
 * LED_IRQSENABLED   0     NC    NC    NC    NC
 * LED_STACKCREATED  2     ON    ON    NC    NC
 * LED_INIRQ         0     NC    NC    NC    NC
 * LED_SIGNAL        0     NC    NC    NC    NC
 * LED_ASSERTION     0     NC    NC    NC    NC
 * LED_PANIC         3     OFF   ON    NC    NC  (flashing 2Hz)
 * --------------- ------- ----- ----- ----- ---------------
 *
 * A high output illuminates the LED.
 */

  switch (led)
  {
  default:
  case 0: /* No change */
    break;

  case 1: /* LED1=OFF, LED2=OFF, LED3=NC,  LED4=NC  */
    tiva_gpiowrite(GPIO_LED_D1, true);
    tiva_gpiowrite(GPIO_LED_D2, false);
    break;

  case 2: /* LED1=ON,  LED2=ON,  LED3=NC,  LED4=NC */
    tiva_gpiowrite(GPIO_LED_D1, true);
    tiva_gpiowrite(GPIO_LED_D2, true);
    break;

  case 3: /* LED1=OFF, LED2=ON,  LED3=NC,  LED4=NC */
    tiva_gpiowrite(GPIO_LED_D1, false);
    tiva_gpiowrite(GPIO_LED_D2, true);
    break;
  }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
/* --------------- ------- ----- ----- ----- ----- ----------
 * STATE           VALUE   LED1  LED2  LED3  LED4
 * --------------- ------- ----- ----- ----- ----- ----------
 * LED_STARTED       1     ON    OFF   NC    NC
 * LED_HEAPALLOCATE  0     NC    NC    NC    NC
 * LED_IRQSENABLED   0     NC    NC    NC    NC
 * LED_STACKCREATED  2     ON    ON    NC    NC
 * LED_INIRQ         0     NC    NC    NC    NC
 * LED_SIGNAL        0     NC    NC    NC    NC
 * LED_ASSERTION     0     NC    NC    NC    NC
 * LED_PANIC         3     ON    OFF   NC    NC  (flashing 2Hz)
 * --------------- ------- ----- ----- ----- ---------------
 *
 * A high output illuminates the LED.
 */

  switch (led)
  {
  case 0: /* No change */
  case 1: /* Will not happen */
  case 2: /* Will not happen */
  default:
    break;

  case 3: /* LED1=ON,  LED2=OFF, LED3=NC, LED4=NC */
    tiva_gpiowrite(GPIO_LED_D1, true);
    tiva_gpiowrite(GPIO_LED_D2, false);
    break;
  }
}

#endif /* CONFIG_ARCH_LEDS */
