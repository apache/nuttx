/****************************************************************************
 * boards/arm/tiva/tm4c1294-launchpad/src/tm4c_autoleds.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "tiva_gpio.h"
#include "tm4c1294-launchpad.h"

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
