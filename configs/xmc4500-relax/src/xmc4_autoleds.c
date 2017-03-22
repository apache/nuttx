/****************************************************************************
 * configs/xmc4500-relax/include/xmc4_autoleds.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

/* The XMC4500 Relax Lite v1 board has two LEDs:
 *
 * LED1 P1.1 High output illuminates
 * LED2 P1.0 High output illuminates
 *
 * These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_autoleds.c. The LEDs are used to encode
 * OS-related events as follows:
 *
 *   SYMBOL              Meaning                  LED state
 *                                               LED1   LED2
 *   ------------------ ------------------------ ------ ------
 *   LED_STARTED        NuttX has been started   OFF    OFF
 *   LED_HEAPALLOCATE   Heap has been allocated  OFF    OFF
 *   LED_IRQSENABLED    Interrupts enabled       OFF    OFF
 *   LED_STACKCREATED   Idle stack created       ON     OFF
 *   LED_INIRQ          In an interrupt           No change
 *   LED_SIGNAL         In a signal handler       No change
 *   LED_ASSERTION      An assertion failed       No change
 *   LED_PANIC          The system has crashed   N/C  Blinking
 *   LED_IDLE           MCU is is sleep mode      Not used
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "xmc4_gpio.h"
#include "xmc4500-relax.h"

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
        break;

      case 1:           /* LED1=ON */
        ledon  = true;
        break;

      case 2:           /* LED1=N/C */
      case 3:           /* LED1=N/C */
      default:
        return;
    }

  xmc4_gpio_write(GPIO_LED1, ledon);
}

static void board_led2_on(int led)
{
  bool ledon = false;

  switch (led)
    {
      case 0:           /* LED2=OFF */
      case 1:           /* LED2=OFF */
        break;

      case 3:           /* LED2=ON */
        ledon  = true;
        break;

      case 2:           /* LED2=N/C */
      default:
        return;
    }

  xmc4_gpio_write(GPIO_LED2, ledon);
}

static void board_led1_off(int led)
{
  switch (led)
    {
      case 0:           /* LED1=OFF */
      case 1:           /* LED1=OFF */
        break;

      case 2:           /* LED1=N/C */
      case 3:           /* LED1=N/C */
      default:
        return;
    }

  xmc4_gpio_write(GPIO_LED1, false);
}

static void board_led2_off(int led)
{
  switch (led)
    {
      case 0:           /* LED2=OFF */
      case 1:           /* LED2=OFF */
      case 3:           /* LED2=OFF */
        break;

      case 2:           /* LED2=N/C */
      default:
        return;
    }

   xmc4_gpio_write(GPIO_LED2, false);
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

  (void)xmc4_gpio_config(GPIO_LED1);
  (void)xmc4_gpio_config(GPIO_LED2);
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
