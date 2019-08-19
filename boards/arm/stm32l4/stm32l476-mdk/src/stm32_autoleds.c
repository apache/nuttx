/****************************************************************************
 * boards/arm/stm32l4/stm32l476-mdk/src/sam_autoleds.c
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

/* The Reference Moto Mod contains three LEDs.  Two LEDs, are by convention,
 * used to indicate the Reference Moto Mod battery state of charge, and the
 * other is available for you to use in your applications.
 *
 *   1. The red LED on PD7.  Part of the (rear-firing) red/green LED.
 *   2. The green LED on PE7.  Part of the (rear-firing) red/green LED.
 *   3. The white (top-firing) LED on PE8
 *
 * When the I/O is HIGH value, the LED is OFF.
 * When the I/O is LOW, the LED is ON.
 *
 * Following this convention, only the white LED is made available even though
 * they all could be user-application controlled if desired.
 *
 * None of the LEDs are used by the board port unless CONFIG_ARCH_LEDS is defined.
 * In that case, the white LED (only) will be controlled.  Usage by the board port
 * is defined in include/board.h and src/stm32_autoleds.c.  The white LED will be
 * used to encode OS-related events as follows:
 *
 *   ------------------ ------------------------ ------
 *   SYMBOL             Meaning                  LED
 *   ------------------ ------------------------ ------
 *
 *   LED_STARTED        NuttX has been started   OFF
 *   LED_HEAPALLOCATE   Heap has been allocated  OFF
 *   LED_IRQSENABLED    Interrupts enabled       OFF
 *   LED_STACKCREATED   Idle stack created       ON
 *   LED_INIRQ          In an interrupt          N/C
 *   LED_SIGNAL         In a signal handler      N/C
 *   LED_ASSERTION      An assertion failed      N/C
 *   LED_PANIC          The system has crashed   FLASH
 *   LED_IDLE           MCU is is sleep mode     Not used
 *
 * Thus if the white LED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If white LED is flashing at approximately 2Hz,
 * then a fatal error has been detected and the system has halted.
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

#include "stm32l4_gpio.h"
#include "stm32l476-mdk.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED GPIOs for output */

  stm32l4_configgpio(GPIO_LED_RED);
  stm32l4_configgpio(GPIO_LED_GREEN);
  stm32l4_configgpio(GPIO_LED_WHITE);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if (led == 1 || led == 3)
    {
      stm32l4_gpiowrite(GPIO_LED_WHITE, false); /* Low illuminates */
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led == 3)
    {
      stm32l4_gpiowrite(GPIO_LED_WHITE, true);  /* High extinguishes */
    }
}

#endif /* CONFIG_ARCH_LEDS */
