/****************************************************************************
 * boards/arm/efm32/efm32gg-stk3700/src/efm32_autoleds.c
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

/* The EFM32 Giant Gecko Start Kit has two yellow LEDs marked LED0 and LED1.
 * These LEDs are controlled by GPIO pins on the EFM32.  The LEDs are
 * connected to pins PE2 and PE3 in an active high configuration:
 *
 * ------------------------------------- --------------------
 * EFM32 PIN                             BOARD SIGNALS
 * ------------------------------------- --------------------
 * E2/BCK_VOUT/EBI_A09 #0/               MCU_PE2 UIF_LED0
 *   TIM3_CC2 #1/U1_TX #3/ACMP0_O #1
 * E3/BCK_STAT/EBI_A10 #0/U1_RX #3/      MCU_PE3 UIF_LED1
 *   ACMP1_O #1
 * ------------------------------------- --------------------
 *
 * All LEDs are grounded and so are illuminated by outputting a high
 * value to the LED.
 *
 * These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/efm32_autoleds.c.  The LEDs are used to
 * encode OS-related events as follows:
 *
 *   SYMBOL               Meaning                      LED state
 *                                                   LED0     LED1
 *   -------------------  -----------------------  -------- --------
 *   LED_STARTED          NuttX has been started     OFF      OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF      OFF
 *   LED_STACKCREATED     Idle stack created         ON       OFF
 *   LED_INIRQ            In an interrupt              No change
 *   LED_SIGNAL           In a signal handler          No change
 *   LED_ASSERTION        An assertion failed          No change
 *   LED_PANIC            The system has crashed     OFF      Blinking
 *   LED_IDLE             STM32 is is sleep mode       Not used
 *
 * Thus if LED0 statically on, NuttX has successfully booted and is,
 * apparently, running normally.  If LED1 is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
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

#include "efm32_gpio.h"
#include "efm32gg-stk3700.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED PIOs for output */

  efm32_configgpio(GPIO_LED0);
  efm32_configgpio(GPIO_LED1);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  bool led0on = false; /* High illuminates */
  bool led1on = false; /* High illuminates */

  switch (led)
    {
      case 0:  /* LED_STARTED, LED_HEAPALLOCATE, LED_IRQSENABLED */
        break;

      case 1:  /* LED_STACKCREATED */
        led0on = true;
        break;

      default:
      case 2:  /* LED_INIRQ, LED_SIGNAL, LED_ASSERTION */
        return;

      case 3:  /* LED_PANIC */
        led1on = true;
        break;
    }

  efm32_gpiowrite(GPIO_LED0, led0on);
  efm32_gpiowrite(GPIO_LED1, led1on);
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led != 2)
    {
      efm32_gpiowrite(GPIO_LED0, false);  /* High illuminates */
      efm32_gpiowrite(GPIO_LED1, false);  /* High illuminates */
    }
}

#endif /* CONFIG_ARCH_LEDS */
