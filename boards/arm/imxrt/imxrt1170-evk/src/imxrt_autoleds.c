/****************************************************************************
 * boards/arm/imxrt/imxrt1170-evk/src/imxrt_autoleds.c
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

/* There are five LED status indicators located on the EVK Board.
 * The functions of these LEDs include:
 *
 *   - Main Power Supply (D16)
 *     Green: DC 5V main supply is normal.
 *     Red:   J43 input voltage is over 5.6V.
 *     Off:   The board is not powered.
 *   - Reset LED - Red (D7)
 *   - OpenSDA LED - Red (D5)
 *   - USER LED 1 - Green (D6)
 *   - USER LED 2 - Red (D34)
 *
 * Only two LEDs, D6 & D34, are under software control.
 *
 * This LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/imxrt_autoleds.c. The LED is used to encode
 * OS-related events as follows:
 *
 *   -------------------- ----------------------- ------- -------
 *   SYMBOL               Meaning                  LED1    LED2
 *                                                 GREEN   RED
 *   -------------------- ----------------------- ------- -------
 *
 *   LED_STARTED       0  NuttX has been started   OFF     OFF
 *   LED_HEAPALLOCATE  0  Heap has been allocated  OFF     OFF
 *   LED_IRQSENABLED   0  Interrupts enabled       OFF     OFF
 *   LED_STACKCREATED  1  Idle stack created       ON      OFF
 *   LED_INIRQ         2  In an interrupt         (No change)
 *   LED_SIGNAL        2  In a signal handler     (No change)
 *   LED_ASSERTION     2  An assertion failed     (No change)
 *   LED_PANIC         3  The system has crashed   OFF     FLASH
 *   LED_IDLE             Not used                (Not used)
 *
 * Thus if the LED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If the LED is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/board.h>

#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"
#include <arch/board/board.h>
#include "imxrt1170-evk.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_autoled_initialize
 *
 * Description:
 *   Initialize NuttX-controlled LED logic
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imxrt_autoled_initialize(void)
{
  /* Configure LED GPIOs for output */

  imxrt_config_gpio(GPIO_LED1);
  imxrt_config_gpio(GPIO_LED2);
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *   Turn on the "logical" LED state
 *
 * Input Parameters:
 *   led - Identifies the "logical" LED state (see definitions in
 *         include/board.h)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_autoled_on(int led)
{
  bool greenon = true;
  bool redon   = true;

  switch (led)
    {
      case 0:  /* Both LEDs off */
        greenon = false;
        redon   = false;
        break;

      case 2:  /* No change */
        return;

      case 1:  /* Only green LED on */
        greenon = true;
        redon   = false;
        break;

      case 3:  /* Only red LED on */
        greenon = false;
        redon   = true;
        break;
    }

  imxrt_gpio_write(GPIO_LED1, greenon); /* High illuminates */
  imxrt_gpio_write(GPIO_LED2, redon);   /* High illuminates */
}

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *   Turn off the "logical" LED state
 *
 * Input Parameters:
 *   led - Identifies the "logical" LED state (see definitions in
 *         include/board.h)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
      case 0:  /* LED Off */
      case 1:  /* LED Off */
      case 3:  /* LED Off */
        break;

      case 2:  /* LED No change */
        return;
    }

  imxrt_gpio_write(GPIO_LED1, false); /* High illuminates */
  imxrt_gpio_write(GPIO_LED2, false); /* High illuminates */
}

#endif /* CONFIG_ARCH_LEDS */
