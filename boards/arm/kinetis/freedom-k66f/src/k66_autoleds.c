/****************************************************************************
 * boards/arm/kinetis/freedom-k66f/src/k66_autoleds.c
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

/* The Freedom K66F has a single RGB LED driven by the K66F as follows:
 *
 *   LED    K66
 *   ------ -------------------------------------------------------
 *   RED    PTB22/SPI2_SOUT/FB_AD29/CMP2_OUT
 *   BLUE   PTB21/SPI2_SCK/FB_AD30/CMP1_OUT
 *   GREEN  PTE26/ENET_1588_CLKIN/UART4_CTS_b/RTC_CLKOUT/USB0_CLKIN
 *
 *
 * If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the Freedom K66F.  The following definitions describe how NuttX controls
 * the LEDs:
 *
 *   SYMBOL                Meaning                 LED state
 *                                                 RED   GREEN  BLUE
 *   -------------------  -----------------------  -----------------
 *   LED_STARTED          NuttX has been started    OFF  OFF  OFF
 *   LED_HEAPALLOCATE     Heap has been allocated   OFF  OFF  ON
 *   LED_IRQSENABLED      Interrupts enabled        OFF  OFF  ON
 *   LED_STACKCREATED     Idle stack created        OFF  ON   OFF
 *   LED_INIRQ            In an interrupt          (no change)
 *   LED_SIGNAL           In a signal handler      (no change)
 *   LED_ASSERTION        An assertion failed      (no change)
 *   LED_PANIC            The system has crashed    FLASH OFF OFF
 *   LED_IDLE             K66 is in sleep mode     (Optional, not used)
 */

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
#include "kinetis.h"
#include "freedom-k66f.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Summary of all possible settings */

#define LED_NOCHANGE      0 /* LED_IRQSENABLED, LED_INIRQ, LED_SIGNAL, LED_ASSERTION */
#define LED_OFF_OFF_OFF   1 /* LED_STARTED */
#define LED_OFF_OFF_ON    2 /* LED_HEAPALLOCATE */
#define LED_OFF_ON_OFF    3 /* LED_STACKCREATED */
#define LED_ON_OFF_OFF    4 /* LED_PANIC */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 *
 * Description:
 *   Initialize the on-board LED
 *
 ****************************************************************************/

void board_autoled_initialize(void)
{
  kinetis_pinconfig(GPIO_LED_R);
  kinetis_pinconfig(GPIO_LED_G);
  kinetis_pinconfig(GPIO_LED_B);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if (led != LED_NOCHANGE)
    {
      bool redoff   = true;
      bool greenoff = true;
      bool blueoff  = true;

      switch (led)
        {
          default:
          case LED_OFF_OFF_OFF:
            break;

          case LED_OFF_OFF_ON:
            blueoff = false;
            break;

          case LED_OFF_ON_OFF:
            greenoff = false;
            break;

          case LED_ON_OFF_OFF:
            redoff = false;
            break;
        }

      kinetis_gpiowrite(GPIO_LED_R, redoff);
      kinetis_gpiowrite(GPIO_LED_G, greenoff);
      kinetis_gpiowrite(GPIO_LED_B, blueoff);
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led == LED_ON_OFF_OFF)
    {
      kinetis_gpiowrite(GPIO_LED_R, true);
      kinetis_gpiowrite(GPIO_LED_G, true);
      kinetis_gpiowrite(GPIO_LED_B, true);
    }
}

#endif /* CONFIG_ARCH_LEDS */
