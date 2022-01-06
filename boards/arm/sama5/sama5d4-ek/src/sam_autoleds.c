/****************************************************************************
 * boards/arm/sama5/sama5d4-ek/src/sam_autoleds.c
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

/* There are 3 LEDs on the SAMA5D4-EK:
 *
 * --------------------------- ----------------- -----------------------
 * SAMA5D4 PIO                 SIGNAL            USAGE
 * --------------------------- ----------------- -----------------------
 * PE28/NWAIT/RTS4/A1          1Wire_PE28        1-WIRE ROM, LCD, D8 (green)
 * PE8/A8/TCLK3/PWML3          LED_USER_PE8      LED_USER (D10)
 * PE9/A9/TIOA2                LED_POWER_PE9     LED_POWER (D9, Red)
 * --------------------------- ----------------- -----------------------
 *
 * - D8: D8 is shared with other functions and cannot be used if the
 *   1-Wire ROM is used.
 *   I am not sure of the LCD function, but the LED may not be available
 *   if the LCD is used either.  We will avoid using D8 just for simplicity.
 * - D10:  Nothing special here.  A low output illuminates.
 * - D9: The Power ON LED.  Connects to the via an IRLML2502 MOSFET.
 *   This LED will be on when power is applied but otherwise a low output
 *   value will turn it off.
 *
 * These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *   SYMBOL                Meaning                     LED state
 *                                                 USER D10 POWER D9
 *   -------------------  -----------------------  -------- --------
 *   LED_STARTED          NuttX has been started     OFF      ON
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF      ON
 *   LED_IRQSENABLED      Interrupts enabled         OFF      ON
 *   LED_STACKCREATED     Idle stack created         ON       ON
 *   LED_INIRQ            In an interrupt              No change
 *   LED_SIGNAL           In a signal handler          No change
 *   LED_ASSERTION        An assertion failed          No change
 *   LED_PANIC            The system has crashed     OFF      Blinking
 *   LED_IDLE             MCU is is sleep mode         Not used
 *
 * Thus if the D0 and D9 are statically on, NuttX has successfully booted and
 * is, apparently, running normally.  If the red D9 LED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
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

#include "sam_pio.h"
#include "sama5d4-ek.h"

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

  sam_configpio(PIO_LED_USER);
  sam_configpio(PIO_LED_POWER);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      default:
      case 0:  /* LED_STARTED, LED_HEAPALLOCATE, LED_IRQSENABLED */
        break; /* Leave USER LED off */

      case 1:   /* LED_STACKCREATED */
        {
          /* User LED is ON (Low illuminates) */

          sam_piowrite(PIO_LED_USER, false);
        }
        break;

      case 2:  /* LED_INIRQ, LED_SIGNAL, LED_ASSERTION */
        break; /* Ignored */

      case 3:   /* LED_PANIC */
        {
          /* Power LED is ON (High illuminates) */

          sam_piowrite(PIO_LED_POWER, true);
        }
        break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
      default:
      case 0:  /* LED_STARTED, LED_HEAPALLOCATE, LED_IRQSENABLED,  */
      case 1:  /* LED_STACKCREATED */
        break; /* Will not happen */

      case 2:  /* LED_INIRQ, LED_SIGNAL, LED_ASSERTION */
        break; /* Ignored */

      case 3:  /* LED_PANIC */
        {
          /* Power LED is OFF (High illuminates) */

          sam_piowrite(PIO_LED_POWER, false);
        }
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
