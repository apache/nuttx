/****************************************************************************
 * boards/arm/sama5/sama5d4-ek/src/sam_userleds.c
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
 * -------------------------- ------------------- -----------------------
 * SAMA5D4 PIO                SIGNAL              USAGE
 * -------------------------- ------------------- -----------------------
 * PE28/NWAIT/RTS4/A19        1Wire_PE28          1-WIRE ROM, LCD, D8 (green)
 * PE8/A8/TCLK3/PWML3         LED_USER_PE8        LED_USER (D10)
 * PE9/A9/TIOA2               LED_POWER_PE9       LED_POWER (D9, Red)
 * -------------------------- ------------------- -----------------------
 *
 * - D8: D8 is shared with other functions and cannot be used if the
 *   1-Wire ROM is used.
 *   I am not sure of the LCD function, but the LED may not be available
 *   if the LCD is used either.  We will avoid using D8 just for simplicity.
 * - D10:  Nothing special here.  A low output illuminates.
 * - D9: The Power ON LED.  Connects to the via an IRLML2502 MOSFET.
 *       This LED will be on when power is applied but otherwise,
 *       I think it works like D10.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "sam_pio.h"
#include "sama5d4-ek.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure LED PIOs for output */

  sam_configpio(PIO_LED_USER);
  sam_configpio(PIO_LED_POWER);
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  uint32_t ledcfg;

  if (led == BOARD_BLUE)
    {
      /* Low illuminates */

      ledcfg = PIO_LED_USER;
      ledon  = !ledon;
    }
  else if (led == BOARD_RED)
    {
      /* High illuminates */

      ledcfg = PIO_LED_POWER;
    }
  else
    {
      return;
    }

  sam_piowrite(ledcfg, ledon);
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  bool ledon;

  /* Low illuminates */

  ledon = ((ledset & BOARD_BLUE_BIT) == 0);
  sam_piowrite(PIO_LED_USER, ledon);

  /* High illuminates */

  ledon = ((ledset & BOARD_RED_BIT) != 0);
  sam_piowrite(PIO_LED_POWER, ledon);
}

#endif /* !CONFIG_ARCH_LEDS */
