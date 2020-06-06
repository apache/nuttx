/****************************************************************************
 *  boards/arm/sama5/sama5d2-xult/src/sam_userleds.c
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

/* There is an RGB LED on board the SAMA5D2-XULT.  The RED component is
 * driven by the SDHC_CD pin (PA13) and so will not be used.  The LEDs are
 * provided VDD_LED and so bringing the LED low will will illuminated the
 * LED.
 *
 *   ------------------------------ ------------------- ---------------------
 *   SAMA5D2 PIO                    SIGNAL              USAGE
 *   ------------------------------ ------------------- ---------------------
 *   PA13                           SDHC_CD_PA13        Red LED
 *   PB5                            LED_GREEN_PB5       Green LED
 *   PB0                            LED_BLUE_PB0        Blue LED
 *   ------------------------------ ------------------- ---------------------
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
#include "sama5d2-xult.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure LED PIOs for output */

#ifndef CONFIG_ARCH_LEDS
  sam_configpio(PIO_LED_GREEN);
#endif
  sam_configpio(PIO_LED_BLUE);
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  uint32_t ledcfg;

#ifndef CONFIG_ARCH_LEDS
  if (led == BOARD_GREEN)
    {
      ledcfg = PIO_LED_GREEN;
    }
  else
#endif
  if (led == BOARD_BLUE)
    {
      ledcfg = PIO_LED_BLUE;
    }
  else
    {
      return;
    }

  /* Low illuminates */

  sam_piowrite(ledcfg, !ledon);
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  bool ledon;

#ifndef CONFIG_ARCH_LEDS
  /* Low illuminates */

  ledon = ((ledset & BOARD_GREEN_BIT) == 0);
  sam_piowrite(PIO_LED_GREEN, ledon);
#endif

  /* Low illuminates */

  ledon = ((ledset &BOARD_BLUE_BIT) != 0);
  sam_piowrite(PIO_LED_BLUE, ledon);
}
