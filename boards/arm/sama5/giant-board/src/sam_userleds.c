/****************************************************************************
 *  boards/arm/sama5/giant-board/src/sam_userleds.c
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

/* There is an orange status LED on board the Giant Board
 * driven by pin (PA6).
 *
 *   ------------------------------ ------------------- ---------------------
 *   SAMA5D27 PIO                   SIGNAL              USAGE
 *   ------------------------------ ------------------- ---------------------
 *   PA6                            STATUS_LED          Orange LED
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
#include "giant-board.h"

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
  sam_configpio(PIO_LED_ORANGE);
#endif
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  uint32_t ledcfg;

  if (led == BOARD_ORANGE)
    {
      ledcfg = PIO_LED_ORANGE;
    }
  else
    {
      return;
    }

  /* High illuminates */

  sam_piowrite(ledcfg, ledon);
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  bool ledon;

  /* High illuminates */

  ledon = ((ledset & BOARD_ORANGE_BIT) != 0);
  sam_piowrite(PIO_LED_ORANGE, ledon);
}
