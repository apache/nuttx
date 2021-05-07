/****************************************************************************
 * boards/arm/sama5/sama5d3x-ek/src/sam_userleds.c
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

/* There are two LEDs on the SAMA5D3 series-CM board that can be controlled
 * by software.  A  blue LED is controlled via PIO pins.  A red LED normally
 * provides an indication that power is supplied to the board but can also
 * be controlled via software.
 *
 *   PE25.  This blue LED is pulled high and is illuminated by pulling PE25
 *   low.
 *
 *   PE24.  The red LED is also pulled high but is driven by a transistor so
 *   that it is illuminated when power is applied even if PE24 is not
 *   configured as an output.  If PE24 is configured as an output, then the
 *   LCD is illuminated by a low output.
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
#include "sama5d3x-ek.h"

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

  sam_configpio(PIO_BLUE);
#ifndef CONFIG_SAMA5D3XEK_NOREDLED
  sam_configpio(PIO_RED);
#endif
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

      ledcfg = PIO_BLUE;
      ledon  = !ledon;
    }
#ifndef CONFIG_SAMA5D3XEK_NOREDLED
  else if (led == BOARD_RED)
    {
      /* High illuminates */

      ledcfg = PIO_RED;
    }
#endif
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
  sam_piowrite(PIO_BLUE, ledon);

#ifndef CONFIG_SAMA5D3XEK_NOREDLED
  /* High illuminates */

  ledon = ((ledset & BOARD_RED_BIT) != 0);
  sam_piowrite(PIO_RED, ledon);
#endif
}

#endif /* !CONFIG_ARCH_LEDS */
