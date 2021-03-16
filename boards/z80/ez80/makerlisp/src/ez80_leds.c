/****************************************************************************
 * boards/z80/ez80/makerlisp/src/ez80_leds.c
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

/* The D3 GREEN LED is driven by an eZ80 GPI/O pin.  However, it has some
 * additional properties:
 *
 * 1. On input, it will be '1' if the I/O expansion board is present.
 * 2. Setting it to an output of '0' will generate a system reset.
 * 3. Setting it to an output of '1' will not only illuminate the LED
 *    take the card out of reset and enable power to the SD card slot.
 *
 * As a consequence, the GREEN LED will not be illuminated if SD card
 * support or SPI is disabled.  The only effect of CONFIG_ARCH_LEDS is that
 * the GREEN LED will turned off in the event of a crash.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "z80_internal.h"
#include "makerlisp.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if (led != 0)  /* LED_ASSERTION or LED_PANIC */
    {
      /* To be provided */
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  /* Ignored */
}

#endif /* CONFIG_ARCH_LEDS */
