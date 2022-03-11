/****************************************************************************
 * boards/mips/pic32mz/pic32mz-starterkit/src/pic32mz_userleds.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "mips_internal.h"
#include "pic32mz_gpio.h"
#include "pic32mz-starterkit.h"

#ifndef CONFIG_ARCH_LEDS

/* LED Configuration ********************************************************/

/* The PIC32MZ Ethernet Starter kit has 3 user LEDs labelled LED1-3 on the
 * board:
 *
 *   PIN  LED   Notes
 *   ---  ----- -------------------------
 *   RH0  LED1  High illuminates (RED)
 *   RH1  LED3  High illuminates (YELLOW)
 *   RH2  LED2  High illuminates (GREEN)
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user is free to control the
 * LEDs through the functions provided in this file
 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The following array simply maps the PIC32MZ_STARTERKIT_LEDn index values
 * to the correct LED pin configuration.
 */

static const pinset_t g_ledpincfg[PIC32MZ_STARTERKIT_NLEDS] =
{
  GPIO_LED_1, GPIO_LED_2, GPIO_LED_3
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure output pins */

  pic32mz_configgpio(GPIO_LED_1);
  pic32mz_configgpio(GPIO_LED_2);
  pic32mz_configgpio(GPIO_LED_3);
  return 3;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  if ((unsigned)led < PIC32MZ_STARTERKIT_NLEDS)
    {
      pic32mz_gpiowrite(g_ledpincfg[led], ledon);
    }
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  board_userled(PIC32MZ_STARTERKIT_LED1,
                (ledset & PIC32MZ_STARTERKIT_LED1_BIT) != 0);
  board_userled(PIC32MZ_STARTERKIT_LED2,
                (ledset & PIC32MZ_STARTERKIT_LED2_BIT) != 0);
  board_userled(PIC32MZ_STARTERKIT_LED3,
                (ledset & PIC32MZ_STARTERKIT_LED3_BIT) != 0);
}

#endif /* !CONFIG_ARCH_LEDS */
