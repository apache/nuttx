/****************************************************************************
 * boards/mips/pic32mz-starterkit/src/pic32mz_userleds.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include "mips_arch.h"
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
