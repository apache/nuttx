/****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_leds.c
 *
 *   Copyright (C) 2011-2013, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 *    Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void led_clrbits(unsigned int clrbits)
{
  if ((clrbits & BOARD_LED1_BIT) != 0)
    {
      cxd56_gpio_write(GPIO_LED1, false);
    }

  if ((clrbits & BOARD_LED2_BIT) != 0)
    {
      cxd56_gpio_write(GPIO_LED2, false);
    }

  if ((clrbits & BOARD_LED3_BIT) != 0)
    {
      cxd56_gpio_write(GPIO_LED3, false);
    }

  if ((clrbits & BOARD_LED4_BIT) != 0)
    {
      cxd56_gpio_write(GPIO_LED4, false);
    }
}

static inline void led_setbits(unsigned int setbits)
{
  if ((setbits & BOARD_LED1_BIT) != 0)
    {
      cxd56_gpio_write(GPIO_LED1, true);
    }

  if ((setbits & BOARD_LED2_BIT) != 0)
    {
      cxd56_gpio_write(GPIO_LED2, true);
    }

  if ((setbits & BOARD_LED3_BIT) != 0)
    {
      cxd56_gpio_write(GPIO_LED3, true);
    }

  if ((setbits & BOARD_LED4_BIT) != 0)
    {
      cxd56_gpio_write(GPIO_LED4, true);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  cxd56_gpio_config(GPIO_LED1, false);
  cxd56_gpio_config(GPIO_LED2, false);
  cxd56_gpio_config(GPIO_LED3, false);
  cxd56_gpio_config(GPIO_LED4, false);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  led_clrbits(BOARD_LED1_BIT | BOARD_LED2_BIT |
              BOARD_LED3_BIT | BOARD_LED4_BIT);
  led_setbits(led);
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  led_clrbits(led);
}

#endif /* CONFIG_ARCH_LEDS */
