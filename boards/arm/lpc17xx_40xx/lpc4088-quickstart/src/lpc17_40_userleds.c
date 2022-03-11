/****************************************************************************
 * boards/arm/lpc17xx_40xx/lpc4088-quickstart/src/lpc17_40_userleds.c
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

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "lpc17_40_gpio.h"
#include "lpc4088-quickstart.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Dump GPIO registers */

#ifdef CONFIG_DEBUG_LEDS_INFO
#  define led_dumpgpio(m) lpc17_40_dumpgpio(???, m)
#else
#  define led_dumpgpio(m)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This array maps an LED number to GPIO pin configuration */

static uint32_t g_ledcfg[BOARD_NLEDS] =
{
  GPIO_LED1, GPIO_LED2, GPIO_LED3, GPIO_LED4
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure LED1-4 GPIOs for output */

  lpc17_40_configgpio(GPIO_LED1);
  lpc17_40_configgpio(GPIO_LED2);
  lpc17_40_configgpio(GPIO_LED3);
  lpc17_40_configgpio(GPIO_LED4);
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  switch ((unsigned)led)
    {
      case BOARD_LED1:

        /* LED1 and LED2 are inverted */

      case BOARD_LED2:
        lpc17_40_gpiowrite(g_ledcfg[led], !ledon);
        break;

      case BOARD_LED3:

        /* LED3 and LED4 are driven normally */

      case BOARD_LED4:
        lpc17_40_gpiowrite(g_ledcfg[led], ledon);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  lpc17_40_gpiowrite(GPIO_LED1, (ledset & BOARD_LED1_BIT) != 0);
  lpc17_40_gpiowrite(GPIO_LED2, (ledset & BOARD_LED2_BIT) != 0);
  lpc17_40_gpiowrite(GPIO_LED3, (ledset & BOARD_LED3_BIT) == 0);
  lpc17_40_gpiowrite(GPIO_LED4, (ledset & BOARD_LED4_BIT) == 0);
}

#endif /* !CONFIG_ARCH_LEDS */
