/****************************************************************************
 * boards/arm/stm32f0l0g0/stm32g071b-disco/src/stm32_userleds.c
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

#include <nuttx/board.h>

#include "stm32_gpio.h"
#include "stm32g071b-disco.h"

#include <arch/board/board.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This array maps an LED number to GPIO pin configuration */

static uint32_t g_ledcfg[BOARD_NLEDS] =
{
  GPIO_LEDSINK, GPIO_LEDSOURCE, GPIO_LEDSPY, GPIO_LEDCC
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure LED GPIOs for output */

  stm32_configgpio(GPIO_LEDSINK);
  stm32_configgpio(GPIO_LEDSOURCE);
  stm32_configgpio(GPIO_LEDSPY);
  stm32_configgpio(GPIO_LEDCC);

  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  if ((unsigned)led < BOARD_NLEDS)
    {
      stm32_gpiowrite(g_ledcfg[led], !ledon);
    }
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  stm32_gpiowrite(GPIO_LEDSINK, (ledset & BOARD_LEDSINK_BIT) != 0);
  stm32_gpiowrite(GPIO_LEDSOURCE, (ledset & BOARD_LEDSOURCE_BIT) != 0);
  stm32_gpiowrite(GPIO_LEDSPY, (ledset & BOARD_LEDSPY_BIT) != 0);
  stm32_gpiowrite(GPIO_LEDCC, (ledset & BOARD_LEDCC_BIT) != 0);
}
