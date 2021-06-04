/****************************************************************************
 * boards/risc-v/rv32m1/rv32m1-vega/src/rv32m1_autoleds.c
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
#include <debug.h>
#include <arch/board/board.h>

#include "rv32m1_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_LED  (GPIO_OUTPUT|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN24)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  rv32m1_gpio_config(GPIO_LED);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if (LED_CPU == led)
    {
      rv32m1_gpio_write(GPIO_LED, true);
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (LED_CPU == led)
    {
      rv32m1_gpio_write(GPIO_LED, false);
    }
}
