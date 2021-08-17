/****************************************************************************
 * boards/renesas/rx65n/rx65n-rsk1mb/src/rx65n_main.c
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

#include "rx65n_macrodriver.h"
#include "arch/board/board.h"
#include "rx65n_definitions.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled1_on
 *
 * Description:
 *   Turns on LED 0
 *
 ****************************************************************************/

void board_autoled1_on(int led)
{
  LED0 = LED_ON;
}

/****************************************************************************
 * Name: board_autoled2_on
 *
 * Description:
 *   Turns on LED 1
 *
 ****************************************************************************/

void board_autoled2_on(int led)
{
  LED1 = LED_ON;
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *   Turns on LED 0 & LED 1
 *
 ****************************************************************************/

void board_autoled_on(int led)
{
  LED0 = LED_ON;
  LED1 = LED_ON;
}

/****************************************************************************
 * Name: board_autoled1_off
 *
 * Description:
 *   Turns off LED 0
 *
 ****************************************************************************/

void board_autoled1_off(int led)
{
  LED0 = LED_OFF;
}

/****************************************************************************
 * Name: board_autoled2_off
 *
 * Description:
 *   Turns off LED 1
 *
 ****************************************************************************/

void board_autoled2_off(int led)
{
  LED1 = LED_OFF;
}

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *   Turns off LED 0 & LED 1
 *
 ****************************************************************************/

void board_autoled_off(int led)
{
  LED0 = LED_OFF;
  LED1 = LED_OFF;
}
