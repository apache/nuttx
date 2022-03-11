/****************************************************************************
 * boards/arm/lpc31xx/olimex-lpc-h3131/src/lpc31_leds.c
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
#include "lpc31.h"

#include "lpc_h3131.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 *
 * Description:
 *   Configure LEDs.  LEDs are left in the OFF state.
 *
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Turn off both LEDs */

  gpio_outputlow(LPC31_IOCONFIG_GPIO, GPIO_LED1);
  gpio_outputlow(LPC31_IOCONFIG_GPIO, GPIO_LED2);
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *   Select the "logical" ON state:
 *
 *    SYMBOL          Value  Meaning                     LED state
 *                                                   LED2     LED1
 *   ---------------- -----  -----------------------  -------- --------
 *   LED_STARTED        0  NuttX has been started     OFF      OFF
 *   LED_HEAPALLOCATE   0  Heap has been allocated    OFF      OFF
 *   LED_IRQSENABLED    0  Interrupts enabled         OFF      OFF
 *   LED_STACKCREATED   1  Idle stack created         ON       OFF
 *   LED_INIRQ          2  In an interrupt            N/C      N/C
 *   LED_SIGNAL         2  In a signal handler        N/C      N/C
 *   LED_ASSERTION      2  An assertion failed        N/C      N/C
 *   LED_PANIC          3  The system has crashed     N/C      Blinking
 *   LED_IDLE           -  MCU is is sleep mode         Not used
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_on(int led)
{
  switch (led)
    {
    case 0:
      gpio_outputlow(LPC31_IOCONFIG_GPIO, GPIO_LED1);
      gpio_outputlow(LPC31_IOCONFIG_GPIO, GPIO_LED2);
      break;

    case 1:
      gpio_outputlow(LPC31_IOCONFIG_GPIO, GPIO_LED1);
      gpio_outputhigh(LPC31_IOCONFIG_GPIO, GPIO_LED2);
      break;

    case 2:
      break;

    case 3:
      gpio_outputhigh(LPC31_IOCONFIG_GPIO, GPIO_LED1);
      break;
    }
}
#endif

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *   Select the "logical" OFF state:
 *
 *    SYMBOL          Value  Meaning                     LED state
 *                                                   LED2     LED1
 *   ---------------- -----  -----------------------  -------- --------
 *   LED_STARTED        0  NuttX has been started     OFF      OFF
 *   LED_HEAPALLOCATE   0  Heap has been allocated    OFF      OFF
 *   LED_IRQSENABLED    0  Interrupts enabled         OFF      OFF
 *   LED_STACKCREATED   1  Idle stack created         ON       OFF
 *   LED_INIRQ          2  In an interrupt            N/C      N/C
 *   LED_SIGNAL         2  In a signal handler        N/C      N/C
 *   LED_ASSERTION      2  An assertion failed        N/C      N/C
 *   LED_PANIC          3  The system has crashed     N/C      Blinking
 *   LED_IDLE           -  MCU is is sleep mode         Not used
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_off(int led)
{
  switch (led)
    {
    case 0:
    case 1:
    case 2:
      break;

    case 3:
      gpio_outputlow(LPC31_IOCONFIG_GPIO, GPIO_LED1);
      break;
    }
}
#endif

/****************************************************************************
 * Name:  board_userled_initialize, board_userled, and board_userled_all
 *
 * Description:
 *   These interfaces allow user control of the board LEDs.
 *
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control both on-board
 *   LEDs up until the completion of boot.
 *   Then it will continue to control LED2;
 *   LED1 is available for application use.
 *
 *   If CONFIG_ARCH_LEDS is not defined, then both LEDs are available
 *   for application use.
 *
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* All initialization performed in board_autoled_initialize() */

  return BOARD_NLEDS;
}

void board_userled(int led, bool ledon)
{
  uint32_t bit;

#ifndef CONFIG_ARCH_LEDS
  if (led == BOARD_LED1)
    {
      bit = GPIO_LED1;
    }
  else
#endif
  if (led == BOARD_LED2)
    {
      bit = GPIO_LED2;
    }
  else
    {
      return;
    }

  if (ledon)
    {
      gpio_outputhigh(LPC31_IOCONFIG_GPIO, bit);
    }
  else
    {
      gpio_outputlow(LPC31_IOCONFIG_GPIO, bit);
    }
}

void board_userled_all(uint32_t ledset)
{
#ifndef CONFIG_ARCH_LEDS
  board_userled(BOARD_LED1, (ledset & BOARD_LED1_BIT) != 0);
#endif
  board_userled(BOARD_LED2, (ledset & BOARD_LED2_BIT) != 0);
}
