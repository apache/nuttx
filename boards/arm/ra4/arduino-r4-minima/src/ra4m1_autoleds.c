/****************************************************************************
 * boards/arm/ra4/arduino-r4-minima/src/ra4m1_autoleds.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <assert.h>
#include <debug.h>

#include <nuttx/board.h>

#include "chip.h"
#include "ra_gpio.h"
#include "arduino-r4-minima.h"

/* The board.h file may override pin configurations defined in ra_pinmap.h */

#include <arch/board/board.h>

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*  There are three user-controllable LEDs on board the Arduino Due board:
 *
 *     LED              GPIO
 *     ---------------- -----
 *     L   Amber LED    P111
 *     TX  Yellow LED   P012
 *     RX  Yellow LED   P013
 *
 * LED L is connected to ground and can be illuminated by driving the PB27
 * output high. The TX and RX LEDs are pulled high and can be illuminated by
 * driving the corresponding
 * GPIO output to low.
 *
 * These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/ra_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *   SYMBOL                MEANING                         LED STATE
 *                                                   L         TX       RX
 *   -------------------  -----------------------  -------- -------- --------
 *   LED_STARTED          NuttX has been started     OFF      OFF      OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF      OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF      OFF      OFF
 *   LED_STACKCREATED     Idle stack created         ON       OFF      OFF
 *   LED_INIRQ            In an interrupt            N/C      GLOW     OFF
 *   LED_SIGNAL           In a signal handler        N/C      GLOW     OFF
 *   LED_ASSERTION        An assertion failed        N/C      GLOW     OFF
 *   LED_PANIC            The system has crashed     N/C      N/C    Blinking
 *   LED_IDLE             MCU is is sleep mode       ------ Not used --------
 *
 * Thus if LED L is statically on, NuttX has successfully booted and is,
 * apparently, running normmally.  If LED RX is glowing, then NuttX is
 * handling interrupts (and also signals and assertions).  If TX is flashing
 * at approximately 2Hz, then a fatal error has been detected and the system
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure RX and TX LED GPIOs for output */

  ra_configgpio(GPIO_L_LED);
  ra_configgpio(GPIO_RX_LED);
  ra_configgpio(GPIO_TX_LED);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      /* 0: LED_STARTED, LED_HEAPALLOCATE, LED_IRQSENABLED: L=OFF TX=OFF
       *    RX=OFF
       *
       * Since the LEDs were initially all OFF and since this state only
       * occurs one time, nothing need be done.
       */

      default:
      case 0:
        break;

      /* 1: LED_STACKCREATED: L=ON TX=OFF RX=OFF
       *
       * This case will also occur only once.  Note that unlike the other
       * LEDs, LED L is active high.
       */

      case 1:
        ra_gpiowrite(GPIO_L_LED, true);
        break;

      /* 2: LED_INIRQ, LED_SIGNAL, LED_ASSERTION: L=N/C TX=ON RX=N/C
       *
       * This case will occur many times.  LED TX is active low.
       */

     case 2:
        ra_gpiowrite(GPIO_TX_LED, false);
        break;

      /* 3: LED_PANIC: L=N/X TX=N/C RX=ON
       *
       * This case will also occur many times. LED RX is active low.
       */

      case 3:
        ra_gpiowrite(GPIO_RX_LED, false);
        break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
      /* 0: LED_STARTED, LED_HEAPALLOCATE, LED_IRQSENABLED: L=OFF TX=OFF
       *    RX=OFF
       * 1: LED_STACKCREATED: L=ON TX=OFF RX=OFF
       *
       * These cases should never happen.
       */

      default:
      case 1:
      case 0:
        break;

      /* 2: LED_INIRQ, LED_SIGNAL, LED_ASSERTION: L=N/C TX=OFF RX=N/C
       *
       * This case will occur many times.  LED TX is active low.
       */

     case 2:
        ra_gpiowrite(GPIO_TX_LED, true);
        break;

      /* 3: LED_PANIC: L=N/X TX=N/C RX=OFF
       *
       * This case will also occur many times. LED RX is active low.
       */

      case 3:
        ra_gpiowrite(GPIO_RX_LED, true);
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
