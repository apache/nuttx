/****************************************************************************
 * boards/risc-v/qemu-rv/rv-virt/src/qemu_rv_autoleds.c
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
#include <debug.h>

#include <sys/param.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 *
 * Description:
 *   This function is called very early in initialization to perform board-
 *   specific initialization of LED-related resources.  This includes such
 *   things as, for example, configure GPIO pins to drive the LEDs and also
 *   putting the LEDs in their correct initial state.
 *
 *   NOTE: In most architectures, board_autoled_initialize() is called from
 *   board-specific initialization logic.  But there are a few architectures
 *   where this initialization function is still called from common chip
 *   architecture logic.  This interface is not, however, a common board
 *   interface in any event and, hence, the usage of the name
 *   board_autoled_initialize is deprecated.
 *
 *   WARNING: This interface name will eventually be removed; do not use it
 *   in new board ports.  New implementations should use the naming
 *   conventions for "Microprocessor-Specific Interfaces" or the "Board-
 *   Specific Interfaces" as described above.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_autoled_initialize(void)
{
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *   Set the LED configuration into the ON condition for the state provided
 *   by the led parameter.  This may be one of:
 *
 *     LED_STARTED       NuttX has been started
 *     LED_HEAPALLOCATE  Heap has been allocated
 *     LED_IRQSENABLED   Interrupts enabled
 *     LED_STACKCREATED  Idle stack created
 *     LED_INIRQ         In an interrupt
 *     LED_SIGNAL        In a signal handler
 *     LED_ASSERTION     An assertion failed
 *     LED_PANIC         The system has crashed
 *     LED_IDLE          MCU is in sleep mode
 *
 *   Where these values are defined in a board-specific way in the standard
 *   board.h header file exported by every architecture.
 *
 * Input Parameters:
 *   led - Identifies the LED state to put in the ON state (which may or may
 *         not equate to turning an LED on)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_autoled_on(int led)
{
  /* This stub is required for supporting User LEDs */

  UNUSED(led);
}

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *   Set the LED configuration into the OFF condition for the state provided
 *   by the led parameter.  This may be one of:
 *
 *     LED_INIRQ         Leaving an interrupt
 *     LED_SIGNAL        Leaving a signal handler
 *     LED_ASSERTION     Recovering from an assertion failure
 *     LED_PANIC         The system has crashed (blinking).
 *     LED_IDLE          MCU is not in sleep mode
 *
 *   Where these values are defined in a board-specific way in the standard
 *   board.h header file exported by every architecture.
 *
 * Input Parameters:
 *   led - Identifies the LED state to put in the OFF state (which may or may
 *         not equate to turning an LED off)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_autoled_off(int led)
{
  /* This stub is required for supporting User LEDs */

  UNUSED(led);
}

#endif /* CONFIG_ARCH_LEDS */