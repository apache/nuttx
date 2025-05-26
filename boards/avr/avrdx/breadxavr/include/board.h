/****************************************************************************
 * boards/avr/avrdx/breadxavr/include/board.h
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

#ifndef __BOARDS_AVR_AVRDX_BREADXAVR_INCLUDE_BOARD_H
#define __BOARDS_AVR_AVRDX_BREADXAVR_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Clocking *****************************************************************/

/* Variables similar to what Atmega1284-xplained sets,
 * taken from menuconfig.
 */
#define BOARD_XTAL_FREQ        CONFIG_AVRDX_HFO_CLOCK_FREQ
#define BOARD_CPU_CLOCK        BOARD_XTAL_FREQ

/* LED definitions **********************************************************/

/* The breadboard has one LED dedicated to displaying OS operation.
 */

#define LED_STARTED            0
#define LED_HEAPALLOCATE       0
#define LED_IRQSENABLED        0
#define LED_STACKCREATED       1
#define LED_INIRQ              2
#define LED_SIGNAL             2
#define LED_ASSERTION          2
#define LED_PANIC              0

/* Button definitions
 *
 * The breadboard has 4 buttons connected to port A pins 2, 3 and port C
 * pins 2, 3
 */

enum breadboard_buttons_e
{
  BBRD_BUTTON_1 = 0,
  BBRD_BUTTON_2 = 1,
  BBRD_BUTTON_3 = 2,
  BBRD_BUTTON_4 = 3,
  BBRD_NUM_BUTTONS
};

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_AVR_AVRDX_BREADXAVR_INCLUDE_BOARD_H */
