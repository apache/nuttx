/****************************************************************************
 * boards/xtensa/esp32s2/esp32s2-kaluga-1/include/board.h
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

#ifndef __BOARDS_XTENSA_ESP32S2_ESP32S2_KALUGA_1_INCLUDE_BOARD_H
#define __BOARDS_XTENSA_ESP32S2_ESP32S2_KALUGA_1_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The ESP32-S2-Kaluga-1 is fitted with a 40MHz crystal */

#define BOARD_XTAL_FREQUENCY  40000000

#ifdef CONFIG_ESP32S2_DEFAULT_CPU_FREQ_MHZ
#  define BOARD_CLOCK_FREQUENCY (CONFIG_ESP32S2_DEFAULT_CPU_FREQ_MHZ * 1000000)
#else
#  define BOARD_CLOCK_FREQUENCY 80000000
#endif

/* Button definitions *******************************************************/

#define BUTTON_BTN1                 0 /* BUTTON_BOOT      */
#define BUTTON_BTN1_BIT             (1 << BUTTON_BTN1)

#ifdef CONFIG_ESP32S2_TOUCH
#  define BUTTON_BTN2               1 /* TOUCHPAD_PHOTO   */
#  define BUTTON_BTN2_BIT           (1 << BUTTON_BTN2)
#  define BUTTON_BTN3               2 /* TOUCHPAD_PLAY    */
#  define BUTTON_BTN3_BIT           (1 << BUTTON_BTN3)
#  define BUTTON_BTN4               3 /* TOUCHPAD_RECORD  */
#  define BUTTON_BTN4_BIT           (1 << BUTTON_BTN4)
#  define BUTTON_BTN5               4 /* TOUCHPAD_NETWORK */
#  define BUTTON_BTN5_BIT           (1 << BUTTON_BTN5)
#  define BUTTON_BTN6               5 /* TOUCHPAD_VOLUP   */
#  define BUTTON_BTN6_BIT           (1 << BUTTON_BTN6)
#  define BUTTON_BTN7               6 /* TOUCHPAD_VOLDN   */
#  define BUTTON_BTN7_BIT           (1 << BUTTON_BTN7)
#  define NUM_BUTTONS               7
#else
#  define NUM_BUTTONS               1
#endif

/* LED definitions **********************************************************/

/* Define how many LEDs this board has (needed by userleds) */

#define BOARD_NLEDS       1

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOOUT    2 /* Amount of GPIO Output pins */
#define BOARD_NGPIOIN     1 /* Amount of GPIO Input without Interruption */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

#endif /* __BOARDS_XTENSA_ESP32S2_ESP32S2_KALUGA_1_INCLUDE_BOARD_H */
