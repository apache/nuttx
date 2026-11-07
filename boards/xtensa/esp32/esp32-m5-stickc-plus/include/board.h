/****************************************************************************
 * boards/xtensa/esp32/esp32-m5-stickc-plus/include/board.h
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

#ifndef __BOARDS_XTENSA_ESP32_ESP32_M5_STICKC_PLUS_INCLUDE_BOARD_H
#define __BOARDS_XTENSA_ESP32_ESP32_M5_STICKC_PLUS_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The ESP32-PICO-D4 SiP integrates a 40MHz crystal */

#define BOARD_XTAL_FREQUENCY  40000000

#ifdef CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ
#  define BOARD_CLOCK_FREQUENCY (CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ * 1000000)
#else
#  define BOARD_CLOCK_FREQUENCY 80000000
#endif

/* LED definitions **********************************************************/

/* Define how many LEDs this board has (needed by userleds) */

#define BOARD_NLEDS       1

#define BOARD_LED1        0

/* Button definitions *******************************************************/

#define BUTTON_A_ID       0
#define BUTTON_B_ID       1
#define NUM_BUTTONS       2

#define BUTTON_A_BIT      (1 << BUTTON_A_ID)
#define BUTTON_B_BIT      (1 << BUTTON_B_ID)

/* ST7789V2 LCD *************************************************************/

/* SPI2 carries CLK on GPIO13, MOSI on GPIO15 and CS on GPIO5.  The panel
 * backlight is powered by AXP192 LDO2, so there is no DISPLAY_BCKL GPIO.
 */

#define DISPLAY_SPI       2
#define DISPLAY_DC        23
#define DISPLAY_RST       18

#endif /* __BOARDS_XTENSA_ESP32_ESP32_M5_STICKC_PLUS_INCLUDE_BOARD_H */
