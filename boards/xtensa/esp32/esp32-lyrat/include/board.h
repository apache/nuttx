/****************************************************************************
 * boards/xtensa/esp32/esp32-lyrat/include/board.h
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

#ifndef __BOARDS_XTENSA_ESP32_ESP32_LYRAT_INCLUDE_BOARD_H
#define __BOARDS_XTENSA_ESP32_ESP32_LYRAT_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The ESP32-LyraT board is fitted with either a 26 or a 40MHz crystal */

#ifdef CONFIG_ESP32_XTAL_26MHz
#  define BOARD_XTAL_FREQUENCY  26000000
#else
#  define BOARD_XTAL_FREQUENCY  40000000
#endif

#ifdef CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ
#  define BOARD_CLOCK_FREQUENCY (CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ * 1000000)
#else
#  define BOARD_CLOCK_FREQUENCY 80000000
#endif

/* Button definitions *******************************************************/

#define BUTTON_BTN1                 0 /* BUTTON_REC    */
#define BUTTON_BTN1_BIT             (1 << BUTTON_BTN1)
#define BUTTON_BTN2                 1 /* BUTTON_MODE   */
#define BUTTON_BTN2_BIT             (1 << BUTTON_BTN2)

#ifdef CONFIG_ESP32_TOUCH
#  define BUTTON_BTN3               2 /* TOUCHPAD_PLAY */
#  define BUTTON_BTN3_BIT           (1 << BUTTON_BTN3)
#  define BUTTON_BTN4               3 /* TOUCHPAD_SET  */
#  define BUTTON_BTN4_BIT           (1 << BUTTON_BTN4)
#  define BUTTON_BTN5               4 /* TOUCHPAD_VOLM */
#  define BUTTON_BTN5_BIT           (1 << BUTTON_BTN5)
#  define BUTTON_BTN6               5 /* TOUCHPAD_VOLP */
#  define BUTTON_BTN6_BIT           (1 << BUTTON_BTN6)
#  define NUM_BUTTONS               6
#else
#  define NUM_BUTTONS               2
#endif

/* GPIO definitions *********************************************************/

/* ES8388 CODEC */

#define ES8388_I2C_CCLK       23     /* SCL */
#define ES8388_I2C_CDATA      18     /* SDA */

#define ES8388_I2S_MCLK       0      /* Master clock */
#define ES8388_I2S_SCLK       5      /* Audio data bit clock */
#define ES8388_I2S_LRCK       25     /* Audio data left and right clock */
#define ES8388_I2S_DSDIN      26     /* DAC audio data (to speakers) */
#define ES8388_I2S_ASDOUT     35     /* ADC audio data (from microphone) */

/* LED definitions **********************************************************/

/* Define how many LEDs this board has (needed by userleds) */

#define BOARD_NLEDS       1

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOIN     1 /* Amount of GPIO Input without Interruption */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

/* Peripherals definitions **************************************************/

/* ES8388 CODEC */

#define ES8388_I2C_FREQ       400000
#define ES8388_I2C_ADDR       0x10

#endif /* __BOARDS_XTENSA_ESP32_ESP32_LYRAT_INCLUDE_BOARD_H */
