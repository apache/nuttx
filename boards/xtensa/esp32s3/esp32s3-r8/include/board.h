/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-r8/include/board.h
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

#ifndef __BOARDS_XTENSA_ESP32S3_ESP32S3_R8_INCLUDE_BOARD_H
#define __BOARDS_XTENSA_ESP32S3_ESP32S3_R8_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The ESP32-S3-EYE board is fitted with a 40MHz crystal */

#define BOARD_XTAL_FREQUENCY    40000000

#ifdef CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ
#  define BOARD_CLOCK_FREQUENCY (CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ * 1000000)
#else
#  define BOARD_CLOCK_FREQUENCY 80000000
#endif

/* LED definitions **********************************************************/

/* Define how many LEDs this board has (needed by userleds) */

#define BOARD_NLEDS             1

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOIN     0
#define BOARD_NGPIOINT    1

/* Oneshot timer resolution *************************************************/

#define ONESHOT_RESOLUTION_US 1

/* Audio codec definitions **************************************************/

#ifdef CONFIG_AUDIO_ES8311
#  define ES8311_I2C_FREQ       100000
#  define ES8311_I2C_ADDR       0x18
#  define ES8311_I2C_PORT       0
#  define ES8311_I2S_PORT       0
#  define ES8311_PA_CTRL        46
#endif

#ifdef CONFIG_AUDIO_ES7210
#  define ES7210_I2C_FREQ       100000
#  define ES7210_I2C_ADDR       0x43
#  define ES7210_I2C_PORT       0
#  define ES7210_I2S_PORT       0
#endif

#endif /* __BOARDS_XTENSA_ESP32S3_ESP32S3_R8_INCLUDE_BOARD_H */
