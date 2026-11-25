/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-touch-amoled-216/include/board.h
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

#ifndef __BOARDS_XTENSA_ESP32S3_ESP32S3_TOUCH_AMOLED_216_INCLUDE_BOARD_H
#define __BOARDS_XTENSA_ESP32S3_ESP32S3_TOUCH_AMOLED_216_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The ESP32-S3-Touch-AMOLED-2.16 board is fitted with a 40MHz crystal */

#define BOARD_XTAL_FREQUENCY    40000000

#ifdef CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ
#  define BOARD_CLOCK_FREQUENCY (CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ * 1000000)
#else
#  define BOARD_CLOCK_FREQUENCY 80000000
#endif

/* LED definitions **********************************************************/

/* The board has no user LEDs */

#define BOARD_NLEDS       0

/* Button definitions *******************************************************/

/* The board has three keys:
 *
 *   BOOT  - GPIO0, active low.
 *   IO18  - GPIO18, active low (labelled "IO18" on the silkscreen).
 *   PWR   - The AXP2101 PWRON key.  Its level is mirrored to GPIO16
 *           (SYS_OUT) through a MOSFET, so the key reads high on GPIO16
 *           while it is pressed.  Holding it for several seconds makes
 *           the AXP2101 power the board off.
 */

#define BUTTON_BOOT       0
#define BUTTON_IO18       1
#define BUTTON_PWR        2
#define NUM_BUTTONS       3

#define BUTTON_BOOT_BIT   (1 << BUTTON_BOOT)
#define BUTTON_IO18_BIT   (1 << BUTTON_IO18)
#define BUTTON_PWR_BIT    (1 << BUTTON_PWR)

/* QMI8658 IMU **************************************************************/

/* The QMI8658 sits on the shared I2C0 bus with SA0 tied low (0x6B) */

#define QMI8658_I2C_PORT  0
#define QMI8658_I2C_ADDR  0x6b

#endif /* __BOARDS_XTENSA_ESP32S3_ESP32S3_TOUCH_AMOLED_216_INCLUDE_BOARD_H */
