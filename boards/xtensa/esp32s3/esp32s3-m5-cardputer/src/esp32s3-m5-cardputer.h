/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-m5-cardputer/src/esp32s3-m5-cardputer.h
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

#ifndef __BOARDS_XTENSA_ESP32S3_ESP32S3_M5_CARDPUTER_SRC_ESP32S3_M5_CARDPUTER_H
#define __BOARDS_XTENSA_ESP32S3_ESP32S3_M5_CARDPUTER_SRC_ESP32S3_M5_CARDPUTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ST7789 display - wired to SPI2; DC/RST/backlight are plain GPIOs.
 * (Mirrors the CARDPUTER_GPIO_LCD_* pins documented in include/board.h.)
 */

#define DISPLAY_SPI    2
#define DISPLAY_DC     34  /* Data/command */
#define DISPLAY_RST    33  /* Reset */
#define DISPLAY_BCKL   38  /* Backlight enable */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_bringup
 *
 * Description:
 *   Perform architecture-specific initialization.
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp32s3_bringup(void);

/****************************************************************************
 * Name: board_i2c_init
 *
 * Description:
 *   Configure the I2C driver(s) for the Grove port.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_DRIVER
int board_i2c_init(void);
#endif

/****************************************************************************
 * Name: esp32s3_kbd_initialize
 *
 * Description:
 *   Initialize and register the M5Stack Cardputer matrix keyboard driver as
 *   a NuttX keyboard device (/dev/kbdN).
 *
 * Input Parameters:
 *   devpath - The full path to the keyboard device, e.g. "/dev/kbd0".
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_M5_CARDPUTER_KEYBOARD
int esp32s3_kbd_initialize(FAR const char *devpath);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_XTENSA_ESP32S3_ESP32S3_M5_CARDPUTER_SRC_ESP32S3_M5_CARDPUTER_H */
