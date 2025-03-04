/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-eye/src/esp32s3-eye.h
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

#ifndef __BOARDS_XTENSA_ESP32S3_ESP32S3_EYE_SRC_ESP32S3_EYE_H
#define __BOARDS_XTENSA_ESP32S3_ESP32S3_EYE_SRC_ESP32S3_EYE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ESP32-S3-EYE GPIOs *******************************************************/

/* BOOT Button */

#define BUTTON_BOOT         0
/* Display */

#define ESP32S3_EYE_DISPLAY_SPI         2
#define ESP32S3_EYE_DISPLAY_DC          43
#define ESP32S3_EYE_DISPLAY_BCKL        48

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library via board_app_initialize()
 *
 ****************************************************************************/

int esp32s3_bringup(void);

/****************************************************************************
 * Name: esp32s3_gpio_init
 *
 * Description:
 *   Configure the GPIO driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int esp32s3_gpio_init(void);
#endif

/****************************************************************************
 * Name: board_spiflash_init
 *
 * Description:
 *   Initialize the SPIFLASH and register the MTD device.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPIFLASH
int board_spiflash_init(void);
#endif

/****************************************************************************
 * Name: board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware. The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with
 *   the power setting at 0 (full off).
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_EYE_LCD
int board_lcd_initialize(void);
#endif

/****************************************************************************
 * Name: board_lcd_getdev
 *
 * Description:
 *   Return a reference to the LCD object for the specified LCD. This allows
 *   support for multiple LCD devices.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_EYE_LCD
struct lcd_dev_s *board_lcd_getdev(int lcddev);
#endif

/****************************************************************************
 * Name: board_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the LCD support.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_EYE_LCD
void board_lcd_uninitialize(void);
#endif

/****************************************************************************
 * Name: board_i2c_init
 *
 * Description:
 *   Configure the I2C driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_DRIVER
int board_i2c_init(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_XTENSA_ESP32S3_ESP32S3_EYE_SRC_ESP32S3_EYE_H */
