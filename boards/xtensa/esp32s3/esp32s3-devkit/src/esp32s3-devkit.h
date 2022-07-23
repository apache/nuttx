/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-devkit/src/esp32s3-devkit.h
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

#ifndef __BOARDS_XTENSA_ESP32S3_ESP32S3_DEVKIT_SRC_ESP32S3_DEVKIT_H
#define __BOARDS_XTENSA_ESP32S3_ESP32S3_DEVKIT_SRC_ESP32S3_DEVKIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ESP32-S3-DEVKIT GPIOs ****************************************************/

/* LCD pins, i.e. used by ST7735 */

#define GPIO_LCD_DC         14
#define GPIO_LCD_RST        9

/* BOOT Button */

#define BUTTON_BOOT  0

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

/****************************************************************************
 * Name: board_bmp180_initialize
 *
 * Description:
 *   Initialize and register the BMP180 Pressure Sensor driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/pressN
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMP180
int board_bmp180_initialize(int devno, int busno);
#endif

/****************************************************************************
 * Name: esp32s3_djoy_initialize
 *
 * Description:
 *   Initialize and register the discrete joystick driver
 *
 ****************************************************************************/

#ifdef CONFIG_INPUT_DJOYSTICK
int esp32s3_djoy_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_XTENSA_ESP32S3_ESP32S3_DEVKIT_SRC_ESP32S3_DEVKIT_H */
