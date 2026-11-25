/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-touch-amoled-216/src/esp32s3-touch-amoled-216.h
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

#ifndef __BOARDS_XTENSA_ESP32S3_ESP32S3_TOUCH_AMOLED_216_SRC_ESP32S3_TOUCH_AMOLED_216_H
#define __BOARDS_XTENSA_ESP32S3_ESP32S3_TOUCH_AMOLED_216_SRC_ESP32S3_TOUCH_AMOLED_216_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Keys *********************************************************************/

#define GPIO_BUTTON_BOOT   0   /* BOOT key, active low */
#define GPIO_BUTTON_IO18   18  /* IO18 key, active low */
#define GPIO_BUTTON_PWR    16  /* SYS_OUT, AXP2101 PWRON key, active high */

/* I2C **********************************************************************/

/* A single I2C bus, GPIO15 (SDA) and GPIO14 (SCL), is shared by the
 * touch controller, AXP2101, QMI8658, PCF85063, ES8311 and ES7210.
 */

#define BOARD_I2C_BUS      0

/* QSPI AMOLED panel (CO5300), on SPI2 in quad mode *************************/

#define GPIO_LCD_CS        12
#define GPIO_LCD_SCLK      38
#define GPIO_LCD_SIO0      4
#define GPIO_LCD_SIO1      5
#define GPIO_LCD_SIO2      6
#define GPIO_LCD_SIO3      7
#define GPIO_LCD_RST       39

/* Touch controller (CST9220) ***********************************************/

#define GPIO_TP_INT        11
#define GPIO_TP_RST        40

/* QMI8658 IMU **************************************************************/

#define GPIO_IMU_INT1      17
#define GPIO_IMU_INT2      21

/* PCF85063 RTC *************************************************************/

#define GPIO_RTC_INT       13

/* Audio ********************************************************************/

/* ES8311 (playback) and ES7210 (capture) share the I2S0 clocks:
 * MCLK GPIO42, BCLK GPIO9, LRCK GPIO45, DOUT GPIO8 and DIN GPIO10.
 * GPIO46 enables the speaker power amplifier.
 */

#define GPIO_SPK_PA_EN     46

/* microSD card, on SPI3 ****************************************************/

#define GPIO_SD_MOSI       1
#define GPIO_SD_SCLK       2
#define GPIO_SD_MISO       3
#define GPIO_SD_CS         41

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: esp32s3_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library via board_app_initialize().
 *
 ****************************************************************************/

int esp32s3_bringup(void);

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
 * Name: board_spiflash_init
 *
 * Description:
 *   Initialize the SPI Flash and register the MTD device.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPIFLASH
int board_spiflash_init(void);
#endif

/****************************************************************************
 * Name: esp32s3_qmi8658_initialize
 *
 * Description:
 *   Initialize and register the QMI8658 6-axis IMU uORB driver.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_QMI8658
int esp32s3_qmi8658_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_XTENSA_ESP32S3_ESP32S3_TOUCH_AMOLED_216_SRC_ESP32S3_TOUCH_AMOLED_216_H */
