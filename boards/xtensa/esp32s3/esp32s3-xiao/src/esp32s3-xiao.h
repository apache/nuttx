/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-xiao/src/esp32s3-xiao.h
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

#ifndef __BOARDS_XTENSA_ESP32S3_ESP32S3_XIAO_SRC_ESP32S3_XIAO_H
#define __BOARDS_XTENSA_ESP32S3_ESP32S3_XIAO_SRC_ESP32S3_XIAO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 ****************************************************************************/

int esp32s3_bringup(void);

/****************************************************************************
 * Name: esp32s3_gpio_init
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int esp32s3_gpio_init(void);
#endif

/****************************************************************************
 * Name: board_i2c_init
 ****************************************************************************/

#ifdef CONFIG_I2C_DRIVER
int board_i2c_init(void);
#endif

/****************************************************************************
 * Name: board_lsm6ds3trc_initialize
 *
 * Description:
 *   Initialize and register the LSM6DS3TR-C 6-axis IMU driver, exposing
 *   it through uORB as /dev/uorb/sensor_accelN and /dev/uorb/sensor_gyroN.
 *
 * Input Parameters:
 *   devno - The device number, used to build the uORB device paths
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_LSM6DS3TRC
int board_lsm6ds3trc_initialize(int devno, int busno);
#endif

/****************************************************************************
 * Name: esp_openeth_initialize
 *
 * Description:
 *   Register the openeth MAC driver, the NIC emulated by QEMU's esp32s3
 *   machine.  Implemented by the shared Espressif code in
 *   arch/xtensa/src/common/espressif/esp_openeth.c.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_OPENETH
int esp_openeth_initialize(void);
#endif

/****************************************************************************
 * Name: board_spiflash_init
 *
 * Description:
 *   Initialize the SPI flash MTD partition and mount the file system
 *   selected by CONFIG_ESP32S3_SPIFLASH_FS on it.  Implemented by the
 *   shared ESP32-S3 board code in
 *   boards/xtensa/esp32s3/common/src/esp32s3_board_spiflash.c.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPIFLASH
int board_spiflash_init(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_XTENSA_ESP32S3_ESP32S3_XIAO_SRC_ESP32S3_XIAO_H */
