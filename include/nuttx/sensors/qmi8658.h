/****************************************************************************
 * include/nuttx/sensors/qmi8658.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_QMI8658_H
#define __INCLUDE_NUTTX_SENSORS_QMI8658_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/i2c/i2c_master.h>
#include <stdint.h>
#include <sys/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Accelerometer Full Scale Ranges */
#define QMI8658_ACC_FS_2G          (0x00)
#define QMI8658_ACC_FS_4G          (0x01)
#define QMI8658_ACC_FS_8G          (0x02)
#define QMI8658_ACC_FS_16G         (0x03)

/* Gyroscope Full Scale Ranges */
#define QMI8658_GYRO_FS_16DPS      (0x00)
#define QMI8658_GYRO_FS_32DPS      (0x01)
#define QMI8658_GYRO_FS_64DPS      (0x02)
#define QMI8658_GYRO_FS_128DPS     (0x03)
#define QMI8658_GYRO_FS_256DPS     (0x04)
#define QMI8658_GYRO_FS_512DPS     (0x05)
#define QMI8658_GYRO_FS_1024DPS    (0x06)

/* ODR (Output Data Rate) Definitions */

/* Accelerometer ODR */
#define QMI8658_ACC_ODR_1000Hz         (0x03)
#define QMI8658_ACC_ODR_500Hz          (0x04)
#define QMI8658_ACC_ODR_250Hz          (0x05)
#define QMI8658_ACC_ODR_125Hz          (0x06)
#define QMI8658_ACC_ODR_62_5Hz         (0x07)
#define QMI8658_ACC_ODR_31_25Hz        (0x08)
#define QMI8658_ACC_ODR_LOWPOWER_128Hz (0x0C)
#define QMI8658_ACC_ODR_LOWPOWER_21Hz  (0x0D)
#define QMI8658_ACC_ODR_LOWPOWER_11Hz  (0x0E)
#define QMI8658_ACC_ODR_LOWPOWER_3Hz   (0x0F)

/* Gyroscope ODR */
#define QMI8658_GYRO_ODR_7174_4Hz  (0x00)
#define QMI8658_GYRO_ODR_3587_2Hz  (0x01)
#define QMI8658_GYRO_ODR_1793_6Hz  (0x02)
#define QMI8658_GYRO_ODR_896_8Hz   (0x03)
#define QMI8658_GYRO_ODR_448_4Hz   (0x04)
#define QMI8658_GYRO_ODR_224_2Hz   (0x05)
#define QMI8658_GYRO_ODR_112_1Hz   (0x06)
#define QMI8658_GYRO_ODR_56_05Hz   (0x07)
#define QMI8658_GYRO_ODR_28_025Hz  (0x08)

/* Scale Factors */

/* Accelerometer scale factors (LSB/g) */
#define QMI8658_ACC_SCALE_2G       (16384.0f)
#define QMI8658_ACC_SCALE_4G       (8192.0f)
#define QMI8658_ACC_SCALE_8G       (4096.0f)
#define QMI8658_ACC_SCALE_16G      (2048.0f)

/* Gyroscope scale factors (LSB/dps) */
#define QMI8658_GYRO_SCALE_16DPS   (2048.0f)
#define QMI8658_GYRO_SCALE_32DPS   (1024.0f)
#define QMI8658_GYRO_SCALE_64DPS   (512.0f)
#define QMI8658_GYRO_SCALE_128DPS  (256.0f)
#define QMI8658_GYRO_SCALE_256DPS  (128.0f)
#define QMI8658_GYRO_SCALE_512DPS  (64.0f)
#define QMI8658_GYRO_SCALE_1024DPS (32.0f)

/* Temperature Scale Factor (LSB/°C) */
#define QMI8658_TEMP_SCALE         (256.0f)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: qmi8658_uorb_register
 *
 * Description:
 *   Register the QMI8658 uORB sensor device
 *
 * Input Parameters:
 *   devno   - Device number to use
 *   i2c     - Pointer to the I2C master device
 *   addr    - I2C address (use QMI8658_I2C_ADDR for default)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int qmi8658_uorb_register(int devno, FAR struct i2c_master_s *i2c,
                          uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_QMI8658_H */
