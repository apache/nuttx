/****************************************************************************
 * include/nuttx/sensors/lsm9ds1.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_LSM9DS1
#define __INCLUDE_NUTTX_SENSORS_LSM9DS1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LSM9DS1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C Addresses ************************************************************/

/* Accelerometer addresses */

#define LSM9DS1ACCEL_ADDR0  0x6a
#define LSM9DS1ACCEL_ADDR1  0x6b

/* Gyroscope addresses */

#define LSM9DS1GYRO_ADDR0   LSM9DS1ACCEL_ADDR0
#define LSM9DS1GYRO_ADDR1   LSM9DS1ACCEL_ADDR1

/* Magnetometer addresses */

#define LSM9DS1MAG_ADDR0    0x1c
#define LSM9DS1MAG_ADDR1    0x1e

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: lsm9ds1accel_register
 *
 * Description:
 *   Register the LSM9DS1 accelerometer character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/accel0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the LSM9DS1 accelerometer.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm9ds1accel_register(FAR const char *devpath,
                          FAR struct i2c_master_s *i2c,
                          uint8_t addr);

/****************************************************************************
 * Name: lsm9ds1gyro_register
 *
 * Description:
 *   Register the LSM9DS1 gyroscope character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/gyro0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the LSM9DS1 gyroscope.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm9ds1gyro_register(FAR const char *devpath,
                         FAR struct i2c_master_s *i2c,
                         uint8_t addr);

/****************************************************************************
 * Name: lsm9ds1mag_register
 *
 * Description:
 *   Register the LSM9DS1 magnetometer character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/mag0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the LSM9DS1 magnetometer.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm9ds1mag_register(FAR const char *devpath,
                        FAR struct i2c_master_s *i2c,
                        uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_LSM9DS1 */
#endif /* __INCLUDE_NUTTX_SENSORS_LSM9DS1 */
