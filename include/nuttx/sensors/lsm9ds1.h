/****************************************************************************
 * include/nuttx/sensors/lsm9ds1.h
 *
 *   Copyright (C) 2016 Omni Hoverboards Inc. All rights reserved.
 *   Author: Paul Alexander Patience <paul-a.patience@polymtl.ca>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

int lsm9ds1accel_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
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

int lsm9ds1gyro_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
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

int lsm9ds1mag_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                        uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_LSM9DS1 */
#endif /* __INCLUDE_NUTTX_SENSORS_LSM9DS1 */
