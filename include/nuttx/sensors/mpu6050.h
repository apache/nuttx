/****************************************************************************
 * include/nuttx/sensors/mpu6050.h
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
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_MPU6050_H
#define __INCLUDE_NUTTX_SENSORS_MPU6050_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>
#include <nuttx/sensors/sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MPU6050 I2C addresses */

#define MPU6050_ADDR_LOW   0x68
#define MPU6050_ADDR_HIGH  0x69

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/* Board specific configuration.  With CONFIG_SENSORS_MPU6050_INT the board
 * must supply attach(), which wires the MPU6050 INT pin to the given
 * handler; the driver then delivers samples from that interrupt instead of
 * reading the device on demand.
 */

struct mpu6050_config_s
{
  CODE int (*attach)(FAR const struct mpu6050_config_s *config,
                     xcpt_t isr, FAR void *arg);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: mpu6050_register
 *
 * Description:
 *   Register the MPU6050 6-axis IMU sensor device with the NuttX uORB
 *   sensor framework.
 *
 * Input Parameters:
 *   devno  - Device number for sensor registration (e.g. 0)
 *   i2c    - Pointer to the I2C master interface
 *   addr   - I2C slave address of the MPU6050 device
 *   config - Board configuration, required with CONFIG_SENSORS_MPU6050_INT
 *            and otherwise unused; may be NULL
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mpu6050_register(int devno, FAR struct i2c_master_s *i2c, uint8_t addr,
                     FAR const struct mpu6050_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_MPU6050_H */
