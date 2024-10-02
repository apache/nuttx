/****************************************************************************
 * include/nuttx/sensors/kxtj9.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_KXTJ9_H
#define __INCLUDE_NUTTX_SENSORS_KXTJ9_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_KXTJ9)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Data control register bits */

enum kxtj9_odr_e
{
  ODR0_781F  = 8,
  ODR1_563F  = 9,
  ODR3_125F  = 10,
  ODR6_25F   = 11,
  ODR12_5F   = 0,
  ODR25F     = 1,
  ODR50F     = 2,
  ODR100F    = 3,
  ODR200F    = 4,
  ODR400F    = 5,
  ODR800F    = 6
};

/* Data returned by reading from the KXTJ9 is returned in this format.
 * In order for the read to be successful, a buffer of size >= sizeof(struct
 * kxtj9_sensor_data) must be provided with the read.
 */

struct kxtj9_sensor_data
{
  uint16_t x;
  uint16_t y;
  uint16_t z;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: kxtj9_register
 *
 * Description:
 *   Register the KXTJ9 accelerometer device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/accel0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the KXTJ9 accelerometer, gyroscope or
 *             magnetometer.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

struct i2c_master_s;
int kxtj9_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                   uint8_t address);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_KXTJ9 */
#endif /* __INCLUDE_NUTTX_SENSORS_KXTJ9_H */
