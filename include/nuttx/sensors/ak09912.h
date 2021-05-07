/****************************************************************************
 * include/nuttx/sensors/ak09912.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_AK09912_H
#define __INCLUDE_NUTTX_SENSORS_AK09912_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#if defined(CONFIG_I2C) && (defined(CONFIG_SENSORS_AK09912) || defined (CONFIG_SENSORS_AK09912_SCU))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Prerequisites:
 *
 * CONFIG_AK09912
 *   Enables support for the AK09912 driver
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* IOCTL Commands ***********************************************************/

/* Arg: 0: Disable compensated
 *      1: Enable compensated
 */
#define ENABLE_COMPENSATED (1)
#define DISABLE_COMPENSATED (0)
#define SNIOC_ENABLE_COMPENSATED   _SNIOC(0x0001)

#define SNIOC_GETADJ               _SNIOC(0x0002)

struct mag_data_s
{
  int16_t x; /* X raw data */
  int16_t y; /* Y raw data */
  int16_t z; /* Z raw data */
};

struct ak09912_sensadj_s
{
  uint8_t x;
  uint8_t y;
  uint8_t z;
};

#ifdef CONFIG_SENSORS_AK09912_SCU
/****************************************************************************
 * Name: ak09912_init
 *
 * Description:
 *   Initialize AK09912 magnetometer device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             AK09912
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ak09912_init(FAR struct i2c_master_s *i2c, int port);
#endif

/****************************************************************************
 * Name: ak09912_register
 *
 * Description:
 *   Register the AK09912 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/mag0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             AK09912
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
#ifdef CONFIG_SENSORS_AK09912_SCU
int ak09912_register(FAR const char *devpath, int minor,
                     FAR struct i2c_master_s *i2c, int port);
#else
int ak09912_register(FAR const char *devpath, FAR struct i2c_master_s *i2c);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_AK09912 */
#endif /* __INCLUDE_NUTTX_SENSORS_AK09912_H */
