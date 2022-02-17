/****************************************************************************
 * include/nuttx/sensors/isl29023.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_ISL29023_H
#define __INCLUDE_NUTTX_SENSORS_ISL29023_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_ISL29023)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

enum isl29023_resolution_e
{
  ISL29023_RESOLUTION_16BITS =      0x0,
  ISL29023_RESOLUTION_12BITS =      0x1,
  ISL29023_RESOLUTION_8BITS =       0x2,
  ISL29023_RESOLUTION_4BITS =       0x3,
};

enum isl29023_als_range_e
{
  ISL29023_ALS_RANGE_1000 =         0x0,
  ISL29023_ALS_RANGE_4000 =         0x1,
  ISL29023_ALS_RANGE_16000 =        0x2,
  ISL29023_ALS_RANGE_64000 =        0x3,
};

/* ISL2923 goes to power dowm mode after mode *once */

enum isl29023_operational_mode_e
{
  ISL29023_OP_MODE_POWER_DOWN =     0x0,
  ISL29023_OP_MODE_ALS_ONCE =       0x1,
  ISL29023_OP_MODE_IR_ONCE =        0x2,
  ISL29023_OP_MODE_ALS_CONTINUES =  0x5,
  ISL29023_OP_MODE_IR_CONTINUES =   0x6,
};

/* Data transfer structure */

struct isl29023_data_s
{
  uint16_t lux;               /* Converted lux value */
  uint16_t raw;               /* Raw unconverted value */
};

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

/****************************************************************************
 * Name: isl29023_register
 *
 * Description:
 *   Register the ISL29023 ALS device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/als0"
 *   i2c - An instance of the I2C interface to use to communicate with ALS
 *   addr - The I2C address of the ALS.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int isl29023_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                      uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_ISL29023 */
#endif /* __INCLUDE_NUTTX_SENSORS_ISL29023_H */
