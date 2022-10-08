/****************************************************************************
 * include/nuttx/sensors/ltr308.h
 * Character driver for the LTR-308ALS-01 Lite-On ambient light sensor.
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

#ifndef __INCLUDE_NUTTX_SENSORS_LTR308_H
#define __INCLUDE_NUTTX_SENSORS_LTR308_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

/* Configuration ************************************************************
 * Prerequisites:
 *
 * CONFIG_I2C
 *   Enables support for I2C drivers
 * CONFIG_SENSORS_LTR308
 *   Enables support for the LTR308 driver
 */

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LTR308)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

struct ltr308_calibval
{
  uint8_t integration_time;
  uint8_t measurement_rate;
  uint8_t gain;
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
 * Name: ltr308_register
 *
 * Description:
 *   Register the LTR308 character device as 'devpath'.
 *
 * Input Parameters:
 *   devno   - Number of device (i.e. ltr0, ltr1, ...)
 *   i2c     - An I2C driver instance.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ltr308_register(int devno, FAR struct i2c_master_s *i2c);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_LTR308 */
#endif /* __INCLUDE_NUTTX_SENSORS_LTR308_H */
