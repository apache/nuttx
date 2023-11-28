/****************************************************************************
 * include/nuttx/sensors/ms56xx.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_MS56XX_H
#define __INCLUDE_NUTTX_SENSORS_MS56XX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MS56XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************
 * Prerequisites:
 *
 * CONFIG_I2C
 *   Enables support for I2C drivers
 * CONFIG_SENSORS_MS56XX
 *   Enables support for the MS56XX driver
 */

/* I2C Address **************************************************************/

#define MS56XX_ADDR0       0x77
#define MS56XX_ADDR1       0x76

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum ms56xx_model_e
{
  MS56XX_MODEL_MS5607 = 0,
  MS56XX_MODEL_MS5611 = 1,
};

struct ms56xx_measure_s
{
  int32_t temperature;  /* in Degree   x100    */
  int32_t pressure;     /* in mBar     x10     */
};

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

/****************************************************************************
 * Name: ms56xx_register
 *
 * Description:
 *   Register the MS56XX character device as 'devpath'.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance.
 *   devno   - Number of device (i.e. baro0, baro1, ...)
 *   addr    - The I2C address of the MS56XX.
 *   model   - The MS56XX model.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ms56xx_register(FAR struct i2c_master_s *i2c, int devno, uint8_t addr,
                    enum ms56xx_model_e model);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_MS56XX */
#endif /* __INCLUDE_NUTTX_SENSORS_MS56XX_H */
