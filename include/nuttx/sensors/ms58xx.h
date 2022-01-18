/****************************************************************************
 * include/nuttx/sensors/ms58xx.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_MS58XX_H
#define __INCLUDE_NUTTX_SENSORS_MS58XX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MS58XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************
 * Prerequisites:
 *
 * CONFIG_I2C
 *   Enables support for I2C drivers
 * CONFIG_SENSORS_MS58XX
 *   Enables support for the MS58XX driver
 * CONFIG_MS58XX_VDD
 */

/* I2C Address **************************************************************/

#define MS58XX_ADDR0       0x76
#define MS58XX_ADDR1       0x77

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum ms58xx_model_e
{
  MS58XX_MODEL_MS5803_02 = 0,
  MS58XX_MODEL_MS5803_05 = 1,
  MS58XX_MODEL_MS5803_07 = 2,
  MS58XX_MODEL_MS5803_14 = 3,
  MS58XX_MODEL_MS5803_30 = 4,
  MS58XX_MODEL_MS5805_02 = 5,
  MS58XX_MODEL_MS5806_02 = 6,
  MS58XX_MODEL_MS5837_30 = 7
};

struct ms58xx_measure_s
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
 * Name: ms58xx_register
 *
 * Description:
 *   Register the MS58XX character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/press0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the MS58XX.
 *   osr     - The oversampling ratio.
 *   model   - The MS58XX model.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ms58xx_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr, uint16_t osr, enum ms58xx_model_e model);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_MS58XX */
#endif /* __INCLUDE_NUTTX_SENSORS_MS58XX_H */
