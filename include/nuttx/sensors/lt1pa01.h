/****************************************************************************
 * include/nuttx/sensors/lt1pa01.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_LT1PA01_H
#define __INCLUDE_NUTTX_SENSORS_LT1PA01_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_I2C) && (defined(CONFIG_SENSORS_LT1PA01) || defined(CONFIG_SENSORS_LT1PA01_SCU))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Prerequisites:
 *
 * CONFIG_LT1PA01
 *   Enables support for the LT1PA01 driver
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

/* IOCTL Commands ***********************************************************
 *   These commands are valid for CONFIG_LT1PA01_PROXIMITY_INTERRUPT=y
 */

#define SNIOC_SETPROXLTHRESHOLD    _SNIOC(0x0001)  /* Set low threshold */
#define SNIOC_SETPROXHTHRESHOLD    _SNIOC(0x0002)  /* Set high threshold */
#define SNIOC_STARTPROXMEASUREMENT _SNIOC(0x0003)  /* Start measurement */
#define SNIOC_STOPPROXMEASUREMENT  _SNIOC(0x0004)  /* Stop measurement */
#define SNIOC_GETINTSTATUS         _SNIOC(0x0005)  /* Get interrupt status */

/****************************************************************************
 * Name: lt1pa01_init
 *
 * Description:
 *   Initialize LT1PA01 proximity and ambient light sensor device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             LT1PA01
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lt1pa01_init(FAR struct i2c_master_s *i2c, int port);

/****************************************************************************
 * Name: lt1pa01als_register
 *
 * Description:
 *   Register the LT1PA01 ambient light sensor character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/light0"
 *   minor   - minor device number
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             LT1PA01
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lt1pa01als_register(FAR const char *devpath, int minor,
                        FAR struct i2c_master_s *i2c, int port);

/****************************************************************************
 * Name: lt1pa01prox_register
 *
 * Description:
 *   Register the LT1PA01 proximity sensor character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/proxim0"
 *   minor   - minor device number
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             LT1PA01
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lt1pa01prox_register(FAR const char *devpath, int minor,
                         FAR struct i2c_master_s *i2c, int port);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_LT1PA01 */
#endif /* __INCLUDE_NUTTX_SENSORS_LT1PA01_H */
