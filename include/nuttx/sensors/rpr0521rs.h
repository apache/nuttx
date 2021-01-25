/****************************************************************************
 * include/nuttx/sensors/rpr0521rs.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_RPR0521RS_H
#define __INCLUDE_NUTTX_SENSORS_RPR0521RS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_I2C) && (defined(CONFIG_SENSORS_RPR0521RS) || defined(CONFIG_SENSORS_RPR0521RS_SCU))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Prerequisites:
 *
 * CONFIG_RPR0521RS
 *   Enables support for the RPR0521RS driver
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

#define SNIOC_SETTHRESHOLD      _SNIOC(0x0001)   /* Set proximity threshold */
#define SNIOC_SETPERSISTENCE    _SNIOC(0x0002)   /* Set proximity interrupt persistence */
#define SNIOC_STARTMEASUREMENT  _SNIOC(0x0003)   /* Start proximity measurement */
#define SNIOC_STOPMEASUREMENT   _SNIOC(0x0004)   /* Stop proximity measurement */
#define SNIOC_GETINTSTATUS      _SNIOC(0x0005)   /* Get proximity interrupt status */

/****************************************************************************
 * Name: rpr0521rs_init
 *
 * Description:
 *   Initialize RPR0521RS proximity and ambient light sensor device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             RPR0521RS
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rpr0521rs_init(FAR struct i2c_master_s *i2c, int port);

/****************************************************************************
 * Name: rpr0521rsals_register
 *
 * Description:
 *   Register the RPR0521RS ambient light sensor character device as
 *   'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/light0"
 *   minor   - minor device number
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             RPR0521RS
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rpr0521rsals_register(FAR const char *devpath, int minor,
                          FAR struct i2c_master_s *i2c, int port);

/****************************************************************************
 * Name: rpr0521rsps_register
 *
 * Description:
 *   Register the RPR0521RS proximity sensor character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/proxim0"
 *   minor   - minor device number
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             RPR0521RS
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rpr0521rsps_register(FAR const char *devpath, int minor,
                         FAR struct i2c_master_s *i2c, int port);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_RPR0521RS */
#endif /* __INCLUDE_NUTTX_SENSORS_RPR0521RS_H */
