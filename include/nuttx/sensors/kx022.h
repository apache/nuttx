/****************************************************************************
 * include/nuttx/sensors/kx022.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_KX022_H
#define __INCLUDE_NUTTX_SENSORS_KX022_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_I2C) && (defined(CONFIG_SENSORS_KX022) || defined(CONFIG_SENSORS_KX022_SCU))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Prerequisites:
 *
 * CONFIG_SENSORS_KX022
 *   Enables support for the KX022 driver
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

/****************************************************************************
 * Name: kx022_init
 *
 * Description:
 *   Initialize KX022 accelerometer device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             KX022
 *   port    - I2C port (0 or 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int kx022_init(FAR struct i2c_master_s *i2c, int port);

/****************************************************************************
 * Name: kx022_register
 *
 * Description:
 *   Register the KX022 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/accel0"
 *   minor   - minor device number
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             KX022
 *   port    - I2C port (0 or 1)
 *   seq     - Sequencer instance
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int kx022_register(FAR const char *devpath, int minor,
                   FAR struct i2c_master_s *i2c, int port);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_KX022 */
#endif /* __INCLUDE_NUTTX_SENSORS_KX022_H */
