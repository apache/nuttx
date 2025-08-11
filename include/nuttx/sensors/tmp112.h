/****************************************************************************
 * include/nuttx/sensors/tmp112.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_TMP112_H
#define __INCLUDE_NUTTX_SENSORS_TMP112_H

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_TMP112)

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TMP112_REG_CONFIG   0x01
#define TMP112_REG_TEMP     0x00

/* Configuration ************************************************************/

/* Prerequisites:
 *
 * CONFIG_SENSORS_TMP112
 *   Enables support for the TMP112 driver
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN  extern "C"
extern "C"
{
#else
#define EXTERN  extern
#endif

/****************************************************************************
 * Name: tmp112_register
 *
 * Description:
 *   Register the TMP112 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   devnum  - The index of the TMP112 sensor to interface with.
 *             When multiple sensors are connected to the same bus, each one
 *             must be electrically configured to use a different I2C
 *             address as specified in the datasheet.
 *   i2c     - An instance of the I2C interface to use.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int tmp112_register(FAR const char *devpath, const uint8_t devnum,
                    FAR struct i2c_master_s *i2c);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_TMP112 */
#endif /* __INCLUDE_NUTTX_SENSORS_TMP112_H */
