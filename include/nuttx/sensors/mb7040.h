/****************************************************************************
 * include/nuttx/sensors/mb7040.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_MB7040_H
#define __INCLUDE_NUTTX_SENSORS_MB7040_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MB7040)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************
 * Prerequisites:
 *
 * CONFIG_I2C
 *   Enables support for I2C drivers
 * CONFIG_SENSORS_MB7040
 *   Enables support for the MB7040 driver
 */

/* I2C Addresses ************************************************************/

#define MB7040_DEFAULTADDR     0x70 /* Default I2C Address */

/* Register Definitions *****************************************************/

/* Register Addresses */

#define MB7040_RANGE_REG       0x51
#define MB7040_ADDRUNLOCK1_REG 0xaa
#define MB7040_ADDRUNLOCK2_REG 0xa5

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
 * Name: mb7040_register
 *
 * Description:
 *   Register the MB7040 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/sonar0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the MB7040.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mb7040_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_MB7040 */
#endif /* __INCLUDE_NUTTX_SENSORS_MB7040_H */
