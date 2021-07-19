/****************************************************************************
 * include/nuttx/sensors/ak09919c.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_AK09919C_H
#define __INCLUDE_NUTTX_SENSORS_AK09919C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_AK09919C)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ak09919c_config_s
{
  uint8_t addr;                          /* I2C address */
  int freq;                              /* I2C frequency */
  FAR struct i2c_master_s *i2c;          /* I2C interface */
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
 * Name: ak09919c_register
 *
 * Description:
 *   Register the AK09919C character device as 'devpath'.
 *
 * Input Parameters:
 *   devno   - The device number, used to build the device path
 *             as /dev/sensor/magN
 *   i2c     - An I2C driver instance.
 *   config  - configuration for the AK09919C driver. For details see
 *             description above.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ak09919c_register(int devno, FAR const struct ak09919c_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_AK09919C */
#endif /* __INCLUDE_NUTTX_SENSORS_AK09919C_H */
