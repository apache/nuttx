/****************************************************************************
 * include/nuttx/sensors/ccs811.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_CCS811_H
#define __INCLUDE_NUTTX_SENSORS_CCS811_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ccs811_config_s
{
  /* Optional board callback to control the CCS811 nWAKE line around each
   * I2C transaction (on=true asserts nWAKE, on=false releases it).  Leave
   * NULL when nWAKE is tied low in hardware.
   */

  CODE void (*wake)(bool on);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: ccs811_register_uorb
 *
 * Description:
 *   Register the CCS811 eCO2 and TVOC sensors as UORB devices.
 *
 * Input Parameters:
 *   devno  - The device number, used to build the device path.
 *   i2c    - The I2C bus driver instance.
 *   addr   - The I2C address of the CCS811.
 *   config - Board-specific configuration (may be NULL).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ccs811_register_uorb(int devno, FAR struct i2c_master_s *i2c,
                         uint8_t addr,
                         FAR const struct ccs811_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_CCS811_H */
