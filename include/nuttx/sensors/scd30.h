/****************************************************************************
 * include/nuttx/sensors/scd30.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_SCD30_H
#define __INCLUDE_NUTTX_SENSORS_SCD30_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_SCD30_ADDR 0x61

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;  /* Forward reference */

struct scd30_conv_data_s
{
  float temperature; /* Celsius */
  float humidity;    /* RH-% */
  float co2;         /* CO₂ PPM */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_SCD30_I2C
/****************************************************************************
 * Name: scd30_register_i2c
 *
 * Description:
 *   Register the SCD30 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/co2_0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the SCD30
 *   addr    - The I2C address of the SCD30. The I2C address of SCD30
 *             is always 0x61.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int scd30_register_i2c(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr);
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_SCD30_H */
