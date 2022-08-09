/****************************************************************************
 * include/nuttx/sensors/scd41.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_SCD41_H
#define __INCLUDE_NUTTX_SENSORS_SCD41_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_SCD41_ADDR 0x62

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;  /* Forward reference */

struct scd41_conv_data_s
{
  float temperature; /* Celsius */
  float humidity;    /* RH-% */
  float co2;         /* CO₂ PPM */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_SCD41_I2C
/****************************************************************************
 * Name: scd41_register_i2c
 *
 * Description:
 *   Register the SCD41 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/co2_0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the SCD41
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int scd41_register_i2c(FAR const char *devpath,
                       FAR struct i2c_master_s *i2c);

#endif

#endif /* __INCLUDE_NUTTX_SENSORS_SCD41_H */
