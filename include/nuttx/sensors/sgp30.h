/****************************************************************************
 * include/nuttx/sensors/sgp30.h
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

#ifndef __INCLUDE_NUTT_SENSORS_SGP30_H
#define __INCLUDE_NUTT_SENSORS_SGP30_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_SGP30_ADDR 0x58

#define CONFIG_SGP30_RESET_ADDR 0x00
#define CONFIG_SGP30_RESET_SECOND_BYTE 0x06

#define CONFIG_SGP30_RESET_DELAY_US 300000

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;  /* Forward reference */

struct sgp30_conv_data_s
{
  uint16_t co2eq_ppm;
  uint16_t tvoc_ppb;
};

struct sgp30_raw_data_s
{
  uint16_t h2_signal;
  uint16_t ethanol_signal;
};

struct sgp30_baseline_s
{
  uint16_t co2eq_baseline;
  uint16_t tvoc_baseline;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sgp30_register
 *
 * Description:
 *   Register the SGP30 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/gas0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the SGP30
 *   addr    - The I2C address of the SGP30. The I2C address of SGP30
 *             is always 0x58.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sgp30_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                   uint8_t addr);

#endif /* __INCLUDE_NUTT_SENSORS_SHT21_H */
