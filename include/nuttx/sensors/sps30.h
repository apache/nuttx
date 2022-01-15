/****************************************************************************
 * include/nuttx/sensors/sps30.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_SPS30_H
#define __INCLUDE_NUTTX_SENSORS_SPS30_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_SPS30_ADDR 0x69

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;  /* Forward reference */

struct sps30_conv_data_s
{
  /* Mass Concentrations for particle ranges PM1.0, PM2.5, PM4.0, PM10.
   * Unit is [µg/m³] (microgram per cubicmeter).
   */

  float mass_concenration_pm1_0;
  float mass_concenration_pm2_5;
  float mass_concenration_pm4_0;
  float mass_concenration_pm10;

  /* Number Concentrations for particle ranges PM1.0, PM2.5, PM4.0, PM10.
   * Unit is [#/cm³] (number per cubic-centimeter).
   */

  float number_concenration_pm0_5;
  float number_concenration_pm1_0;
  float number_concenration_pm2_5;
  float number_concenration_pm4_0;
  float number_concenration_pm10;

  /* Typical particle size. Unit is [µm] (micrometer). */

  float typical_particle_size;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_SPS30_I2C
/****************************************************************************
 * Name: sps30_register_i2c
 *
 * Description:
 *   Register the SPS30 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g.,
 *             "/dev/particle0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the SPS30
 *   addr    - The I2C address of the SPS30. The I2C address of SPS30
 *             is always 0x69.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sps30_register_i2c(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr);
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_SPS30_H */
