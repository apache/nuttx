/****************************************************************************
 * include/nuttx/sensors/sps30.h
 *
 *   Copyright (C) 2019 Haltian Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the followirng conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTT_SENSORS_SPS30_H
#define __INCLUDE_NUTT_SENSORS_SPS30_H

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
 *   devpath - The full path to the driver to register. E.g., "/dev/particle0"
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

#endif /* __INCLUDE_NUTT_SENSORS_SHT21_H */
