/****************************************************************************
 * include/nuttx/sensors/sht4x.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_SHT4X_H
#define __INCLUDE_NUTTX_SENSORS_SHT4X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s; /* Forward reference */

/* Precision for reading. */

enum sht4x_precision_e
{
  SHT4X_PREC_LOW = 0,  /* Low precision. */
  SHT4X_PREC_MED = 1,  /* Medium precision. */
  SHT4X_PREC_HIGH = 2, /* High precision. */
};

/* Durations and power for heating. */

enum sht4x_heater_e
{
  SHT4X_HEATER_200MW_1 = 0,  /* Activate heater with 200mW for 1s. */
  SHT4X_HEATER_200MW_01 = 1, /* Activate heater with 200mW for 0.1s. */
  SHT4X_HEATER_110MW_1 = 2,  /* Activate heater with 110mW for 1s. */
  SHT4X_HEATER_110MW_01 = 3, /* Activate heater with 110mW for 0.1s. */
  SHT4X_HEATER_20MW_1 = 4,   /* Activate heater with 20mW for 1s. */
  SHT4X_HEATER_20MW_01 = 5,  /* Activate heater with 20mW for 0.1s. */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sht4x_register
 *
 * Description:
 *   Register the SHT4X character device as 'devpath'
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the SHT4X
 *   devno   - The device number that this device should have.
 *   addr    - The I2C address of the SHT4X. The I2C address is one of 0x44,
 *             0x45 and 0x46.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sht4x_register(FAR struct i2c_master_s *i2c, int devno, uint8_t addr);

#endif /* __INCLUDE_NUTTX_SENSORS_SHT4X_H */
