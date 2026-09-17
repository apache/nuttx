/****************************************************************************
 * include/nuttx/sensors/tc74.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_TC74_H
#define __INCLUDE_NUTTX_SENSORS_TC74_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_TC74)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TC74 Factory I2C Addresses (7-bit)
 * The address is factory programmed according to the part suffix (A0-A7).
 */

#define TC74_ADDR_A0               0x48
#define TC74_ADDR_A1               0x49
#define TC74_ADDR_A2               0x4a
#define TC74_ADDR_A3               0x4b
#define TC74_ADDR_A4               0x4c
#define TC74_ADDR_A5               0x4d
#define TC74_ADDR_A6               0x4e
#define TC74_ADDR_A7               0x4f

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
 * Name: tc74_register
 *
 * Description:
 *   Register the TC74 sensor as a uORB temperature sensor, appearing as
 *   /dev/uorb/sensor_temp<devno>.
 *
 * Input Parameters:
 *   devno - The topic number, giving /dev/uorb/sensor_temp<devno>
 *   i2c   - An instance of the I2C interface to communicate with the TC74
 *   addr  - The 7-bit I2C address of the TC74 (e.g., TC74_ADDR_A5)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int tc74_register(int devno, FAR struct i2c_master_s *i2c, uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_TC74 */
#endif /* __INCLUDE_NUTTX_SENSORS_TC74_H */
