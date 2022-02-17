/****************************************************************************
 * include/nuttx/sensors/hdc1008.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_HDC1008_H
#define __INCLUDE_NUTTX_SENSORS_HDC1008_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default address if both address pins are grounded */

#define CONFIG_HDC1008_ADDR 0x40

/* Modes of operation. Used in the calls to ioctl(). */

#define HDC1008_MEAS_TEMPERATURE  0x00
#define HDC1008_MEAS_HUMIDITY     0x01
#define HDC1008_MEAS_T_AND_RH     0x02

/* IOCTL commands */

#define SNIOC_RESET                 _SNIOC(0x0001) /* Soft Reset */
#define SNIOC_SET_OPERATIONAL_MODE  _SNIOC(0x0002) /* Mode: 0, 1, 2 */
#define SNIOC_SET_RESOLUTION_T      _SNIOC(0x0003) /* Resolution: 11, 14 */
#define SNIOC_SET_RESOLUTION_RH     _SNIOC(0x0004) /* Resolution: 8, 11, 14 */
#define SNIOC_SET_HEATER_MODE       _SNIOC(0x0005) /* Heater on or off */
#define SNIOC_GET_CONFIGURATION     _SNIOC(0x0006) /* Read config register */
#define SNIOC_MEASURE               _SNIOC(0x0007) /* Perform measurement */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;  /* Forward reference */

/* Structure with measurement data. Temperature is scaled by 100 and humidity
 * by 10.
 */

struct hdc1008_conv_data_s
{
  int temperature;
  int humidity;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: hdc1008_register
 *
 * Description:
 *   Register the HDC1008 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the HDC1008
 *   addr    - The I2C address of the HDC1008. The I2C address is
 *             configurable by two address pins, in the range of 0x40-0x43
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int hdc1008_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr);

#endif /* __INCLUDE_NUTTX_SENSORS_HDC1008_H */
