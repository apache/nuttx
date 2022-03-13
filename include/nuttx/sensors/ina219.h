/****************************************************************************
 * include/nuttx/sensors/ina219.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_INA219_H
#define __INCLUDE_NUTTX_SENSORS_INA219_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_INA219)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* INA219 Register Definitions **********************************************/

/* INA219 Config Register bits */

#define INA219_CONFIG_SADC_SHIFT     3
#define INA219_CONFIG_SADC_MASK      (15 << INA219_CONFIG_SADC_SHIFT)
#define INA219_CONFIG_SADC_9BIT      ( 0 << INA219_CONFIG_SADC_SHIFT)
#define INA219_CONFIG_SADC_10BIT     ( 1 << INA219_CONFIG_SADC_SHIFT)
#define INA219_CONFIG_SADC_11BIT     ( 2 << INA219_CONFIG_SADC_SHIFT)
#define INA219_CONFIG_SADC_12BIT     ( 3 << INA219_CONFIG_SADC_SHIFT)
#define INA219_CONFIG_SADC_AVG2      ( 9 << INA219_CONFIG_SADC_SHIFT)
#define INA219_CONFIG_SADC_AVG4      (10 << INA219_CONFIG_SADC_SHIFT)
#define INA219_CONFIG_SADC_AVG8      (11 << INA219_CONFIG_SADC_SHIFT)
#define INA219_CONFIG_SADC_AVG16     (12 << INA219_CONFIG_SADC_SHIFT)
#define INA219_CONFIG_SADC_AVG32     (13 << INA219_CONFIG_SADC_SHIFT)
#define INA219_CONFIG_SADC_AVG64     (14 << INA219_CONFIG_SADC_SHIFT)
#define INA219_CONFIG_SADC_AVG128    (15 << INA219_CONFIG_SADC_SHIFT)

#define INA219_CONFIG_BADC_SHIFT     7
#define INA219_CONFIG_BADC_MASK      (15 << INA219_CONFIG_BADC_SHIFT)
#define INA219_CONFIG_BADC_9BIT      ( 0 << INA219_CONFIG_BADC_SHIFT)
#define INA219_CONFIG_BADC_10BIT     ( 1 << INA219_CONFIG_BADC_SHIFT)
#define INA219_CONFIG_BADC_11BIT     ( 2 << INA219_CONFIG_BADC_SHIFT)
#define INA219_CONFIG_BADC_12BIT     ( 3 << INA219_CONFIG_BADC_SHIFT)
#define INA219_CONFIG_BADC_AVG2      ( 9 << INA219_CONFIG_BADC_SHIFT)
#define INA219_CONFIG_BADC_AVG4      (10 << INA219_CONFIG_BADC_SHIFT)
#define INA219_CONFIG_BADC_AVG8      (11 << INA219_CONFIG_BADC_SHIFT)
#define INA219_CONFIG_BADC_AVG16     (12 << INA219_CONFIG_BADC_SHIFT)
#define INA219_CONFIG_BADC_AVG32     (13 << INA219_CONFIG_BADC_SHIFT)
#define INA219_CONFIG_BADC_AVG64     (14 << INA219_CONFIG_BADC_SHIFT)
#define INA219_CONFIG_BADC_AVG128    (15 << INA219_CONFIG_BADC_SHIFT)

#define INA219_CONFIG_PGA_X1         (0 << 11) /*  40 mV shunt voltage range */
#define INA219_CONFIG_PGA_X2         (1 << 11) /*  80 mV shunt voltage range */
#define INA219_CONFIG_PGA_X4         (2 << 11) /* 160 mV shunt voltage range */
#define INA219_CONFIG_PGA_X8         (3 << 11) /* 320 mV shunt voltage range */

#define INA219_CONFIG_RANGE_16V      0
#define INA219_CONFIG_RANGE_32V      (1 << 13)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

struct ina219_s
{
  uint32_t voltage;  /* [microvolt] max 4.2 kV - device max 26V */
  int32_t  current;  /* [microampere] max 2.1 kA - sensor is bidirectional */
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
 * Name: ina219_register
 *
 * Description:
 *  Register the ina219 character device as 'devpath'
 *
 * Input Parameters:
 *  devpath - The full path to the driver to register. E.g., "/dev/pwrmntr0"
 *  i2c - An instance of the I2C interface to use to communicate with INA219
 *  addr - The I2C address of the INA219.  The base I2C address of the INA219
 *  is 0x40.  Bits 0-3 can be controlled to get 16 unique addresses from 0x40
 *  through 0x4f.
 *  shuntval - resistor value in microohms
 *  config - a combination of the constants defined earlier in this file
 *
 * Returned Value:
 *  Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ina219_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr, int32_t shuntval, uint16_t config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_INA219 */
#endif /* __INCLUDE_NUTTX_SENSORS_INA219_H */
