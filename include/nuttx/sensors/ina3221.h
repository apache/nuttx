/****************************************************************************
 * include/nuttx/sensors/ina3221.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_INA3221_H
#define __INCLUDE_NUTTX_SENSORS_INA3221_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_INA3221)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define INA3221_I2C_ADDR0   0x40
#define INA3221_I2C_ADDR1   0x41
#define INA3221_I2C_ADDR2   0x42
#define INA3221_I2C_ADDR3   0x43

/* INA3221 Register Definitions *********************************************/

/* INA3221 Config Register bits */

#define INA3221_CONFIG_MODE_POWERDOWN   0x0
#define INA3221_CONFIG_MODE_SHUNT_TRIG  0x1
#define INA3221_CONFIG_MODE_BUS_TRIG    0x2
#define INA3221_CONFIG_MODE_BOTH_TRIG   0x3
#define INA3221_CONFIG_MODE_POWERDOWN_1 0x4
#define INA3221_CONFIG_MODE_SHUNT_CONT  0x5
#define INA3221_CONFIG_MODE_BUS_CONT    0x6
#define INA3221_CONFIG_MODE_BOTH_CONT   0x7

#define INA3221_CONFIG_VSHUNTCT_140_US  (0x0  << 3)
#define INA3221_CONFIG_VSHUNTCT_204_US  (0x1  << 3)
#define INA3221_CONFIG_VSHUNTCT_332_US  (0x2  << 3)
#define INA3221_CONFIG_VSHUNTCT_588_US  (0x3  << 3)
#define INA3221_CONFIG_VSHUNTCT_1100_US (0x4  << 3)
#define INA3221_CONFIG_VSHUNTCT_2116_US (0x5  << 3)
#define INA3221_CONFIG_VSHUNTCT_4156_US (0x6  << 3)
#define INA3221_CONFIG_VSHUNTCT_8244_US (0x7  << 3)

#define INA3221_CONFIG_VBUSCT_140_US  (0x0 << 6)
#define INA3221_CONFIG_VBUSCT_204_US  (0x1 << 6)
#define INA3221_CONFIG_VBUSCT_332_US  (0x2 << 6)
#define INA3221_CONFIG_VBUSCT_588_US  (0x3 << 6)
#define INA3221_CONFIG_VBUSCT_1100_US (0x4 << 6)
#define INA3221_CONFIG_VBUSCT_2116_US (0x5 << 6)
#define INA3221_CONFIG_VBUSCT_4156_US (0x6 << 6)
#define INA3221_CONFIG_VBUSCT_8244_US (0x7 << 6)

#define INA3221_CONFIG_AVG_NSAMPLES_1     (0x0 << 9)
#define INA3221_CONFIG_AVG_NSAMPLES_4     (0x1 << 9)
#define INA3221_CONFIG_AVG_NSAMPLES_16    (0x2 << 9)
#define INA3221_CONFIG_AVG_NSAMPLES_64    (0x3 << 9)
#define INA3221_CONFIG_AVG_NSAMPLES_128   (0x4 << 9)
#define INA3221_CONFIG_AVG_NSAMPLES_256   (0x5 << 9)
#define INA3221_CONFIG_AVG_NSAMPLES_512   (0x6 << 9)
#define INA3221_CONFIG_AVG_NSAMPLES_1024  (0x7 << 9)

#define INA3221_CONFIG_CH1_EN (1 << 14)
#define INA3221_CONFIG_CH2_EN (1 << 13)
#define INA3221_CONFIG_CH3_EN (1 << 12)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

struct ina3221_config_s
{
  uint8_t addr;
  int32_t shunt_resistor[3];
  uint16_t cfgreg;
};

struct ina3221_channel_s
{
  uint32_t voltage;  /* [microvolt] max 4.2 kV - device max 26V */
  int32_t  current;  /* [microampere] max 2.1 kA - sensor is bidirectional */
};

struct ina3221_s
{
  struct ina3221_channel_s ch[3];
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
 * Name: ina3221_register
 *
 * Description:
 *  Register the ina3221 character device as 'devpath'
 *
 * Input Parameters:
 *  devpath - The full path to the driver to register. E.g., "/dev/pwrmntr0"
 *  i2c - An instance of the I2C interface to use to communicate with INA3221
 *  addr - The I2C address of the INA3221.
 *         The base I2C address of the INA3221 is 0x80.
 *         Bits 0-1 can be controlled to get 4 unique addresses from 0x80
 *         through 0x83.
 *  shuntval - resistor value in microohms
 *  config - a combination of the constants defined earlier in this file
 *
 * Returned Value:
 *  Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ina3221_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     FAR const struct ina3221_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_INA3221 */
#endif /* __INCLUDE_NUTTX_SENSORS_INA3221_H */
