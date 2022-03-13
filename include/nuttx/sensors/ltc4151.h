/****************************************************************************
 * include/nuttx/sensors/ltc4151.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_LTC4151_H
#define __INCLUDE_NUTTX_SENSORS_LTC4151_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>
#include <fixedmath.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LTC4151)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_LTC4151_BASEADDR 0x66
#define CONFIG_LTC4151_ADDR0    (CONFIG_LTC4151_BASEADDR + 0)
#define CONFIG_LTC4151_ADDR1    (CONFIG_LTC4151_BASEADDR + 1)
#define CONFIG_LTC4151_ADDR2    (CONFIG_LTC4151_BASEADDR + 2)
#define CONFIG_LTC4151_ADDR3    (CONFIG_LTC4151_BASEADDR + 3)
#define CONFIG_LTC4151_ADDR4    (CONFIG_LTC4151_BASEADDR + 4)
#define CONFIG_LTC4151_ADDR5    (CONFIG_LTC4151_BASEADDR + 5)
#define CONFIG_LTC4151_ADDR6    (CONFIG_LTC4151_BASEADDR + 6)
#define CONFIG_LTC4151_ADDR7    (CONFIG_LTC4151_BASEADDR + 7)
#define CONFIG_LTC4151_ADDR8    (CONFIG_LTC4151_BASEADDR + 8)
#define CONFIG_LTC4151_ADDR9    (CONFIG_LTC4151_BASEADDR + 9)

/* LTC4151 Register Definitions *********************************************/

/* LTC4151 Registers addresses */

#define LTC4151_CURR_REG        0x00     /* Current Register start address */
#define LTC4151_VOLT_REG        0x02     /* Voltage Register start address*/

#define LTC4151_VALUE_MSB_MASK  0x00ff
#define LTC4151_VALUE_LSB_MASK  0x0f00

/* NOTE: When meassurement values are read, they are return as b16_t, fixed
 * precision integer values (see include/fixedmath.h).
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

struct ltc4151_s
{
  b16_t current;  /* [milliampere] */
  b16_t voltage;  /* [volt] */
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
 * Name: ltc4151_register
 *
 * Description:
 *  Register the ltc4151 character device as 'devpath'
 *
 * Input Parameters:
 *  devpath - The full path to the driver to register. E.g., "/dev/pwrmntr0"
 *  i2c - An instance of the I2C interface to use to communicate with LTC4151
 *  addr - The I2C address of the LTC4151.
 *         The base I2C address of the LTC4151 is 0x18.
 *         Bits 0-3 can be controlled to get 8 unique addresses from 0x18
 *         through 0x1f.
 *   shunt_resistor_value - resistor value in ohm
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ltc4151_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr, float shunt_resistor_value);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_LTC4151 */
#endif /* __INCLUDE_NUTTX_SENSORS_LTC4151_H */
