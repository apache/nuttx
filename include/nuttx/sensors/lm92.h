/****************************************************************************
 * include/nuttx/sensors/lm92.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_LM92_H
#define __INCLUDE_NUTTX_SENSORS_LM92_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LM92)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration
 *
 * CONFIG_I2C - Enables support for I2C drivers
 * CONFIG_SENSORS_LM92 - Enables support for the LM92 driver
 */

#define CONFIG_LM92_BASEADDR 0x48
#define CONFIG_LM92_ADDR0 (CONFIG_LM92_BASEADDR + 0)
#define CONFIG_LM92_ADDR1 (CONFIG_LM92_BASEADDR + 1)
#define CONFIG_LM92_ADDR2 (CONFIG_LM92_BASEADDR + 2)
#define CONFIG_LM92_ADDR3 (CONFIG_LM92_BASEADDR + 3)

/* LM92 Register Definitions ************************************************/

/* LM92 Register Addresses */

#define LM92_TEMP_REG      0x00     /* Temperature Register */
#define LM92_CONF_REG      0x01     /* Configuration Register */
#define LM92_THYS_REG      0x02     /* Temperature Register */
#define LM92_TCRIT_REG     0x03     /* Critical Temperature Register */
#define LM92_TLOW_REG      0x04     /* Low Temperature Register */
#define LM92_THIGH_REG     0x05     /* High Temperature Register */
#define LM92_ID_REG        0x07     /* Manufacturer's Identification Register */

/* Configuration Register Bit Definitions */

#define LM92_CONF_SHUTDOWN      (1 << 0) /* Bit 0: Put LM92 goes in low power shutdown mode */
#define LM92_CONF_INTMODE       (1 << 1) /* Bit 1: 0=Comparator 1=Interrupt mode */
#define LM92_CONF_TCRITPOLARITY (1 << 2) /* Bit 2: 0=Active low 1=Active high */
#define LM92_CONF_INTPOLARITY   (1 << 3) /* Bit 3: 0=Active low 1=Active high */
#define LM92_CONF_FAULTQ        (1 << 4) /* Bit 4: 0=Disabled 1=Enabled */

/* NOTE: When temperature values are read, they are returned as b16_t, fixed
 * precision integer values (see include/fixedmath.h).
 */

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
 * Name: lm92_register
 *
 * Description:
 *   Register the LM92 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0".
 *   i2c     - An instance of the I2C interface to use to communicate
 *             with the LM92.
 *   addr    - The I2C address of the LM92.  The base I2C address of the LM92
 *             is 0x48.  Bits 0-2 can be controlled to get 4 unique addresses
 *             from 0x48 through 0x4b.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lm92_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                  uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_LM92 */
#endif /* __INCLUDE_NUTTX_SENSORS_LM92_H */
