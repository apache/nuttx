/****************************************************************************
 * include/nuttx/sensors/veml6070.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_VEML6070_H
#define __INCLUDE_NUTTX_SENSORS_VEML6070_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_SENSORS_VEML6070)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device I2C Address */

#define VEML6070_I2C_DATA_LSB_CMD_ADDR  0x38
#define VEML6070_I2C_DATA_MSB_ADDR      0x39

/* Command Register Format
 * Bits:
 *  7  |  6  |  5  |    4    |  3  |  2  |  1  |  0 |
 * RSV | RSV | ACK | ACK_THD | IT1 | IT0 | RSV | SD |
 *
 * NOTE: The RSV Bit 1 needs to be always 1
 */

#define VEML6070_CMD_SD           0x01 /* Shutdown command */
#define VEML6070_CMD_RSV          0x02
#define VEML6070_CMD_IT_0_5T      0x00 /* IT1=0 : IT0=0 */
#define VEML6070_CMD_IT_1T        0x04 /* IT1=0 : IT0=1 */
#define VEML6070_CMD_IT_2T        0x08 /* IT1=1 : IT0=0 */
#define VEML6070_CMD_IT_4T        0x0c /* IT1=1 : IT0=1 */
#define VEML6070_CMD_ACK_THD      0x10 /* Acknowledge threshold:
                                        *  0 = 102 steps
                                        *  1 = 145 steps */
#define VEML6070_CMD_ACK          0x20 /* Acknowledge activity */

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
 * Name: veml6070_register
 *
 * Description:
 *   Register the VEML6070 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/uvlight0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *              VEML6070
 *   addr    - The I2C address of the VEML6070.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

struct i2c_master_s;
int veml6070_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_VEML6070 */
#endif /* __INCLUDE_NUTTX_SENSORS_VEML6070_H */
