/****************************************************************************
 * include/nuttx/sensors/mcp9844.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_MCP9844_H
#define __INCLUDE_NUTTX_SENSORS_MCP9844_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MCP9844)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MCP9844 Register Definitions *********************************************/

/* MCP9844 Registers addresses */

#define MCP9844_CAPA_REG      (0x00)     /* Sensor Capability Register */
#define MCP9844_CONF_REG      (0x01)     /* Sensor Configuration Register */
#define MCP9844_TEMP_REG      (0x05)     /* Sensor Temperature Register */
#define MCP9844_RESO_REG      (0x09)     /* Register to control the resolution of the temperature sensor */

/* Configuration Register Bit definitions */

#define MCP9844_CONF_REG_SHDN   (1<<8)

/* Resolution Register Bit definitions */

#define MCP9844_RESO_REG_BIT_0  (1<<0)
#define MCP9844_RESO_REG_BIT_1  (1<<1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mcp9844_temp_arg_s
{
  int8_t temp_pre_comma;
  uint8_t temp_post_comma;
};

enum mcp9844_resolution_e
{
  RES_0_5    = 0,
  RES_0_25   = MCP9844_RESO_REG_BIT_0,
  RES_0_125  = MCP9844_RESO_REG_BIT_1,
  RES_0_0625 = MCP9844_RESO_REG_BIT_1 | MCP9844_RESO_REG_BIT_0
};

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
 * Name: mcp9844_register
 *
 * Description:
 *  Register the MCP9844 character device as 'devpath'
 *
 * Input Parameters:
 *  devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *  i2c - An instance of the I2C interface to use to communicate with MCP9844
 *  addr - The I2C address of the MCP9844.
 *
 * Returned Value:
 *  Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mcp9844_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_MCP9844 */
#endif /* __INCLUDE_NUTTX_SENSORS_MCP9844_H */
