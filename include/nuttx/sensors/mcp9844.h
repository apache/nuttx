/****************************************************************************
 * include/nuttx/sensors/mcp9844.h
 *
 *   Copyright (C) 2016, DS-Automotion GmbH. All rights reserved.
 *   Author: Alexander Entinger <a.entinger@ds-automotion.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
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
 *   Register the MCP9844 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c - An instance of the I2C interface to use to communicate with MCP9844
 *   addr - The I2C address of the MCP9844.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
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
