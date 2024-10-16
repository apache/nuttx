/****************************************************************************
 * arch/arm64/src/imx9/imx9_pmic.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <debug.h>
#include <sys/types.h>
#include <syslog.h>
#include <errno.h>
#include <nuttx/i2c/i2c_master.h>
#include "imx9_lpi2c.h"
#include "chip.h"
#include "arm64_internal.h"
#include "imx9_trdc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PCA9451A in IMX9 platform */

#define PCA9451A_I2C_ADDR       0x25
#define REG_SW_RST              0x06
#define REG_POWERON_STAT        0x05
#define COLD_RESET              0x64
#define WARM_RESET              0x35

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_pmic_reset
 *
 * Description:
 *   Reset SoC via pmic
 *
 ****************************************************************************/

void imx9_pmic_reset(void)
{
  struct i2c_master_s *i2c;
  struct i2c_msg_s msg;
  uint8_t buffer[2];
  int ret;

  i2c = imx9_i2cbus_initialize(CONFIG_IMX9_PMIC_I2C);

  if (i2c == NULL)
    {
      return;
    }

  buffer[0] = REG_SW_RST;
  buffer[1] = COLD_RESET;

  msg.frequency = 400000;
  msg.addr      = PCA9451A_I2C_ADDR;
  msg.flags     = 0;
  msg.buffer    = buffer;
  msg.length    = 2;

  ret = I2C_TRANSFER(i2c, &msg, 1);
  if (ret < 0)
    {
      _err("ERROR: Failed to write reset via I2C2 interface\n");
    }
}

/****************************************************************************
 * Name: imx9_get_pmic_reset_reason
 *
 * Description:
 *   Read reset reason from pmic
 *
 ****************************************************************************/

uint32_t imx9_pmic_getreset_reason(void)
{
  struct i2c_master_s *i2c;
  struct i2c_msg_s msgs[2];
  uint8_t reg_addr = REG_POWERON_STAT;
  uint8_t data;
  int ret;

  i2c = imx9_i2cbus_initialize(CONFIG_IMX9_PMIC_I2C);
  if (i2c == NULL)
    {
      _err("Failed to initialize I2C bus\n");
      return -ENODEV;
    }

  msgs[0].frequency = 400000;
  msgs[0].addr      = PCA9451A_I2C_ADDR;
  msgs[0].flags     = 0;
  msgs[0].buffer    = &reg_addr;
  msgs[0].length    = 1;

  msgs[1].frequency = 400000;
  msgs[1].addr      = PCA9451A_I2C_ADDR;
  msgs[1].flags     = I2C_M_READ;
  msgs[1].buffer    = &data;
  msgs[1].length    = 1;

  ret = I2C_TRANSFER(i2c, msgs, 2);
  if (ret < 0)
    {
      _err("I2C transfer failed: %d\n", ret);
      imx9_i2cbus_uninitialize(i2c);
      return ret;
    }

  imx9_i2cbus_uninitialize(i2c);
  return (uint32_t)data;
}
