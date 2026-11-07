/****************************************************************************
 * boards/xtensa/esp32/esp32-m5-stickc-plus/src/esp32_axp192.c
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

/* Minimal board-level setup of the AXP192 PMIC.
 *
 * On the M5StickC Plus the AXP192 supplies the LCD: LDO2 drives the
 * backlight and LDO3 the panel logic.  Neither rail is guaranteed to be on
 * after a cold power-up, so they must be enabled before the LCD driver
 * talks to the panel.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>

#include "esp32_i2c.h"
#include "esp32-m5-stickc-plus.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AXP192_I2C_ADDR       0x34
#define AXP192_I2C_FREQ       400000

#define AXP192_POWER_OUT_CTL  0x12  /* DCDC1/2/3, LDO2/3 and external output enables */
#define AXP192_LDO23_VOLTAGE  0x28  /* [7:4] LDO2, [3:0] LDO3 */
#define AXP192_VBUS_IPSOUT    0x30  /* VBUS-IPSOUT path management */
#define AXP192_CHARGE_CTL1    0x33  /* Charge target voltage / current */

#define POWER_OUT_DCDC1       (1 << 0)
#define POWER_OUT_DCDC3       (1 << 1)
#define POWER_OUT_LDO2        (1 << 2)
#define POWER_OUT_LDO3        (1 << 3)

/* LDOx voltage = 1.8V + 0.1V * n, n = 12 -> 3.0V for both rails */

#define LDO23_3V0             0xcc

/* 4.2V target, 100mA: the battery is a small 120mAh cell */

#define CHARGE_4V2_100MA      0xc0

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct i2c_master_s *g_axp192_i2c;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int axp192_write(uint8_t reg, uint8_t val)
{
  struct i2c_config_s config;
  uint8_t buf[2];

  config.frequency = AXP192_I2C_FREQ;
  config.address   = AXP192_I2C_ADDR;
  config.addrlen   = 7;

  buf[0] = reg;
  buf[1] = val;

  return i2c_write(g_axp192_i2c, &config, buf, sizeof(buf));
}

static int axp192_read(uint8_t reg, uint8_t *buf, size_t len)
{
  struct i2c_config_s config;

  config.frequency = AXP192_I2C_FREQ;
  config.address   = AXP192_I2C_ADDR;
  config.addrlen   = 7;

  return i2c_writeread(g_axp192_i2c, &config, &reg, 1, buf, len);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_axp192_initialize
 *
 * Description:
 *   Power up the LCD rails (LDO2 backlight, LDO3 logic) and apply the
 *   battery charging settings for this board.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_axp192_initialize(void)
{
  uint8_t val;
  int ret;

  g_axp192_i2c = esp32_i2cbus_initialize(AXP192_I2C_PORT);
  if (g_axp192_i2c == NULL)
    {
      return -ENODEV;
    }

  ret = axp192_read(AXP192_POWER_OUT_CTL, &val, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: AXP192 not responding: %d\n", ret);
      return ret;
    }

  ret = axp192_write(AXP192_VBUS_IPSOUT, 0x80);
  if (ret >= 0)
    {
      ret = axp192_write(AXP192_CHARGE_CTL1, CHARGE_4V2_100MA);
    }

  if (ret >= 0)
    {
      ret = axp192_write(AXP192_LDO23_VOLTAGE, LDO23_3V0);
    }

  if (ret >= 0)
    {
      /* DCDC3 is unused on this board.  DCDC1 powers the ESP32 itself and
       * must stay enabled.
       */

      val &= ~POWER_OUT_DCDC3;
      val |= POWER_OUT_DCDC1 | POWER_OUT_LDO2 | POWER_OUT_LDO3;
      ret = axp192_write(AXP192_POWER_OUT_CTL, val);
    }

  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: AXP192 setup failed: %d\n", ret);
      return ret;
    }

  return OK;
}
