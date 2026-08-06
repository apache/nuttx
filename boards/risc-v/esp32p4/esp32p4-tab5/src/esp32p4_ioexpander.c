/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-tab5/src/esp32p4_ioexpander.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/pi4ioe5v6408.h>

#include "espressif/esp_i2c.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAB5_PI4IOE_FREQUENCY       400000

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32P4_TAB5_IOEXPANDER_LOW
static struct pi4ioe5v6408_config_s g_pi4ioe_config_low =
{
  .address   = PI4IOE5V6408_I2C_ADDRESS_LOW,
  .frequency = TAB5_PI4IOE_FREQUENCY,
};
static FAR struct ioexpander_dev_s *g_pi4ioe_low;
#endif

#ifdef CONFIG_ESP32P4_TAB5_IOEXPANDER_HIGH
static struct pi4ioe5v6408_config_s g_pi4ioe_config_high =
{
  .address   = PI4IOE5V6408_I2C_ADDRESS_HIGH,
  .frequency = TAB5_PI4IOE_FREQUENCY,
};
static FAR struct ioexpander_dev_s *g_pi4ioe_high;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tab5_pi4ioe_init
 *
 * Description:
 *   Initialize the IO expanders.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

int tab5_pi4ioe_init(void)
{
  FAR struct i2c_master_s *i2c;

  i2c = esp_i2cbus_initialize(ESPRESSIF_I2C0);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "tab5_pi4ioe_init: failed to get I2C0\n");
      return -ENODEV;
    }

#ifdef CONFIG_ESP32P4_TAB5_IOEXPANDER_LOW
  g_pi4ioe_low = pi4ioe5v6408_initialize(i2c, &g_pi4ioe_config_low);
  if (g_pi4ioe_low == NULL)
    {
      syslog(LOG_ERR, "tab5_pi4ioe_init: expander (low) init failed\n");
      return -ENODEV;
    }

  syslog(LOG_INFO, "tab5_pi4ioe_init: PI4IOE5V6408 (low) initialized\n");
  #endif

#ifdef CONFIG_ESP32P4_TAB5_IOEXPANDER_HIGH
  g_pi4ioe_high = pi4ioe5v6408_initialize(i2c, &g_pi4ioe_config_high);
  if (g_pi4ioe_high == NULL)
    {
      syslog(LOG_ERR, "tab5_pi4ioe_init: expander (high) init failed\n");
      return -ENODEV;
    }

  syslog(LOG_INFO, "tab5_pi4ioe_init: PI4IOE5V6408 (high) initialized\n");
#endif

  return OK;
}

/****************************************************************************
 * Name: tab5_pi4ioe_low_write_pin
 *
 * Description:
 *   Write a pin on the IO expander (low).
 *
 * Input Parameters:
 *   pin - The pin to write on the IO expander (low).
 *   enable - True to set the pin high, false to set the pin low.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32P4_TAB5_IOEXPANDER_LOW
int tab5_pi4ioe_low_write_pin(uint8_t pin, bool enable)
{
  IOEXP_SETDIRECTION(g_pi4ioe_low, pin, IOEXPANDER_DIRECTION_OUT);
  return IOEXP_WRITEPIN(g_pi4ioe_low, pin, enable);
}
#endif

/****************************************************************************
 * Name: tab5_pi4ioe_high_write_pin
 *
 * Description:
 *   Write a pin on the IO expander (high).
 *
 * Input Parameters:
 *   pin - The pin to write on the IO expander (high).
 *   enable - True to set the pin high, false to set the pin low.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32P4_TAB5_IOEXPANDER_HIGH
int tab5_pi4ioe_high_write_pin(uint8_t pin, bool enable)
{
  return IOEXP_WRITEPIN(g_pi4ioe_high, pin, enable);
}
#endif
