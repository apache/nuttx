/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-tab5/src/esp32p4_hmi_power.c
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
#include <nuttx/sched.h>

#include "espressif/esp_ldo.h"
#include "espressif/esp_gpio.h"

#include "esp32p4-tab5.h"
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MIPI PHY LDO channel and voltage */

#define ESP_LDO_MIPI_PHY_CHAN       3
#define ESP_LDO_MIPI_PHY_VOLTAGE_MV 2500

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp_ldo_config_t g_mipi_phy_ldo_config =
{
  .chan_id = ESP_LDO_MIPI_PHY_CHAN,
  .voltage_mv = ESP_LDO_MIPI_PHY_VOLTAGE_MV,
  .handler = NULL,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tab5_touchscreen_power_init
 *
 * Description:
 *   Reset the touchscreen by pulsing the TOUCH_EN pin
 *   via PI4IOE P5 (BSP_TOUCH_EN). Datasheet suggests 2 ms low pulse to
 *   reset the controller and at least 20 ms in high state to stabilize the
 *   controller.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

int tab5_touchscreen_power_init(void)
{
  int ret;

  ret = tab5_pi4ioe_low_write_pin(TAB5_TOUCH_EN_PIN, true);
  nxsched_msleep(10);
  ret |= tab5_pi4ioe_low_write_pin(TAB5_TOUCH_EN_PIN, false);
  nxsched_msleep(30);
  ret |= tab5_pi4ioe_low_write_pin(TAB5_TOUCH_EN_PIN, true);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: failed to reset touchscreen: %d\n", ret);
      return ret;
    }

  nxsched_msleep(100);

  syslog(LOG_INFO, "Touchscreen reset complete\n");
  return OK;
}

/****************************************************************************
 * Name: tab5_mipi_phy_power
 *
 * Description:
 *   Enable or disable the MIPI PHY LDO.
 *
 * Input Parameters:
 *   on - True to enable the MIPI PHY LDO, false to disable it.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

int tab5_mipi_phy_power(bool on)
{
  int ret = OK;
  if (on)
    {
      ret = esp_ldo_channel_acquire(&g_mipi_phy_ldo_config);
    }
  else
    {
      ret = esp_ldo_channel_release(&g_mipi_phy_ldo_config);
    }

  return ret;
}

/****************************************************************************
 * Name: tab5_lcd_enable
 *
 * Description:
 *   Enable or disable the LCD rail via IO Expander (low).
 *
 * Input Parameters:
 *   enable - True to enable the LCD rail, false to disable it.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

int tab5_lcd_enable(bool enable)
{
  int ret;

  ret = tab5_pi4ioe_low_write_pin(TAB5_LCD_EN_PIN, enable);
  if (ret < 0)
    {
      syslog(LOG_ERR,
         "ERROR: failed to write LCD_EN pin: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "LCD %s\n", enable ? "enabled" : "disabled");
  return OK;
}

/****************************************************************************
 * Name: tab5_lcd_backlight
 *
 * Description:
 *   Configure GPIO22 as output and drive the backlight enable.
 *
 * Input Parameters:
 *   on - True to enable the backlight, false to disable it.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

int tab5_lcd_backlight(bool on)
{
  int ret;

  ret = esp_configgpio(TAB5_GPIO_LCD_BL_EN, OUTPUT);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: failed to configure backlight GPIO%d: %d\n",
             TAB5_GPIO_LCD_BL_EN, ret);
      return ret;
    }

  esp_gpiowrite(TAB5_GPIO_LCD_BL_EN, on);
  syslog(LOG_INFO, "Backlight %s\n", on ? "enabled" : "disabled");
  return OK;
}

/****************************************************************************
 * Name: tab5_lcd_power_init
 *
 * Description:
 *   Power MIPI PHY LDO, init PI4IOE, and enable LCD (P4 high).
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

int tab5_lcd_power_init(void)
{
  int ret;

  ret = tab5_mipi_phy_power(true);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: failed to power on MIPI PHY: %d\n", ret);
      return ret;
    }

  ret = tab5_lcd_enable(true);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: failed to enable LCD rail: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "LCD power/reset path ready\n");
  return OK;
}
