/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-tab5/src/esp32p4_bringup.c
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

#include <nuttx/debug.h>
#include <syslog.h>
#include <sys/types.h>

#include <nuttx/fs/fs.h>

#include "espressif/esp_start.h"

#ifdef CONFIG_ESPRESSIF_SPIFLASH
#  include "esp_board_spiflash.h"
#endif

#ifdef CONFIG_I2C_DRIVER
#  include "esp_board_i2c.h"
#endif

#ifdef CONFIG_ESP32P4_TAB5_MIPI_DSI
#  include "espressif/esp_mipi_dsi.h"
#endif

#ifdef CONFIG_ESP32P4_TAB5_LCD
#  include <nuttx/video/fb.h>
#endif

#include <arch/board/board.h>

#include "esp32p4-tab5.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_bringup
 *
 * Description:
 *   Perform architecture-specific initialization.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      _err("Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_SPIFLASH
  ret = board_spiflash_init();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI Flash\n");
    }
#endif

#ifdef CONFIG_I2C_DRIVER
  /* Configure I2C peripheral interfaces */

  ret = board_i2c_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32P4_TAB5_IOEXPANDER
  /* Initialize the IO expanders */

  ret = tab5_pi4ioe_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: failed to init IO expanders: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_ESP32P4_TAB5_LCD_POWER
  /* MIPI PHY LDO + PI4IOE LCD_EN (before optional DSI host) */

  ret = tab5_lcd_power_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: failed to init LCD power path: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_ESP32P4_TAB5_TOUCHSCREEN
  /* Reset the touchscreen by pulsing the TOUCH_EN pin */

  ret = tab5_touchscreen_power_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: failed to reset touchscreen: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_ESP32P4_TAB5_MIPI_DSI
  /* Tab5 ST712x bus: 2 lanes @ board.h bitrate. */

  struct esp_mipi_dsi_bus_config_s bus_cfg =
    {
      .num_data_lanes = TAB5_MIPI_DSI_LANES,
      .lane_bit_rate_mbps = TAB5_MIPI_DSI_LANE_BITRATE_MBPS,
    };

  ret = esp_mipi_dsi_initialize(&bus_cfg);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: failed to init MIPI-DSI host: %d\n",
              ret);
      return ret;
    }

  syslog(LOG_INFO, "Tab5: MIPI-DSI host registered\n");
#endif

#ifdef CONFIG_ESP32P4_TAB5_LCD
  /* Selected panel DCS + DPI + DW-GDMA FB → /dev/fb0 */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: failed to register /dev/fb0: %d\n", ret);
    }
  else
    {
      /* fb_register() clears the plane; restore test red for DMA. */

      ret = tab5_fb_reload_test_pattern();
      if (ret < 0)
        {
          syslog(LOG_ERR,
                  "ERROR: failed to reload FB test pattern: %d\n", ret);
        }

      syslog(LOG_INFO, "/dev/fb0 registered (%s)\n",
              TAB5_LCD_PANEL_NAME);
    }
#endif

#ifdef CONFIG_ESP32P4_TAB5_TOUCHSCREEN
  /* Touch screen controller init. Must come after LCD and power init. */

  ret = tab5_touchscreen_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: failed to initialize touchscreen: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_TMPFS
  /* Mount the tmpfs file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      _err("Failed to mount tmpfs at %s: %d\n", CONFIG_LIBC_TMPDIR, ret);
    }
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  return ret;
}
