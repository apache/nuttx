/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-tab5/src/esp32p4_lcd_st7123.c
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

/* ST7123 panel bring-up for M5Stack Tab5.
 *
 * Init init table sourced from https://github.com/espressif/esp-iot-solution
 * under components/display/lcd/esp_lcd_st7123/esp_lcd_st7123.c.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <syslog.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/video/mipi_dsi.h>

#include <arch/board/board.h>

#include "esp32p4-tab5.h"
#include "esp32p4_lcd_st7123.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ST7123_NAME  "st7123"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct st7123_init_cmd_s
{
  uint8_t cmd;
  FAR const uint8_t *data;
  uint8_t len;
  uint16_t delay_ms;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ESP-BSP Tab5 ST7123 vendor table (disp_init_data_st7123). */

static const uint8_t g_st7123_cmd_60a[] =
{
  0x71, 0x23, 0xa2
};

static const uint8_t g_st7123_cmd_60b[] =
{
  0x71, 0x23, 0xa3
};

static const uint8_t g_st7123_cmd_60c[] =
{
  0x71, 0x23, 0xa4
};

static const uint8_t g_st7123_cmd_a4[] =
{
  0x31
};

static const uint8_t g_st7123_cmd_d7[] =
{
  0x10, 0x0a, 0x10, 0x2a, 0x80, 0x80
};

static const uint8_t g_st7123_cmd_90[] =
{
  0x71, 0x23, 0x5a, 0x20, 0x24, 0x09, 0x09
};

static const uint8_t g_st7123_cmd_a3[] =
{
  0x80, 0x01, 0x88, 0x30, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46, 0x00,
  0x00, 0x1e, 0x5c, 0x1e, 0x80, 0x00, 0x4f, 0x05, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x46, 0x00, 0x00, 0x1e, 0x5c, 0x1e, 0x80, 0x00, 0x6f, 0x58, 0x00,
  0x00, 0x00, 0xff
};

static const uint8_t g_st7123_cmd_a6[] =
{
  0x03, 0x00, 0x24, 0x55, 0x36, 0x00, 0x39, 0x00, 0x6e, 0x6e, 0x91, 0xff,
  0x00, 0x24, 0x55, 0x38, 0x00, 0x37, 0x00, 0x6e, 0x6e, 0x91, 0xff, 0x00,
  0x24, 0x11, 0x00, 0x00, 0x00, 0x00, 0x6e, 0x6e, 0x91, 0xff, 0x00, 0xec,
  0x11, 0x00, 0x03, 0x00, 0x03, 0x6e, 0x6e, 0xff, 0xff, 0x00, 0x08, 0x80,
  0x08, 0x80, 0x06, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t g_st7123_cmd_a7[] =
{
  0x19, 0x19, 0x80, 0x64, 0x40, 0x07, 0x16, 0x40, 0x00, 0x44, 0x03, 0x6e,
  0x6e, 0x91, 0xff, 0x08, 0x80, 0x64, 0x40, 0x25, 0x34, 0x40, 0x00, 0x02,
  0x01, 0x6e, 0x6e, 0x91, 0xff, 0x08, 0x80, 0x64, 0x40, 0x00, 0x00, 0x40,
  0x00, 0x00, 0x00, 0x6e, 0x6e, 0x91, 0xff, 0x08, 0x80, 0x64, 0x40, 0x00,
  0x00, 0x00, 0x00, 0x20, 0x00, 0x6e, 0x6e, 0x84, 0xff, 0x08, 0x80, 0x44
};

static const uint8_t g_st7123_cmd_ac[] =
{
  0x03, 0x19, 0x19, 0x18, 0x18, 0x06, 0x13, 0x13, 0x11, 0x11, 0x08, 0x08,
  0x0a, 0x0a, 0x1c, 0x1c, 0x07, 0x07, 0x00, 0x00, 0x02, 0x02, 0x01, 0x19,
  0x19, 0x18, 0x18, 0x06, 0x12, 0x12, 0x10, 0x10, 0x09, 0x09, 0x0b, 0x0b,
  0x1c, 0x1c, 0x07, 0x07, 0x03, 0x03, 0x01, 0x01
};

static const uint8_t g_st7123_cmd_ad[] =
{
  0xf0, 0x00, 0x46, 0x00, 0x03, 0x50, 0x50, 0xff, 0xff, 0xf0, 0x40, 0x06,
  0x01, 0x07, 0x42, 0x42, 0xff, 0xff, 0x01, 0x00, 0x00, 0xff, 0xff, 0xff,
  0xff
};

static const uint8_t g_st7123_cmd_ae[] =
{
  0xfe, 0x3f, 0x3f, 0xfe, 0x3f, 0x3f, 0x00
};

static const uint8_t g_st7123_cmd_b2[] =
{
  0x15, 0x19, 0x05, 0x23, 0x49, 0xaf, 0x03, 0x2e, 0x5c, 0xd2, 0xff, 0x10,
  0x20, 0xfd, 0x20, 0xc0, 0x00
};

static const uint8_t g_st7123_cmd_e8[] =
{
  0x20, 0x6f, 0x04, 0x97, 0x97, 0x3e, 0x04, 0xdc, 0xdc, 0x3e, 0x06, 0xfa,
  0x26, 0x3e
};

static const uint8_t g_st7123_cmd_75[] =
{
  0x03, 0x04
};

static const uint8_t g_st7123_cmd_e7[] =
{
  0x3b, 0x00, 0x00, 0x7c, 0xa1, 0x8c, 0x20, 0x1a, 0xf0, 0xb1, 0x50, 0x00,
  0x50, 0xb1, 0x50, 0xb1, 0x50, 0xd8, 0x00, 0x55, 0x00, 0xb1, 0x00, 0x45,
  0xc9, 0x6a, 0xff, 0x5a, 0xd8, 0x18, 0x88, 0x15, 0xb1, 0x01, 0x01, 0x77
};

static const uint8_t g_st7123_cmd_ea[] =
{
  0x13, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x2c
};

static const uint8_t g_st7123_cmd_b0[] =
{
  0x22, 0x43, 0x11, 0x61, 0x25, 0x43, 0x43
};

static const uint8_t g_st7123_cmd_b7[] =
{
  0x00, 0x00, 0x73, 0x73
};

static const uint8_t g_st7123_cmd_bf[] =
{
  0xa6, 0xaa
};

static const uint8_t g_st7123_cmd_a9[] =
{
  0x00, 0x00, 0x73, 0xff, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03
};

static const uint8_t g_st7123_cmd_c8[] =
{
  0x00, 0x00, 0x10, 0x1f, 0x36, 0x00, 0x5d, 0x04, 0x9d, 0x05, 0x10, 0xf2,
  0x06, 0x60, 0x03, 0x11, 0xad, 0x00, 0xef, 0x01, 0x22, 0x2e, 0x0e, 0x74,
  0x08, 0x32, 0xdc, 0x09, 0x33, 0x0f, 0xf3, 0x77, 0x0d, 0xb0, 0xdc, 0x03,
  0xff
};

static const uint8_t g_st7123_cmd_c9[] =
{
  0x00, 0x00, 0x10, 0x1f, 0x36, 0x00, 0x5d, 0x04, 0x9d, 0x05, 0x10, 0xf2,
  0x06, 0x60, 0x03, 0x11, 0xad, 0x00, 0xef, 0x01, 0x22, 0x2e, 0x0e, 0x74,
  0x08, 0x32, 0xdc, 0x09, 0x33, 0x0f, 0xf3, 0x77, 0x0d, 0xb0, 0xdc, 0x03,
  0xff
};

static const uint8_t g_st7123_cmd_36[] =
{
  0x03
};

static const uint8_t g_st7123_cmd_35[] =
{
  0x00
};

static const struct st7123_init_cmd_s g_st7123_init[] =
{
  {
    0x60, g_st7123_cmd_60a, sizeof(g_st7123_cmd_60a), 0
  },
  {
    0x60, g_st7123_cmd_60b, sizeof(g_st7123_cmd_60b), 0
  },
  {
    0x60, g_st7123_cmd_60c, sizeof(g_st7123_cmd_60c), 0
  },
  {
    0xa4, g_st7123_cmd_a4, sizeof(g_st7123_cmd_a4), 0
  },
  {
    0xd7, g_st7123_cmd_d7, sizeof(g_st7123_cmd_d7), 0
  },
  {
    0x90, g_st7123_cmd_90, sizeof(g_st7123_cmd_90), 0
  },
  {
    0xa3, g_st7123_cmd_a3, sizeof(g_st7123_cmd_a3), 0
  },
  {
    0xa6, g_st7123_cmd_a6, sizeof(g_st7123_cmd_a6), 0
  },
  {
    0xa7, g_st7123_cmd_a7, sizeof(g_st7123_cmd_a7), 0
  },
  {
    0xac, g_st7123_cmd_ac, sizeof(g_st7123_cmd_ac), 0
  },
  {
    0xad, g_st7123_cmd_ad, sizeof(g_st7123_cmd_ad), 0
  },
  {
    0xae, g_st7123_cmd_ae, sizeof(g_st7123_cmd_ae), 0
  },
  {
    0xb2, g_st7123_cmd_b2, sizeof(g_st7123_cmd_b2), 0
  },
  {
    0xe8, g_st7123_cmd_e8, sizeof(g_st7123_cmd_e8), 0
  },
  {
    0x75, g_st7123_cmd_75, sizeof(g_st7123_cmd_75), 0
  },
  {
    0xe7, g_st7123_cmd_e7, sizeof(g_st7123_cmd_e7), 0
  },
  {
    0xea, g_st7123_cmd_ea, sizeof(g_st7123_cmd_ea), 0
  },
  {
    0xb0, g_st7123_cmd_b0, sizeof(g_st7123_cmd_b0), 0
  },
  {
    0xb7, g_st7123_cmd_b7, sizeof(g_st7123_cmd_b7), 0
  },
  {
    0xbf, g_st7123_cmd_bf, sizeof(g_st7123_cmd_bf), 0
  },
  {
    0xa9, g_st7123_cmd_a9, sizeof(g_st7123_cmd_a9), 0
  },
  {
    0xc8, g_st7123_cmd_c8, sizeof(g_st7123_cmd_c8), 0
  },
  {
    0xc9, g_st7123_cmd_c9, sizeof(g_st7123_cmd_c9), 0
  },
  {
    0x36, g_st7123_cmd_36, sizeof(g_st7123_cmd_36), 0
  },

  /* 0x11 / 0x29: 0-param DCS (dummy 0x00 as 1-param broke sleep-out).
   * Delays match ESP-BSP disp_init_data_st7123 (120 / 50).
   */

  {
    0x11, NULL, 0, 120
  },
  {
    0x29, NULL, 0, 50
  },
  {
    0x35, g_st7123_cmd_35, sizeof(g_st7123_cmd_35), 0
  },
};

#define ST7123_INIT_COUNT (sizeof(g_st7123_init) / sizeof(g_st7123_init[0]))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: st7123_send_init
 *
 * Description:
 *   Send the ST7123 vendor init table.
 *
 * Input Parameters:
 *   device - Pointer to the MIPI-DSI device structure.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 ****************************************************************************/

static int st7123_send_init(FAR struct mipi_dsi_device *device)
{
  unsigned int i;
  ssize_t n;

  for (i = 0; i < ST7123_INIT_COUNT; i++)
    {
      FAR const struct st7123_init_cmd_s *cmd = &g_st7123_init[i];

      n = mipi_dsi_dcs_write(device, cmd->cmd, cmd->data, cmd->len);
      if (n < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: DCS 0x%02x failed: %zd (cmd %u/%u)\n",
                 cmd->cmd, n, i + 1, ST7123_INIT_COUNT);
          return (int)n;
        }

      if (cmd->delay_ms > 0)
        {
          up_mdelay(cmd->delay_ms);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tab5_st7123_initialize
 *
 * Description:
 *   Register the ST7123 as a mipi_dsi_device, attach to the Espressif host,
 *   pulse LCD reset via PI4IOE, and send the ESP-BSP Tab5 vendor DCS init
 *   table (disp_init_data_st7123[]). Does not start video; board code
 *   calls display_on after video_start.
 *
 * Input Parameters:
 *   host - Pointer to the MIPI-DSI host structure.
 *
 * Returned Value:
 *   Pointer to the MIPI-DSI device structure on success, NULL on failure.
 ****************************************************************************/

FAR struct mipi_dsi_device *tab5_st7123_initialize(
      FAR struct mipi_dsi_host *host)
{
  FAR struct mipi_dsi_device *device;
  int ret;

  if (host == NULL)
    {
      return NULL;
    }

  ret = tab5_lcd_enable(true);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: LCD_EN enable failed: %d\n", ret);
      return NULL;
    }

  device = mipi_dsi_device_register(host, ST7123_NAME, 0);
  if (device == NULL)
    {
      syslog(LOG_ERR, "ERROR: mipi_dsi_device_register failed\n");
      return NULL;
    }

  device->lanes = TAB5_MIPI_DSI_LANES;
  device->format = MIPI_DSI_FMT_RGB565;
  device->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
                       MIPI_DSI_MODE_LPM;
  device->hs_rate = TAB5_MIPI_DSI_LANE_BITRATE_MBPS * 1000000UL;
  device->lp_rate = 0;

  ret = mipi_dsi_attach(device);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: attach failed: %d\n", ret);
      return NULL;
    }

  ret = st7123_send_init(device);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: DCS init failed: %d\n", ret);
      return NULL;
    }

  return device;
}
