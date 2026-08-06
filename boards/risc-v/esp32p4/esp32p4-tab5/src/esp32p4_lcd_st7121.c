/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-tab5/src/esp32p4_lcd_st7121.c
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

/* ST7121 panel bring-up for M5Stack Tab5.
 *
 * Init init table sourced from https://github.com/espressif/esp-iot-solution
 * under components/display/lcd/esp_lcd_st7121/esp_lcd_st7121.c.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/video/mipi_dsi.h>

#include <arch/board/board.h>

#include "esp32p4-tab5.h"
#include "esp32p4_lcd_st7121.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ST7121_NAME               "st7121"
#define ST7121_COMMAND_DELAY_MS    5

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct st7121_init_cmd_s
{
  uint8_t cmd;
  FAR const uint8_t *data;
  uint8_t len;
  uint16_t delay_ms;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_st7121_cmd_60a[] =
{
  0x71, 0x21, 0xa2
};

static const uint8_t g_st7121_cmd_60b[] =
{
  0x71, 0x21, 0xa3
};

static const uint8_t g_st7121_cmd_60c[] =
{
  0x71, 0x21, 0xa4
};

static const uint8_t g_st7121_cmd_78[] =
{
  0x21
};

static const uint8_t g_st7121_cmd_79[] =
{
  0xef
};

static const uint8_t g_st7121_cmd_a4[] =
{
  0x31
};

static const uint8_t g_st7121_cmd_b7[] =
{
  0x00, 0x00, 0x5f, 0x5f, 0x44, 0x1a
};

static const uint8_t g_st7121_cmd_b0[] =
{
  0x22, 0x6b, 0x11, 0x89, 0x25, 0x43, 0x43
};

static const uint8_t g_st7121_cmd_bf[] =
{
  0xa7, 0xa7
};

static const uint8_t g_st7121_cmd_a5[] =
{
  0xf0, 0x03
};

static const uint8_t g_st7121_cmd_d7[] =
{
  0x10, 0x2c, 0x14, 0x2a, 0x80, 0x80
};

static const uint8_t g_st7121_cmd_90[] =
{
  0x71, 0x23, 0x5a, 0x20, 0x24, 0x11, 0x21
};

static const uint8_t g_st7121_cmd_a3[] =
{
  0x80, 0x01, 0x8c, 0xff, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46, 0x00,
  0x00, 0x1e, 0x5c, 0x1e, 0x80, 0x10, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x46, 0x00, 0x00, 0x1e, 0x5c, 0x1e, 0x80, 0x10, 0xef, 0x58, 0x00,
  0x00, 0x00, 0xff
};

static const uint8_t g_st7121_cmd_a6[] =
{
  0x0a, 0x00, 0x24, 0x71, 0x36, 0x00, 0x00, 0x00, 0x68, 0x68, 0x91, 0xff,
  0x00, 0x24, 0x71, 0x37, 0x00, 0x00, 0x00, 0x68, 0x68, 0x91, 0xff, 0x00,
  0x24, 0x71, 0x00, 0x00, 0x00, 0x00, 0x68, 0x68, 0x91, 0xff, 0x00, 0x2c,
  0x71, 0x00, 0x01, 0x00, 0x00, 0x68, 0x68, 0xff, 0xff, 0x00, 0x08, 0x80,
  0x08, 0x80, 0x06, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t g_st7121_cmd_a7[] =
{
  0x1a, 0x1a, 0xc0, 0x64, 0x40, 0x04, 0x15, 0x40, 0x00, 0x40, 0x00, 0x68,
  0x68, 0x91, 0xff, 0x08, 0x80, 0x64, 0x40, 0x26, 0x37, 0x40, 0x00, 0x00,
  0x00, 0x68, 0x68, 0x91, 0xff, 0x08, 0x80, 0x64, 0x40, 0x8c, 0x9d, 0x40,
  0x00, 0x00, 0x00, 0x68, 0x68, 0x91, 0xff, 0x08, 0x80, 0x64, 0x40, 0xae,
  0xbf, 0x00, 0x00, 0x20, 0x00, 0x68, 0x68, 0x91, 0xff, 0x08, 0x80, 0x79
};

static const uint8_t g_st7121_cmd_ac[] =
{
  0x1d, 0x18, 0x19, 0x1d, 0x18, 0x19, 0x04, 0x1c, 0x1d, 0x08, 0x0a, 0x10,
  0x12, 0x0c, 0x0e, 0x14, 0x16, 0x00, 0x1d, 0x1d, 0x1d, 0x1d, 0x1d, 0x18,
  0x19, 0x1d, 0x18, 0x19, 0x06, 0x1c, 0x1d, 0x09, 0x0b, 0x11, 0x13, 0x0d,
  0x0f, 0x15, 0x17, 0x02, 0x1d, 0x1d, 0x1d, 0x1d
};

static const uint8_t g_st7121_cmd_ad[] =
{
  0x0c, 0x40, 0x46, 0x00, 0x07, 0x4b, 0x4b, 0xff, 0xff, 0xf0, 0x40, 0x0e,
  0x01, 0x07, 0x42, 0x42, 0xff, 0xff, 0x01, 0x00, 0x00, 0xff, 0xff, 0xff,
  0xff
};

static const uint8_t g_st7121_cmd_ae[] =
{
  0xf0, 0xff, 0x03, 0xf0, 0xff, 0x03, 0x00
};

static const uint8_t g_st7121_cmd_b2[] =
{
  0x15, 0x19, 0x05, 0x23, 0x49, 0x2d, 0x03, 0x2e, 0x5c, 0xd2, 0xff, 0x10,
  0x60, 0xfd, 0x20, 0xc0, 0x00
};

static const uint8_t g_st7121_cmd_e8[] =
{
  0x20, 0x60, 0x04, 0x8e, 0x8e, 0x3e, 0x04, 0xdc, 0xdc, 0x3e, 0x06, 0xfa,
  0x26, 0x3e
};

static const uint8_t g_st7121_cmd_75[] =
{
  0x03, 0x04
};

static const uint8_t g_st7121_cmd_e7[] =
{
  0x4b, 0x00, 0x00, 0xbe, 0x4b, 0x8c, 0x20, 0x1a, 0xf0, 0x7d, 0x14, 0x7d,
  0x14, 0x7d, 0x14, 0x7d, 0x14, 0xff, 0x00, 0x32, 0x30, 0x73, 0x00, 0x00,
  0xc8, 0x6a, 0xff, 0x5a, 0x64, 0x38, 0x88, 0x15, 0xb1, 0x01, 0x01, 0x64,
  0x01, 0x01, 0x7c, 0xff, 0x1a, 0x51
};

static const uint8_t g_st7121_cmd_e1[] =
{
  0x0c, 0x0c
};

static const uint8_t g_st7121_cmd_ea[] =
{
  0x15, 0x00, 0x01
};

static const uint8_t g_st7121_cmd_c8[] =
{
  0x00, 0x00, 0x04, 0x08, 0x10, 0x00, 0x1f, 0x01, 0x39, 0x3e, 0x00, 0x78,
  0x06, 0xe2, 0x02, 0x11, 0x33, 0x01, 0x7a, 0x0d, 0x21, 0xc4, 0x0b, 0x19,
  0x08, 0x32, 0xa0, 0x08, 0x1a, 0x0a, 0xf3, 0x7f, 0x0e, 0xc5, 0xe8, 0x03,
  0xff
};

static const uint8_t g_st7121_cmd_c9[] =
{
  0x00, 0x00, 0x04, 0x08, 0x10, 0x00, 0x1f, 0x01, 0x39, 0x3e, 0x00, 0x78,
  0x06, 0xe2, 0x02, 0x11, 0x33, 0x01, 0x7a, 0x0d, 0x21, 0xc4, 0x0b, 0x19,
  0x08, 0x32, 0xa0, 0x08, 0x1a, 0x0a, 0xf3, 0x7f, 0x0e, 0xc5, 0xe8, 0x03,
  0xff
};

static const uint8_t g_st7121_cmd_60d[] =
{
  0x71, 0x21, 0x00
};

static const uint8_t g_st7121_cmd_35[] =
{
  0x00
};

static const struct st7121_init_cmd_s g_st7121_init[] =
{
  {0x60, g_st7121_cmd_60a, sizeof(g_st7121_cmd_60a), 0},
  {0x60, g_st7121_cmd_60b, sizeof(g_st7121_cmd_60b), 0},
  {0x60, g_st7121_cmd_60c, sizeof(g_st7121_cmd_60c), 0},
  {0x78, g_st7121_cmd_78, sizeof(g_st7121_cmd_78), 0},
  {0x79, g_st7121_cmd_79, sizeof(g_st7121_cmd_79), 0},
  {0xa4, g_st7121_cmd_a4, sizeof(g_st7121_cmd_a4), 0},
  {0xb7, g_st7121_cmd_b7, sizeof(g_st7121_cmd_b7), 0},
  {0xb0, g_st7121_cmd_b0, sizeof(g_st7121_cmd_b0), 0},
  {0xbf, g_st7121_cmd_bf, sizeof(g_st7121_cmd_bf), 0},
  {0xa5, g_st7121_cmd_a5, sizeof(g_st7121_cmd_a5), 0},
  {0xd7, g_st7121_cmd_d7, sizeof(g_st7121_cmd_d7), 0},
  {0x90, g_st7121_cmd_90, sizeof(g_st7121_cmd_90), 0},
  {0xa3, g_st7121_cmd_a3, sizeof(g_st7121_cmd_a3), 0},
  {0xa6, g_st7121_cmd_a6, sizeof(g_st7121_cmd_a6), 0},
  {0xa7, g_st7121_cmd_a7, sizeof(g_st7121_cmd_a7), 0},
  {0xac, g_st7121_cmd_ac, sizeof(g_st7121_cmd_ac), 0},
  {0xad, g_st7121_cmd_ad, sizeof(g_st7121_cmd_ad), 0},
  {0xae, g_st7121_cmd_ae, sizeof(g_st7121_cmd_ae), 0},
  {0xb2, g_st7121_cmd_b2, sizeof(g_st7121_cmd_b2), 0},
  {0xe8, g_st7121_cmd_e8, sizeof(g_st7121_cmd_e8), 0},
  {0x75, g_st7121_cmd_75, sizeof(g_st7121_cmd_75), 0},
  {0xe7, g_st7121_cmd_e7, sizeof(g_st7121_cmd_e7), 0},
  {0xe1, g_st7121_cmd_e1, sizeof(g_st7121_cmd_e1), 0},
  {0xea, g_st7121_cmd_ea, sizeof(g_st7121_cmd_ea), 0},
  {0xc8, g_st7121_cmd_c8, sizeof(g_st7121_cmd_c8), 0},
  {0xc9, g_st7121_cmd_c9, sizeof(g_st7121_cmd_c9), 0},
  {0x60, g_st7121_cmd_60d, sizeof(g_st7121_cmd_60d), 0},
  {0x11, NULL, 0, 80},
  {0x29, NULL, 0, 800},
  {0x35, g_st7121_cmd_35, sizeof(g_st7121_cmd_35), 0},
};

#define ST7121_INIT_COUNT (sizeof(g_st7121_init) / sizeof(g_st7121_init[0]))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: st7121_send_init
 ****************************************************************************/

static int st7121_send_init(FAR struct mipi_dsi_device *device)
{
  unsigned int i;
  ssize_t n;

  for (i = 0; i < ST7121_INIT_COUNT; i++)
    {
      FAR const struct st7121_init_cmd_s *cmd = &g_st7121_init[i];

      n = mipi_dsi_dcs_write(device, cmd->cmd, cmd->data, cmd->len);
      if (n < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: DCS 0x%02x failed: %zd (cmd %u/%u)\n",
                 cmd->cmd, n, i + 1, ST7121_INIT_COUNT);
          return (int)n;
        }

      if (cmd->delay_ms > 0)
        {
          up_mdelay(cmd->delay_ms);
        }

      up_mdelay(ST7121_COMMAND_DELAY_MS);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tab5_st7121_initialize
 ****************************************************************************/

FAR struct mipi_dsi_device *tab5_st7121_initialize(
      FAR struct mipi_dsi_host *host)
{
  FAR struct mipi_dsi_device *device;
  int ret;

  if (host == NULL)
    {
      return NULL;
    }

  tab5_lcd_backlight(false);

  ret = tab5_lcd_enable(true);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: LCD_EN enable failed: %d\n", ret);
      return NULL;
    }

  device = mipi_dsi_device_register(host, ST7121_NAME, 0);
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

  ret = mipi_dsi_dcs_soft_reset(device);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: soft reset failed: %d\n", ret);
      return NULL;
    }

  up_mdelay(120);

  ret = st7121_send_init(device);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: DCS init failed: %d\n", ret);
      return NULL;
    }

  return device;
}
