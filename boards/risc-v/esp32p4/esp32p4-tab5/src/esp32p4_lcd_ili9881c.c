/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-tab5/src/esp32p4_lcd_ili9881c.c
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

/* ILI9881C panel bring-up for M5Stack Tab5, fitted to the units that also
 * carry the GT911 touch controller.  Init table sourced from
 * https://github.com/espressif/esp-bsp under
 * bsp/m5stack_tab5/priv_include/disp_init_data.h.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>
#include <sys/param.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/video/mipi_display.h>
#include <nuttx/video/mipi_dsi.h>

#include <arch/board/board.h>

#include "esp32p4-tab5.h"
#include "esp32p4_lcd_ili9881c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ILI9881C_NAME           "ili9881c"

/* Command page selection: 0xff 0x98 0x81 <page> */

#define ILI9881C_CMD_PAGE       0xff
#define ILI9881C_PAGE_BYTE0     0x98
#define ILI9881C_PAGE_BYTE1     0x81
#define ILI9881C_PAGE0          0x00
#define ILI9881C_PAGE1          0x01

/* On command page 1, registers 0x00..0x02 hold the panel identification */

#define ILI9881C_REG_ID1        0x00
#define ILI9881C_ID1_ILITEK     0x98
#define ILI9881C_ID2_ILITEK     0x81

/* Time the panel needs after leaving sleep mode */

#define ILI9881C_SLEEP_OUT_MS   120

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Vendor initialization table, as a flat byte stream: DCS command, payload
 * length, payload.  It walks the panel through its command pages, so the
 * order matters, and ends back on page 0 with display on (0x29).
 */

static const uint8_t g_ili9881c_init[] =
{
  0xff, 3, 0x98, 0x81, 0x01,
  0xb7, 1, 0x03,
  0xff, 3, 0x98, 0x81, 0x03,
  0x01, 1, 0x00,
  0x02, 1, 0x00,
  0x03, 1, 0x73,
  0x04, 1, 0x00,
  0x05, 1, 0x00,
  0x06, 1, 0x08,
  0x07, 1, 0x00,
  0x08, 1, 0x00,
  0x09, 1, 0x1b,
  0x0a, 1, 0x01,
  0x0b, 1, 0x01,
  0x0c, 1, 0x0d,
  0x0d, 1, 0x01,
  0x0e, 1, 0x01,
  0x0f, 1, 0x26,
  0x10, 1, 0x26,
  0x11, 1, 0x00,
  0x12, 1, 0x00,
  0x13, 1, 0x02,
  0x14, 1, 0x00,
  0x15, 1, 0x00,
  0x16, 1, 0x00,
  0x17, 1, 0x00,
  0x18, 1, 0x00,
  0x19, 1, 0x00,
  0x1a, 1, 0x00,
  0x1b, 1, 0x00,
  0x1c, 1, 0x00,
  0x1d, 1, 0x00,
  0x1e, 1, 0x40,
  0x1f, 1, 0x00,
  0x20, 1, 0x06,
  0x21, 1, 0x01,
  0x22, 1, 0x00,
  0x23, 1, 0x00,
  0x24, 1, 0x00,
  0x25, 1, 0x00,
  0x26, 1, 0x00,
  0x27, 1, 0x00,
  0x28, 1, 0x33,
  0x29, 1, 0x03,
  0x2a, 1, 0x00,
  0x2b, 1, 0x00,
  0x2c, 1, 0x00,
  0x2d, 1, 0x00,
  0x2e, 1, 0x00,
  0x2f, 1, 0x00,
  0x30, 1, 0x00,
  0x31, 1, 0x00,
  0x32, 1, 0x00,
  0x33, 1, 0x00,
  0x34, 1, 0x00,
  0x35, 1, 0x00,
  0x36, 1, 0x00,
  0x37, 1, 0x00,
  0x38, 1, 0x00,
  0x39, 1, 0x00,
  0x3a, 1, 0x00,
  0x3b, 1, 0x00,
  0x3c, 1, 0x00,
  0x3d, 1, 0x00,
  0x3e, 1, 0x00,
  0x3f, 1, 0x00,
  0x40, 1, 0x00,
  0x41, 1, 0x00,
  0x42, 1, 0x00,
  0x43, 1, 0x00,
  0x44, 1, 0x00,
  0x50, 1, 0x01,
  0x51, 1, 0x23,
  0x52, 1, 0x45,
  0x53, 1, 0x67,
  0x54, 1, 0x89,
  0x55, 1, 0xab,
  0x56, 1, 0x01,
  0x57, 1, 0x23,
  0x58, 1, 0x45,
  0x59, 1, 0x67,
  0x5a, 1, 0x89,
  0x5b, 1, 0xab,
  0x5c, 1, 0xcd,
  0x5d, 1, 0xef,
  0x5e, 1, 0x11,
  0x5f, 1, 0x02,
  0x60, 1, 0x00,
  0x61, 1, 0x07,
  0x62, 1, 0x06,
  0x63, 1, 0x0e,
  0x64, 1, 0x0f,
  0x65, 1, 0x0c,
  0x66, 1, 0x0d,
  0x67, 1, 0x02,
  0x68, 1, 0x02,
  0x69, 1, 0x02,
  0x6a, 1, 0x02,
  0x6b, 1, 0x02,
  0x6c, 1, 0x02,
  0x6d, 1, 0x02,
  0x6e, 1, 0x02,
  0x6f, 1, 0x02,
  0x70, 1, 0x02,
  0x71, 1, 0x02,
  0x72, 1, 0x02,
  0x73, 1, 0x05,
  0x74, 1, 0x01,
  0x75, 1, 0x02,
  0x76, 1, 0x00,
  0x77, 1, 0x07,
  0x78, 1, 0x06,
  0x79, 1, 0x0e,
  0x7a, 1, 0x0f,
  0x7b, 1, 0x0c,
  0x7c, 1, 0x0d,
  0x7d, 1, 0x02,
  0x7e, 1, 0x02,
  0x7f, 1, 0x02,
  0x80, 1, 0x02,
  0x81, 1, 0x02,
  0x82, 1, 0x02,
  0x83, 1, 0x02,
  0x84, 1, 0x02,
  0x85, 1, 0x02,
  0x86, 1, 0x02,
  0x87, 1, 0x02,
  0x88, 1, 0x02,
  0x89, 1, 0x05,
  0x8a, 1, 0x01,
  0xff, 3, 0x98, 0x81, 0x04,
  0x38, 1, 0x01,
  0x39, 1, 0x00,
  0x6c, 1, 0x15,
  0x6e, 1, 0x1a,
  0x6f, 1, 0x25,
  0x3a, 1, 0xa4,
  0x8d, 1, 0x20,
  0x87, 1, 0xba,
  0x3b, 1, 0x98,
  0xff, 3, 0x98, 0x81, 0x01,
  0x22, 1, 0x0a,
  0x31, 1, 0x00,
  0x50, 1, 0x6b,
  0x51, 1, 0x66,
  0x53, 1, 0x73,
  0x55, 1, 0x8b,
  0x60, 1, 0x1b,
  0x61, 1, 0x01,
  0x62, 1, 0x0c,
  0x63, 1, 0x00,
  0xa0, 1, 0x00,
  0xa1, 1, 0x15,
  0xa2, 1, 0x1f,
  0xa3, 1, 0x13,
  0xa4, 1, 0x11,
  0xa5, 1, 0x21,
  0xa6, 1, 0x17,
  0xa7, 1, 0x1b,
  0xa8, 1, 0x6b,
  0xa9, 1, 0x1e,
  0xaa, 1, 0x2b,
  0xab, 1, 0x5d,
  0xac, 1, 0x19,
  0xad, 1, 0x14,
  0xae, 1, 0x4b,
  0xaf, 1, 0x1d,
  0xb0, 1, 0x27,
  0xb1, 1, 0x49,
  0xb2, 1, 0x5d,
  0xb3, 1, 0x39,
  0xc0, 1, 0x00,
  0xc1, 1, 0x01,
  0xc2, 1, 0x0c,
  0xc3, 1, 0x11,
  0xc4, 1, 0x15,
  0xc5, 1, 0x28,
  0xc6, 1, 0x1b,
  0xc7, 1, 0x1c,
  0xc8, 1, 0x62,
  0xc9, 1, 0x1c,
  0xca, 1, 0x29,
  0xcb, 1, 0x60,
  0xcc, 1, 0x16,
  0xcd, 1, 0x17,
  0xce, 1, 0x4a,
  0xcf, 1, 0x23,
  0xd0, 1, 0x24,
  0xd1, 1, 0x4f,
  0xd2, 1, 0x5f,
  0xd3, 1, 0x39,
  0xff, 3, 0x98, 0x81, 0x00,
  0x35, 0,
  0xfe, 0,
  0x29, 0,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ili9881c_select_page
 *
 * Description:
 *   Select one of the ILI9881C command pages.
 *
 * Input Parameters:
 *   device - The MIPI DSI device.
 *   page - The command page to select.
 *
 * Returned Value:
 *   Zero on success, a negated errno value on failure.
 *
 ****************************************************************************/

static int ili9881c_select_page(FAR struct mipi_dsi_device *device,
                                uint8_t page)
{
  const uint8_t sel[3] =
    {
      ILI9881C_PAGE_BYTE0, ILI9881C_PAGE_BYTE1, page
    };

  ssize_t n = mipi_dsi_dcs_write(device, ILI9881C_CMD_PAGE, sel,
                                 sizeof(sel));

  return n < 0 ? (int)n : OK;
}

/****************************************************************************
 * Name: ili9881c_report_id
 *
 * Description:
 *   Read and log the panel identification from registers 0x00..0x02 of
 *   command page 1, which must already be selected.  An Ilitek panel
 *   answers 0x98 0x81 in the first two.  A failed read is not fatal.
 *
 * Input Parameters:
 *   device - The MIPI DSI device.
 *
 ****************************************************************************/

static void ili9881c_report_id(FAR struct mipi_dsi_device *device)
{
  uint8_t id[3] =
    {
      0, 0, 0
    };

  ssize_t n;
  int i;

  for (i = 0; i < 3; i++)
    {
      n = mipi_dsi_dcs_read(device, ILI9881C_REG_ID1 + i, &id[i], 1);
      if (n < 0)
        {
          syslog(LOG_WARNING, "ili9881c: ID read %d failed: %d\n",
                 i, (int)n);
          return;
        }
    }

  syslog(LOG_INFO, "ili9881c: panel ID %02x %02x %02x%s\n",
         id[0], id[1], id[2],
         (id[0] == ILI9881C_ID1_ILITEK && id[1] == ILI9881C_ID2_ILITEK) ?
         "" : " (unexpected: not an Ilitek panel?)");
}

/****************************************************************************
 * Name: ili9881c_send_init
 *
 * Description:
 *   Send the vendor initialization table.
 *
 * Input Parameters:
 *   device - The MIPI DSI device.
 *
 * Returned Value:
 *   Zero on success, a negated errno value on failure.
 *
 ****************************************************************************/

static int ili9881c_send_init(FAR struct mipi_dsi_device *device)
{
  size_t i = 0;
  uint8_t cmd;
  uint8_t len;
  ssize_t n;

  while (i + 1 < nitems(g_ili9881c_init))
    {
      cmd = g_ili9881c_init[i];
      len = g_ili9881c_init[i + 1];
      i += 2;

      n = mipi_dsi_dcs_write(device, cmd,
                             len > 0 ? &g_ili9881c_init[i] : NULL, len);
      if (n < 0)
        {
          syslog(LOG_ERR, "ili9881c: cmd %02x failed: %d\n",
                 cmd, (int)n);
          return (int)n;
        }

      i += len;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tab5_ili9881c_initialize
 *
 * Description:
 *   Register the ILI9881C as a mipi_dsi_device, attach it to the Espressif
 *   host, report the panel identification and send the vendor DCS
 *   initialization table.  Does not start video.
 *
 * Input Parameters:
 *   host - Registered MIPI-DSI host (esp_mipi_dsi_host_get())
 *
 * Returned Value:
 *   Pointer to the registered device on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct mipi_dsi_device *tab5_ili9881c_initialize(
      FAR struct mipi_dsi_host *host)
{
  FAR struct mipi_dsi_device *device;
  const uint8_t colmod = MIPI_DCS_PIXEL_FMT_16BIT;
  const uint8_t madctl = 0x00;
  int ret;

  if (host == NULL)
    {
      return NULL;
    }

  device = mipi_dsi_device_register(host, ILI9881C_NAME, 0);
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

  /* The identification registers live on command page 1 */

  ret = ili9881c_select_page(device, ILI9881C_PAGE1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: page 1 select failed: %d\n", ret);
      return NULL;
    }

  ili9881c_report_id(device);

  ret = ili9881c_select_page(device, ILI9881C_PAGE0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: page 0 select failed: %d\n", ret);
      return NULL;
    }

  ret = mipi_dsi_dcs_exit_sleep_mode(device);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: exit sleep mode failed: %d\n", ret);
      return NULL;
    }

  up_mdelay(ILI9881C_SLEEP_OUT_MS);

  if (mipi_dsi_dcs_write(device, MIPI_DCS_SET_ADDRESS_MODE, &madctl, 1) < 0
      || mipi_dsi_dcs_write(device, MIPI_DCS_SET_PIXEL_FORMAT,
                            &colmod, 1) < 0)
    {
      syslog(LOG_ERR, "ERROR: address/pixel format setup failed\n");
      return NULL;
    }

  ret = ili9881c_send_init(device);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: DCS init failed: %d\n", ret);
      return NULL;
    }

  return device;
}
