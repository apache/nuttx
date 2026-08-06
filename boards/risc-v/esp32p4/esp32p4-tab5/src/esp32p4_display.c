/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-tab5/src/esp32p4_display.c
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

/* Tab5 display / framebuffer glue (PinePhone-style up_fbinitialize split).
 *
 * Sequence (ESP-IDF DPI panel + selected ST712x controller):
 *   1. Allocate RGB565 FB (PSRAM)
 *   2. Host already registered by bringup
 *   3. configure_dpi -> panel DCS -> bind FB -> video_start -> display_on
 *   4. Backlight on
 *   5. Optional All-Pixels-On bring-up flash
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>
#include <nuttx/video/fb.h>
#include <nuttx/video/mipi_display.h>
#include <nuttx/video/mipi_dsi.h>

#include <arch/board/board.h>

#include "espressif/esp_mipi_dsi.h"

#include "esp32p4-tab5.h"

#ifdef CONFIG_ESP32P4_TAB5_LCD_ST7123
#  include "esp32p4_lcd_st7123.h"
#  define tab5_panel_initialize tab5_st7123_initialize
#else
#  include "esp32p4_lcd_st7121.h"
#  define tab5_panel_initialize tab5_st7121_initialize
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAB5_FB_BPP        16
#define TAB5_FB_WIDTH      TAB5_MIPI_DSI_H_RES
#define TAB5_FB_HEIGHT     TAB5_MIPI_DSI_V_RES
#define TAB5_FB_STRIDE     (TAB5_FB_WIDTH * (TAB5_FB_BPP / 8))
#define TAB5_FB_SIZE       (TAB5_FB_STRIDE * TAB5_FB_HEIGHT)

#define TAB5_FB_TEST_COLOR  0xf800

#define TAB5_DCS_ALL_PIXELS_ON   0x23
#define TAB5_DCS_ALL_PIXELS_OFF  0x22
#define TAB5_APO_TEST_MS         1000

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int tab5_getvideoinfo(FAR struct fb_vtable_s *vtable,
                             FAR struct fb_videoinfo_s *vinfo);
static int tab5_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                             FAR struct fb_planeinfo_s *pinfo);
#ifdef CONFIG_FB_UPDATE
static int tab5_updatearea(FAR struct fb_vtable_s *vtable,
                           FAR const struct fb_area_s *area);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct fb_vtable_s g_tab5_vtable =
{
  .getvideoinfo = tab5_getvideoinfo,
  .getplaneinfo = tab5_getplaneinfo,
#ifdef CONFIG_FB_UPDATE
  .updatearea   = tab5_updatearea,
#endif
};

static struct fb_videoinfo_s g_tab5_video =
{
  .fmt     = FB_FMT_RGB16_565,
  .xres    = TAB5_FB_WIDTH,
  .yres    = TAB5_FB_HEIGHT,
  .nplanes = 1,
};

static struct fb_planeinfo_s g_tab5_plane =
{
  .fbmem   = NULL,
  .fblen   = TAB5_FB_SIZE,
  .stride  = TAB5_FB_STRIDE,
  .display = 0,
  .bpp     = TAB5_FB_BPP,
  .xres_virtual = TAB5_FB_WIDTH,
  .yres_virtual = TAB5_FB_HEIGHT,
  .xoffset = 0,
  .yoffset = 0,
};

static FAR uint16_t *g_tab5_fb;
static bool g_tab5_fb_ready;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tab5_getvideoinfo
 *
 * Description:
 *   Return the video information.
 *
 * Input Parameters:
 *   vtable - The vtable.
 *   vinfo - The video information.
 *
 * Returned Value:
 *   Zero on success.
 *
 ****************************************************************************/

static int tab5_getvideoinfo(FAR struct fb_vtable_s *vtable,
                             FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_tab5_vtable && vinfo != NULL);
  memcpy(vinfo, &g_tab5_video, sizeof(*vinfo));
  return OK;
}

/****************************************************************************
 * Name: tab5_getplaneinfo
 *
 * Description:
 *   Return the plane information.
 *
 * Input Parameters:
 *   vtable - The vtable.
 *   planeno - The plane number.
 *   pinfo - The plane information.
 *
 * Returned Value:
 *   Zero on success, -EINVAL if the plane number is invalid.
 *
 ****************************************************************************/

static int tab5_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                             FAR struct fb_planeinfo_s *pinfo)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_tab5_vtable && pinfo != NULL);

  if (planeno != 0 || g_tab5_plane.fbmem == NULL)
    {
      return -EINVAL;
    }

  memcpy(pinfo, &g_tab5_plane, sizeof(*pinfo));
  return OK;
}

#ifdef CONFIG_FB_UPDATE
/****************************************************************************
 * Name: tab5_updatearea
 *
 * Description:
 *   Update a region of the framebuffer.
 *
 * Input Parameters:
 *   vtable - The vtable.
 *   area - The area to update.
 *
 * Returned Value:
 *   Zero on success, -EAGAIN if the framebuffer is not ready,
 *   -EINVAL if the area is out of bounds.
 ****************************************************************************/

static int tab5_updatearea(FAR struct fb_vtable_s *vtable,
                           FAR const struct fb_area_s *area)
{
  size_t offset;
  size_t len;

  DEBUGASSERT(vtable != NULL && vtable == &g_tab5_vtable);

  if (g_tab5_fb == NULL)
    {
      return -EAGAIN;
    }

  if (area == NULL)
    {
      return esp_mipi_dsi_flush_framebuffer(g_tab5_fb, TAB5_FB_SIZE);
    }

  if (area->y >= TAB5_FB_HEIGHT || area->x >= TAB5_FB_WIDTH)
    {
      return OK;
    }

  offset = (size_t)area->y * TAB5_FB_STRIDE +
           (size_t)area->x * (TAB5_FB_BPP / 8);
  len = (size_t)area->h * TAB5_FB_STRIDE;
  if (offset + len > TAB5_FB_SIZE)
    {
      len = TAB5_FB_SIZE - offset;
    }

  return esp_mipi_dsi_flush_framebuffer((FAR uint8_t *)g_tab5_fb + offset,
                                        len);
}
#endif /* CONFIG_FB_UPDATE */

/****************************************************************************
 * Name: tab5_fb_fill
 *
 * Description:
 *   Fill the framebuffer with a given color.
 *
 * Input Parameters:
 *   color - The color to fill the framebuffer with.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void tab5_fb_fill(uint16_t color)
{
  size_t i;
  size_t npix = (size_t)TAB5_FB_WIDTH * TAB5_FB_HEIGHT;
  int ret;

  for (i = 0; i < npix; i++)
    {
      g_tab5_fb[i] = color;
    }

  ret = esp_mipi_dsi_flush_framebuffer(g_tab5_fb, TAB5_FB_SIZE);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: FB cache C2M sync failed: %d\n", ret);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Initialize the framebuffer.
 *
 * Input Parameters:
 *   display - The display number.
 *
 * Returned Value:
 *   Zero on success, -ENODEV on failure.
 *
 ****************************************************************************/

int up_fbinitialize(int display)
{
  FAR struct mipi_dsi_host *host;
  FAR struct mipi_dsi_device *device;
  struct esp_mipi_dsi_dpi_config_s dpi;
  int ret;

  if (display != 0)
    {
      return -ENODEV;
    }

  if (g_tab5_fb_ready)
    {
      return OK;
    }

  g_tab5_fb = kumm_memalign(64, TAB5_FB_SIZE);
  if (g_tab5_fb == NULL)
    {
      syslog(LOG_ERR,
             "ERROR: FB alloc failed (%u bytes; enable PSRAM)\n",
             (unsigned int)TAB5_FB_SIZE);
      return -ENOMEM;
    }

  memset(g_tab5_fb, 0, TAB5_FB_SIZE);
  g_tab5_plane.fbmem = g_tab5_fb;

  host = esp_mipi_dsi_host_get();
  if (host == NULL)
    {
      syslog(LOG_ERR,
             "ERROR: MIPI-DSI host not ready "
             "(enable ESP32P4_TAB5_MIPI_DSI)\n");
      ret = -EAGAIN;
      goto errout_fb;
    }

  /* IDF order: DPI config before panel DCS; display_on after video_start. */

  syslog(LOG_INFO, "Configuring DPI...\n");
  tab5_mipi_dsi_dpi_config(&dpi);
  ret = esp_mipi_dsi_configure_dpi(&dpi);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: configure_dpi failed: %d\n", ret);
      goto errout_fb;
    }

  syslog(LOG_INFO, "%s panel init...\n",
         TAB5_LCD_PANEL_NAME);
  device = tab5_panel_initialize(host);
  if (device == NULL)
    {
      ret = -EIO;
      goto errout_fb;
    }

  ret = esp_mipi_dsi_bind_framebuffer(g_tab5_fb, TAB5_FB_SIZE,
                                      TAB5_FB_WIDTH, TAB5_FB_HEIGHT,
                                      TAB5_FB_BPP);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: bind_framebuffer failed: %d\n", ret);
      goto errout_fb;
    }

  tab5_fb_fill(TAB5_FB_TEST_COLOR);

  ret = mipi_dsi_dcs_exit_sleep_mode(device);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pre-video sleep_out failed: %d\n",
             ret);
      goto errout_fb;
    }

  up_mdelay(120);

  syslog(LOG_INFO, "Starting video...\n");
  ret = esp_mipi_dsi_video_start();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: video_start failed: %d\n", ret);
      goto errout_fb;
    }

  ret = mipi_dsi_dcs_set_display_on(device);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: post-video display_on failed: %d\n",
             ret);
      goto errout_fb;
    }

  tab5_fb_fill(TAB5_FB_TEST_COLOR);

  ret = tab5_lcd_backlight(true);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: backlight failed: %d\n", ret);
    }

  g_tab5_fb_ready = true;
  syslog(LOG_INFO,
         "/dev/fb0 ready %ux%u RGB565 @ %p\n",
         TAB5_FB_WIDTH, TAB5_FB_HEIGHT, g_tab5_fb);

#ifdef CONFIG_ESP32P4_TAB5_LCD_APO_TEST
  /* White flash proves DCS/BL; then restore normal mode + red FB. */

  mipi_dsi_dcs_write(device, TAB5_DCS_ALL_PIXELS_ON, NULL, 0);
  up_mdelay(TAB5_APO_TEST_MS);
  mipi_dsi_dcs_write(device, TAB5_DCS_ALL_PIXELS_OFF, NULL, 0);
  mipi_dsi_dcs_write(device, MIPI_DCS_ENTER_NORMAL_MODE, NULL, 0);
  mipi_dsi_dcs_set_display_on(device);
  tab5_fb_fill(TAB5_FB_TEST_COLOR);
#endif

  return OK;

errout_fb:
  kumm_free(g_tab5_fb);
  g_tab5_fb = NULL;
  g_tab5_plane.fbmem = NULL;
  return ret;
}

/****************************************************************************
 * Name: up_fbgetvplane
 *
 * Description:
 *   Return the framebuffer vtable.
 *
 * Input Parameters:
 *   display - The display number.
 *   vplane - The vplane number.
 *
 * Returned Value:
 *   Pointer to the vtable on success, NULL on failure.
 *
 ****************************************************************************/

FAR struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  if (display != 0 || vplane != 0 || !g_tab5_fb_ready)
    {
      return NULL;
    }

  return &g_tab5_vtable;
}

/****************************************************************************
 * Name: up_fbuninitialize
 *
 * Description:
 *   Uninitialize the framebuffer.
 *
 * Input Parameters:
 *   display - The display number.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_fbuninitialize(int display)
{
  UNUSED(display);
}

/****************************************************************************
 * Name: tab5_fb_reload_test_pattern
 *
 * Description:
 *   Call after fb_register(): the generic FB driver memset()s the
 *   framebuffer and would otherwise leave a black DMA buffer.
 *   This sets the framebuffer to the test color (red).
 *
 * Returned Value:
 *   Zero on success, -EAGAIN on failure.
 *
 ****************************************************************************/

int tab5_fb_reload_test_pattern(void)
{
  if (!g_tab5_fb_ready || g_tab5_fb == NULL)
    {
      return -EAGAIN;
    }

  tab5_fb_fill(TAB5_FB_TEST_COLOR);
  return OK;
}
