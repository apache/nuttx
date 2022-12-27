/****************************************************************************
 * boards/arm64/a64/pinephone/src/pinephone_display.c
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

/* Reference:
 *
 * "Understanding PinePhone's Display (MIPI DSI)"
 * https://lupyuen.github.io/articles/dsi
 *
 * "NuttX RTOS for PinePhone: Display Driver in Zig"
 * https://lupyuen.github.io/articles/dsi2
 *
 * "Rendering PinePhone's Display (DE and TCON0)"
 * https://lupyuen.github.io/articles/de
 *
 * "NuttX RTOS for PinePhone: Render Graphics in Zig"
 * https://lupyuen.github.io/articles/de2
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <nuttx/video/fb.h>
#include "chip.h"
#include "arm64_internal.h"
#include "a64_de.h"
#include "a64_mipi_dphy.h"
#include "a64_mipi_dsi.h"
#include "a64_tcon0.h"
#include "pinephone_lcd.h"
#include "pinephone_pmic.h"
#include "pinephone_display.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LCD Backlight Brightness (percentage) */

#define BACKLIGHT_BRIGHTNESS_PERCENT 90

/* Number of UI Channels to render: 1 or 3 */

#define UI_CHANNELS  3

/* LCD Panel Width and Height (pixels) */

#define PANEL_WIDTH  PINEPHONE_LCD_PANEL_WIDTH   /* 720 pixels */
#define PANEL_HEIGHT PINEPHONE_LCD_PANEL_HEIGHT  /* 1440 pixels */

/* Framebuffer 1 Width and Height (pixels) */

#define FB1_WIDTH    600
#define FB1_HEIGHT   600

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pinephone_getvideoinfo(struct fb_vtable_s *vtable,
                                  struct fb_videoinfo_s *vinfo);

static int pinephone_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                                  struct fb_planeinfo_s *pinfo);

static int pinephone_getoverlayinfo(struct fb_vtable_s *vtable,
                                    int overlayno,
                                    struct fb_overlayinfo_s *oinfo);

static int pinephone_settransp(struct fb_vtable_s *vtable,
                               const struct fb_overlayinfo_s *oinfo);

static int pinephone_setchromakey(struct fb_vtable_s *vtable,
                                  const struct fb_overlayinfo_s *oinfo);

static int pinephone_setcolor(struct fb_vtable_s *vtable,
                              const struct fb_overlayinfo_s *oinfo);

static int pinephone_setblank(struct fb_vtable_s *vtable,
                              const struct fb_overlayinfo_s *oinfo);

static int pinephone_setarea(struct fb_vtable_s *vtable,
                             const struct fb_overlayinfo_s *oinfo);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Vtable for Frame Buffer Operations */

static struct fb_vtable_s g_pinephone_vtable =
{
  .getvideoinfo    = pinephone_getvideoinfo,
  .getplaneinfo    = pinephone_getplaneinfo,
  .getoverlayinfo  = pinephone_getoverlayinfo,
  .settransp       = pinephone_settransp,
  .setchromakey    = pinephone_setchromakey,
  .setcolor        = pinephone_setcolor,
  .setblank        = pinephone_setblank,
  .setarea         = pinephone_setarea
};

/* Frame Buffers for Display Engine *****************************************/

/* Frame Buffer 0: (Base UI Channel)
 * Fullscreen 720 x 1440 (4 bytes per XRGB 8888 pixel)
 */

static uint32_t g_pinephone_fb0[PANEL_WIDTH * PANEL_HEIGHT];

/* Frame Buffer 1: (First Overlay UI Channel)
 * Square 600 x 600 (4 bytes per ARGB 8888 pixel)
 */

static uint32_t g_pinephone_fb1[FB1_WIDTH * FB1_HEIGHT];

/* Frame Buffer 2: (Second Overlay UI Channel)
 * Fullscreen 720 x 1440 (4 bytes per ARGB 8888 pixel)
 */

static uint32_t g_pinephone_fb2[PANEL_WIDTH * PANEL_HEIGHT];

/* Video Controller for 3 UI Channels:
 * Fullscreen 720 x 1440 (4 bytes per ARGB 8888 pixel)
 */

static struct fb_videoinfo_s g_pinephone_video =
{
  .fmt       = FB_FMT_RGBA32,  /* Pixel format (XRGB 8888) */
  .xres      = PANEL_WIDTH,    /* Horizontal resolution in pixel columns */
  .yres      = PANEL_HEIGHT,   /* Vertical resolution in pixel rows */
  .nplanes   = 1,              /* Color planes: Base UI Channel */
  .noverlays = 2               /* Overlays: 2 Overlay UI Channels) */
};

/* Color Plane for Base UI Channel:
 * Fullscreen 720 x 1440 (4 bytes per XRGB 8888 pixel)
 */

static struct fb_planeinfo_s g_pinephone_plane =
{
  .fbmem        = &g_pinephone_fb0,
  .fblen        = sizeof(g_pinephone_fb0),
  .stride       = PANEL_WIDTH * 4,  /* Length of a line (4-byte pixel) */
  .display      = 0,                /* Display number (Unused) */
  .bpp          = 32,               /* Bits per pixel (XRGB 8888) */
  .xres_virtual = PANEL_WIDTH,      /* Virtual Horizontal resolution */
  .yres_virtual = PANEL_HEIGHT,     /* Virtual Vertical resolution */
  .xoffset      = 0,                /* Offset from virtual to visible */
  .yoffset      = 0                 /* Offset from virtual to visible */
};

/* Overlays for 2 Overlay UI Channels */

static struct fb_overlayinfo_s g_pinephone_overlays[2] =
{
  /* First Overlay UI Channel:
   * Square 600 x 600 (4 bytes per ARGB 8888 pixel)
   */

  {
    .fbmem     = &g_pinephone_fb1,
    .fblen     = sizeof(g_pinephone_fb1),
    .stride    = FB1_WIDTH * 4,    /* Length of a line (4-byte pixel) */
    .overlay   = 0,                /* Overlay number (First Overlay) */
    .bpp       = 32,               /* Bits per pixel (ARGB 8888) */
    .blank     = 0,                /* TODO: Blank or unblank */
    .chromakey = 0,                /* TODO: Chroma key argb8888 formatted */
    .color     = 0,                /* TODO: Color argb8888 formatted */
    .transp    =                   /* TODO: Transparency */
    {
      .transp      = 0,
      .transp_mode = 0
    },
    .sarea     =                   /* Selected area within the overlay */
    {
      .x = 52,
      .y = 52,
      .w = FB1_WIDTH,
      .h = FB1_HEIGHT
    },
    .accl      = 0                 /* TODO: Supported hardware acceleration */
  },

  /* Second Overlay UI Channel:
   * Fullscreen 720 x 1440 (4 bytes per ARGB 8888 pixel)
   */

  {
    .fbmem     = &g_pinephone_fb2,
    .fblen     = sizeof(g_pinephone_fb2),
    .stride    = PANEL_WIDTH * 4,  /* Length of a line (4-byte pixel) */
    .overlay   = 1,                /* Overlay number (First Overlay) */
    .bpp       = 32,               /* Bits per pixel (ARGB 8888) */
    .blank     = 0,                /* TODO: Blank or unblank */
    .chromakey = 0,                /* TODO: Chroma key argb8888 formatted */
    .color     = 0,                /* TODO: Color argb8888 formatted */
    .transp    =                   /* TODO: Transparency */
    {
      .transp      = 0,
      .transp_mode = 0
    },
    .sarea     =                   /* Selected area within the overlay */
    {
      .x = 0,
      .y = 0,
      .w = PANEL_WIDTH,
      .h = PANEL_HEIGHT
    },
    .accl      = 0                 /* TODO: Supported hardware acceleration */
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: render_framebuffers
 *
 * Description:
 *   Render the 3 Frame Buffers with the Display Engine.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int render_framebuffers(void)
{
  int i;
  int ret;
  const int overlay_len = sizeof(g_pinephone_overlays) /
                          sizeof(g_pinephone_overlays[0]);

  /* Validate the Frame Buffer Sizes */

  DEBUGASSERT(UI_CHANNELS == 1 || UI_CHANNELS == 3);
  DEBUGASSERT(g_pinephone_plane.xres_virtual == g_pinephone_video.xres);
  DEBUGASSERT(g_pinephone_plane.yres_virtual == g_pinephone_video.yres);
  DEBUGASSERT(g_pinephone_plane.fblen ==
              g_pinephone_plane.xres_virtual *
              g_pinephone_plane.yres_virtual * 4);
  DEBUGASSERT(g_pinephone_plane.stride ==
              g_pinephone_plane.xres_virtual * 4);
  DEBUGASSERT(g_pinephone_overlays[0].fblen ==
             (g_pinephone_overlays[0].sarea.w) *
              g_pinephone_overlays[0].sarea.h * 4);
  DEBUGASSERT(g_pinephone_overlays[0].stride ==
              g_pinephone_overlays[0].sarea.w * 4);
  DEBUGASSERT(g_pinephone_overlays[1].fblen ==
             (g_pinephone_overlays[1].sarea.w) *
              g_pinephone_overlays[1].sarea.h * 4);
  DEBUGASSERT(g_pinephone_overlays[1].stride ==
              g_pinephone_overlays[1].sarea.w * 4);

  /* Init the UI Blender for Display Engine */

  ret = a64_de_blender_init();
  if (ret < 0)
    {
      gerr("Init UI Blender failed: %d\n", ret);
      return ret;
    }

  /* Init the Base UI Channel. Frame Buffer Address should be 32-bit. */

  ret = a64_de_ui_channel_init(1,  /* UI Channel 1 (Base UI Channel) */
                               g_pinephone_plane.fbmem,
                               g_pinephone_plane.fblen,
                               g_pinephone_plane.xres_virtual,
                               g_pinephone_plane.yres_virtual,
                               g_pinephone_plane.xoffset,
                               g_pinephone_plane.yoffset);
  if (ret < 0)
    {
      gerr("Init UI Channel 1 failed: %d\n", ret);
      return ret;
    }

  /* Init the 2 Overlay UI Channels */

  for (i = 0; i < overlay_len; i++)
    {
      const struct fb_overlayinfo_s *ov = &g_pinephone_overlays[i];

      /* Pass Frame Buffer as null if UI Channel should be disabled */

      const void *fb = (UI_CHANNELS == 3) ? ov->fbmem : NULL;

      ret = a64_de_ui_channel_init(i + 2,        /* UI Channel 2 or 3 */
                                   fb,           /* 32-bit address */
                                   ov->fblen,    /* Frame Buffer length */
                                   ov->sarea.w,  /* Horiz res */
                                   ov->sarea.h,  /* Vert res */
                                   ov->sarea.x,  /* Horiz offset */
                                   ov->sarea.y); /* Vert offset */
      if (ret < 0)
        {
          gerr("Init UI Channel %d failed: %d\n", i + 2, ret);
          return ret;
        }
    }

  /* Set UI Blender Route, enable Blender Pipes and apply the settings */

  ret = a64_de_enable(UI_CHANNELS);
  if (ret < 0)
    {
      gerr("Enable Display Engine failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: pinephone_getvideoinfo
 *
 * Description:
 *   Get the videoinfo for the framebuffer. (ioctl Entrypoint:
 *   FBIOGET_VIDEOINFO)
 *
 * Input Parameters:
 *   vtable - Framebuffer driver object
 *   vinfo  - Returned videoinfo object
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int pinephone_getvideoinfo(struct fb_vtable_s *vtable,
                                  struct fb_videoinfo_s *vinfo)
{
  static int stage = 0;

  ginfo("vtable=%p vinfo=%p\n", vtable, vinfo);
  DEBUGASSERT(vtable != NULL && vtable == &g_pinephone_vtable &&
              vinfo != NULL);

  /* Copy and return the videoinfo object */

  memcpy(vinfo, &g_pinephone_video, sizeof(struct fb_videoinfo_s));

  /* Keep track of the stages during startup:
   * Stage 0: Initialize driver at startup
   * Stage 1: First call by apps
   * Stage 2: Subsequent calls by apps
   * We erase the framebuffers at stages 0 and 1. This allows the
   * Test Pattern to be displayed for as long as possible before erasure.
   */

  if (stage < 2)
    {
      stage++;
      memset(g_pinephone_fb0, 0, sizeof(g_pinephone_fb0));
      memset(g_pinephone_fb1, 0, sizeof(g_pinephone_fb1));
      memset(g_pinephone_fb2, 0, sizeof(g_pinephone_fb2));
    }

  return OK;
}

/****************************************************************************
 * Name: pinephone_getplaneinfo
 *
 * Description:
 *   Get the planeinfo for the framebuffer. (ioctl Entrypoint:
 *   FBIOGET_PLANEINFO)
 *
 * Input Parameters:
 *   vtable - Framebuffer driver object
 *   pinfo  - Returned planeinfo object
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int pinephone_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                                  struct fb_planeinfo_s *pinfo)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_pinephone_vtable);
  ginfo("vtable=%p planeno=%d pinfo=%p\n", vtable, planeno, pinfo);

  /* Copy and return the planeinfo object */

  if (planeno == 0)
    {
      memcpy(pinfo, &g_pinephone_plane, sizeof(struct fb_planeinfo_s));
      return OK;
    }

  gerr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: pinephone_getoverlayinfo
 *
 * Description:
 *   Get the overlayinfo for the framebuffer. (ioctl Entrypoint:
 *   FBIOGET_OVERLAYINFO)
 *
 * Input Parameters:
 *   vtable    - Framebuffer driver object
 *   overlayno - Overlay number
 *   oinfo     - Returned overlayinfo object
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int pinephone_getoverlayinfo(struct fb_vtable_s *vtable,
                                    int overlayno,
                                    struct fb_overlayinfo_s *oinfo)
{
  const int overlay_len = sizeof(g_pinephone_overlays) /
                          sizeof(g_pinephone_overlays[0]);

  ginfo("vtable=%p overlay=%d oinfo=%p\n", vtable, overlayno, oinfo);
  DEBUGASSERT(vtable != NULL && vtable == &g_pinephone_vtable);

  /* Copy and return the overlayinfo object */

  if (overlayno >= 0 && overlayno < overlay_len)
    {
      struct fb_overlayinfo_s *overlay = &g_pinephone_overlays[overlayno];

      memcpy(oinfo, overlay, sizeof(struct fb_overlayinfo_s));
      return OK;
    }

  gerr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: pinephone_settransp
 *
 * Description:
 *   Set the overlay transparency for the framebuffer. (ioctl Entrypoint:
 *   FBIOSET_TRANSP)
 *
 * Input Parameters:
 *   vtable    - Framebuffer driver object
 *   oinfo     - overlayinfo object
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int pinephone_settransp(struct fb_vtable_s *vtable,
                               const struct fb_overlayinfo_s *oinfo)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_pinephone_vtable);
  ginfo("vtable=%p, overlay=%d, transp=%02x, transp_mode=%02x\n", vtable,
        oinfo->overlay, oinfo->transp.transp, oinfo->transp.transp_mode);
  gerr("Not implemented");
  return OK;
}

/****************************************************************************
 * Name: pinephone_setchromakey
 *
 * Description:
 *   Set the overlay chroma key for the framebuffer. (ioctl Entrypoint:
 *   FBIOSET_CHROMAKEY)
 *
 * Input Parameters:
 *   vtable    - Framebuffer driver object
 *   oinfo     - overlayinfo object
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int pinephone_setchromakey(struct fb_vtable_s *vtable,
                                  const struct fb_overlayinfo_s *oinfo)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_pinephone_vtable &&
              oinfo != NULL);
  ginfo("vtable=%p, overlay=%d, chromakey=%08x\n", vtable,
        oinfo->overlay, oinfo->chromakey);
  gerr("Not implemented");
  return OK;
}

/****************************************************************************
 * Name: pinephone_setcolor
 *
 * Description:
 *   Set the overlay color for the framebuffer. (ioctl Entrypoint:
 *   FBIOSET_COLOR)
 *
 * Input Parameters:
 *   vtable    - Framebuffer driver object
 *   oinfo     - overlayinfo object
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int pinephone_setcolor(struct fb_vtable_s *vtable,
                              const struct fb_overlayinfo_s *oinfo)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_pinephone_vtable &&
              oinfo != NULL);
  ginfo("vtable=%p, overlay=%d, color=%08x\n",
        vtable, oinfo->overlay, oinfo->color);
  gerr("Not implemented");
  return OK;
}

/****************************************************************************
 * Name: pinephone_setblank
 *
 * Description:
 *   Set the framebuffer overlay to blank. (ioctl Entrypoint: FBIOSET_BLANK)
 *
 * Input Parameters:
 *   vtable    - Framebuffer driver object
 *   oinfo     - overlayinfo object
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int pinephone_setblank(struct fb_vtable_s *vtable,
                              const struct fb_overlayinfo_s *oinfo)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_pinephone_vtable &&
              oinfo != NULL);
  ginfo("vtable=%p, overlay=%d, blank=%02x\n",
        vtable, oinfo->overlay, oinfo->blank);
  gerr("Not implemented");
  return OK;
}

/****************************************************************************
 * Name: pinephone_setarea
 *
 * Description:
 *   Set the overlay area for the framebuffer. (ioctl Entrypoint:
 *   FBIOSET_AREA)
 *
 * Input Parameters:
 *   vtable    - Framebuffer driver object
 *   oinfo     - overlayinfo object
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int pinephone_setarea(struct fb_vtable_s *vtable,
                             const struct fb_overlayinfo_s *oinfo)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_pinephone_vtable &&
              oinfo != NULL);
  ginfo("vtable=%p, overlay=%d, x=%d, y=%d, w=%d, h=%d\n", vtable,
        oinfo->overlay, oinfo->sarea.x, oinfo->sarea.y, oinfo->sarea.w,
        oinfo->sarea.h);
  gerr("Not implemented");
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Initialize the framebuffer video hardware associated with the display.
 *
 *   There are multiple logic paths that may call up_fbinitialize() so any
 *   implementation of up_fbinitialize() should be tolerant of being called
 *   multiple times.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *             specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int up_fbinitialize(int display)
{
  int ret;
  static bool initialized = false;

  /* Allow multiple calls */

  DEBUGASSERT(display == 0);
  if (initialized)
    {
      return OK;
    }

  initialized = true;

  /* Turn on Display Backlight */

  ret = pinephone_lcd_backlight_enable(BACKLIGHT_BRIGHTNESS_PERCENT);
  if (ret < 0)
    {
      gerr("Enable Backlight failed: %d\n", ret);
      return ret;
    }

  /* Init Timing Controller TCON0 */

  ret = a64_tcon0_init(PANEL_WIDTH, PANEL_HEIGHT);
  if (ret < 0)
    {
      gerr("Init Timing Controller TCON0 failed: %d\n", ret);
      return ret;
    }

  /* Reset LCD Panel to Low */

  ret = pinephone_lcd_panel_reset(false);
  if (ret < 0)
    {
      gerr("Reset LCD Panel failed: %d\n", ret);
      return ret;
    }

  /* Init PMIC */

  ret = pinephone_pmic_init();
  if (ret < 0)
    {
      gerr("Init PMIC failed: %d\n", ret);
      return ret;
    }

  /* Wait 15 milliseconds for power supply and power-on init */

  up_mdelay(15);

  /* Enable MIPI DSI */

  ret = a64_mipi_dsi_enable();
  if (ret < 0)
    {
      gerr("Enable MIPI DSI failed: %d\n", ret);
      return ret;
    }

  /* Enable MIPI D-PHY */

  ret = a64_mipi_dphy_enable();
  if (ret < 0)
    {
      gerr("Enable MIPI D-PHY failed: %d\n", ret);
      return ret;
    }

  /* Reset LCD Panel to High */

  ret = pinephone_lcd_panel_reset(true);
  if (ret < 0)
    {
      gerr("Reset LCD Panel failed: %d\n", ret);
      return ret;
    }

  /* Wait 15 milliseconds for LCD Panel */

  up_mdelay(15);

  /* Initialise ST7703 LCD Controller */

  ret = pinephone_lcd_panel_init();
  if (ret < 0)
    {
      gerr("Init ST7703 LCD Controller failed: %d\n", ret);
      return ret;
    }

  /* Start MIPI DSI Bus in HSC and HSD modes */

  ret = a64_mipi_dsi_start();
  if (ret < 0)
    {
      gerr("Start MIPI DSI failed: %d\n", ret);
      return ret;
    }

  /* Init Display Engine */

  ret = a64_de_init();
  if (ret < 0)
    {
      gerr("Init Display Engine failed: %d\n", ret);
      return ret;
    }

  /* Wait 160 milliseconds for Display Engine */

  up_mdelay(160);

  /* Render Frame Buffers with Display Engine */

  ret = render_framebuffers();
  if (ret < 0)
    {
      gerr("Display Engine Frame Buffers failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: up_fbgetvplane
 *
 * Description:
 *   Return a reference to the framebuffer object for the specified video
 *   plane of the specified plane.  Many OSDs support multiple planes of
 *   video.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *             specifies the display.  Normally this is zero.
 *   vplane  - Identifies the plane being queried.
 *
 * Returned Value:
 *   A non-NULL pointer to the frame buffer access structure is returned on
 *   success; NULL is returned on any failure.
 *
 ****************************************************************************/

struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  ginfo("vplane: %d\n", vplane);

  DEBUGASSERT(display == 0);
  if (vplane == 0)
    {
      return &g_pinephone_vtable;
    }

  return NULL;
}

/****************************************************************************
 * Name: up_fbuninitialize
 *
 * Description:
 *   Uninitialize the framebuffer support for the specified display.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *             specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_fbuninitialize(int display)
{
  /* Uninitialize is not supported */

  UNUSED(display);
}

/****************************************************************************
 * Name: pinephone_display_test_pattern
 *
 * Description:
 *   Fill the 3 Frame Buffers with a Test Pattern.  Should be called after
 *   Display Engine is Enabled, or the rendered image will have missing
 *   rows.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pinephone_display_test_pattern(void)
{
  int i;
  int x;
  int y;
  const int fb0_len = sizeof(g_pinephone_fb0) / sizeof(g_pinephone_fb0[0]);
  const int fb1_len = sizeof(g_pinephone_fb1) / sizeof(g_pinephone_fb1[0]);

  /* Zero the Framebuffers */

  memset(g_pinephone_fb0, 0, sizeof(g_pinephone_fb0));
  memset(g_pinephone_fb1, 0, sizeof(g_pinephone_fb1));
  memset(g_pinephone_fb2, 0, sizeof(g_pinephone_fb2));

  /* Init Framebuffer 0:
   * Fill with Blue, Green and Red
   */

  for (i = 0; i < fb0_len; i++)
    {
      /* Colours are in XRGB 8888 format */

      if (i < fb0_len / 4)
        {
          /* Blue for top quarter */

          g_pinephone_fb0[i] = 0x80000080;
        }
      else if (i < fb0_len / 2)
        {
          /* Green for next quarter */

          g_pinephone_fb0[i] = 0x80008000;
        }
      else
        {
          /* Red for lower half */

          g_pinephone_fb0[i] = 0x80800000;
        }

      /* Fixes missing rows in the rendered image, not sure why */

      ARM64_DMB();
      ARM64_DSB();
      ARM64_ISB();
    }

  /* Init Framebuffer 1:
   * Fill with Semi-Transparent White
   */

  for (i = 0; i < fb1_len; i++)
    {
      /* Semi-Transparent White in ARGB 8888 format */

      g_pinephone_fb1[i] = 0x40ffffff;

      /* Fixes missing rows in the rendered image, not sure why */

      ARM64_DMB();
      ARM64_DSB();
      ARM64_ISB();
    }

  /* Init Framebuffer 2:
   * Fill with Semi-Transparent Green Circle
   */

  for (y = 0; y < PANEL_HEIGHT; y++)
    {
      for (x = 0; x < PANEL_WIDTH; x++)
        {
          /* Get pixel index */

          const int p = (y * PANEL_WIDTH) + x;

          /* Shift coordinates so that centre of screen is (0,0) */

          const int half_width  = PANEL_WIDTH  / 2;
          const int half_height = PANEL_HEIGHT / 2;
          const int x_shift = x - half_width;
          const int y_shift = y - half_height;

          /* If x^2 + y^2 < radius^2, set to Semi-Transparent Green */

          if (x_shift*x_shift + y_shift*y_shift < half_width*half_width)
            {
              /* Semi-Transparent Green in ARGB 8888 Format */

              g_pinephone_fb2[p] = 0x80008000;
            }
          else  /* Otherwise set to Transparent Black */
            {
              /* Transparent Black in ARGB 8888 Format */

              g_pinephone_fb2[p] = 0x00000000;
            }

          /* Fixes missing rows in the rendered image, not sure why */

          ARM64_DMB();
          ARM64_DSB();
          ARM64_ISB();
        }
    }
}
