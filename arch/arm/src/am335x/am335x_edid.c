/****************************************************************************
 * arch/arm/src/am335x/am335x_edid.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The LCD driver derives from the LPC54xx LCD driver but also includes
 * information from the FreeBSD AM335x LCD driver which was released under
 * a two-clause BSD license:
 *
 *   Copyright 2013 Oleksandr Tymoshenko <gonzo@freebsd.org>
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/video/edid.h>

#include "am335x_lcdc.h"

/****************************************************************************
 * Pre-processor definitions Functions
 ****************************************************************************/

#define MODE_HBP(mode)  ((mode)->htotal - (mode)->hsync_end)
#define MODE_HFP(mode)  ((mode)->hsync_start - (mode)->hdisplay)
#define MODE_HSW(mode)  ((mode)->hsync_end - (mode)->hsync_start)
#define MODE_VBP(mode)  ((mode)->vtotal - (mode)->vsync_end)
#define MODE_VFP(mode)  ((mode)->vsync_start - (mode)->vdisplay)
#define MODE_VSW(mode)  ((mode)->vsync_end - (mode)->vsync_start)

#define MAX_PIXEL_CLOCK 126000
#define MAX_BANDWIDTH   (1280*1024*60)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_lcd_edid
 *
 * Description:
 *   Return the vertical refresh rate for this video mode.
 *
 ****************************************************************************/

static uint32_t
am335x_videomode_vrefresh(const struct videomode_s *videomode)
{
  uint32_t refresh;

  /* Calculate vertical refresh rate */

  refresh = (videomode->dotclock * 1000 / videomode->htotal);
  refresh = (refresh + videomode->vtotal / 2) / videomode->vtotal;

  if (videomode->flags & VID_INTERLACE)
    {
      refresh *= 2;
    }

  if (videomode->flags & VID_DBLSCAN)
    {
     refresh /= 2;
    }

  return refresh;
}

/****************************************************************************
 * Name: am335x_videomode_valid
 *
 * Description:
 *   Return true if the provided video mode is valid.
 *
 ****************************************************************************/

static bool
am335x_videomode_valid(const struct videomode_s *videomode)
{
  size_t fbstride;
  size_t fbsize;
  uint32_t hbp;
  uint32_t hfp;
  uint32_t hsw;
  uint32_t vbp;
  uint32_t vfp;
  uint32_t vsw;
  uint32_t vrefresh;

  if (videomode->dotclock > MAX_PIXEL_CLOCK)
    {
      return false;
    }

  if (videomode->hdisplay & 0xf)
    {
      return false;
    }

  if (videomode->vdisplay > 2048)
    {
      return false;
    }

  /* Check ranges for timing parameters */

  hbp = MODE_HBP(videomode) - 1;
  hfp = MODE_HFP(videomode) - 1;
  hsw = MODE_HSW(videomode) - 1;
  vbp = MODE_VBP(videomode);
  vfp = MODE_VFP(videomode);
  vsw = MODE_VSW(videomode) - 1;

  if (hbp > 0x3ff)
    {
      return false;
    }

  if (hfp > 0x3ff)
    {
      return false;
    }

  if (hsw > 0x3ff)
    {
      return false;
    }

  if (vbp > 0xff)
    {
      return false;
    }

  if (vfp > 0xff)
    {
      return false;
    }

  if (vsw > 0x3f)
    {
      return false;
    }

  vrefresh = am335x_videomode_vrefresh(videomode);
  if (videomode->vdisplay * videomode->hdisplay * vrefresh > MAX_BANDWIDTH)
    {
      return false;
    }

  /* Finally, make sure that the framebuffer buffer region is large enough
   * to support this video mode.
   */

  fbstride = (videomode->hdisplay * AM335X_BPP + 7) >> 3;
  fbsize   = videomode->vdisplay * fbstride;

  if (fbsize > AM335X_LCDC_FB_SIZE)
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: am335x_lcd_pickmode
 *
 * Description:
 *   If there is access to Extended Display Identification Data (EDIDI),
 *   then the board-specific logic may read the EDID data and use this
 *   function to select an appropriate video mode.
 *
 *   edid_parse() should be used to convert the raw EDID data into the
 *   digested form of struct edid_info.
 *
 *   The returned video mode may be used to both (1) configure HDMI and (2)
 *   initialize the AM335x LCD controller.
 *
 ****************************************************************************/

static const struct videomode_s *
am335x_lcd_pickmode(struct edid_info_s *ei)
{
  const struct videomode_s *videomode;
  int n;

  /* Get standard VGA as default */

  videomode = NULL;

  /* Pick a video mode -- First check if we can support the preferred mode. */

  if (ei->edid_preferred_mode != NULL)
    {
      if (am335x_videomode_valid(ei->edid_preferred_mode))
        {
          videomode = ei->edid_preferred_mode;
          return videomode;
        }
    }

  /* Sort video modes by refresh rate, aspect ratio (*), then resolution.
   * Preferred mode or largest mode is first in the list and other modes
   * are sorted on closest match to that mode.
   */

  sort_videomodes(ei->edid_modes, &ei->edid_preferred_mode, ei->edid_nmodes);

  /* Pick the first valid mode in the list */

  for (n = 0; n < ei->edid_nmodes; n++)
    {
      if (am335x_videomode_valid(&ei->edid_modes[n]))
        {
          videomode = &ei->edid_modes[n];
          break;
        }
    }

  return videomode;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_lcd_videomode
 *
 * Description:
 *   If the video mod is known, then the board-specific logic may read the
 *   use this function to convert the video mode data to an instance of
 *   struct am335x_panel_info_s which then may be used to initialize the
 *   the LCD/
 *
 * Input Parameters:
 *    videomode - A reference to the desired video mode.
 *    panel    - A user provided location to receive the panel data.
 *
 * Returned value:
 *   None.  Always succeeds.
 *
 ****************************************************************************/

void am335x_lcd_videomode(const struct videomode_s *videomode,
                          struct am335x_panel_info_s *panel)
{
  lcdinfo("Detected videomode: %dx%d @ %" PRId32 "KHz\n",
          videomode->hdisplay, videomode->vdisplay,
          am335x_videomode_vrefresh(videomode));

  panel->width           = videomode->hdisplay;
  panel->height          = videomode->vdisplay;
  panel->hfp             = videomode->hsync_start - videomode->hdisplay;
  panel->hbp             = videomode->htotal      - videomode->hsync_end;
  panel->hsw             = videomode->hsync_end   - videomode->hsync_start;
  panel->vfp             = videomode->vsync_start - videomode->vdisplay;
  panel->vbp             = videomode->vtotal      - videomode->vsync_end;
  panel->vsw             = videomode->vsync_end   - videomode->vsync_start;
  panel->pixelclk_active = true;

  /* Logic for HSYNC should be reversed */

  panel->hsync_active    = ((videomode->flags & VID_NHSYNC) != 0);
  panel->vsync_active    = ((videomode->flags & VID_NVSYNC) == 0);
  panel->pixclk          = videomode->dotclock * 1000;

  /* Set other values to the default */

#ifdef CONFIG_AM335X_LCDC_SYNC_EDGE
  panel->sync_edge       = true;
#else
  panel->sync_edge       = false;
#endif

#ifdef CONFIG_AM335X_LCDC_SYNC_CTRL
  panel->sync_ctrl       = true;
#else
  panel->sync_ctrl       = false;
#endif

#ifdef CONFIG_AM335X_LCDC_PIXCLK_INVERT
  panel->pixelclk_active = true;
#else
  panel->pixelclk_active = false;
#endif

  panel->acbias          = CONFIG_AM335X_LCDC_ACBIAS;
  panel->acbias_pint     = CONFIG_AM335X_LCDC_ACBIAS_PINT;
  panel->dma_burstsz     = AM335X_LCD_DMA_BURSTSZ;
  panel->bpp             = AM335X_BPP;                     /* REVISIT */
  panel->fdd             = CONFIG_AM335X_LCDC_FDD;
}

/****************************************************************************
 * Name: am335x_lcd_edid
 *
 * Description:
 *   If there is access to Extended Display Identification Data (EDID),
 *   then the board-specific logic may read the EDID data and use this
 *   function to initialize an instance of struct am335x_panel_info_s.
 *
 *   The returned video mode may optionally be returned to configure HDMI.
 *
 * Input Parameters:
 *   edid     - A reference to the raw EDID data.
 *   len      - The length of the EDID data in bytes
 *   panel    - A user provided location to receive the panel data.
 *   selected - A user provided location to receive the selected video mode.
 *
 * Returned value:
 *   None.  Always succeeds.  The logic will fallback to VGA mode if no
 *   EDID data is provided or if there is no valid video mode in the EDID
 *   data.
 *
 ****************************************************************************/

void am335x_lcd_edid(const uint8_t *edid, size_t edid_len,
                     struct am335x_panel_info_s *panel,
                     const struct videomode_s **selected)
{
  const struct videomode_s *videomode = NULL;
  struct edid_info_s ei;

  /* Do we have EDID data? */

  if (edid != NULL && edid_len > 0)
    {
      /* Parse the EDID data */

      if (edid_parse(edid, &ei) == 0)
        {
          videomode = am335x_lcd_pickmode(&ei);
        }
      else
        {
          lcderr("ERROR: Failed to parse EDID\n");
        }
    }

  /* Use standard VGA as fallback */

  if (videomode == NULL)
    {
      videomode = videomode_lookup_by_name("640x480x60");
      DEBUGASSERT(videomode != NULL);
    }

  /* Initialize the LCD using the selected video mode */

  am335x_lcd_videomode(videomode, panel);

  /* Return the selected video mode */

  if (selected != NULL)
    {
      *selected = videomode;
    }
}
