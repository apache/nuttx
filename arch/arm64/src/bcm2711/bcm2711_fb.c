/****************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_fb.c
 *
 * Contributed by Matteo Golin
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

#include <debug.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/video/fb.h>

#include "bcm2711_mailbox.h"
#include "arm64_arch.h"
#include "arm64_gic.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Screen resolution */

#define FB_WIDTH (1920)
#define FB_HEIGHT (1080)

/* Bits per pixel (32 for RGB) */

#define FB_BPP (32)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bcm2711_fb_s
{
  struct fb_vtable_s vtable; /* vtable device */
  void *fb;                  /* Frame buffer pointer */
  uint32_t fbsize;           /* Size of frame buffer in bytes */
  int dispno;                /* Display number */
  bool inited;               /* True when initialized */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bcm2711_getvideoinfo(struct fb_vtable_s *vtable,
                                struct fb_videoinfo_s *vinfo);
static int bcm2711_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                                struct fb_planeinfo_s *pinfo);

/* TODO: implement functions for other features available with the RPi
 * frame buffer. There is some mailbox commands for a cursor, for instance.
 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* NOTE: It appears that the frame buffer can refer to either HDMI0 or HDMI1,
 * depending which one is plugged into the display. I don't know what happens
 * when both are plugged in at once, since I only have a single display to
 * test with at the moment. I will therefore just refer to it generally as
 * display 0, but some way to differentiate later would be beneficial.
 */

static struct bcm2711_fb_s g_bcm2711_fb0 =
{
  .inited = 0,
  .dispno = 0,
  .vtable =
    {
      .getvideoinfo    = bcm2711_getvideoinfo,
      .getplaneinfo    = bcm2711_getplaneinfo,
    }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2711_getvideoinfo
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

static int bcm2711_getvideoinfo(struct fb_vtable_s *vtable,
                                struct fb_videoinfo_s *vinfo)
{
  int err;
  uint32_t bpp;
  uint32_t xres;
  uint32_t yres;

  DEBUGASSERT(vtable != NULL);
  DEBUGASSERT(vinfo != NULL);

  vinfo->nplanes = 1; /* Only one plane supported */

  err = bcm2711_mbox_getdisp(&xres, &yres);
  if (err < 0)
    {
      gerr("Couldn't get display dimensions: %d", err);
      return err;
    }

  vinfo->xres = xres;
  vinfo->yres = yres;

  err = bcm2711_mbox_getdepth(&bpp);
  if (err < 0)
    {
      gerr("Couldn't get display depth: %d", err);
      return err;
    }

  /* NOTE: there appears to be no FB_FMT_* descriptors for BGR pixel
   * ordering, so I will assume RGB always.
   *
   * TODO: not even sure how to check for some of the formats. No idea if 16
   * bpp is the 555 or 565 version.
   * Should I also be making an attempt to set pixel order to RGB if the
   * mailbox reports BGR?
   */

  err = bcm2711_mbox_getalpha(&xres);
  if (err < 0)
    {
      gerr("Couldn't get alpha mode: %d", err);
      return err;
    }

  /* TODO: this method ignores the case when the alpha channel is reversed.
   * Not sure how the frame buffer upper-half can handle that.
   */

  if (xres == 1)
    {
      gwarn("Alpha channel reversed, this is not handled");
    }
  else
    {
      ginfo("Alpha channel: %s", xres == 0 ? "enabled" : "ignored");
    }

  switch (bpp)
    {
    case 4:
      DEBUGASSERT(xres == 2);
      vinfo->fmt = FB_FMT_RGB4;
      break;
    case 8:
      DEBUGASSERT(xres == 2);
      vinfo->fmt = FB_FMT_RGB8;
      break;
    case 16:
      vinfo->fmt = xres == 0 ? FB_FMT_RGBA16 : FB_FMT_RGB16_555;
      break;
    case 24:
      DEBUGASSERT(xres == 2);
      vinfo->fmt = FB_FMT_RGB24;
      break;
    case 32:
      vinfo->fmt = xres == 0 ? FB_FMT_RGBA32 : FB_FMT_RGB32;
      break;
    default:
      gerr("Unknown depth of %u bpp", bpp);
      return -EIO;
    }

#ifdef CONFIG_FB_OVERLAY
  vinfo->noverlays = 0; /* No overlays supported */
#endif

  return 0;
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

static int bcm2711_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                                struct fb_planeinfo_s *pinfo)
{
  int err;
  uint32_t result;
#ifdef CONFIG_DEBUG_GRAPHICS_INFO
  uint32_t overscan[4];
#endif
  struct bcm2711_fb_s *priv = (struct bcm2711_fb_s *)vtable;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(pinfo != NULL);
  DEBUGASSERT(planeno == 0); /* Only one supported plane */

#ifdef CONFIG_DEBUG_GRAPHICS_INFO

  /* Get overscan for debugging purposes */

  err = bcm2711_mbox_getoscan(&overscan[0], &overscan[1], &overscan[2],
                              &overscan[3]);
  if (err < 0)
    {
      gerr("Couldn't get overscan amounts: %d", err);
    }

  ginfo("Overscan px: top=%u, bot=%u, left=%u, right=%u", overscan[0],
        overscan[1], overscan[2], overscan[3]);
#endif /* CONFIG_DEBUG_GRAPHICS_INFO */

  /* With over-scan, the frame buffer is actually larger than the screen
   * resolution. We have an extra `top + bottom` pixels in the y, and `left +
   * right` in the x.
   */

  pinfo->fbmem = priv->fb;
  pinfo->fblen = priv->fbsize;
  pinfo->display = priv->dispno;

  /* Get VideoCore information about stride */

  err = bcm2711_mbox_getpitch(&result);
  if (err < 0)
    {
      gerr("Couldn't get pitch: %d", err);
      return err;
    }

  pinfo->stride = result;

  /* Get VideoCore information about bits per pixel */

  err = bcm2711_mbox_getdepth(&result);
  if (err < 0)
    {
      gerr("Couldn't get display depth: %d", err);
      return err;
    }

  pinfo->bpp = result;

  /* Get virtual resolution */

  err = bcm2711_mbox_getvirtres(&pinfo->xres_virtual, &pinfo->yres_virtual);
  if (err)
    {
      gerr("Couldn't get virtual resolution: %d", err);
      return err;
    }

  ginfo("Virtual resolution: %u x %u px",
        pinfo->xres_virtual, pinfo->yres_virtual);

  /* Get offset from virtual resolution to visible resolution */

  err = bcm2711_mbox_getvirtoff(&pinfo->xoffset, &pinfo->yoffset);
  if (err)
    {
      gerr("Couldn't get virtual offset: %d", err);
      return err;
    }

  ginfo("Virtual offset: (%u, %u) px", pinfo->xoffset, pinfo->yoffset);

  return 0;
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
  int err;
  uint8_t tries = 0;
  uint32_t xres = FB_WIDTH;
  uint32_t yres = FB_HEIGHT;
  uint32_t bpp = FB_BPP;
  struct bcm2711_fb_s *priv;

  switch (display)
    {
    case 0:
      priv = &g_bcm2711_fb0;
      break;
    default:
      gerr("Unsupported display: %d", display);
      return -EINVAL;
    }

  /* Already initialized */

  if (priv->inited)
    {
      return 0;
    }

  /* Basic initialization of members */

  priv->fb = NULL;
  priv->fbsize = 0;

  /* Initialize the frame-buffer in a bulk request. Doing this piece-wise
   * never seems to work, always resulting a virtual resolution of 2x2 px.
   * This attempts the initialization three times since the operation always
   * seems to fail right after boot time.
   */

  do
    {
      err = bcm2711_mbox_fbinit(&xres, &yres, &bpp,
                                &priv->fb, &priv->fbsize);
      tries++;
    }
  while (err < 0 && tries < 3);

  if (err < 0)
    {
      gerr("Failed to initialize frame buffer: %d", err);
      return err;
    }

  if (xres != FB_WIDTH || yres != FB_HEIGHT)
    {
      gerr("Display mismatch: wanted %u x %u px, but got %u x %u px",
           FB_WIDTH, FB_HEIGHT, xres, yres);
      return -EIO;
    }

  if (bpp != FB_BPP)
    {
      gerr("Depth mismatch: wanted %u bpp, got %u bpp", FB_BPP, bpp);
      return -EIO;
    }

  priv->inited = true;
  ginfo("Display %d initialized.", display);
  return 0;
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
  DEBUGASSERT(vplane == 0); /* Only one plane supported */

  switch (display)
    {
    case 0:
      return &g_bcm2711_fb0.vtable;
      break;
    default:
      gerr("Invalid display: %d", display);
      return NULL;
    }
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
  int err;
  struct bcm2711_fb_s *priv;

  switch (display)
    {
    case 0:
      priv = &g_bcm2711_fb0;
      break;
    default:
      gerr("Invalid display: %d", display);
      return;
    }

  /* Already uninitialized */

  if (!priv->inited)
    {
      return;
    }

  /* Release frame buffer via mailbox. */

  err = bcm2711_mbox_releasefb();
  if (err < 0)
    {
      gerr("Couldn't release frame buffer: %d", err);
      return;
    }

  priv->inited = false;
  ginfo("Display %d uninitialized.", display);
}
