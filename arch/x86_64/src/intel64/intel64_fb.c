/****************************************************************************
 * arch/x86_64/src/intel64/intel64_fb.c
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

#include <string.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <arch/multiboot2.h>

#include <nuttx/video/fb.h>

#include "x86_64_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct multiboot_fb_s
{
  /* Framebuffer interface */

  struct fb_vtable_s vtable;

  /* Framebuffer video information */

  struct fb_videoinfo_s videoinfo;

  /* Framebuffer plane information */

  struct fb_planeinfo_s planeinfo;

  /* Framebuffer base address */

  void *baseaddr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Framebuffer interface */

static int intel64_getvideoinfo(struct fb_vtable_s *vtable,
                                struct fb_videoinfo_s *vinfo);
static int intel64_getplaneinfo(struct fb_vtable_s *vtable,
                                int planeno,
                                struct fb_planeinfo_s *pinfo);

/* Helpers */

static uint8_t intel64_fb_getfmt(uint8_t bpp, uint8_t type);
static void intel64_fb_clear(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern struct multiboot_tag_framebuffer *g_mb_fb_tag;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct multiboot_fb_s g_fb =
{
  .vtable =
  {
    .getvideoinfo = intel64_getvideoinfo,
    .getplaneinfo = intel64_getplaneinfo
  },

  .baseaddr = NULL
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: intel64_getvideoinfo
 ****************************************************************************/

static int intel64_getvideoinfo(struct fb_vtable_s *vtable,
                                struct fb_videoinfo_s *vinfo)
{
  struct multiboot_fb_s *priv = (struct multiboot_fb_s *)vtable;

  memcpy(vinfo, &priv->videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/****************************************************************************
 * Name: intel64_getplaneinfo
 ****************************************************************************/

static int intel64_getplaneinfo(struct fb_vtable_s *vtable,
                                int planeno,
                                struct fb_planeinfo_s *pinfo)
{
  struct multiboot_fb_s *priv = (struct multiboot_fb_s *)vtable;

  if (planeno == 0)
    {
      memcpy(pinfo, &priv->planeinfo, sizeof(struct fb_planeinfo_s));
      return OK;
    }

  gerr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: intel64_fb_getfmt
 ****************************************************************************/

static uint8_t intel64_fb_getfmt(uint8_t bpp, uint8_t type)
{
  if (type != MULTIBOOT_FRAMEBUFFER_TYPE_RGB)
    {
      /* Only RGB type supported */

      gerr("ERROR: not supported type=%d\n", type);
      PANIC();
    }

  switch (bpp)
    {
      case 8:
        {
          return FB_FMT_RGB8;
        }

      case 24:
        {
          return FB_FMT_RGB24;
        }

      case 32:
        {
          return FB_FMT_RGB32;
        }

      default:
        {
          gerr("ERROR: not supported BPP=%d\n", bpp);
          PANIC();
        }
    }
}

/****************************************************************************
 * Name: intel64_fb_clear
 ****************************************************************************/

static void intel64_fb_clear(void)
{
  if (g_fb.baseaddr == NULL)
    {
      return;
    }

  memset(g_fb.baseaddr, 0, g_fb.planeinfo.stride * g_fb.videoinfo.yres);
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
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int up_fbinitialize(int display)
{
  struct multiboot_tag_framebuffer *fbt = g_mb_fb_tag;
  struct multiboot_fb_s            *fb  = &g_fb;

  UNUSED(display);

  /* Return error if multiboot tag is empty */

  if (fbt == NULL)
    {
      return -ENODEV;
    }

  /* Get framebuffer base address */

  fb->baseaddr = (void *)(uintptr_t)fbt->common.framebuffer_addr;

  /* Get video info */

  fb->videoinfo.xres    = fbt->common.framebuffer_width;
  fb->videoinfo.yres    = fbt->common.framebuffer_height;
  fb->videoinfo.nplanes = 1;
  fb->videoinfo.fmt     = intel64_fb_getfmt(fbt->common.framebuffer_bpp,
                                            fbt->common.framebuffer_type);

  /* Get plane info */

  fb->planeinfo.fbmem  = fb->baseaddr;
  fb->planeinfo.fblen  = (fbt->common.framebuffer_pitch *
                          fb->videoinfo.yres);
  fb->planeinfo.stride = fbt->common.framebuffer_pitch;
  fb->planeinfo.bpp    = fbt->common.framebuffer_bpp;

  /* Map framebuffer memory.
   * NOTE: framebuffer base address may lie above 4GB for real hardware,
   *       in that case CONFIG_MM_PGALLOC=y must be enabled so
   *       up_map_region() can map this address.
   */

  up_map_region(fb->baseaddr, fb->planeinfo.stride * fb->videoinfo.yres,
                X86_PAGE_WR | X86_PAGE_PRESENT |
                X86_PAGE_NOCACHE | X86_PAGE_GLOBAL);

  /* Clear frambufer */

  intel64_fb_clear();

  return OK;
}

/****************************************************************************
 * Name: up_fbgetvplane
 *
 * Description:
 *   Return a a reference to the framebuffer object for the specified video
 *   plane of the specified plane.
 *   Many OSDs support multiple planes of video.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *   vplane - Identifies the plane being queried.
 *
 * Returned Value:
 *   A non-NULL pointer to the frame buffer access structure is returned on
 *   success; NULL is returned on any failure.
 *
 ****************************************************************************/

struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  if (vplane == 0)
    {
      return &g_fb.vtable;
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
 *     specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_fbuninitialize(int display)
{
  UNUSED(display);

  /* Clear frambufer */

  intel64_fb_clear();
}
