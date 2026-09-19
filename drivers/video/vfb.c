/****************************************************************************
 * drivers/video/vfb.c
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

/* A framebuffer with no display behind it:  memory, registered as
 * /dev/fbN, with the geometry the configuration asks for.
 *
 * Applications draw into it exactly as they would into a panel, and
 * whatever wants the pixels, a VNC server, a screen recorder, a test
 * harness comparing renders, reads the same memory and learns what
 * changed through the framebuffer's dirty-area reporting.  That makes a
 * board with no display, or one whose display is the wrong size for what
 * is being developed, run the same graphics stack as one with a panel.
 *
 * There is deliberately nothing here about who consumes it.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/video/fb.h>
#include <nuttx/video/vfb.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VFB_WIDTH   CONFIG_VIDEO_VFB_WIDTH
#define VFB_HEIGHT  CONFIG_VIDEO_VFB_HEIGHT

#ifdef CONFIG_VIDEO_VFB_FMT_RGB32
#  define VFB_FMT   FB_FMT_RGB32
#  define VFB_BPP   32
#else
#  define VFB_FMT   FB_FMT_RGB16_565
#  define VFB_BPP   16
#endif

#define VFB_STRIDE  (VFB_WIDTH * ((VFB_BPP + 7) >> 3))
#define VFB_SIZE    (VFB_STRIDE * VFB_HEIGHT)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int vfb_getvideoinfo(FAR struct fb_vtable_s *vtable,
                            FAR struct fb_videoinfo_s *vinfo);
static int vfb_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                            FAR struct fb_planeinfo_s *pinfo);
#ifdef CONFIG_FB_UPDATE
static int vfb_updatearea(FAR struct fb_vtable_s *vtable,
                          FAR const struct fb_area_s *area);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct fb_vtable_s g_vfb_vtable =
{
  .getvideoinfo = vfb_getvideoinfo,
  .getplaneinfo = vfb_getplaneinfo,
#ifdef CONFIG_FB_UPDATE
  .updatearea   = vfb_updatearea,
#endif
};

static FAR uint8_t *g_vfb_buffer;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vfb_getvideoinfo
 ****************************************************************************/

static int vfb_getvideoinfo(FAR struct fb_vtable_s *vtable,
                            FAR struct fb_videoinfo_s *vinfo)
{
  if (vtable == NULL || vinfo == NULL)
    {
      return -EINVAL;
    }

  memset(vinfo, 0, sizeof(*vinfo));
  vinfo->fmt     = VFB_FMT;
  vinfo->xres    = VFB_WIDTH;
  vinfo->yres    = VFB_HEIGHT;
  vinfo->nplanes = 1;

  return OK;
}

/****************************************************************************
 * Name: vfb_getplaneinfo
 ****************************************************************************/

static int vfb_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                            FAR struct fb_planeinfo_s *pinfo)
{
  if (vtable == NULL || pinfo == NULL || planeno != 0)
    {
      return -EINVAL;
    }

  memset(pinfo, 0, sizeof(*pinfo));
  pinfo->fbmem   = g_vfb_buffer;
  pinfo->fblen   = VFB_SIZE;
  pinfo->stride  = VFB_STRIDE;
  pinfo->display = 0;
  pinfo->bpp     = VFB_BPP;

  return OK;
}

/****************************************************************************
 * Name: vfb_updatearea
 *
 * Description:
 *   There is no panel to push anything to.  The call exists so that the
 *   framebuffer core sees a driver that supports updates and passes the
 *   areas on to whoever is watching.
 *
 ****************************************************************************/

#ifdef CONFIG_FB_UPDATE
static int vfb_updatearea(FAR struct fb_vtable_s *vtable,
                          FAR const struct fb_area_s *area)
{
  UNUSED(vtable);
  UNUSED(area);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vfb_register
 *
 * Description:
 *   Allocate the framebuffer and register it at /dev/fbN.  Nothing here
 *   is board specific, so a board that has a panel of its own can build
 *   this as well and give the two different display numbers.
 *
 ****************************************************************************/

int vfb_register(int display)
{
  bool allocated = false;
  int ret;

  if (g_vfb_buffer == NULL)
    {
      g_vfb_buffer = kmm_zalloc(VFB_SIZE);
      if (g_vfb_buffer == NULL)
        {
          gerr("ERROR: Failed to allocate %d bytes for the virtual "
               "framebuffer\n", VFB_SIZE);
          return -ENOMEM;
        }

      allocated = true;

      ginfo("Virtual framebuffer: %dx%d, %d bpp, %d bytes at %p\n",
            VFB_WIDTH, VFB_HEIGHT, VFB_BPP, VFB_SIZE, g_vfb_buffer);
    }

  ret = fb_register_device(display, 0, &g_vfb_vtable);
  if (ret < 0)
    {
      gerr("ERROR: fb_register_device() failed for display %d: %d\n",
           display, ret);

      /* Only undo what this call did.  Another display may already be
       * registered against the same memory.
       */

      if (allocated)
        {
          kmm_free(g_vfb_buffer);
          g_vfb_buffer = NULL;
        }
    }

  return ret;
}
