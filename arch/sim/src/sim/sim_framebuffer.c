/****************************************************************************
 * arch/sim/src/sim/sim_framebuffer.c
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

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/video/fb.h>

#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef FB_FMT
#if CONFIG_SIM_FBBPP == 1
#  define FB_FMT FB_FMT_Y1
#elif CONFIG_SIM_FBBPP == 4
#  define FB_FMT FB_FMT_RGB4
#elif CONFIG_SIM_FBBPP == 8
#  define FB_FMT FB_FMT_RGB8
#elif CONFIG_SIM_FBBPP == 16
#  define FB_FMT FB_FMT_RGB16_565
#elif CONFIG_SIM_FBBPP == 24
#  define FB_FMT FB_FMT_RGB24
#elif CONFIG_SIM_FBBPP == 32
#  define FB_FMT FB_FMT_RGB32
#else
#  error "Unsupported BPP"
#endif

/* Framebuffer characteristics in bytes */

#define FB_WIDTH ((CONFIG_SIM_FBWIDTH * CONFIG_SIM_FBBPP + 7) / 8)
#define FB_SIZE  (FB_WIDTH * CONFIG_SIM_FBHEIGHT)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Get information about the video controller configuration and the
 * configuration of each color plane.
 */

static int sim_getvideoinfo(struct fb_vtable_s *vtable,
                           struct fb_videoinfo_s *vinfo);
static int sim_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                           struct fb_planeinfo_s *pinfo);

/* The following is provided only if the video hardware supports
 * RGB color mapping.
 */

#ifdef CONFIG_FB_CMAP
static int sim_getcmap(struct fb_vtable_s *vtable,
                      struct fb_cmap_s *cmap);
static int sim_putcmap(struct fb_vtable_s *vtable,
                      const struct fb_cmap_s *cmap);
#endif
  /* The following is provided only if the video hardware supports
   * a hardware cursor
   */

#ifdef CONFIG_FB_HWCURSOR
static int sim_getcursor(struct fb_vtable_s *vtable,
                        struct fb_cursorattrib_s *attrib);
static int sim_setcursor(struct fb_vtable_s *vtable,
                        struct fb_setcursor_s *settings);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The simulated framebuffer memory */

#ifndef CONFIG_SIM_X11FB
static uint8_t g_fb[FB_SIZE];
#endif

/* This structure describes the simulated video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt      = FB_FMT,
  .xres     = CONFIG_SIM_FBWIDTH,
  .yres     = CONFIG_SIM_FBHEIGHT,
  .nplanes  = 1,
};

#ifndef CONFIG_SIM_X11FB
/* This structure describes the single, simulated color plane */

static const struct fb_planeinfo_s g_planeinfo =
{
  .fbmem    = (void *)&g_fb,
  .fblen    = FB_SIZE,
  .stride   = FB_WIDTH,
  .display  = 0,
  .bpp      = CONFIG_SIM_FBBPP,
};
#else
static struct work_s g_updatework;

/* This structure describes the single, X11 color plane */

static struct fb_planeinfo_s g_planeinfo;
#endif

/* Current cursor position */

#ifdef CONFIG_FB_HWCURSOR
static struct fb_cursorpos_s g_cpos;

/* Current cursor size */

#ifdef CONFIG_FB_HWCURSORSIZE
static struct fb_cursorsize_s g_csize;
#endif
#endif

/* The framebuffer object -- There is no private state information
 * in this simple framebuffer simulation.
 */

struct fb_vtable_s g_fbobject =
{
  .getvideoinfo  = sim_getvideoinfo,
  .getplaneinfo  = sim_getplaneinfo,
#ifdef CONFIG_FB_CMAP
  .getcmap       = sim_getcmap,
  .putcmap       = sim_putcmap,
#endif
#ifdef CONFIG_FB_HWCURSOR
  .getcursor     = sim_getcursor,
  .setcursor     = sim_setcursor,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_getvideoinfo
 ****************************************************************************/

static int sim_getvideoinfo(struct fb_vtable_s *vtable,
                           struct fb_videoinfo_s *vinfo)
{
  ginfo("vtable=%p vinfo=%p\n", vtable, vinfo);
  if (vtable && vinfo)
    {
      memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
      return OK;
    }

  gerr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: sim_getplaneinfo
 ****************************************************************************/

static int sim_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                           struct fb_planeinfo_s *pinfo)
{
  ginfo("vtable=%p planeno=%d pinfo=%p\n", vtable, planeno, pinfo);
  if (vtable && planeno == 0 && pinfo)
    {
      memcpy(pinfo, &g_planeinfo, sizeof(struct fb_planeinfo_s));
      return OK;
    }

  gerr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: sim_getcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int sim_getcmap(struct fb_vtable_s *vtable,
                      struct fb_cmap_s *cmap)
{
  int len;
  int i;

  ginfo("vtable=%p cmap=%p len=%d\n", vtable, cmap, cmap->len);
  if (vtable && cmap)
    {
      for (i = cmap->first, len = 0; i < 256 && len < cmap->len; i++, len++)
        {
          cmap->red[i]    = i;
          cmap->green[i]  = i;
          cmap->blue[i]   = i;
#ifdef CONFIG_FB_TRANSPARENCY
          cmap->transp[i] = i;
#endif
        }

      cmap->len = len;
      return OK;
    }

  gerr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: sim_putcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int sim_putcmap(struct fb_vtable_s *vtable,
                      const struct fb_cmap_s *cmap)
{
#ifdef CONFIG_SIM_X11FB
  return sim_x11cmap(cmap->first, cmap->len, cmap->red, cmap->green,
                    cmap->blue, NULL);
#else
  ginfo("vtable=%p cmap=%p len=%d\n", vtable, cmap, cmap->len);
  if (vtable && cmap)
    {
      return OK;
    }

  gerr("ERROR: Returning EINVAL\n");
  return -EINVAL;
#endif
}
#endif

/****************************************************************************
 * Name: sim_getcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int sim_getcursor(struct fb_vtable_s *vtable,
                        struct fb_cursorattrib_s *attrib)
{
  ginfo("vtable=%p attrib=%p\n", vtable, attrib);
  if (vtable && attrib)
    {
#ifdef CONFIG_FB_HWCURSORIMAGE
      attrib->fmt      = FB_FMT;
#endif
      ginfo("pos:      (x=%d, y=%d)\n", g_cpos.x, g_cpos.y);
      attrib->pos      = g_cpos;
#ifdef CONFIG_FB_HWCURSORSIZE
      attrib->mxsize.h = CONFIG_SIM_FBHEIGHT;
      attrib->mxsize.w = CONFIG_SIM_FBWIDTH;
      ginfo("size:     (h=%d, w=%d)\n", g_csize.h, g_csize.w);
      attrib->size     = g_csize;
#endif
      return OK;
    }

  gerr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: sim_setcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int sim_setcursor(struct fb_vtable_s *vtable,
                       struct fb_setcursor_s *settings)
{
  ginfo("vtable=%p settings=%p\n", vtable, settings);
  if (vtable && settings)
    {
      ginfo("flags:   %02x\n", settings->flags);
      if ((flags & FB_CUR_SETPOSITION) != 0)
        {
          g_cpos = settings->pos;
          ginfo("pos:     (h:%d, w:%d)\n", g_cpos.x, g_cpos.y);
        }

#ifdef CONFIG_FB_HWCURSORSIZE
      if ((flags & FB_CUR_SETSIZE) != 0)
        {
          g_csize = settings->size;
          ginfo("size:    (h:%d, w:%d)\n", g_csize.h, g_csize.w);
        }
#endif

#ifdef CONFIG_FB_HWCURSORIMAGE
      if ((flags & FB_CUR_SETIMAGE) != 0)
        {
          ginfo("image:   (h:%d, w:%d) @ %p\n",
               settings->img.height, settings->img.width,
               settings->img.image);
        }

#endif
      return OK;
    }

  gerr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: sim_updatework
 ****************************************************************************/

#ifdef CONFIG_SIM_X11FB
static void sim_updatework(void *arg)
{
  work_queue(LPWORK, &g_updatework, sim_updatework, NULL, MSEC2TICK(33));
  sim_x11update();
  fb_pollnotify(&g_fbobject);
}
#endif

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
  int ret = OK;

#ifdef CONFIG_SIM_X11FB
  ret = sim_x11initialize(CONFIG_SIM_FBWIDTH, CONFIG_SIM_FBHEIGHT,
                         &g_planeinfo.fbmem, &g_planeinfo.fblen,
                         &g_planeinfo.bpp, &g_planeinfo.stride);
  if (ret == OK)
    {
      work_queue(LPWORK, &g_updatework, sim_updatework, NULL, MSEC2TICK(33));
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: up_fbgetvplane
 *
 * Description:
 *   Return a a reference to the framebuffer object for the specified video
 *   plane of the specified plane.  Many OSDs support multiple planes of
 *   video.
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
      return &g_fbobject;
    }
  else
    {
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
 *     specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_fbuninitialize(int display)
{
}
