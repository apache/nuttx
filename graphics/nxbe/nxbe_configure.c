/****************************************************************************
 * graphics/nxbe/nxbe_configure.c
 *
 *   Copyright (C) 2008-2010, 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stddef.h>
#include <errno.h>
#include <debug.h>

#include "nxglib.h"
#include "nxbe.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NX_BGCOLOR
#  define CONFIG_NX_BGCOLOR 0
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const nxgl_mxpixel_t g_bgcolor[CONFIG_NX_NPLANES] =
{
  CONFIG_NX_BGCOLOR

#if CONFIG_NX_NPLANES > 1
#  warning Missing logic for multiple color planes
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_configure
 *
 * Description:
 *   Configure the back end state structure based on information from the
 *   framebuffer driver
 *
 ****************************************************************************/

int nxbe_configure(FAR NX_DRIVERTYPE *dev, FAR struct nxbe_state_s *be)
{
  int ret;
  int i;

  /* Get the video controller configuration */

  ret = dev->getvideoinfo(dev, &be->vinfo);
  if (ret < 0)
    {
      gerr("ERROR: Failed to get vinfo\n");
      return ret;
    }

  /* Set the initial background color */

  nxgl_colorcopy(be->bgcolor, g_bgcolor);

  /* Check the number of color planes */

#ifdef CONFIG_DEBUG_GRAPHICS
  if (be->vinfo.nplanes > CONFIG_NX_NPLANES)
    {
      gerr("ERROR: NX configured for only %d planes, controller wants %d\n",
           CONFIG_NX_NPLANES, be->vinfo.nplanes);
      return -E2BIG;
    }
#endif

  /* Then get information about each color plane */

  for (i = 0; i < be->vinfo.nplanes; i++)
    {
      ret = dev->getplaneinfo(dev, i, &be->plane[i].pinfo);
      if (ret < 0)
        {
          gerr("ERROR: Failed to get pinfo[%d]\n", i);
          return ret;
        }

      be->plane[i].driver = dev;

      /* Select rasterizers to match the BPP reported for this plane.
       * NOTE that there are configuration options to eliminate support
       * for unused BPP values.  If the unused BPP values are not suppressed
       * in this way, then ALL rasterizers will be drawn into the link and
       * will signicantly increase the size
       */

#ifndef CONFIG_NX_DISABLE_1BPP
      if (be->plane[i].pinfo.bpp == 1)
        {
          be->plane[i].dev.setpixel       = nxgl_setpixel_1bpp;
          be->plane[i].dev.fillrectangle  = nxgl_fillrectangle_1bpp;
          be->plane[i].dev.getrectangle   = nxgl_getrectangle_1bpp;
          be->plane[i].dev.filltrapezoid  = nxgl_filltrapezoid_1bpp;
          be->plane[i].dev.moverectangle  = nxgl_moverectangle_1bpp;
          be->plane[i].dev.copyrectangle  = nxgl_copyrectangle_1bpp;

#ifdef CONFIG_NX_RAMBACKED
          be->plane[i].pwfb.setpixel      = pwfb_setpixel_1bpp;
          be->plane[i].pwfb.fillrectangle = pwfb_fillrectangle_1bpp;
          be->plane[i].pwfb.getrectangle  = pwfb_getrectangle_1bpp;
          be->plane[i].pwfb.filltrapezoid = pwfb_filltrapezoid_1bpp;
          be->plane[i].pwfb.moverectangle = pwfb_moverectangle_1bpp;
          be->plane[i].pwfb.copyrectangle = pwfb_copyrectangle_1bpp;
#endif
        }
      else
#endif
#ifndef CONFIG_NX_DISABLE_2BPP
      if (be->plane[i].pinfo.bpp == 2)
        {
          be->plane[i].dev.setpixel       = nxgl_setpixel_2bpp;
          be->plane[i].dev.fillrectangle  = nxgl_fillrectangle_2bpp;
          be->plane[i].dev.getrectangle   = nxgl_getrectangle_2bpp;
          be->plane[i].dev.filltrapezoid  = nxgl_filltrapezoid_2bpp;
          be->plane[i].dev.moverectangle  = nxgl_moverectangle_2bpp;
          be->plane[i].dev.copyrectangle  = nxgl_copyrectangle_2bpp;

#ifdef CONFIG_NX_RAMBACKED
          be->plane[i].pwfb.setpixel      = pwfb_setpixel_2bpp;
          be->plane[i].pwfb.fillrectangle = pwfb_fillrectangle_2bpp;
          be->plane[i].pwfb.getrectangle  = pwfb_getrectangle_2bpp;
          be->plane[i].pwfb.filltrapezoid = pwfb_filltrapezoid_2bpp;
          be->plane[i].pwfb.moverectangle = pwfb_moverectangle_2bpp;
          be->plane[i].pwfb.copyrectangle = pwfb_copyrectangle_2bpp;
#endif
        }
      else
#endif
#ifndef CONFIG_NX_DISABLE_4BPP
      if (be->plane[i].pinfo.bpp == 4)
        {
          be->plane[i].dev.setpixel       = nxgl_setpixel_4bpp;
          be->plane[i].dev.fillrectangle  = nxgl_fillrectangle_4bpp;
          be->plane[i].dev.getrectangle   = nxgl_getrectangle_4bpp;
          be->plane[i].dev.filltrapezoid  = nxgl_filltrapezoid_4bpp;
          be->plane[i].dev.moverectangle  = nxgl_moverectangle_4bpp;
          be->plane[i].dev.copyrectangle  = nxgl_copyrectangle_4bpp;

#ifdef CONFIG_NX_RAMBACKED
          be->plane[i].pwfb.setpixel      = pwfb_setpixel_4bpp;
          be->plane[i].pwfb.fillrectangle = pwfb_fillrectangle_4bpp;
          be->plane[i].pwfb.getrectangle  = pwfb_getrectangle_4bpp;
          be->plane[i].pwfb.filltrapezoid = pwfb_filltrapezoid_4bpp;
          be->plane[i].pwfb.moverectangle = pwfb_moverectangle_4bpp;
          be->plane[i].pwfb.copyrectangle = pwfb_copyrectangle_4bpp;
#endif
        }
      else
#endif
#ifndef CONFIG_NX_DISABLE_8BPP
      if (be->plane[i].pinfo.bpp == 8)
        {
          be->plane[i].dev.setpixel       = nxgl_setpixel_8bpp;
          be->plane[i].dev.fillrectangle  = nxgl_fillrectangle_8bpp;
          be->plane[i].dev.getrectangle   = nxgl_getrectangle_8bpp;
          be->plane[i].dev.filltrapezoid  = nxgl_filltrapezoid_8bpp;
          be->plane[i].dev.moverectangle  = nxgl_moverectangle_8bpp;
          be->plane[i].dev.copyrectangle  = nxgl_copyrectangle_8bpp;

#ifdef CONFIG_NX_RAMBACKED
          be->plane[i].pwfb.setpixel      = pwfb_setpixel_8bpp;
          be->plane[i].pwfb.fillrectangle = pwfb_fillrectangle_8bpp;
          be->plane[i].pwfb.getrectangle  = pwfb_getrectangle_8bpp;
          be->plane[i].pwfb.filltrapezoid = pwfb_filltrapezoid_8bpp;
          be->plane[i].pwfb.moverectangle = pwfb_moverectangle_8bpp;
          be->plane[i].pwfb.copyrectangle = pwfb_copyrectangle_8bpp;
#endif

#ifdef CONFIG_NX_SWCURSOR
          be->plane[i].cursor.draw        = nxglib_cursor_draw_8bpp;
          be->plane[i].cursor.erase       = nxglib_cursor_erase_8bpp;
          be->plane[i].cursor.backup      = nxglib_cursor_backup_8bpp;
#endif
        }
      else
#endif
#ifndef CONFIG_NX_DISABLE_16BPP
      if (be->plane[i].pinfo.bpp == 16)
        {
          be->plane[i].dev.setpixel       = nxgl_setpixel_16bpp;
          be->plane[i].dev.fillrectangle  = nxgl_fillrectangle_16bpp;
          be->plane[i].dev.getrectangle   = nxgl_getrectangle_16bpp;
          be->plane[i].dev.filltrapezoid  = nxgl_filltrapezoid_16bpp;
          be->plane[i].dev.moverectangle  = nxgl_moverectangle_16bpp;
          be->plane[i].dev.copyrectangle  = nxgl_copyrectangle_16bpp;

#ifdef CONFIG_NX_RAMBACKED
          be->plane[i].pwfb.setpixel      = pwfb_setpixel_16bpp;
          be->plane[i].pwfb.fillrectangle = pwfb_fillrectangle_16bpp;
          be->plane[i].pwfb.getrectangle  = pwfb_getrectangle_16bpp;
          be->plane[i].pwfb.filltrapezoid = pwfb_filltrapezoid_16bpp;
          be->plane[i].pwfb.moverectangle = pwfb_moverectangle_16bpp;
          be->plane[i].pwfb.copyrectangle = pwfb_copyrectangle_16bpp;
#endif

#ifdef CONFIG_NX_SWCURSOR
          be->plane[i].cursor.draw        = nxglib_cursor_draw_16bpp;
          be->plane[i].cursor.erase       = nxglib_cursor_erase_16bpp;
          be->plane[i].cursor.backup      = nxglib_cursor_backup_16bpp;
#endif
        }
      else
#endif
#ifndef CONFIG_NX_DISABLE_24BPP
      if (be->plane[i].pinfo.bpp == 24)
        {
          be->plane[i].dev.setpixel       = nxgl_setpixel_24bpp;
          be->plane[i].dev.fillrectangle  = nxgl_fillrectangle_24bpp;
          be->plane[i].dev.getrectangle   = nxgl_getrectangle_24bpp;
          be->plane[i].dev.filltrapezoid  = nxgl_filltrapezoid_24bpp;
          be->plane[i].dev.moverectangle  = nxgl_moverectangle_24bpp;
          be->plane[i].dev.copyrectangle  = nxgl_copyrectangle_24bpp;

#ifdef CONFIG_NX_RAMBACKED
          be->plane[i].pwfb.setpixel      = pwfb_setpixel_24bpp;
          be->plane[i].pwfb.fillrectangle = pwfb_fillrectangle_24bpp;
          be->plane[i].pwfb.getrectangle  = pwfb_getrectangle_24bpp;
          be->plane[i].pwfb.filltrapezoid = pwfb_filltrapezoid_24bpp;
          be->plane[i].pwfb.moverectangle = pwfb_moverectangle_24bpp;
          be->plane[i].pwfb.copyrectangle = pwfb_copyrectangle_24bpp;
#endif

#ifdef CONFIG_NX_SWCURSOR
          be->plane[i].cursor.draw        = nxglib_cursor_draw_24bpp;
          be->plane[i].cursor.erase       = nxglib_cursor_erase_24bpp;
          be->plane[i].cursor.backup      = nxglib_cursor_backup_24bpp;
#endif
        }
      else
#endif
#ifndef CONFIG_NX_DISABLE_32BPP
      if (be->plane[i].pinfo.bpp == 32)
        {
          be->plane[i].dev.setpixel       = nxgl_setpixel_32bpp;
          be->plane[i].dev.fillrectangle  = nxgl_fillrectangle_32bpp;
          be->plane[i].dev.getrectangle   = nxgl_getrectangle_32bpp;
          be->plane[i].dev.filltrapezoid  = nxgl_filltrapezoid_32bpp;
          be->plane[i].dev.moverectangle  = nxgl_moverectangle_32bpp;
          be->plane[i].dev.copyrectangle  = nxgl_copyrectangle_32bpp;

#ifdef CONFIG_NX_RAMBACKED
          be->plane[i].pwfb.setpixel      = pwfb_setpixel_1bpp;
          be->plane[i].pwfb.fillrectangle = pwfb_fillrectangle_32bpp;
          be->plane[i].pwfb.getrectangle  = pwfb_getrectangle_32bpp;
          be->plane[i].pwfb.filltrapezoid = pwfb_filltrapezoid_32bpp;
          be->plane[i].pwfb.moverectangle = pwfb_moverectangle_32bpp;
          be->plane[i].pwfb.copyrectangle = pwfb_copyrectangle_32bpp;
#endif

#ifdef CONFIG_NX_SWCURSOR
          be->plane[i].cursor.draw        = nxglib_cursor_draw_32bpp;
          be->plane[i].cursor.erase       = nxglib_cursor_erase_32bpp;
          be->plane[i].cursor.backup      = nxglib_cursor_backup_32bpp;
#endif
        }
      else
#endif
        {
          gerr("ERROR: Unsupported pinfo[%d] BPP: %d\n",
               i, be->plane[i].pinfo.bpp);
          return -ENOSYS;
        }
    }

  return OK;
}
