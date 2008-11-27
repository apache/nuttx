/****************************************************************************
 * graphics/nxbe/nxbe_fbconfigure.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include "nxbe.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_fbconfigure
 *
 * Description:
 *   Configure the back end state structure based on information from the
 *   framebuffer driver
 *
 ****************************************************************************/

int nxbe_fbconfigure(FAR struct fb_vtable_s *fb, FAR struct nxbe_state_s *be)
{
  int ret;
  int i;

  /* Get the video controller configuration */

  ret = fb->getvideoinfo(fb, &be->vinfo);
  if (ret < 0)
    {
      gdbg("Failed to get vinfo\n");
      return ret;
    }

  /* Check the number of color planes */

  if (be->vinfo.nplanes > CONFIG_NX_NPLANES)
    {
      gdbg("NX configured for only %d planes, controller wants %d\n",
           CONFIG_NX_NPLANES, be->vinfo.nplanes);
      return -E2BIG;
    }
  else if (be->vinfo.nplanes < CONFIG_NX_NPLANES)
    {
      gdbg("NX configured for %d planes, controller only needs %d\n",
           CONFIG_NX_NPLANES, be->vinfo.nplanes);
    }

  /* Then get information about each color plane */

  for (i = 0; i < be->vinfo.nplanes; i++)
    {
      ret = fb->getplaneinfo(fb, i, &be->plane[i].pinfo);
      if (ret < 0)
        {
          gdbg("Failed to get pinfo[%d]\n", i);
          return ret;
        }

      /* Select rasterizers to match the BPP reported for this plane.
       * NOTE that there are configuration options to eliminate support
       * for unused BPP values.  If the unused BPP values are not suppressed
       * in this way, then ALL rasterizers will be drawn into the link and
       * will signicantly increase the size
       */

#ifndef CONFIG_NXGLIB_DISABLE_1BPP
      if (be->plane[i].pinfo.bpp == 1)
        {
          be->plane[i].fillrectangle = nxgl_fillrectangle_1bpp;
          be->plane[i].moverectangle = nxgl_moverectangle_1bpp;
          be->plane[i].copyrectangle = nxgl_copyrectangle_1bpp;
        }
      else
#endif
#ifndef CONFIG_NXGLIB_DISABLE_2BPP
      if (be->plane[i].pinfo.bpp == 2)
        {
          be->plane[i].fillrectangle = nxgl_fillrectangle_2bpp;
          be->plane[i].moverectangle = nxgl_moverectangle_2bpp;
          be->plane[i].copyrectangle = nxgl_copyrectangle_2bpp;
        }
      else
#endif
#ifndef CONFIG_NXGLIB_DISABLE_4BPP
      if (be->plane[i].pinfo.bpp == 4)
        {
          be->plane[i].fillrectangle = nxgl_fillrectangle_4bpp;
          be->plane[i].moverectangle = nxgl_moverectangle_4bpp;
          be->plane[i].copyrectangle = nxgl_copyrectangle_4bpp;
        }
      else
#endif
#ifndef CONFIG_NXGLIB_DISABLE_8BPP
      if (be->plane[i].pinfo.bpp == 8)
        {
          be->plane[i].fillrectangle = nxgl_fillrectangle_8bpp;
          be->plane[i].moverectangle = nxgl_moverectangle_8bpp;
          be->plane[i].copyrectangle = nxgl_copyrectangle_8bpp;
        }
      else
#endif
#ifndef CONFIG_NXGLIB_DISABLE_16BPP
      if (be->plane[i].pinfo.bpp == 16)
        {
          be->plane[i].fillrectangle = nxgl_fillrectangle_16bpp;
          be->plane[i].moverectangle = nxgl_moverectangle_16bpp;
          be->plane[i].copyrectangle = nxgl_copyrectangle_16bpp;
        }
      else
#endif
#ifndef CONFIG_NXGLIB_DISABLE_24BPP
      if (be->plane[i].pinfo.bpp == 24)
        {
          be->plane[i].fillrectangle = nxgl_fillrectangle_24bpp;
          be->plane[i].moverectangle = nxgl_moverectangle_24bpp;
          be->plane[i].copyrectangle = nxgl_copyrectangle_24bpp;
        }
      else
#endif
#ifndef CONFIG_NXGLIB_DISABLE_32BPP
      if (be->plane[i].pinfo.bpp == 32)
        {
          be->plane[i].fillrectangle = nxgl_fillrectangle_32bpp;
          be->plane[i].moverectangle = nxgl_moverectangle_32bpp;
          be->plane[i].copyrectangle = nxgl_copyrectangle_32bpp;
        }
      else
#endif
        {
          gdbg("Unsupported pinfo[%d] BPP: %d\n", i, be->plane[i].pinfo.bpp);
          return -ENOSYS;
        }
    }
  return OK;
}
