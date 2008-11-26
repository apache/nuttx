/****************************************************************************
 * graphics/nxglib/nxsglib_copyrectangle.c
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
#include <nuttx/fb.h>
#include <nuttx/nxglib.h>

#include "nxglib_bitblit.h"

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
 * Name: nxgl_copyrectangle_*bpp
 *
 * Descripton:
 *   Copy a rectangular bitmap image into the specific position in the
 *   framebuffer memory.
 *
 ****************************************************************************/

void NXGL_FUNCNAME(nxgl_copyrectangle,NXGLIB_SUFFIX)
(FAR struct fb_planeinfo_s *pinfo, FAR const struct nxgl_rect_s *dest,
 FAR const void *src, FAR const struct nxgl_point_s *origin,
 unsigned int srcstride)
{
  const ubyte *sptr;
  ubyte *dptr;
  unsigned int width;
  unsigned int deststride;
  unsigned int rows;

  /* Get the width of the framebuffer in bytes */

  deststride = pinfo->stride;

  /* Get the dimensions of the rectange to fill:  height in rows and width in bytes */

  width = NXGL_SCALEX(dest->pt2.x - dest->pt1.x);
  rows = dest->pt2.y - dest->pt1.y;

  /* Then copy the image */

  sptr = (const ubyte*)src + NXGL_SCALEX(dest->pt1.x - origin->x) + (dest->pt1.y - origin->y) * srcstride;
  dptr = pinfo->fbmem + dest->pt1.y * deststride + NXGL_SCALEX(dest->pt1.x);

  while (rows--)
    {
      NXGL_MEMCPY((NXGL_PIXEL_T*)dest, (NXGL_PIXEL_T*)sptr, width);
      dptr += deststride;
      sptr += srcstride;
    }
}
