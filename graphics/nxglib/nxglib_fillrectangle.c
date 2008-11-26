/****************************************************************************
 * graphics/nxglib/nxglib_fillrectangle.c
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

#ifndef NXGLIB_SUFFIX
#  error "NXGLIB_SUFFIX must be defined before including this header file"
#endif

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
 * Name: nxgl_fillrectangle_*bpp
 *
 * Descripton:
 *   Fill a rectangle region in the framebuffer memory with a fixed color
 *
 ****************************************************************************/

void NXGL_FUNCNAME(nxgl_fillrectangle,NXGLIB_SUFFIX)
(FAR struct fb_planeinfo_s *pinfo, FAR const struct nxgl_rect_s *rect, nxgl_mxpixel_t color)
{
  ubyte *line;
  unsigned int width;
  unsigned int stride;
  unsigned int rows;

  /* Get the width of the framebuffer in bytes */

  stride = pinfo->stride;

  /* Get the dimensions of the rectange to fill in pixels */

  width  = rect->pt2.x - rect->pt1.x;
  rows   = rect->pt2.y - rect->pt1.y;

  /* Get the address of the first byte in the first line to write */

  line   = pinfo->fbmem + rect->pt1.y * stride + NXGL_SCALEX(rect->pt1.x);

  /* Then fill the rectangle line-by-line */

  while (rows--)
    {
      NXGL_MEMSET(line, (NXGL_PIXEL_T)color, width);
      line += stride;
    }
}
