/****************************************************************************
 * graphics/nxbe/nxbe_colormap.c
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
 * Name: nxbe_colormap
 *
 * Description:
 *   Set the harware color map to the palette expected by NX
 *
 ****************************************************************************/

#if CONFIG_FB_CMAP
int nxbe_colormap(FAR const fb_vtable_s *fb)
{
  struct fb_cmap cmap;
  ubyte  *alloc;
  ubyte  *red;
  ubyte  *green;
  ubyte  *blue;
  ubyte   rval;
  ubyte   gval;
  int     size;
  int     ndx;
  int     ret;
  int     i, j, k;

  /* Allocate the color map tables */

  size  = 3 * NX_NCOLORS * sizeof(uint16);
  alloc = (ubyte*)malloc(size);
  if (alloc < 0)
    {
      return -ENOMEM;
    }
  memset(alloc, 0xff, size);

  red   = alloc;
  green = &alloc[NX_NCOLORS];
  blue  = &alloc[2*NX_NCOLORS];

  /* Initialize the color map tables. 6*6*6 = 216, the rest
   * are (0xffff, 0xffff, 0xffff)
   */

  ndx = 0;
  for (i = 0; i < 6; i++)
    {
      rval = (i * (NX_NCOLORS-1) / 5) << 8;
      for (j = 0; j < 6; j++)
        {
          gval = (j * (NX_NCOLORS-1) / 5) << 8;
          for (k = 0; k < 6; k++)
            {
              red[ndx]   = rval;
              green[ndx] = gval;
              blue[ndx]  = k * (NX_NCOLORS-1) / 5;
              ndx++;
            }
        }
    }

  /* Now configure the cmap structure */

  cmap.len    = NX_NCOLORS;
  cmap.red    = red;
  cmap.green  = green;
  cmap.blue   = blue;
#ifdef CONFIG_FB_TRANSPARENCY
  cmap.transp = NULL;
#endif

  /* Then set the color map */

  ret =fb->putcmap(fb, &cmap);

  free(cmap);
  return ret;
}
#endif
