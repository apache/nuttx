/****************************************************************************
 * graphics/nxglib/pwfb/pwfb_copyrectangle.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#include <stdint.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxbe.h>

#include "nxglib_bitblit.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwfb_copyrectangle_*bpp
 *
 * Description:
 *   Copy a rectangular bitmap image into the specific position in the
 *   framebuffer memory.
 *
 ****************************************************************************/

void NXGL_FUNCNAME(pwfb_copyrectangle, NXGLIB_SUFFIX)
(FAR struct nxbe_window_s *bwnd, FAR const struct nxgl_rect_s *dest,
 FAR const void *src, FAR const struct nxgl_point_s *origin,
 unsigned int srcstride)
{
  FAR const uint8_t *sline;
  FAR uint8_t *dline;
  unsigned int width;
  unsigned int deststride;
  unsigned int rows;

#if NXGLIB_BITSPERPIXEL < 8
  FAR const uint8_t *sptr;
  FAR uint8_t *dptr;
  uint8_t leadmask;
  uint8_t tailmask;
  uint8_t mask;
  int lnlen;
#endif

  /* Get the width of the framebuffer in bytes */

  deststride = bwnd->stride;

  /* Get the dimensions of the rectangle to fill: width in pixels,
   * height in rows
   */

  width = dest->pt2.x - dest->pt1.x + 1;
  rows  = dest->pt2.y - dest->pt1.y + 1;

#if NXGLIB_BITSPERPIXEL < 8
  /* REVISIT: Doesn't the following assume 8 pixels in a byte */

# ifdef CONFIG_NX_PACKEDMSFIRST
  /* Get the mask for pixels that are ordered so that they pack from the
   * MS byte down.
   */

  leadmask = (uint8_t)(0xff >> (8 - NXGL_REMAINDERX(dest->pt1.x)));
  tailmask = (uint8_t)(0xff << (8 - NXGL_REMAINDERX(dest->pt2.x-1)));
# else
  /* Get the mask for pixels that are ordered so that they pack from the
   * LS byte up.
   */

  leadmask = (uint8_t)(0xff << (8 - NXGL_REMAINDERX(dest->pt1.x)));
  tailmask = (uint8_t)(0xff >> (8 - NXGL_REMAINDERX(dest->pt1.x-1)));
# endif
#endif

  /* Then copy the image */

  sline = (FAR const uint8_t *)src +
          NXGL_SCALEX(dest->pt1.x - origin->x) +
          (dest->pt1.y - origin->y) * srcstride;
  dline = (FAR uint8_t *)bwnd->fbmem + dest->pt1.y * deststride +
          NXGL_SCALEX(dest->pt1.x);

  while (rows--)
    {
#if NXGLIB_BITSPERPIXEL < 8
     /* Handle masking of the fractional initial byte */

     mask  = leadmask;
     sptr  = sline;
     dptr  = dline;
     lnlen = width;

     if (lnlen > 1 && mask)
        {
          dptr[0] = (dptr[0] & ~mask) | (sptr[0] & mask);
          mask = 0xff;
          dptr++;
          sptr++;
          lnlen--;    /* REVISIT:  Is this correct? */
        }

      /* Handle masking of the fractional final byte */

      mask &= tailmask;
      if (lnlen > 0 && mask)
        {
          dptr[lnlen-1] = (dptr[lnlen-1] & ~mask) | (sptr[lnlen-1] & mask);
          lnlen--;    /* REVISIT:  Is this correct? */
        }

      /* Handle all of the unmasked bytes in-between */

      if (lnlen > 0)
        {
          NXGL_MEMCPY(dptr, sptr, lnlen);
        }
#else
      /* Copy the whole line */

      NXGL_MEMCPY((NXGL_PIXEL_T *)dline, (NXGL_PIXEL_T *)sline, width);
#endif
      dline += deststride;
      sline += srcstride;
    }
}
