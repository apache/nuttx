/****************************************************************************
 * graphics/nxglib/fb/nxglib_copyrectangle.c
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

#include <nuttx/video/fb.h>
#include <nuttx/nx/nxglib.h>

#include "nxglib_bitblit.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_copyrectangle_*bpp
 *
 * Description:
 *   Copy a rectangular bitmap image into the specific position in the
 *   framebuffer memory.
 *
 ****************************************************************************/

void NXGL_FUNCNAME(nxgl_copyrectangle, NXGLIB_SUFFIX)
(
  FAR struct fb_planeinfo_s *pinfo,
  FAR const struct nxgl_rect_s *dest,
  FAR const void *src,
  FAR const struct nxgl_point_s *origin,
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

  deststride = pinfo->stride;

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
  tailmask = (uint8_t)(0xff << (8 - NXGL_REMAINDERX(dest->pt2.x - 1)));
# else
  /* Get the mask for pixels that are ordered so that they pack from the
   * LS byte up.
   */

  leadmask = (uint8_t)(0xff << (8 - NXGL_REMAINDERX(dest->pt1.x)));
  tailmask = (uint8_t)(0xff >> (8 - NXGL_REMAINDERX(dest->pt1.x - 1)));
# endif
#endif

  /* Then copy the image */

  sline = (FAR const uint8_t *)src +
          NXGL_SCALEX(dest->pt1.x - origin->x) +
          (dest->pt1.y - origin->y) * srcstride;
  dline = pinfo->fbmem + dest->pt1.y * deststride +
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
          dptr[lnlen - 1] = (dptr[lnlen - 1] & ~mask) |
                            (sptr[lnlen - 1] & mask);
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
