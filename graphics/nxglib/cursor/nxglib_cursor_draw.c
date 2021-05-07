/****************************************************************************
 * graphics/nxglib/cursor/nxglib_cursor_draw.c
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

#include <assert.h>

#include <nuttx/nx/nxglib.h>

#include "nxglib_bitblit.h"
#include "../nxbe/nxbe.h"
#include "nxglib.h"

#ifdef CONFIG_NX_SWCURSOR

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_map_color
 *
 * Description:
 *   Map a 2-bit cursor pixel value to a device pixel value
 *
 *   REVISIT:  If we really were to support multiple displays (or planes)
 *   with differing pixel depth, then the cursor.color1,2,3 would be
 *   insufficient:  There would have to be multiple color sets for each
 *   plane.
 *
 * Input Parameters:
 *   be   - The back-end state structure instance
 *   pixel - Pixel to be mapped
 *
 * Returned Value:
 *   The mapped pixel.
 *
 ****************************************************************************/

static NXGL_PIXEL_T nxbe_map_color(FAR struct nxbe_state_s *be, int plane,
                                   uint8_t pixel)
{
  switch (pixel)
    {
      case 0:
      default:
        return 0;  /* Should not happen */

      case 1:
        return be->cursor.color1[plane];

      case 2:
        return be->cursor.color2[plane];

      case 3:
        return be->cursor.color3[plane];
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxglib_cursor_draw
 *
 * Description:
 *   Update the cursor region by drawing directly in device memory.
 *
 * Input Parameters:
 *   be      - The back-end state structure instance
 *   bounds  - The region of the display that has been modified.
 *   planeno - The color plane being drawn
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void NXGL_FUNCNAME(nxglib_cursor_draw, NXGLIB_SUFFIX)
(
  FAR struct nxbe_state_s *be,
  FAR const struct nxgl_rect_s *bounds,
  int planeno)
{
  struct nxgl_rect_s intersection;
  struct nxgl_point_s origin;
  FAR struct nxbe_plane_s *plane;
  FAR uint8_t *fbmem;
  FAR const uint8_t *src;
  FAR const uint8_t *sline;
  FAR uint8_t *dline;
  FAR NXGL_PIXEL_T *dest;
  nxgl_coord_t width;
  nxgl_coord_t height;
  nxgl_coord_t swidth;
  nxgl_coord_t sstride;
  nxgl_coord_t dstride;
  nxgl_coord_t sshift;
  int shift;
  int row;
  int col;

  /* Handle the case some or all of the cursor image is off of the display. */

  nxgl_rectintersect(&intersection, &be->cursor.bounds, &be->bkgd.bounds);

  /* Check if there is anything in the modified region that we
   * need to handle.
   */

  nxgl_rectintersect(&intersection, &intersection, bounds);
  if (!nxgl_nullrect(&intersection))
    {
      /* Get the width and the height of the images to copy in pixels/rows */

      width   = intersection.pt2.x - intersection.pt1.x + 1;
      height  = intersection.pt2.y - intersection.pt1.y + 1;

      /* Get the width of the images in bytes. */

      swidth  = be->cursor.bounds.pt2.x - be->cursor.bounds.pt1.x + 1;
      sstride = (swidth + 3) >> 2;  /* 2 bits per pixel, 4 pixels per byte */

      plane   = &be->plane[planeno];
      dstride = plane->pinfo.stride;

      /* Get the origin position in the cursor image */

      nxgl_vectsubtract(&origin, &intersection.pt1, &be->cursor.bounds.pt1);

      /* Update any cursor graphics on top of the device display to include
       * the modified cursor.
       *
       * REVISIT:  This will only work for a single plane and for bits per
       * pixel greater than or equal to 8.
       */

      fbmem  = (FAR uint8_t *)plane->pinfo.fbmem;
      sline  = be->cursor.image + sstride * origin.y + (origin.x >> 2);
      dline  = (FAR uint8_t *)fbmem + dstride * intersection.pt1.y +
                NXGL_SCALEX(intersection.pt1.x);

      sshift = (3 - (origin.y & 3)) << 1;    /* MS first {0, 2, 4, 6} */

      /* Loop for each row */

      for (row = 0; row < height; row++)
        {
          /* Reset to the beginning of the line */

          src   = sline;
          dest  = (FAR NXGL_PIXEL_T *)dline;
          shift = sshift;

          /* Loop for each column */

          for (col = 0; col < width; col++)
            {
              /* Extract the 2-bit pixel.  Data is always packed MS first.
               * Shift for first pixel=6, shift for last pixel=0
               */

              uint8_t pixel = (*src >> shift) & 3;

              /* Skip over invisible pixels */

              if (pixel != 0)
                {
                  *dest = nxbe_map_color(be, 0, pixel);
                }

              dest++;

              /* Was that the last pixel in the byte? */

              if (shift == 0)
                {
                  /* Update source column addresses and reset the shift
                   * value
                   */

                  src++;
                  shift = 6;
                }
              else
                {
                  /* The shift value is one of {2, 4, 6}.  Update the shift
                   * value following this order: 6, 4, 2, 0
                   */

                  shift = (shift - 2) & 6;
                }
            }

          /* Update the row addresses to the next row */

          sline += sstride;
          dline += dstride;
        }
    }
}

#endif /* CONFIG_NX_SWCURSOR */
