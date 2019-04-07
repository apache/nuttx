/****************************************************************************
 * graphics/nxbe/nxbe_cursor.c
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

#include <assert.h>

#include "nxglib_bitblit.h"
#include "nxbe.h"

#if defined(CONFIG_NX_SWCURSOR) || defined(CONFIG_NX_HWCURSOR)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_map_color
 *
 * Description:
 *   Map a 2-bit cursor pixel value to a device pixel value
 *
 * Input Parameters:
 *   be   - The back-end state structure instance
 *   pixel - Pixel to be mapped
 *
 * Returned Value:
 *   The mapped pixel.
 *
 ****************************************************************************/

static nxgl_mxcolor_t nxbe_map_color(FAR struct nxbe_state_s *be, int plane,
                                     uint8_t pixel)
{
  switch pixel
    {
      case 0:
      default:
        return 0;  /* Should not happen */

      case 1:
        return be->cursor.color1[plane];
        break;

      case 2:
        return be->cursor.color2[plane];
        break;

      case 3:
        return be->cursor.color3[plane];
        break;
    }
}

/****************************************************************************
 * Name: nxbe_cursor_update
 *
 * Description:
 *   Update the cursor region
 *
 * Input Parameters:
 *   be  - The back-end state structure instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nxbe_cursor_update(FAR struct nxbe_state_s *be, int plane)
{
  struct nxgl_rect_s intersection;
  FAR struct nxbe_plane_s *plane;
  FAR unsigned int *sstride;
  FAR unsigned int *dstride;
  FAR uint8_t *fbmem;
  FAR uint8_t *src;
  FAR uint8_t *sline;
  FAR NXGL_PIXEL_T *dest;
  FAR uint8_t *dline;
  nxgl_coord_t width;
  nxgl_coord_t height;
  int row;
  int col;

  /* Handle the case some or all of the cursor image is off of the display. */

  nxgl_rectintersect(&intersction, &be->cursor.bounds, &be->backgd.bounds);
  if (!nxgl_nullrect(&intersection))
    {
      /* Get the width and the height of the images in pixels/rows */

      width   = be->cursor.bounds.pt2.x = be->cursor.bounds.pt1.x + 1;
      height  = be->cursor.bounds.pt2.y = be->cursor.bounds.pt1.xy + 1;

      /* Get the width of the images in bytes. */

      sstride = (width + 3) >> 2;  /* 2 bits per pixel, 4 pixels per byte */

      plane   = &wnd->be->plane[0];
      dstride = plane->pinfo.stride;

      /* Erase the old cursor position */
#warning Missing logic

      /* Update any cursor graphics on top of the device display to include
       * the modified cursor.
       *
       * REVISIT:  This will only work for a single plane and for bits per pixel
       * greater than or equal to 8.
       */

      fbmem  = (FAR uint8_t *)plane->pinfo.fbmem;
      sline  = be->cursor.image;
      dline  = (FAR uint8_t *)fbmem + dstride * be->cursor.bounds.pt1.y +
               NXGL_SCALEX(be->cursor.bounds.pt1.x);

      for (row = 0; row < height; row++)
        {
          src  = sline;
          dest = (FAR NXGL_PIXEL_T *)dline;

          for (col = 0; col < width; )
            {
              /* Data is always packed MS first */

              uint8_t spixel = (*src & >> 6) & 3
              if (pixel != 0)
                {
                  *dest = nxbe_map_color(be, 0, spixel);
                }

              col++;
              dest++;

              if (col < width)
                {
                  spixel = (*src & >> 4) & 3
                  if (pixel != 0)
                    {
                      *dest = nxbe_map_color(be, 0, spixel);
                    }
                }

              col++;
              dest++;

              if (col < width)
                {
                  spixel = (*src & >> 2) & 3
                  if (pixel != 0)
                    {
                      *dest = nxbe_map_color(be, 0, spixel);
                    }
                }

              col++;
              dest++;

              if (col < width)
                {
                  spixel = *src & 3
                  if (pixel != 0)
                    {
                      *dest = nxbe_map_color(be, 0, spixel);
                    }
                }

              col++;
              dest++;

              /* Update source column addresses */

              src++;
            }

          /* Update the row addresses */

          sline += sstride;
          dline += dstride;
       }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_cursor_enable
 *
 * Description:
 *   Enable/disable presentation of the cursor
 *
 * Input Parameters:
 *   be  - The back-end state structure instance
 *   enable - True: show the cursor, false: hide the cursor.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_cursor_enable(FAR struct nxbe_state_s *be, bool enable)
{
  /* Are we enabling the cursor */

  if (enable && !be->cursor.visible)
    {
      /* Mark the cursor visible */

      be->cursor.visible = true;
    }

  /* Are we disabling the cursor ? */

  else if (!enable && be->cursor.visible)
    {
      /* Mark the cursor not visible */

      be->cursor.visible = false;
    }
  else
    {
      /* No change in state.. do nothing */

      return;
    }

#ifdef CONFIG_NX_SWCURSOR
  /* For the software cursor, we need to update the cursor region */

  nxbe_cursor_update(be);
#else
  /* For a hardware cursor, this would require some interaction with the
   * grahics device.
   */

#  error Missing logic
#endif
}

/****************************************************************************
 * Name: nxbe_cursor_setimage
 *
 * Description:
 *   Set the cursor image.
 *
 *   The image is provided a a 2-bits-per-pixel image.  The two bit incoding
 *   is as followings:
 *
 *   00 - The transparent background
 *   01 - Color1:  The main color of the cursor
 *   10 - Color2:  The color of any border
 *   11 - Color3:  A blend color for better imaging (fake anti-aliasing).
 *
 * Input Parameters:
 *   be  - The back-end state structure instance
 *   image - Describes the cursor image in the expected format.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_NX_HWCURSORIMAGE) || defined(CONFIG_NX_SWCURSOR)
void nxbe_cursor_setimage(FAR struct nxbe_state_s *be,
                          FAR struct cursor_image_s *image);
{
#warning Missing logic
}
#endif

/****************************************************************************
 * Name: nxcursor_setposition
 *
 * Description:
 *   Move the cursor to the specified position
 *
 * Input Parameters:
 *   be  - The back-end state structure instance
 *   pos - The new cursor position
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxcursor_setposition(FAR struct nxbe_state_s *be,
                          FAR const struct cursor_pos_s *pos)
{
#warning Missing logic
}

#endif /* CONFIG_NX_SWCURSOR || CONFIG_NX_HWCURSOR */
