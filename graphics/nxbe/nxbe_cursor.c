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

#include <nuttx/kmalloc.h>

#include "nxglib.h"
#include "nxbe.h"

#if defined(CONFIG_NX_SWCURSOR) || defined(CONFIG_NX_HWCURSOR)

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
  /* Are we enabling the cursor?  Don't allow the cursor to be enabled if no
   * image has been assigned to the cursor.
   */

  ginfo("enable=%d visible=%u\n", enable, be->cursor.visible);

  if (enable && !be->cursor.visible)
    {
#ifdef CONFIG_NX_SWCURSOR
      /* Don't allow the cursor to be enabled if no image has been assigned
       * to the cursor
       */

      if (be->cursor.image != NULL)
        {
          struct nxgl_rect_s bounds;

          DEBUGASSERT(be->cursor.bkgd != NULL);

          /* Handle the case where some or all of the cursor is off the
           * display.
           */

          nxgl_rectintersect(&bounds, &be->cursor.bounds, &be->bkgd.bounds);
          if (!nxgl_nullrect(&bounds))
            {
              /* Save the cursor background image */

              be->plane[0].cursor.backup(be, &bounds, 0);

              /* Write the new cursor image to device memory */

              be->plane[0].cursor.draw(be, &bounds, 0);
            }

          /* Mark the cursor visible */

          be->cursor.visible = true;
        }
#else
      /* Mark the cursor visible */

      be->cursor.visible = true;

      /* For a hardware cursor, this would require some interaction with the
       * graphics device.
       */

#  error Missing logic
#endif
    }

  /* Are we disabling the cursor ? */

  else if (!enable && be->cursor.visible)
    {
      struct nxgl_rect_s bounds;

      /* Mark the cursor not visible */

      be->cursor.visible = false;

#ifdef CONFIG_NX_SWCURSOR
      DEBUGASSERT(be->cursor.bkgd != NULL);

      /* Handle the case where some or all of the cursor is off the display. */

      nxgl_rectintersect(&bounds, &be->cursor.bounds, &be->bkgd.bounds);
      if (!nxgl_nullrect(&bounds))
        {
          /* Erase the old cursor image by writing the saved background
           * image.
           */

          be->plane[0].cursor.erase(be, &bounds, 0);
        }
#else
      /* For a hardware cursor, this would require some interaction with the
       * graphics device.
       */

#  error Missing logic
#endif
    }
}

/****************************************************************************
 * Name: nxbe_cursor_setimage
 *
 * Description:
 *   Set the cursor image.
 *
 *   The image is provided a a 2-bits-per-pixel image.  The two bit encoding
 *   is as follows:
 *
 *   00 - The transparent background
 *   01 - Color1:  The main color of the cursor
 *   10 - Color2:  The color of any border
 *   11 - Color3:  A blend color for better imaging (fake anti-aliasing).
 *
 * Input Parameters:
 *   be    - The back-end state structure instance
 *   image - Describes the cursor image in the expected format.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_NX_HWCURSORIMAGE) || defined(CONFIG_NX_SWCURSOR)
void nxbe_cursor_setimage(FAR struct nxbe_state_s *be,
                          FAR struct nx_cursorimage_s *image)
{
#ifdef CONFIG_NX_SWCURSOR
  struct nxgl_size_s oldsize;
  struct nxgl_rect_s bounds;
  size_t allocsize;
  unsigned int bpp;

  ginfo("image=%p\n", image);

  /* If the cursor is visible, then we need to erase the old cursor from the
   * device graphics memory.
   */

  if (be->cursor.visible)
    {
      /* Handle the case where some or all of the cursor is off the display. */

      nxgl_rectintersect(&bounds, &be->cursor.bounds, &be->bkgd.bounds);
      if (!nxgl_nullrect(&bounds))
        {
          /* Erase the old cursor image by writing the saved background
           * image.
           */

          DEBUGASSERT(be->cursor.bkgd != NULL);
          be->plane[0].cursor.erase(be, &bounds, 0);
        }
    }

  /* Has the cursor changed size? */

  oldsize.w = be->cursor.bounds.pt2.x - be->cursor.bounds.pt2.y + 1;
  oldsize.h = be->cursor.bounds.pt2.y - be->cursor.bounds.pt2.y + 1;

  if (image->size.w != oldsize.w || image->size.h != oldsize.h)
    {
      /* Check the size of the allocation we need to hold the backup image. */

      bpp       = be->plane[0].pinfo.bpp;
      allocsize = (image->size.w * image->size.h * bpp + 7) >> 3;

      /* Reallocate the buffer only if a larger one is needed */

      if (allocsize > be->cursor.allocsize)
        {
          FAR void *tmp = kmm_realloc(be->cursor.bkgd, allocsize);
          if (tmp == NULL)
            {
              goto errout_with_erase;
            }

          /* Save the new allocation information */

          be->cursor.allocsize = allocsize;
          be->cursor.bkgd      = (FAR nxgl_mxpixel_t *)tmp;
        }

      /* Calculate the new image bounds.  The position (pt1), does not
       * change.
       */

      be->cursor.bounds.pt2.x = be->cursor.bounds.pt1.x + image->size.w - 1;
      be->cursor.bounds.pt2.y = be->cursor.bounds.pt1.y + image->size.h - 1;

      /* Read in the new background image */

      be->plane[0].cursor.backup(be, &be->cursor.bounds, 0);
    }

  /* Save the new colors */

  nxgl_colorcopy(be->cursor.color1, image->color1);
  nxgl_colorcopy(be->cursor.color2, image->color2);
  nxgl_colorcopy(be->cursor.color3, image->color3);

  /* Save the new image.  This is a reference to an image in user space.
   * which we assume will persist while we use it.
   *
   * REVISIT:  There is an issue in KERNEL build mode. For FLAT and
   * PROTECTED builds, the cursor image resides in the common application
   * space and is assumed to pesist as long as needed.  But with the KERNEL
   * build, the image will lie in a process space and will not be generally
   * available.  In that case, we could keep the image in a shared memory
   * region or perhaps copy the image into a kernel internal buffer.
   * Neither of those are implemented.
   */

  be->cursor.image = image->image;

errout_with_erase:
  /* If the cursor is visible, then put write the new cursor image into
   * device graphics memory now.
   */

  if (be->cursor.visible)
    {
      /* Write the new cursor image to the device graphics memory. */

      be->plane[0].cursor.draw(be, &be->cursor.bounds, 0);
    }

#else
  /* For a hardware cursor, this would require some interaction with the
   * graphics device.
   */

#  error Missing logic
#endif
}
#endif

/****************************************************************************
 * Name: nxbe_cursor_setposition
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

void nxbe_cursor_setposition(FAR struct nxbe_state_s *be,
                             FAR const struct nxgl_point_s *pos)
{
#ifdef CONFIG_NX_SWCURSOR
  nxgl_coord_t dx;
  nxgl_coord_t dy;

  ginfo("pos=(%d,%d)\n", pos->x, pos->y);

  /* If the cursor is visible, then we need to erase the cursor from the
   * old position in device graphics memory.
   */

  if (be->cursor.visible)
    {
      /* Erase the old cursor image by writing the saved background image. */

      be->plane[0].cursor.erase(be, &be->cursor.bounds, 0);
    }

  /* Calculate the cursor movement */

  dx = pos->x - be->cursor.bounds.pt1.x;
  dy = pos->y - be->cursor.bounds.pt1.y;

  /* Calculate the new image bounds. */

  nxgl_rectoffset(&be->cursor.bounds, &be->cursor.bounds, dx, dy);

  /* If the cursor is visible, then put write the new cursor image into
   * device graphics memory now.
   */

  if (be->cursor.visible)
    {
      struct nxgl_rect_s bounds;

      /* Handle the case where some or all of the cursor is off the display. */

      nxgl_rectintersect(&bounds, &be->cursor.bounds, &be->bkgd.bounds);
      if (!nxgl_nullrect(&bounds))
        {
          /* Read in the new background image at this offset */

          be->plane[0].cursor.backup(be, &bounds, 0);

          /* Write the new cursor image to the device graphics memory. */

          be->plane[0].cursor.draw(be, &bounds, 0);
        }
    }

#else
  /* For a hardware cursor, this would require some interaction with the
   * graphics device.
   */

#  error Missing logic
#endif
}

#endif /* CONFIG_NX_SWCURSOR || CONFIG_NX_HWCURSOR */
