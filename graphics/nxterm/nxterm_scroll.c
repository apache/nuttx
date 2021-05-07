/****************************************************************************
 * graphics/nxterm/nxterm_scroll.c
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
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxfonts.h>

#include "nxterm.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxterm_movedisplay
 *
 * Description:
 *   This function implements the data movement for the scroll operation.  If
 *   we can read the displays framebuffer memory, then the job is pretty
 *   easy.  However, many displays (such as SPI-based LCDs) are often read-
 *   only.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_WRITEONLY
static inline void nxterm_movedisplay(FAR struct nxterm_state_s *priv,
                                     int bottom, int scrollheight)
{
  FAR struct nxterm_bitmap_s *bm;
  struct nxgl_rect_s rect;
  nxgl_coord_t row;
  int ret;
  int i;

  /* Move each row, one at a time.  They could all be moved at once (by
   * calling nxterm_redraw), but the since the region is cleared, then
   * re-written, the effect would not be good.  Below the region is also
   * cleared and re-written, however, in much smaller chunks.
   */

  rect.pt1.x = 0;
  rect.pt2.x = priv->wndo.wsize.w - 1;

  for (row = CONFIG_NXTERM_LINESEPARATION; row < bottom; row += scrollheight)
    {
      /* Create a bounding box the size of one row of characters */

      rect.pt1.y = row;
      rect.pt2.y = row + scrollheight - 1;

      /* Clear the region */

      ret = priv->ops->fill(priv, &rect, priv->wndo.wcolor);
      if (ret < 0)
        {
          gerr("ERROR: Fill failed: %d\n", get_errno());
        }

      /* Fill each character that might lie within in the bounding box */

      for (i = 0; i < priv->nchars; i++)
        {
          bm = &priv->bm[i];
          if (bm->pos.y <= rect.pt2.y &&
              bm->pos.y + priv->fheight >= rect.pt1.y)
            {
              nxterm_fillchar(priv, &rect, bm);
            }
        }
    }

  /* Finally, clear the vacated part of the display */

  rect.pt1.y = bottom;
  rect.pt2.y = priv->wndo.wsize.h - 1;

  ret = priv->ops->fill(priv, &rect, priv->wndo.wcolor);
  if (ret < 0)
    {
      gerr("ERROR: Fill failed: %d\n", get_errno());
    }
}
#else
static inline void nxterm_movedisplay(FAR struct nxterm_state_s *priv,
                                     int bottom, int scrollheight)
{
  struct nxgl_rect_s rect;
  struct nxgl_point_s offset;
  int ret;

  /* Add the line separation value to the scroll height */

  scrollheight += CONFIG_NXTERM_LINESEPARATION;

  /* Move the display in the range of 0-height up one scrollheight.  The
   * line at the bottom will be reset to the background color automatically.
   *
   * The source rectangle to be moved.
   */

  rect.pt1.x = 0;
  rect.pt1.y = scrollheight;
  rect.pt2.x = priv->wndo.wsize.w - 1;
  rect.pt2.y = priv->wndo.wsize.h - 1;

  /* The offset that determines how far to move the source rectangle */

  offset.x   = 0;
  offset.y   = -scrollheight;

  /* Move the source rectangle upward by the scrollheight */

  ret = priv->ops->move(priv, &rect, &offset);
  if (ret < 0)
    {
      gerr("ERROR: Move failed: %d\n", get_errno());
    }

  /* Finally, clear the vacated bottom part of the display */

  rect.pt1.y = priv->wndo.wsize.h - scrollheight;

  ret = priv->ops->fill(priv, &rect, priv->wndo.wcolor);
  if (ret < 0)
    {
      gerr("ERROR: Fill failed: %d\n", get_errno());
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxterm_scroll
 ****************************************************************************/

void nxterm_scroll(FAR struct nxterm_state_s *priv, int scrollheight)
{
  int i;
  int j;

  /* Adjust the vertical position of each character */

  for (i = 0; i < priv->nchars; )
    {
      FAR struct nxterm_bitmap_s *bm = &priv->bm[i];

      /* Has any part of this character scrolled off the screen? */

      if (bm->pos.y < scrollheight + CONFIG_NXTERM_LINESEPARATION)
        {
          /* Yes... Delete the character by moving all of the data */

          for (j = i; j < priv->nchars - 1; j++)
            {
              memcpy(&priv->bm[j], &priv->bm[j + 1],
                     sizeof(struct nxterm_bitmap_s));
            }

          /* Decrement the number of cached characters ('i' is not
           * incremented in this case because it already points to the next
           * character)
           */

          priv->nchars--;
        }

      /* No.. just decrement its vertical position (moving it "up" the
       * display by one line).
       */

      else
        {
          bm->pos.y -= scrollheight;

          /* We are keeping this one so increment to the next character */

          i++;
        }
    }

  /* And move the next display position up by one line as well */

  priv->fpos.y -= scrollheight;

  /* Move the display in the range of 0-height up one scrollheight. */

  nxterm_movedisplay(priv, priv->fpos.y, scrollheight);
}
