/****************************************************************************
 * nuttx/graphics/nxterm/nxterm_font.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

#include "nxterm.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxterm_fontsize
 ****************************************************************************/

static int nxterm_fontsize(FAR struct nxterm_state_s *priv, uint8_t ch,
                           FAR struct nxgl_size_s *size)
{
  FAR const struct nx_fontbitmap_s *fbm;
  NXHANDLE hfont;

  /* Get the handle of the font managed by the font cache */

  hfont = nxf_cache_getfonthandle(priv->fcache);
  DEBUGASSERT(hfont != NULL);

  /* Does the character code map to a font? */

  fbm = nxf_getbitmap(hfont, ch);
  if (fbm)
    {
      /* Yes.. return the font size */

      size->w = fbm->metric.width + fbm->metric.xoffset;
      size->h = fbm->metric.height + fbm->metric.yoffset;
      return OK;
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: nxterm_fillspace
 ****************************************************************************/

static void nxterm_fillspace(FAR struct nxterm_state_s *priv,
                             FAR const struct nxgl_rect_s *rect,
                             FAR const struct nxterm_bitmap_s *bm)
{
#if 0 /* Not necessary now, but perhaps in the future with VT100 support. */
  struct nxgl_rect_s bounds;
  struct nxgl_rect_s intersection;
  int ret;

  /* Construct a bounding box for the glyph */

  bounds.pt1.x = bm->pos.x;
  bounds.pt1.y = bm->pos.y;
  bounds.pt2.x = bm->pos.x + priv->spwidth - 1;
  bounds.pt2.y = bm->pos.y + priv->fheight - 1;

# /* Should this also be clipped to a region in the window? */

  if (rect != NULL)
    {
      /* Get the intersection of the redraw region and the character bitmap */

      nxgl_rectintersect(&intersection, rect, &bounds);
    }
  else
    {
      /* The intersection is the whole glyph */

      nxgl_rectcopy(&intersection, &bounds);
    }

  /* Check for empty intersections */

  if (!nxgl_nullrect(&intersection))
    {
      /* Fill the bitmap region with the background color, erasing the
       * character from the display.  NOTE:  This region might actually
       * be obscured... NX will handle that case.
       */

      ret = priv->ops->fill(priv, &intersection, priv->wndo.wcolor);
      if (ret < 0)
        {
          gerr("ERROR: fill() method failed: %d\n", ret);
        }
    }
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxterm_addchar
 *
 * Description:
 *   This is part of the nxterm_putc logic.  It creates and positions a
 *   the character and renders (or re-uses) a glyph for font.
 *
 ****************************************************************************/

FAR const struct nxterm_bitmap_s *
  nxterm_addchar(FAR struct nxterm_state_s *priv, uint8_t ch)
{
  FAR struct nxterm_bitmap_s *bm = NULL;
  FAR const struct nxfonts_glyph_s *glyph;

  /* Is there space for another character on the display? */

  if (priv->nchars < priv->maxchars)
    {
      /* Yes, setup the bitmap information */

      bm        = &priv->bm[priv->nchars];
      bm->code  = ch;
      bm->flags = 0;
      bm->pos.x = priv->fpos.x;
      bm->pos.y = priv->fpos.y;

      /* Find (or create) the matching glyph */

      glyph = nxf_cache_getglyph(priv->fcache, ch);
      if (!glyph)
        {
          /* No, there is no font for this code.  Just mark this as a space. */

          bm->flags |= BMFLAGS_NOGLYPH;

          /* Set up the next character position */

          priv->fpos.x += priv->spwidth;
        }
      else
        {
          /* Set up the next character position */

          priv->fpos.x += glyph->width;
        }

      /* Success.. increment nchars to retain this character */

      priv->nchars++;
    }

  return bm;
}

/****************************************************************************
 * Name: nxterm_hidechar
 *
 * Description:
 *   Erase a character from the window.
 *
 ****************************************************************************/

int nxterm_hidechar(FAR struct nxterm_state_s *priv,
                    FAR const struct nxterm_bitmap_s *bm)
{
  struct nxgl_rect_s bounds;
  struct nxgl_size_s fsize;
  int ret;

  /* Get the size of the font glyph.  If nxterm_fontsize, then the
   * character will have been rendered as a space, and no display
   * modification is required (not an error).
   */

  ret = nxterm_fontsize(priv, bm->code, &fsize);
  if (ret < 0)
    {
      /* It was rendered as a space. */

      return OK;
    }

  /* Construct a bounding box for the glyph */

  bounds.pt1.x = bm->pos.x;
  bounds.pt1.y = bm->pos.y;
  bounds.pt2.x = bm->pos.x + fsize.w - 1;
  bounds.pt2.y = bm->pos.y + fsize.h - 1;

  /* Fill the bitmap region with the background color, erasing the
   * character from the display.  NOTE:  This region might actually
   * be obscured... NX will handle that case.
   */

  return priv->ops->fill(priv, &bounds, priv->wndo.wcolor);
}

/****************************************************************************
 * Name: nxterm_backspace
 *
 * Description:
 *   Remove the last character from the window.
 *
 ****************************************************************************/

int nxterm_backspace(FAR struct nxterm_state_s *priv)
{
  FAR struct nxterm_bitmap_s *bm;
  int ndx;
  int ret = -ENOENT;

  /* Is there a character on the display? */

  if (priv->nchars > 0)
    {
      /* Yes.. Get the index to the last bitmap on the display */

      ndx = priv->nchars - 1;
      bm  = &priv->bm[ndx];

      /* Erase the character from the display */

      ret = nxterm_hidechar(priv, bm);

      /* The current position to the location where the last character was */

      priv->fpos.x = bm->pos.x;
      priv->fpos.y = bm->pos.y;

      /* Decrement nchars to discard this character */

      priv->nchars = ndx;
    }

  return ret;
}

/****************************************************************************
 * Name: nxterm_home
 *
 * Description:
 *   Set the next character position to the top-left corner of the display.
 *
 ****************************************************************************/

void nxterm_home(FAR struct nxterm_state_s *priv)
{
  /* The first character is one space from the left */

  priv->fpos.x = priv->spwidth;

  /* And CONFIG_NXTERM_LINESEPARATION lines from the top */

  priv->fpos.y = CONFIG_NXTERM_LINESEPARATION;
}

/****************************************************************************
 * Name: nxterm_newline
 *
 * Description:
 *   Set the next character position to the beginning of the next line.
 *
 ****************************************************************************/

void nxterm_newline(FAR struct nxterm_state_s *priv)
{
  /* Carriage return: The first character is one space from the left */

  priv->fpos.x = priv->spwidth;

  /* Linefeed: Down the max font height + CONFIG_NXTERM_LINESEPARATION */

  priv->fpos.y += (priv->fheight + CONFIG_NXTERM_LINESEPARATION);
}

/****************************************************************************
 * Name: nxterm_fillchar
 *
 * Description:
 *   This implements the character display.  It is part of the nxterm_putc
 *   operation but may also be used when redrawing an existing display.
 *
 ****************************************************************************/

void nxterm_fillchar(FAR struct nxterm_state_s *priv,
                     FAR const struct nxgl_rect_s *rect,
                     FAR const struct nxterm_bitmap_s *bm)
{
  FAR const struct nxfonts_glyph_s *glyph;
  struct nxgl_rect_s bounds;
  struct nxgl_rect_s intersection;
  struct nxgl_size_s fsize;
  int ret;

  /* Handle the special case of spaces which have no glyph bitmap */

  if (BM_ISSPACE(bm))
    {
      nxterm_fillspace(priv, rect, bm);
      return;
    }

  /* Get the size of the font glyph (which may not have been created yet) */

  ret = nxterm_fontsize(priv, bm->code, &fsize);
  if (ret < 0)
    {
      /* This would mean that there is no bitmap for the character code and
       * that the font would be rendered as a space.  But this case should
       * never happen here because the BM_ISSPACE() should have already
       * found all such cases.
       */

      return;
    }

  /* Construct a bounding box for the glyph */

  bounds.pt1.x = bm->pos.x;
  bounds.pt1.y = bm->pos.y;
  bounds.pt2.x = bm->pos.x + fsize.w - 1;
  bounds.pt2.y = bm->pos.y + fsize.h - 1;

  /* Should this also be clipped to a region in the window? */

  if (rect != NULL)
    {
      /* Get the intersection of the redraw region and the character bitmap */

      nxgl_rectintersect(&intersection, rect, &bounds);
    }
  else
    {
      /* The intersection is the whole glyph */

      nxgl_rectcopy(&intersection, &bounds);
    }

  /* Check for empty intersections */

  if (!nxgl_nullrect(&intersection))
    {
      FAR const void *src;

      /* Find (or create) the glyph that goes with this font */

      glyph = nxf_cache_getglyph(priv->fcache, bm->code);
      if (!glyph)
        {
          /* Shouldn't happen */

          return;
        }

      /* Blit the font bitmap into the window */

      src = (FAR const void *)glyph->bitmap;
      ret = priv->ops->bitmap(priv, &intersection, &src,
                              &bm->pos, (unsigned int)glyph->stride);
      DEBUGASSERT(ret >= 0);
    }
}
