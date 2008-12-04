/****************************************************************************
 * examples/nx/nx_kbdin.c
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
#include <stdlib.h>
#include <ctype.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/nx.h>
#include <nuttx/nxtk.h>
#include <nuttx/nxfonts.h>

#include "nx_internal.h"

#ifdef CONFIG_NX_KBD

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Select renderer -- Some additional logic would be required to support
 * pixel depths that are not directly addressable (1,2,4, and 24).
 */

#ifndef CONFIG_EXAMPLES_NX_RAWWINDOWS
#if CONFIG_EXAMPLES_NX_BPP == 8
#  define RENDERER nxf_convert_8bpp
#elif CONFIG_EXAMPLES_NX_BPP == 16
#  define RENDERER nxf_convert_16bpp
#elif  CONFIG_EXAMPLES_NX_BPP == 32
#  define RENDERER nxf_convert_32bpp
#else
#  error "Unsupported CONFIG_EXAMPLES_NX_BPP"
#endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
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
 * Name: nxeg_fillchar
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_NX_RAWWINDOWS
static void nxeg_fillchar(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                          FAR const struct nxeg_bitmap_s *bm)
{
  struct nxgl_rect_s intersection;
  int ret;

  /* Get the intersection of the redraw region and the characer bitmap */

  nxgl_rectintersect(&intersection, rect, &bm->bounds);
  if (!nxgl_nullrect(&intersection))
    {
      FAR void *src = (FAR void *)bm->glyph->bitmap;
      ret = nxtk_bitmapwindow((NXTKWINDOW)hwnd, &intersection, src,
                              &bm->bounds.pt1,
                              (unsigned int)bm->glyph->stride);
#if 0
EXTERN int nxtk_bitmapwindow(NXTKWINDOW hfwnd,
                             FAR const struct nxgl_rect_s *dest,
                             FAR const void *src[CONFIG_NX_NPLANES],
                             FAR const struct nxgl_point_s *origin,
                             unsigned int stride);
#endif
      if (ret < 0)
        {
          message("nxeg_fillchar: nxtk_bitmapwindow failed: %d\n", errno);
        }
    }
}
#endif

/****************************************************************************
 * Name: nxeg_kbdinfo
 ****************************************************************************/

static void nxeg_kbdinfo(ubyte nch, const ubyte *ch)
{
  int i;
  for (i = 0; i < nch; i++)
    {
      if (isprint(ch[i]))
        {
          message("          ch[%d]=%c (%02x)\n", i, ch[i], ch[i]);
        }
      else
        {
          message("          ch[%d]=  (%02x)\n", i, ch[i]);
        }
    }
}

/****************************************************************************
 * Name: nxeg_renderglyph
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_NX_RAWWINDOWS
static inline FAR const struct nxeg_glyph_s *
nxeg_renderglyph(FAR struct nxeg_state_s *st, ubyte ch)
{
  FAR struct nxeg_glyph_s *glyph = NULL;
  FAR nxgl_mxpixel_t *ptr;
  int bmstride;
  int bmsize;
  int row;
  int col;

  /* Make sure that there is room for another glyph */

  message("nxeg_renderglyph: ch=%02x\n", ch);
  if (st->nglyphs < NXTK_MAXKBDCHARS)
    {
      /* Allocate the glyph */

      glyph = &st->glyph[st->nglyphs];

      /* Allocate the maximum size for the bitmap */

      glyph->stride = (st->width * CONFIG_EXAMPLES_NX_BPP + 4) / 8;
      bmsize        =  glyph->stride * st->height;
      glyph->bitmap = (FAR ubyte *)malloc(bmsize);
      if (glyph->bitmap)
        {
          /* Initialize the glyph memory to the background color */

#if CONFIG_EXAMPLES_NX_BPP < 8 || CONFIG_EXAMPLES_NX_BPP == 24
# error "Additional logic is needed here"
#else
          ptr = (FAR nxgl_mxpixel_t *)glyph->bitmap;
          for (row = 0; row < st->height; row++)
            {
              for (col = 0; col < st->width; col++)
                {
                  *ptr++ = st->color[0];
                }
            }
#endif
          /* Then render the glyph into the allocated memory */

          glyph->width = RENDERER((FAR nxgl_mxpixel_t*)glyph->bitmap,
                                  st->height, st->width, bmstride,
                                  ch, CONFIG_EXAMPLES_NX_FONTCOLOR);
          if (glyph->width <= 0)
            {
              message("nxeg_renderglyph: RENDERER returned width=%d\n", glyph->width);
              free(glyph->bitmap);
              glyph->bitmap = NULL;
              glyph         = NULL;
            }
          else
            {
               /* Make it permanent */

               st->nglyphs++;
            }
        }
    }

  return glyph;
}
#endif

/****************************************************************************
 * Name: nxeg_getglyph
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_NX_RAWWINDOWS
static FAR const struct nxeg_glyph_s *
nxeg_getglyph(FAR struct nxeg_state_s *st, ubyte ch)
{
  int i;

  /* First, try to find the glyph in the cache of pre-rendered glyphs */

  message("nxeg_getglyph: ch=%02x\n", ch);
  for (i = 0; i < st->nglyphs; i++)
    {
      if (st->glyph[i].code == ch)
        {
          return &st->glyph[i];
        }
    }

   /* No, it is not cached... render it now and add it to the cache */

   return nxeg_renderglyph(st, ch);
}
#endif

/****************************************************************************
 * Name: nxeg_addchar
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_NX_RAWWINDOWS
static FAR const struct nxeg_bitmap_s *
nxeg_addchar(FAR struct nxeg_state_s *st, ubyte ch)
{
  FAR struct nxeg_bitmap_s *bm = NULL;
  FAR struct nxeg_bitmap_s *bmleft;
  FAR const struct nx_font_s *fontset;
  nxgl_coord_t leftx;

  /* Is there space for another character on the display? */

  message("nxeg_addchar: ch=%02x\n", ch);
  if (st->nchars < NXTK_MAXKBDCHARS)
    {
       /* Yes, setup the bitmap */

       bm = &st->bm[st->nchars];

       /* Find the matching glyph */

       bm->glyph = nxeg_getglyph(st, ch);
       if (!bm->glyph)
         {
           return NULL;
         }

       /* Set up the bounds for the bitmap */

       if (st->nchars <= 0)
         {
            /* The first character is one space from the left */

            fontset = nxf_getfontset();
            leftx   = fontset->spwidth;
         }
       else
         {
            /* Otherwise, it is to the left of the preceding char */

            bmleft = &st->bm[st->nchars-1];
            leftx  = bmleft->bounds.pt2.x + 1;
         }

       bm->bounds.pt1.x = leftx;
       bm->bounds.pt1.y = 2;
       bm->bounds.pt2.x = leftx + bm->glyph->width - 1;
       bm->bounds.pt2.x = 2 + st->height - 1;

       st->nchars++;
    }
  return bm;
}
#endif

/****************************************************************************
 * Name: nxeg_addchars
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_NX_RAWWINDOWS
static inline void nxeg_addchars(NXWINDOW hwnd, FAR struct nxeg_state_s *st,
                                 ubyte nch, FAR const ubyte *ch)
{
  FAR const struct nxeg_bitmap_s *bm;

  while (nch--)
    {
      bm = nxeg_addchar(st, *ch++);
      nxeg_fillchar(hwnd, &bm->bounds, bm);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxeg_kbdin
 ****************************************************************************/

void nxeg_kbdin(NXWINDOW hwnd, ubyte nch, FAR const ubyte *ch, FAR void *arg)
{
  FAR struct nxeg_state_s *st = (FAR struct nxeg_state_s *)arg;
  message("nxeg_kbdin%d: hwnd=%p nch=%d\n", st->wnum, hwnd, nch);
#ifdef CONFIG_EXAMPLES_NX_RAWWINDOWS
  nxeg_kbdinfo(nch, ch);
#else
  nxeg_addchars(hwnd, st, nch, ch);
#endif
}

/****************************************************************************
 * Name: nxeg_tbkbdin
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_NX_RAWWINDOWS
void nxeg_tbkbdin(NXWINDOW hwnd, ubyte nch, const ubyte *ch, FAR void *arg)
{
  FAR struct nxeg_state_s *st = (FAR struct nxeg_state_s *)arg;
  message("nxeg_tbkbdin: ERROR -- toolbar should not received keyboard input\n");
  message("nxeg_tbkbdin%d: hwnd=%p nch=%d\n", st->wnum, hwnd, nch);
  nxeg_kbdinfo(nch, ch);
}
#endif

/****************************************************************************
 * Name: nxeg_tbkbdin
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_NX_RAWWINDOWS
void nxeg_filltext(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                   FAR struct nxeg_state_s *st)
{
  int i;
  for (i = 0; i < st->nchars; i++)
    {
      nxeg_fillchar(hwnd, rect, &st->bm[i]);
    }
}
#endif

#endif /* CONFIG_NX_KBD */
