/****************************************************************************
 * graphics/vnc/vnc_color.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
#include <assert.h>
#include <errno.h>

#include "vnc_server.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_convert_rgbNN
 *
 * Description:
 *  Convert the native framebuffer color format (either RGB8 3:3:2,
 *  RGB16 5:6:5, or RGB32 8:8:8) to the remote framebuffer color format
 *  (either RGB8 2:2:2, RGB8 3:3:2, RGB16 5:5:5, RGB16 5:6:5, or RGB32
 *  8:8:8)
 *
 * Input Parameters:
 *   pixel - The src color in local framebuffer format.
 *
 * Returned Value:
 *   The pixel in the remote framebuffer color format.
 *
 ****************************************************************************/

#if defined(CONFIG_VNCSERVER_COLORFMT_RGB8)

uint8_t vnc_convert_rgb8_222(lfb_color_t rgb)
{
  /* 76543210
   * --------
   * RRRGGGBB
   * ..RRGGBB
   */

  return (uint8_t)(((rgb >> 2) & 0x30)  |
                   ((rgb >> 1) & 0x0c)  |
                   ( rgb       & 0x03));
}

uint8_t vnc_convert_rgb8_332(lfb_color_t rgb)
{
  /* Identity mapping */

  return (uint8_t)rgb;
}

uint16_t vnc_convert_rgb16_555(lfb_color_t rgb)
{
  /* 111111
   * 54321098 76543210
   * -----------------
   *          RRRGGGBB
   * .RRR..GG G..BB...
   */

  return (uint8_t)((((uint16_t)rgb << 8) & 0x7000)  |
                   (((uint16_t)rgb << 5) & 0x0380)  |
                   (((uint16_t)rgb << 3) & 0x0018));
}

uint16_t vnc_convert_rgb16_565(lfb_color_t rgb)
{
  /* 111111
   * 54321098 76543210
   * -----------------
   *          RRRGGGBB
   * RRR..GGG ...BB...
   */

  return (uint8_t)((((uint16_t)rgb << 8) & 0xe000)  |
                   (((uint16_t)rgb << 6) & 0x0700)  |
                   (((uint16_t)rgb << 3) & 0x0018));
}

uint32_t vnc_convert_rgb32_888(lfb_color_t rgb)
{
  /* 33222222 22221111 111111
   * 10987654 32109876 54321098 76543210
   * ----------------------------------
   *                            RRRGGGBB
   *          RRR..... GGG..... BB......
   */

  return (((uint32_t)rgb << 16) & 0x00e00000) |
         (((uint32_t)rgb << 11) & 0x0000e000) |
         (((uint32_t)rgb << 6)  & 0x000000c0);
}

#elif defined(CONFIG_VNCSERVER_COLORFMT_RGB16)

uint8_t vnc_convert_rgb8_222(lfb_color_t rgb)
{
  /* 111111
   * 54321098 76543210
   * -----------------
   * RRRRRGGG GGGBBBBB
   *          ..RRGGBB
   */

  return (uint8_t)(((rgb >> 10) & 0x0030)  |
                   ((rgb >> 7)  & 0x000c)  |
                   ((rgb >> 3)  & 0x0003));
}

uint8_t vnc_convert_rgb8_332(lfb_color_t rgb)
{
  /* 111111
   * 54321098 76543210
   * -----------------
   * RRRRRGGG GGGBBBBB
   *          RRRGGGBB
   */

  return (uint8_t)(((rgb >> 8) & 0x00e0)  |
                   ((rgb >> 6) & 0x001c)  |
                   ((rgb >> 3) & 0x0003));
}

uint16_t vnc_convert_rgb16_555(lfb_color_t rgb)
{
  /* 111111
   * 54321098 76543210
   * -----------------
   * RRRRRGGG GGGBBBBB
   * .RRRRRGG GGGBBBBB
   */

  return (((rgb >> 1) & ~0x001f) | (rgb & 0x001f));
}

uint16_t vnc_convert_rgb16_565(lfb_color_t rgb)
{
  /* Identity mapping */

  return (uint32_t)rgb;
}

uint32_t vnc_convert_rgb32_888(lfb_color_t rgb)
{
  /* 33222222 22221111 111111
   * 10987654 32109876 54321098 76543210
   * ----------------------------------
   *                   RRRRRGGG GGGBBBBB
   *          RRRRR... GGGGGG.. BBBBB...
   */

  return (((uint32_t)rgb << 8) & 0x00f80000) |
         (((uint32_t)rgb << 6) & 0x0000fc00) |
         (((uint32_t)rgb << 3) & 0x000000f8);
}

#elif defined(CONFIG_VNCSERVER_COLORFMT_RGB32)

uint8_t vnc_convert_rgb8_222(lfb_color_t rgb)
{
  /* 33222222 22221111 111111
   * 10987654 32109876 54321098 76543210
   * -----------------------------------
   *          RRRRRRRR GGGGGGGG BBBBBBBB
   *                            ..RRGGBB
   */

  return (uint8_t)(((rgb >> 18) & 0x00000030)  |
                   ((rgb >> 12) & 0x0000000c)  |
                    (rgb >> 6)  & 0x00000003));
}

uint8_t vnc_convert_rgb8_332(lfb_color_t rgb)
{
  /* 33222222 22221111 111111
   * 10987654 32109876 54321098 76543210
   * -----------------------------------
   *          RRRRRRRR GGGGGGGG BBBBBBBB
   *                            RRRGGGBB
   */

  return (uint8_t)(((rgb >> 16) & 0x00000070)  |
                   ((rgb >> 11) & 0x0000001c)  |
                    (rgb >> 6)  & 0x00000003));
}

uint16_t vnc_convert_rgb16_555(lfb_color_t rgb)
{
  /* 33222222 22221111 111111
   * 10987654 32109876 54321098 76543210
   * -----------------------------------
   *          RRRRRRRR GGGGGGGG BBBBBBBB
   *                   .RRRRRGG GGGBBBBB
   */

  return (uint16_t)(((rgb >> 9) & 0x00007c00)  |
                    ((rgb >> 6) & 0x000003e0)  |
                    ((rgb >> 3) & 0x0000001f));
}

uint16_t vnc_convert_rgb16_565(lfb_color_t rgb)
{
  /* 33222222 22221111 111111
   * 10987654 32109876 54321098 76543210
   * -----------------------------------
   *          RRRRRRRR GGGGGGGG BBBBBBBB
   *                   RRRRRGGG GGGBBBBB
   */

  return (uint16_t)(((rgb >> 8) & 0x0000f800)  |
                    ((rgb >> 5) & 0x000007e0)  |
                    ((rgb >> 3) & 0x0000001f));
}

uint32_t vnc_convert_rgb32_888(lfb_color_t rgb)
{
  /* Identity mapping */

  return rgb;
}
#else
#  error Unspecified/unsupported color format
#endif

/****************************************************************************
 * Name: vnc_colors
 *
 * Description:
 *   Test the update rectangle to see if it contains complex colors.  If it
 *   contains only a few colors, then it may be a candidate for some type
 *   run-length encoding.
 *
 *   REVISIT:  This function is imperfect:  It will fail if there are more
 *   than 8 colors in the region.  For small colors, we can keep a local
 *   array for all color formats and always return the exact result, no
 *   matter now many colors.
 *
 * Input Parameters:
 *   session   - An instance of the session structure.
 *   rect      - The update region in the local frame buffer.
 *   maxcolors - The maximum number of colors that should be returned.  This
 *               currently cannot exceed eight.
 *   colors    - The top 'maxcolors' most frequency colors are returned.
 *
 * Returned Value:
 *   The number of valid colors in the colors[] array are returned, the
 *   first entry being the most frequent.  A negated errno value is returned
 *   if the colors cannot be determined.  This would be the case if the color
 *   there are more than 'maxcolors' colors in the update rectangle.
 *
 ****************************************************************************/

int vnc_colors(FAR struct vnc_session_s *session, FAR struct nxgl_rect_s *rect,
               unsigned int maxcolors, FAR lfb_color_t *colors)
{
  FAR const lfb_color_t *rowstart;
  FAR const lfb_color_t *pixptr;
  lfb_color_t pixel;
  unsigned int counts[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  nxgl_coord_t x;
  nxgl_coord_t y;
  int ncolors = 0;
  int pixndx;
  int maxndx;
  int cmpndx;

  DEBUGASSERT(session != NULL && rect != NULL && maxcolors <= 8 && colors != NULL);

  /* Pointer to the first pixel in the first row in the local framebuffer */

  rowstart = (FAR lfb_color_t *)
    (session->fb + RFB_STRIDE * rect->pt1.y + RFB_BYTESPERPIXEL * rect->pt1.x);

  /* Loop for each row in the rectangle */

  for (y = rect->pt1.y; y <= rect->pt2.y; y++)
    {
      /* Loop for each column in the row */

      pixptr = rowstart;
      for (x = rect->pt1.x; x <= rect->pt2.x; x++)
        {
          /* Compare this pix to all of the others we have seen */

          pixel = *pixptr++;
          for (pixndx = 0; pixndx < ncolors; pixndx++)
            {
              if (colors[pixndx] == pixel)
                {
                  break;
                }
            }

          /* Have we seen this color before? */

          if (pixndx < ncolors)
            {
              /* Yes.. just increment the count of the number of times we
               * have seen it.
               */

              counts[pixndx]++;
            }

           /* Do we have space for another color? */

          else if (ncolors >= maxcolors)
            {
              /* No, then bail.  We don't have enough memory to deal with
               * large number of colors.
               */

              return -E2BIG;
            }

          /* Add the new color to the list of colors that we have found */

          else
            {
              colors[ncolors] = pixel;
              counts[ncolors] = 1;
              ncolors++;
            }
        }

      /* Set the point to the start of the next row */

      rowstart = (FAR lfb_color_t *)((uintptr_t)rowstart + RFB_STRIDE);
    }

  /* Now sort the colors by how often we saw them with the most frequent
   * color in the first position.
   */

  /* Loop for colors N={0..(ncolors-1)} */

  for (pixndx = 0; pixndx < ncolors - 1; pixndx++)
    {
      /* Compare color N with colors M={(N_1)..ncolors} */

      maxndx = pixndx;
      for (cmpndx = maxndx + 1; cmpndx < ncolors; cmpndx++)
        {
          /* Have we seen color M more often that color N? */

          if (counts[cmpndx] > counts[maxndx])
            {
              /* Yes.. then color M has been seen more frequently */

              maxndx = cmpndx;
            }
        }

      /* Do nothing if color N is the most often seen */

      if (maxndx != pixndx)
        {
          /* Otherwise swap color N and color M */
          /* Remember color N */

          lfb_color_t tmpcolor = colors[pixndx];
          int tmpcount         = counts[pixndx];

          /* Set color N to color M */

          colors[pixndx]       = colors[maxndx];
          counts[pixndx]       = counts[maxndx];

          /* Set color M to color N */

          colors[maxndx]       = tmpcolor;
          counts[maxndx]       = tmpcount;
        }
    }

  /* And return the number of colors that we found */

  return ncolors;
}
