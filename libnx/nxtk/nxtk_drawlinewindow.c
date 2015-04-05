/****************************************************************************
 * libnx/nxtk/nxtk_drawlinewindow.c
 *
 *   Copyright (C) 2011, 2015 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxtk.h>

/****************************************************************************
 * Pre-processor Definitions
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
 * Name: nxtk_drawlinewindow
 *
 * Description:
 *  Fill the specified line in the window with the specified color.  This
 *  is simply a wrapper that uses nxgl_splitline() to break the line into
 *  trapezoids and call calls nxtk_filltrapwindow() to render the line.
 *
 * Input Parameters:
 *   hfwnd - The window handle returned by nxtk_openwindow
 *   vector - Describes the line to be drawn
 *   width  - The width of the line
 *   color  - The color to use to fill the line
 *   caps   - Draw a circular cap on the ends of the line to support better
 *            line joins
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_drawlinewindow(NXTKWINDOW hfwnd, FAR struct nxgl_vector_s *vector,
                        nxgl_coord_t width, nxgl_mxpixel_t color[CONFIG_NX_NPLANES],
                        bool caps)

{
  struct nxgl_trapezoid_s trap[3];
  struct nxgl_rect_s rect;
  int ret;

#ifdef CONFIG_DEBUG
  if (!hfwnd || !vector || width < 1 || !color)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Split the line into trapezoids */

  ret = nxgl_splitline(vector, trap, &rect, width);
  switch (ret)
    {
      /* 0: Line successfully broken up into three trapezoids.  Values in
       *    traps[0], traps[1], and traps[2] are valid.
       */

      case 0:
        ret = nxtk_filltrapwindow(hfwnd, &trap[0], color);
        if (ret == OK)
          {
            ret = nxtk_filltrapwindow(hfwnd, &trap[1], color);
            if (ret == OK)
              {
                ret = nxtk_filltrapwindow(hfwnd, &trap[2], color);
              }
          }
        break;

      /* 1: Line successfully represented by one trapezoid. Value in traps[1]
       *    is valid.
       */

      case 1:
        ret = nxtk_filltrapwindow(hfwnd, &trap[1], color);
         break;

      /* 2: Line successfully represented by one rectangle. Value in rect is
       *    valid
       */

       case 2:
         ret = nxtk_fillwindow(hfwnd, &rect, color);
         break;

      /* <0: On errors, a negated errno value is returned. */

      default:
         set_errno(EINVAL);
         return ERROR;
    }

  /* Draw circular caps at each end of the line to support better line joins */

  if (caps != NX_LINECAP_NONE && width >= 3)
    {
      nxgl_coord_t radius = width >> 1;

      /* Draw a circle at pt1 */

      ret = OK;
      if ((caps & NX_LINECAP_PT1) != 0)
        {
          ret = nxtk_fillcirclewindow(hfwnd, &vector->pt1, radius, color);
        }

      /* Draw a circle at pt2 */

      if (ret == OK && (caps & NX_LINECAP_PT2) != 0)
        {
          ret = nxtk_fillcirclewindow(hfwnd, &vector->pt2, radius, color);
        }
    }

  return ret;
}
