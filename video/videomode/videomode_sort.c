/****************************************************************************
 * video/videomode/videomode_sort.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from logic in FreeBSD which has an compatible 2-clause BSD
 * license:
 *
 *   Copyright (c) 2006 The NetBSD Foundation.  All rights reserved.
 *   Author:  Michael Lorenz
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE NETBSD FOUNDATION BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <string.h>

#include <nuttx/video/videomode.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DIVIDE(x, y)  (((x) + ((y) / 2)) / (y))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void videomode_swap(FAR struct videomode_s *left,
                                  FAR struct videomode_s *right)
{
  struct videomode_s temp;

  temp   = *left;
  *left  = *right;
  *right = temp;
}

static inline int _abs(int a)
{
  if (a < 0)
    {
      return -a;
    }
  else
    {
      return a;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sort_videomodes
 *
 * Description:
 *   Sort video modes by refresh rate, aspect ratio, then resolution.
 *   Preferred mode or largest mode is first in the list and other modes
 *   are sorted on closest match to that mode.
 *
 *   Note that the aspect ratio calculation treats "close" aspect ratios
 *   (within 12.5%) as the same for this purpose.
 *
 * Input Parameters:
 *   modes     - A reference to the first entry in a list of video modes
 *   preferred - A pointer to the pointer to the preferred mode in the list
 *   nmodes    - The number of modes in the list
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sort_videomodes(FAR struct videomode_s *modes,
                     FAR struct videomode_s **preferred,
                     unsigned int nmodes)
{
  FAR struct videomode_s *tmpmode = NULL;
  int aspect;
  int refresh;
  int hbest;
  int vbest;
  int abest;
  int atemp;
  int rbest;
  int rtemp;
  int i;
  int j;

  if (nmodes < 2)
    {
      return;
    }

  if (*preferred != NULL)
    {
      /* Put the preferred mode first in the list */

      aspect = (*preferred)->hdisplay * 100 / (*preferred)->vdisplay;
      refresh = DIVIDE(DIVIDE((*preferred)->dotclock * 1000,
                              (*preferred)->htotal), (*preferred)->vtotal);
      if (*preferred != modes)
        {
          videomode_swap(*preferred, modes);
          *preferred = modes;
        }
    }
  else
    {
      /* Find the largest horizontal and vertical mode and put that
       * first in the list.  Preferred refresh rate is taken from
       * the first mode of this size.
       */

      hbest = 0;
      vbest = 0;

      for (i = 0; i < nmodes; i++)
        {
          if (modes[i].hdisplay > hbest)
            {
              hbest   = modes[i].hdisplay;
              vbest   = modes[i].vdisplay;
              tmpmode = &modes[i];
            }
          else if (modes[i].hdisplay == hbest && modes[i].vdisplay > vbest)
            {
              vbest   = modes[i].vdisplay;
              tmpmode = &modes[i];
            }
        }

      aspect  = tmpmode->hdisplay * 100 / tmpmode->vdisplay;
      refresh = DIVIDE(DIVIDE(tmpmode->dotclock * 1000,
                              tmpmode->htotal), tmpmode->vtotal);
      if (tmpmode != modes)
        {
          videomode_swap(tmpmode, modes);
        }
    }

  /* Sort other modes by refresh rate, aspect ratio, then resolution */

  for (j = 1; j < nmodes - 1; j++)
    {
      rbest = 1000;
      abest = 1000;
      hbest = 0;
      vbest = 0;

      for (i = j; i < nmodes; i++)
        {
          rtemp = _abs(refresh -
                       DIVIDE(DIVIDE(modes[i].dotclock * 1000,
                                    modes[i].htotal), modes[i].vtotal));
          atemp = (modes[i].hdisplay * 100 / modes[i].vdisplay);
          if (rtemp < rbest)
            {
              rbest = rtemp;
              tmpmode = &modes[i];
            }

          if (rtemp == rbest)
            {
              /* Treat "close" aspect ratios as identical */

              if (_abs(abest - atemp) > (abest / 8) &&
                  _abs(aspect - atemp) < _abs(aspect - abest))
                {
                  abest = atemp;
                  tmpmode = &modes[i];
                }

              if (atemp == abest ||
                  _abs(abest - atemp) <= (abest / 8))
                {
                  if (modes[i].hdisplay > hbest)
                    {
                      hbest = modes[i].hdisplay;
                      tmpmode = &modes[i];
                    }

                  if (modes[i].hdisplay ==
                      hbest && modes[i].vdisplay > vbest)
                    {
                      vbest = modes[i].vdisplay;
                      tmpmode = &modes[i];
                    }
                }
            }
        }

      if (tmpmode != &modes[j])
        {
          videomode_swap(tmpmode, &modes[j]);
        }
    }
}
