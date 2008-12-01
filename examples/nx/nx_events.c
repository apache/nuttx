/****************************************************************************
 * examples/nx/nx_events.c
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
#include <semaphore.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/nx.h>
#include "nx_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nxeg_redraw1(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                         boolean morem, FAR void *arg);
static void nxeg_redraw2(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                         boolean more, FAR void *arg);
static void nxeg_position1(NXWINDOW hwnd, FAR const struct nxgl_rect_s *size,
                           FAR const struct nxgl_point_s *pos,
                           FAR const struct nxgl_rect_s *bounds,
                           FAR void *arg);
static void nxeg_position2(NXWINDOW hwnd, FAR const struct nxgl_rect_s *size,
                           FAR const struct nxgl_point_s *pos,
                           FAR const struct nxgl_rect_s *bounds,
                           FAR void *arg);
#ifdef CONFIG_NX_MOUSE
static void nxeg_mousein1(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                          ubyte buttons, FAR void *arg);
static void nxeg_mousein2(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                          ubyte buttons, FAR void *arg);
#endif
#ifdef CONFIG_NX_KBD
static void nxeg_kbdin(NXWINDOW hwnd, ubyte nch, const ubyte *ch);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct nx_callback_s g_nxcb1 =
{
  nxeg_redraw1,   /* redraw */
  nxeg_position1  /* position */
#ifdef CONFIG_NX_MOUSE
  , nxeg_mousein1 /* mousein */
#endif
#ifdef CONFIG_NX_KBD
  , nxeg_kbdin1   /* my kbdin */
#endif
};

const struct nx_callback_s g_nxcb2 =
{
  nxeg_redraw2,   /* redraw */
  nxeg_position2  /* position */
#ifdef CONFIG_NX_MOUSE
  , nxeg_mousein2 /* mousein */
#endif
#ifdef CONFIG_NX_KBD
  , nxeg_kbdin2   /* my kbdin */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxeg_redraw1
 ****************************************************************************/

static void nxeg_redraw1(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                         boolean more, FAR void *arg)
{
  message("nxeg_redraw%d: hwnd=%p rect={(%d,%d),(%d,%d)} more=%s\n",
           (int)arg, hwnd,
           rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y,
           more ? "TRUE" : "FALSE");
  nx_fill(hwnd, rect, g_color1);
}

/****************************************************************************
 * Name: nxeg_redraw2
 ****************************************************************************/

static void nxeg_redraw2(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                         boolean more, FAR void *arg)
{
  message("nxeg_redraw%d: hwnd=%p rect={(%d,%d),(%d,%d)} more=%s\n",
           (int)arg, hwnd,
           rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y,
           more ? "TRUE" : "FALSE");
  nx_fill(hwnd, rect, g_color2);
}

/****************************************************************************
 * Name: nxeg_position1
 ****************************************************************************/

static void nxeg_position1(NXWINDOW hwnd, FAR const struct nxgl_rect_s *size,
                           FAR const struct nxgl_point_s *pos,
                           FAR const struct nxgl_rect_s *bounds,
                           FAR void *arg)
{
  /* Report the position */

  message("nxeg_position%d: hwnd=%p size={(%d,%d),(%d,%d)} pos=(%d,%d) bounds={(%d,%d),(%d,%d)}\n",
           arg, hwnd,
           size->pt1.x, size->pt1.y, size->pt2.x, size->pt2.y,
           pos->x, pos->y,
           bounds->pt1.x, bounds->pt1.y, bounds->pt2.x, bounds->pt2.y);

  /* Have we picked off the window bounds yet? */

  if (!b_haveresolution)
    {
      /* Save the window limits (these should be the same for all places and all windows */

      g_xres = bounds->pt2.x;
      g_yres = bounds->pt2.y;

      b_haveresolution = TRUE;
      sem_post(&g_semevent);
      message("nxeg_position2: Have xres=%d yres=%d\n", g_xres, g_yres);
    }
}

/****************************************************************************
 * Name: nxeg_position2
 ****************************************************************************/

static void nxeg_position2(NXWINDOW hwnd, FAR const struct nxgl_rect_s *size,
                           FAR const struct nxgl_point_s *pos,
                           FAR const struct nxgl_rect_s *bounds,
                           FAR void *arg)
{
  /* Report the position */

  message("nxeg_position%d: hwnd=%p size={(%d,%d),(%d,%d)} pos=(%d,%d) bounds={(%d,%d),(%d,%d)}\n",
           arg, hwnd,
           size->pt1.x, size->pt1.y, size->pt2.x, size->pt2.y,
           pos->x, pos->y,
           bounds->pt1.x, bounds->pt1.y, bounds->pt2.x, bounds->pt2.y);
}

/****************************************************************************
 * Name: nxeg_mousein1
 ****************************************************************************/

#ifdef CONFIG_NX_MOUSE
static void nxeg_mousein1(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                          ubyte buttons, FAR void *arg)
{
  message("nxeg_mousein%d: hwnd=%p pos=(%d,%d) button=%02x\n",
           (int)arg, hwnd,  pos->x, pos->y, buttons);
}
#endif

/****************************************************************************
 * Name: nxeg_mousein2
 ****************************************************************************/

#ifdef CONFIG_NX_MOUSE
static void nxeg_mousein2(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                          ubyte buttons, FAR void *arg)
{
  message("nxeg_mousein%d: hwnd=%p pos=(%d,%d) button=%02x\n",
          (int)arg, hwnd,  pos->x, pos->y, buttons);
}
#endif

/****************************************************************************
 * Name: nxeg_kbdinfo
 ****************************************************************************/

#ifdef CONFIG_NX_KBD
static void nxeg_kbdinfo(ubyte nch, const ubyte *ch)
{
  int i;
  for (i = 0; i < nch; i++)
    {
      if (isprint(ch[i]))
        {
          message("          ch[%d]=  (%02x)", i, ch[i]);
        }
      else
        {
          message("          ch[%d]=%c (%02x)", i, ch[i], ch[i]);
        }
    }
}
#endif

/****************************************************************************
 * Name: nxeg_kbdin1
 ****************************************************************************/

#ifdef CONFIG_NX_KBD
static void nxeg_kbdin1(NXWINDOW hwnd, ubyte nch, const ubyte *ch)
{
  message("nxeg_kbdin1: hwnd=%p nch=%d\n", hwnd, nch);
  nxeg_kbdinfo(nch, ch);
}
#endif

/****************************************************************************
 * Name: nxeg_kbdin2
 ****************************************************************************/

#ifdef CONFIG_NX_KBD
static void nxeg_kbdin2(NXWINDOW hwnd, ubyte nch, const ubyte *ch)
{
  message("nxeg_kbdin2: hwnd=%p nch=%d\n", hwnd, nch);
  nxeg_kbdinfo(nch, ch);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_listenerthread
 ****************************************************************************/

#ifdef CONFIG_NX_MULTIUSER
FAR void *nx_listenerthread(FAR void *arg)
{
  int ret;

  /* Set up to catch a signal */

  /* Process events forever */

  for (;;)
    {
      /* Handle the next event.  If we were configured blocking, then
       * we will stay right here until the next event is received.  Since
       * we have dedicated a while thread to servicing events, it would
       * be most natural to also select CONFIG_NX_BLOCKING -- if not, the
       * following would be a tight infinite loop (unless we added addition
       * logic with nx_eventnotify and sigwait to pace it).
       */

      ret = nx_eventhandler(g_hnx);
      if (ret < 0)
        {
          /* An error occurred... assume that we have lost connection with
           * the server.
           */

          message("nx_listenerthread: Lost server connection: %d\n", errno);
          exit(NXEXIT_LOSTSERVERCONN);
        }

      /* If we received a message, we must be connected */

      if (!g_connected)
        {
          g_connected = TRUE;
          sem_post(&g_semevent);
          message("nx_listenerthread: Connected\n");
        }
    }
}
#endif
