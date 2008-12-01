/****************************************************************************
 * examples/nx/nx_main.c
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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>
#include <signal.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fb.h>
#include <nuttx/arch.h>
#include <nuttx/nx.h>
#include "nx_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum exitcode_e
{
  NXEXIT_SUCCESS = 0,
  NXEXIT_SIGPROCMASK,
  NXEXIT_SIGACTION,
  NXEXIT_EVENTNOTIFY,
  NXEXIT_TASKCREATE,
  NXEXIT_FBINITIALIZE,
  NXEXIT_FBGETVPLANE,
  NXEXIT_NXOPEN,
  NXEXIT_NXCONNECT,
  NXEXIT_NXSETBGCOLOR,
  NXEXIT_NXOPENWINDOW,
  NXEXIT_NXSETSIZE,
  NXEXIT_NXSETPOSITION,
  NXEXIT_NXCLOSEWINDOW
};

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

static const struct nx_callback_s g_nxcb1 =
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

static const struct nx_callback_s g_nxcb2 =
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

/* The connecton handler */

static NXHANDLE g_hnx = NULL;
static int g_exitcode = NXEXIT_SUCCESS;

/* Initialized to zero, incremented when connected */

#ifdef CONFIG_NX_MULTIUSER
static boolean g_connected = FALSE;
#endif
static boolean b_haveresolution = FALSE;
static sem_t g_semevent = {0};

/* The screen resolution */

static nxgl_coord_t g_xres;
static nxgl_coord_t g_yres;

/* Colors used to fill window 1 & 2 */

static nxgl_mxpixel_t g_color1[CONFIG_NX_NPLANES];
static nxgl_mxpixel_t g_color2[CONFIG_NX_NPLANES];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxeg_redraw1
 ****************************************************************************/

static void nxeg_redraw1(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                         boolean more, FAR void *arg)
{
  message("nxeg_redraw%d: hwnd=%p rect={(%d,%d),(%d,%d)} more=%s arg=%p\n",
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
  message("nxeg_redraw%d: hwnd=%p rect={(%d,%d),(%d,%d)} more=%s arg=%p\n",
           (int)arg, hwnd,
           rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y,
           more ? "TRUE" : "FALSE",
           arg);
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
           arg, hwnd,  pos->x, pos->y, buttons);
}
#endif

/****************************************************************************
 * Name: nxeg_mousein2
 ****************************************************************************/

#ifdef CONFIG_NX_MOUSE
static void nxeg_mousein2(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                          ubyte buttons, FAR void *arg)
{
  message("nxeg_mousein2: hwnd=%p pos=(%d,%d) button=%02x\n",
           hwnd,  pos->x, pos->y, buttons);
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
 * Name: nxeg_suinitialize
 ****************************************************************************/

#ifdef CONFIG_NX_MULTIUSER
static void nxeg_sigaction(int signo, FAR siginfo_t *info, FAR void *context)
{
  int ret;

  /* I know... you are not supposed to call printf from signal handlers */

  message("nxeg_sigaction: received signo=%d\n", signo);
  ret = nx_eventhandler(g_hnx);

  /* If we received a message, we must be connected */

  if (!g_connected)
    {
      g_connected = TRUE;
      sem_post(&g_semevent);
      message("nxeg_sigaction: Connected\n");
    }

  /* Request notification of further incoming events */

  ret = nx_eventnotify(g_hnx, CONFIG_EXAMPLES_NX_NOTIFYSIGNO);
  if (ret < 0)
     {
       message("nxeg_sigaction: nx_eventnotify failed: %d\n", errno);
     }
}
#endif

/****************************************************************************
 * Name: nxeg_suinitialize
 ****************************************************************************/

#ifndef CONFIG_NX_MULTIUSER
static inline int nxeg_suinitialize(void)
{
  FAR struct fb_vtable_s *fb;
  int ret;

  /* Initialize the frame buffer device */

  message("nxeg_initialize: Initializing framebuffer\n");
  ret = up_fbinitialize();
  if (ret < 0)
    {
      message("nxeg_initialize: up_fbinitialize failed: %d\n", -ret);
      g_exitcode = NXEXIT_FBINITIALIZE;
      return ERROR;
    }

  fb = up_fbgetvplane(CONFIG_EXAMPLES_NX_VPLANE);
  if (!fb)
    {
      message("nxeg_initialize: up_fbgetvplane failed, vplane=%d\n", CONFIG_EXAMPLES_NX_VPLANE);
      g_exitcode = NXEXIT_FBGETVPLANE;
      return ERROR;
    }

  /* Then open NX */

  message("nxeg_initialize: Open NX\n");
  g_hnx = nx_open(fb);
  if (!g_hnx)
    {
      message("user_start: nx_open failed: %d\n", errno);
      g_exitcode = NXEXIT_NXOPEN;
      return ERROR;
    }
  return OK;
}
#endif

/****************************************************************************
 * Name: nxeg_initialize
 ****************************************************************************/

#ifdef CONFIG_NX_MULTIUSER
static inline int nxeg_muinitialize(void)
{
  struct sigaction act;
  sigset_t sigset;
  pid_t servrid;
  int ret;

  /* Set up to catch a signal */

  message("nxeg_initialize: Unmasking signal %d\n" , CONFIG_EXAMPLES_NX_NOTIFYSIGNO);
  (void)sigemptyset(&sigset);
  (void)sigaddset(&sigset, CONFIG_EXAMPLES_NX_NOTIFYSIGNO);
  ret = sigprocmask(SIG_UNBLOCK, &sigset, NULL);
  if (ret < 0)
    {
      message("nxeg_initialize: sigprocmask failed: \n", ret);
      g_exitcode = NXEXIT_SIGPROCMASK;
      return ERROR;
    }

  message("nxeg_initialize: Registering signal handler\n" );
  act.sa_sigaction = nxeg_sigaction;
  act.sa_flags     = SA_SIGINFO;
  (void)sigfillset(&act.sa_mask);
  (void)sigdelset(&act.sa_mask, CONFIG_EXAMPLES_NX_NOTIFYSIGNO);

  ret = sigaction(CONFIG_EXAMPLES_NX_NOTIFYSIGNO, &act, NULL);
  if (ret < 0)
    {
      message("nxeg_initialize: sigaction failed: %d\n" , ret);
      g_exitcode = NXEXIT_SIGACTION;
      return ERROR;
    }

  /* Start the server task */

  message("nxeg_initialize: Starting nx_servertask task\n");
  servrid = task_create("NX Server", CONFIG_EXAMPLES_NX_SERVERPRIO,
                        CONFIG_EXAMPLES_NX_STACKSIZE, nx_servertask, NULL);
  if (servrid < 0)
    {
      message("nxeg_initialize: Failed to create nx_servertask task: %d\n", errno);
      g_exitcode = NXEXIT_TASKCREATE;
      return ERROR;
    }

  /* Wait a bit to let the server get started */

  sleep(2);

  /* Connect to the server */

  g_hnx = nx_connect();
  if (g_hnx)
    {
       /* Request notification of incoming events */

       ret = nx_eventnotify(g_hnx, CONFIG_EXAMPLES_NX_NOTIFYSIGNO);
       if (ret < 0)
         {
            message("nxeg_initialize: nx_eventnotify failed: %d\n", errno);
            (void)nx_disconnect(g_hnx);
            g_hnx      = NULL;
            g_exitcode = NXEXIT_EVENTNOTIFY;
            return ERROR;
         }

       /* Don't return until we are connected to the server */

       while (!g_connected)
         {
           (void)sem_wait(&g_semevent);
         }
    }
  else
    {
      message("user_start: nx_connect failed: %d\n", errno);
      g_exitcode = NXEXIT_NXCONNECT;
      return ERROR;
    }
  return OK;
}
#endif

/****************************************************************************
 * Name: nxeg_initialize
 ****************************************************************************/

static int nxeg_initialize(void)
{
  int i;

  /* Initialize window colors */

  for (i = 0; i < CONFIG_NX_NPLANES; i++)
    {
      g_color1[i] = CONFIG_EXAMPLES_NX_COLOR1;
      g_color2[i] = CONFIG_EXAMPLES_NX_COLOR2;
    }

#ifdef CONFIG_NX_MULTIUSER
  return nxeg_muinitialize();
#else
  return nxeg_suinitialize();
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: user_initialize
 ****************************************************************************/

void user_initialize(void)
{
}

/****************************************************************************
 * Name: user_start
 ****************************************************************************/

int user_start(int argc, char *argv[])
{
  NXWINDOW hwnd1;
  NXWINDOW hwnd2;
  struct nxgl_rect_s rect;
  struct nxgl_point_s pt;
  nxgl_mxpixel_t color;
  int ret;

  /* Initialize */

  ret = nxeg_initialize();
  message("user_start: NX handle=%p\n", g_hnx);
  if (!g_hnx || ret < 0)
    {
      message("user_start: Failed to get NX handle: %d\n", errno);
      g_exitcode = NXEXIT_NXOPEN;
      goto errout;
    }

  /* Set the background to the configured background color */

  message("user_start: Set background color=%d\n", CONFIG_EXAMPLES_NX_BGCOLOR);
  color = CONFIG_EXAMPLES_NX_BGCOLOR;
  ret = nx_setbgcolor(g_hnx, &color);
  if (ret < 0)
    {
      message("user_start: nx_setbgcolor failed: %d\n", errno);
      g_exitcode = NXEXIT_NXSETBGCOLOR;
      goto errout_with_nx;
    }
  (void)nx_eventhandler(g_hnx); /* Check for server events -- normally done in a loop */

  /* Create window #1 */

  message("user_start: Create window #1\n");
  hwnd1 = nx_openwindow(g_hnx, &g_nxcb1, (FAR void *)2);
  message("user_start: hwnd1=%p\n", hwnd1);

  if (!hwnd1)
    {
      message("user_start: nx_openwindow failed: %d\n", errno);
      g_exitcode = NXEXIT_NXOPENWINDOW;
      goto errout_with_nx;
    }

  /* Wait until we have the screen resolution */

  while (!b_haveresolution)
    {
      (void)sem_wait(&g_semevent);
    }
  message("user_start: Screen resolution (%d,%d)\n", g_xres, g_yres);

  /* Set the size of the window 1 */

  rect.pt1.x = 0;
  rect.pt1.y = 0;
  rect.pt2.x = g_xres/2;
  rect.pt2.y = g_yres/2;

  message("user_start: Set hwnd1 size to (%d,%d)\n", rect.pt2.x, rect.pt2.y);
  ret = nx_setsize(hwnd1, &rect);
  if (ret < 0)
    {
      message("user_start: nx_setsize failed: %d\n", errno);
      g_exitcode = NXEXIT_NXSETSIZE;
      goto errout_with_hwnd1;
    }

  /* Sleep a bit -- both so that we can see the result of the above operations
   * but also, in the multi-user case, so that the server can get a chance to
   * actually do them!
   */

  message("user_start: Sleeping\n\n");
  sleep(1);

  /* Set the position of window #1 */

  pt.x = g_xres / 8;
  pt.y = g_yres / 8;

  message("user_start: Set hwnd1 postion to (%d,%d)\n", pt.x, pt.y);
  ret = nx_setposition(hwnd1, &pt);
  if (ret < 0)
    {
      message("user_start: nx_setposition failed: %d\n", errno);
      g_exitcode = NXEXIT_NXSETPOSITION;
      goto errout_with_hwnd1;
    }

  /* Sleep a bit */

  message("user_start: Sleeping\n\n");
  sleep(2);

  /* Create window #2 */

  message("user_start: Create window #2\n");
  hwnd2 = nx_openwindow(g_hnx, &g_nxcb2, (FAR void *)2);
  message("user_start: hwnd2=%p\n", hwnd2);

  if (!hwnd2)
    {
      message("user_start: nx_openwindow failed: %d\n", errno);
      g_exitcode = NXEXIT_NXOPENWINDOW;
      goto errout_with_hwnd1;
    }

  /* Sleep a bit */

  message("user_start: Sleeping\n\n");
  sleep(1);

  /* Set the size of the window 2 == size of window 1*/

  message("user_start: Set hwnd2 size to (%d,%d)\n", rect.pt2.x, rect.pt2.y);
  ret = nx_setsize(hwnd2, &rect);
  if (ret < 0)
    {
      message("user_start: nx_setsize failed: %d\n", errno);
      g_exitcode = NXEXIT_NXSETSIZE;
      goto errout_with_hwnd2;
    }

  /* Sleep a bit */

  message("user_start: Sleeping\n\n");
  sleep(1);

  /* Set the position of window #2 */

  pt.x = g_xres - rect.pt2.x - pt.x;
  pt.y = g_yres - rect.pt2.y - pt.y;

  message("user_start: Set hwnd2 postion to (%d,%d)\n", pt.x, pt.y);
  ret = nx_setposition(hwnd2, &pt);
  if (ret < 0)
    {
      message("user_start: nx_setposition failed: %d\n", errno);
      g_exitcode = NXEXIT_NXSETPOSITION;
      goto errout_with_hwnd2;
    }

  /* Sleep a bit */

  message("user_start: Sleeping\n\n");
  sleep(2);

  /* Lower window 2 */

  message("user_start: Lower window #2\n");
  ret = nx_lower(hwnd2);
  if (ret < 0)
    {
      message("user_start: nx_lower failed: %d\n", errno);
      g_exitcode = NXEXIT_NXSETPOSITION;
      goto errout_with_hwnd2;
    }

  /* Sleep a bit */

  message("user_start: Sleeping\n\n");
  sleep(1);

  /* Lower window 1 */

  message("user_start: Raise window #2\n");
  ret = nx_raise(hwnd2);
  if (ret < 0)
    {
      message("user_start: nx_raise failed: %d\n", errno);
      g_exitcode = NXEXIT_NXSETPOSITION;
      goto errout_with_hwnd2;
    }

  /* Sleep a bit */

  message("user_start: Sleeping\n\n");
  sleep(2);

  /* Close the window 2 */

errout_with_hwnd2:
  message("user_start: Close window\n");
  ret = nx_closewindow(hwnd2);
  if (ret < 0)
    {
      message("user_start: nx_openwindow failed: %d\n", errno);
      g_exitcode = NXEXIT_NXCLOSEWINDOW;
      goto errout_with_nx;
    }

  /* Close the window1 */

errout_with_hwnd1:
  message("user_start: Close window\n");
  ret = nx_closewindow(hwnd1);
  if (ret < 0)
    {
      message("user_start: nx_openwindow failed: %d\n", errno);
      g_exitcode = NXEXIT_NXCLOSEWINDOW;
      goto errout_with_nx;
    }

errout_with_nx:
#ifdef CONFIG_NX_MULTIUSER
  /* Disconnect from the server */

  message("user_start: Disconnect from the server\n");
  nx_disconnect(g_hnx);
#else
  /* Close the server */

  message("user_start: Close NX\n");
  nx_close(g_hnx);
#endif
errout:
  return g_exitcode;
}
