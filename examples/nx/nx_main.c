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
  NXEXIT_TASKCREATE,
  NXEXIT_FBINITIALIZE,
  NXEXIT_FBGETVPLANE,
  NXEXIT_NXOPEN,
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
                         boolean more);
static void nxeg_redraw2(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                         boolean more);
static void nxeg_position1(NXWINDOW hwnd, FAR const struct nxgl_rect_s *size,
                           FAR const struct nxgl_point_s *pos,
                           FAR const struct nxgl_rect_s *bounds);
static void nxeg_position2(NXWINDOW hwnd, FAR const struct nxgl_rect_s *size,
                           FAR const struct nxgl_point_s *pos,
                           FAR const struct nxgl_rect_s *bounds);
#ifdef CONFIG_NX_MOUSE
static void nxeg_mousein1(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                          ubyte buttons);
static void nxeg_mousein2(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                          ubyte buttons);
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

static nxgl_coord_t g_xres;
static nxgl_coord_t g_yres;

static nxgl_mxpixel_t g_color1[CONFIG_NX_NPLANES];
static nxgl_mxpixel_t g_color2[CONFIG_NX_NPLANES];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxeg_redraw1
 ****************************************************************************/

static void nxeg_redraw1(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                         boolean more)
{
  message("nxeg_redraw1: hwnd=%p rect={(%d,%d),(%d,%d)} more=%s\n",
           hwnd,
           rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y,
           more ? "TRUE" : "FALSE");
  nx_fill(hwnd, rect, g_color1);
}

/****************************************************************************
 * Name: nxeg_redraw2
 ****************************************************************************/

static void nxeg_redraw2(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                         boolean more)
{
  message("nxeg_redraw2: hwnd=%p rect={(%d,%d),(%d,%d)} more=%s\n",
           hwnd,
           rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y,
           more ? "TRUE" : "FALSE");
  nx_fill(hwnd, rect, g_color2);
}

/****************************************************************************
 * Name: nxeg_position1
 ****************************************************************************/

static void nxeg_position1(NXWINDOW hwnd, FAR const struct nxgl_rect_s *size,
                           FAR const struct nxgl_point_s *pos,
                           FAR const struct nxgl_rect_s *bounds)
{
  /* Report the position */

  message("nxeg_position1: hwnd=%p size={(%d,%d),(%d,%d)} pos=(%d,%d) bounds={(%d,%d),(%d,%d)}\n",
           hwnd,
           size->pt1.x, size->pt1.y, size->pt2.x, size->pt2.y,
           pos->x, pos->y,
           bounds->pt1.x, bounds->pt1.y, bounds->pt2.x, bounds->pt2.y);

  /* Save the window limits */

  g_xres = bounds->pt2.x;
  g_yres = bounds->pt2.y;
}

/****************************************************************************
 * Name: nxeg_position2
 ****************************************************************************/

static void nxeg_position2(NXWINDOW hwnd, FAR const struct nxgl_rect_s *size,
                           FAR const struct nxgl_point_s *pos,
                           FAR const struct nxgl_rect_s *bounds)
{
  /* Report the position */

  message("nxeg_position2: hwnd=%p size={(%d,%d),(%d,%d)} pos=(%d,%d) bounds={(%d,%d),(%d,%d)}\n",
           hwnd,
           size->pt1.x, size->pt1.y, size->pt2.x, size->pt2.y,
           pos->x, pos->y,
           bounds->pt1.x, bounds->pt1.y, bounds->pt2.x, bounds->pt2.y);

  /* Save the window limits */

  g_xres = bounds->pt2.x;
  g_yres = bounds->pt2.y;
}

/****************************************************************************
 * Name: nxeg_mousein1
 ****************************************************************************/

#ifdef CONFIG_NX_MOUSE
static void nxeg_mousein1(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                          ubyte buttons)
{
  message("nxeg_mousein1: hwnd=%p pos=(%d,%d) button=%02x\n",
           hwnd,  pos->x, pos->y, buttons);
}
#endif

/****************************************************************************
 * Name: nxeg_mousein2
 ****************************************************************************/

#ifdef CONFIG_NX_MOUSE
static void nxeg_mousein2(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                          ubyte buttons)
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
  NXHANDLE hnx;
  NXWINDOW hwnd1;
  NXWINDOW hwnd2;
#ifndef CONFIG_NX_MULTIUSER
  FAR struct fb_vtable_s *fb;
#else
  pid_t servrid;
#endif
  struct nxgl_rect_s rect;
  struct nxgl_point_s pt;
  nxgl_mxpixel_t color;
  int exitcode = NXEXIT_SUCCESS;
  int ret;
  int i;

  /* Initialize window colors */

  for (i = 0; i < CONFIG_NX_NPLANES; i++)
    {
      g_color1[i] = CONFIG_EXAMPLES_NX_COLOR1;
      g_color1[2] = CONFIG_EXAMPLES_NX_COLOR2;
    }

#ifdef CONFIG_NX_MULTIUSER
  /* Start the server task */

  message("user_start: Starting nx_servertask task\n");
  servrid = task_create("NX Server", 50, CONFIG_EXAMPLES_NX_STACKSIZE, nx_servertask, NULL);
  if (servrid < 0)
    {
      message("user_start: Failed to create nx_servertask task: %d\n", errno);
      exitcode = NXEXIT_TASKCREATE;
      goto errout;
    }

  /* Wait a bit to let the server get started */

  sleep(2);

  /* Connect to the server */

  hnx = nx_connect();
#else
  /* Initialize the frame buffer device */

  message("user_start: Initializing framebuffer\n");
  ret = up_fbinitialize();
  if (ret < 0)
    {
      message("user_start: up_fbinitialize failed: %d\n", -ret);
      exitcode = NXEXIT_FBINITIALIZE;
      goto errout;
    }

  fb = up_fbgetvplane(CONFIG_EXAMPLES_NX_VPLANE);
  if (!fb)
    {
      message("user_start: up_fbgetvplane failed, vplane=%d\n", CONFIG_EXAMPLES_NX_VPLANE);
      exitcode = NXEXIT_FBGETVPLANE;
      goto errout;
    }

  /* Then open NX */

  message("user_start: Open NX\n");
  hnx = nx_open(fb);
#endif

  message("user_start: NX handle=%p\n", hnx);
  if (!hnx)
    {
      message("user_start: Failed to get NX handle: %d\n", errno);
      exitcode = NXEXIT_NXOPEN;
      goto errout;
    }

  /* Set the background to the configured background color */

  message("user_start: Set background color=%d\n", CONFIG_EXAMPLES_NX_BGCOLOR);
  color = CONFIG_EXAMPLES_NX_BGCOLOR;
  ret = nx_setbgcolor(hnx, &color);
  if (ret < 0)
    {
      message("user_start: nx_setbgcolor failed: %d\n", errno);
      exitcode = NXEXIT_NXSETBGCOLOR;
      goto errout_with_nx;
    }

  /* Create window #1 */

  message("user_start: Create window #1\n");
  hwnd1 = nx_openwindow(hnx, &g_nxcb1);
  message("user_start: hwnd1=%p\n", hwnd1);

  if (!hwnd1)
    {
      message("user_start: nx_openwindow failed: %d\n", errno);
      exitcode = NXEXIT_NXOPENWINDOW;
      goto errout_with_nx;
    }
  message("user_start: Screen resolution (%d,%d)\n", g_xres, g_yres);

  /* Set the size of the window 2 */

  rect.pt1.x = 0;
  rect.pt1.y = 0;
  rect.pt2.x = g_xres/2;
  rect.pt2.y = g_yres/2;

  message("user_start: Set hwnd1 size to (%d,%d)\n", rect.pt2.x, rect.pt2.y);
  ret = nx_setsize(hwnd1, &rect);
  if (ret < 0)
    {
      message("user_start: nx_setsize failed: %d\n", errno);
      exitcode = NXEXIT_NXSETSIZE;
      goto errout_with_hwnd1;
    }

  pt.x = g_xres / 4;
  pt.y = g_yres / 4;

  message("user_start: Set hwnd1 postion to (%d,%d)\n", pt.x, pt.y);
  ret = nx_setposition(hwnd1, &pt);
  if (ret < 0)
    {
      message("user_start: nx_setposition failed: %d\n", errno);
      exitcode = NXEXIT_NXSETPOSITION;
      goto errout_with_hwnd1;
    }

  /* Create window #2 */

  message("user_start: Create window #1\n");
  hwnd2 = nx_openwindow(hnx, &g_nxcb2);
  message("user_start: hwnd2=%p\n", hwnd2);

  if (!hwnd2)
    {
      message("user_start: nx_openwindow failed: %d\n", errno);
      exitcode = NXEXIT_NXOPENWINDOW;
      goto errout_with_hwnd1;
    }

  /* Set the size of the window 2 == size of window 1*/

  message("user_start: Set hwnd2 size to (%d,%d)\n", rect.pt2.x, rect.pt2.y);
  ret = nx_setsize(hwnd2, &rect);
  if (ret < 0)
    {
      message("user_start: nx_setsize failed: %d\n", errno);
      exitcode = NXEXIT_NXSETSIZE;
      goto errout_with_hwnd2;
    }

  pt.x = g_xres - rect.pt2.x - pt.x;
  pt.y = g_yres - rect.pt2.y - pt.y;

  message("user_start: Set hwnd2 postion to (%d,%d)\n", pt.x, pt.y);
  ret = nx_setposition(hwnd2, &pt);
  if (ret < 0)
    {
      message("user_start: nx_setposition failed: %d\n", errno);
      exitcode = NXEXIT_NXSETPOSITION;
      goto errout_with_hwnd2;
    }

  /* Lower window 2 */

  message("user_start: Lower window #2\n");
  ret = nx_lower(hwnd2);
  if (ret < 0)
    {
      message("user_start: nx_lower failed: %d\n", errno);
      exitcode = NXEXIT_NXSETPOSITION;
      goto errout_with_hwnd2;
    }

  /* Close the window 2 */

errout_with_hwnd2:
  message("user_start: Close window\n");
  ret = nx_closewindow(hwnd2);
  if (ret < 0)
    {
      message("user_start: nx_openwindow failed: %d\n", errno);
      exitcode = NXEXIT_NXCLOSEWINDOW;
      goto errout_with_nx;
    }

  /* Close the window1 */

errout_with_hwnd1:
  message("user_start: Close window\n");
  ret = nx_closewindow(hwnd1);
  if (ret < 0)
    {
      message("user_start: nx_openwindow failed: %d\n", errno);
      exitcode = NXEXIT_NXCLOSEWINDOW;
      goto errout_with_nx;
    }

errout_with_nx:
#ifdef CONFIG_NX_MULTIUSER
  /* Disconnect from the server */

  message("user_start: Disconnect from the server\n");
  nx_disconnect(hnx);
#else
  /* Close the server */

  message("user_start: Close NX\n");
  nx_close(hnx);
#endif
errout:
  return exitcode;
}
