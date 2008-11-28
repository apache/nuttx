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

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void my_redraw(NXWINDOW handle, FAR const struct nxgl_rect_s *rect,
                      boolean more);
static void my_position(NXWINDOW handle, FAR const struct nxgl_rect_s *size,
                        FAR const struct nxgl_point_s *pos);
#ifdef CONFIG_NX_MOUSE
static void my_mousein(NXWINDOW handle, FAR const struct nxgl_point_s *pos,
                       ubyte buttons);
#endif
#ifdef CONFIG_NX_KBD
static void my_kbdin(NXWINDOW handle, ubyte nch, const ubyte *ch);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct nx_callback_s g_nxcb =
{
  my_redraw,   /* redraw */
  my_position  /* position */
#ifdef CONFIG_NX_MOUSE
  , my_mousein /* mousein */
#endif
#ifdef CONFIG_NX_KBD
  , my_kbdin   /* my kbdin */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: my_redraw
 ****************************************************************************/

static void my_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                      boolean more)
{
  message("my_redraw: hwnd=%p rect={(%d,%d),(%d,%d)} more=%s\n",
           hwnd,
           rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y,
           more ? "TRUE" : "FALSE");
}

/****************************************************************************
 * Name: my_position
 ****************************************************************************/

static void my_position(NXWINDOW hwnd, FAR const struct nxgl_rect_s *size,
                        FAR const struct nxgl_point_s *pos)
{
  message("my_position: hwnd=%p size={(%d,%d),(%d,%d)} pos=(%d,%d)\n",
           hwnd,
           size->pt1.x, size->pt1.y, size->pt2.x, size->pt2.y,
           pos->x, pos->y);
}

/****************************************************************************
 * Name: my_mousein
 ****************************************************************************/

#ifdef CONFIG_NX_MOUSE
static void my_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                       ubyte buttons)
{
  message("my_mousein: hwnd=%p pos=(%d,%d) button=%02x\n",
           hwnd,  pos->x, pos->y, buttons);
}
#endif

/****************************************************************************
 * Name: 
 ****************************************************************************/

#ifdef CONFIG_NX_KBD
static void my_kbdin(NXWINDOW hwnd, ubyte nch, const ubyte *ch)
{
  int i;
  message("my_kbdin: hwnd=%p nch=%d\n", hwnd, nch);
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
  NXWINDOW hwnd;
#ifndef CONFIG_NX_MULTIUSER
  FAR struct fb_vtable_s *fb;
#else
  pid_t servrid;
#endif
  nxgl_mxpixel_t color;
  int exitcode = 0;
  int ret;

#ifdef CONFIG_NX_MULTIUSER
  /* Start the server task */

  message("user_start: Starting nx_servertask task\n");
  servrid = task_create("NX Server", 50, CONFIG_EXAMPLES_NX_STACKSIZE, nx_servertask, argv);
  if (servrid < 0)
    {
      message("user_start: Failed to create nx_servertask task: %d\n", errno);
      exitcode = 1;
      goto errout;
    }

  /* Wait a bit to let the server get started */

  sleep(2);

  /* Connect to the server */

  hnx = nx_connect(&g_nxcb);
#else
  /* Initialize the frame buffer device */

  message("user_start: Initializing framebuffer\n");
  ret = up_fbinitialize();
  if (ret < 0)
    {
      message("user_start: up_fbinitialize failed: %d\n", -ret);
      exitcode = 2;
      goto errout;
    }

  fb = up_fbgetvplane(CONFIG_EXAMPLES_NX_VPLANE);
  if (!fb)
    {
      message("user_start: up_fbgetvplane failed, vplane=%d\n", CONFIG_EXAMPLES_NX_VPLANE);
      exitcode = 3;
      goto errout;
    }

  /* Then open NX */

  message("user_start: Open NX\n");
  hnx = nx_open(fb, &g_nxcb);
#endif

  message("user_start: NX handle=%p\n", hnx);
  if (!hnx)
    {
      message("user_start: Failed to get NX handle: %d\n", errno);
      exitcode = 4;
      goto errout;
    }

  /* Set the background to the configured background color */

  message("user_start: Set background color=%d\n", CONFIG_EXAMPLES_NX_BGCOLOR);
  color = CONFIG_EXAMPLES_NX_BGCOLOR;
  ret = nx_setbgcolor(hnx, &color);
  if (ret < 0)
    {
      message("user_start: nx_setbgcolor failed: %d\n", errno);
      exitcode = 5;
      goto errout_with_nx;
    }

  /* Create a window */

  message("user_start: Create a window\n");
  hwnd = nx_openwindow(hnx);
  message("user_start: NX window=%p\n", hwnd);

  if (!hwnd)
    {
      message("user_start: nx_openwindow failed: %d\n", errno);
      exitcode = 6;
      goto errout_with_nx;
    }

  /* Close the window */

//errout_with_window:
  message("user_start: Close window\n");
  ret = nx_closewindow(hwnd);
  if (!hwnd)
    {
      message("user_start: nx_openwindow failed: %d\n", errno);
      exitcode = 7;
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
