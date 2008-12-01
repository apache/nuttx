/****************************************************************************
 * graphics/nxsu/nx_mousein.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/nx.h>
#include "nxfe.h"

#ifdef CONFIG_NX_MOUSE

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct nxgl_point_s g_mpos;
static struct nxgl_rect_s g_mrange;
static struct g_mbutton;

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
 * Name: nxsu_mouseinit
 *
 * Description:
 *   Initialize with the mouse in the center of the display
 *
 ****************************************************************************/

void nxsu_mouseinit(int x, int y)
{
  g_mrange.x = x;
  g_mrange.y = y;
  g_mpos.x   = x / 2;
  g_mpos.y   = y / 2;
  g_mbutton  = 0;
}

/****************************************************************************
 * Name: nxmu_mousereport
 *
 * Description:
 *   Report mouse position info to the specified window
 *
 ****************************************************************************/

void nxsu_mousereport(struct nxbe_window_s *wnd)
{
  /* Give the keypad event only to the top child */

  if (win->cb->mousein)
    {
      win->cb->mousein((NXWINDOW)wnd, &g_mpos, g_mbutton, wnd->arg);
    }
}

/****************************************************************************
 * Name: nx_mousein
 *
 * Description:
 *   Used by a thread or interrupt handler that manages some kind of pointing
 *   hardware to report new positional data to the NX server.  That positional
 *   data will be routed by the NX server to the appropriate window client.
 *
 ****************************************************************************/

int nx_mousein(NXHANDLE handle, nxgl_coord_t x, nxgl_coord_t y, ubyte buttons)
{
  FAR struct nxfe_state_s *fe = (FAR struct nxfe_state_s *)handle;
  struct nxbe_window_s *wnd;

  x_coord_t x = pos->x;
  x_coord_t y = pos->y;

  /* Clip x and y to within the bounding rectangle */

  if (x < 0)
    {
      x = 0;
    }
  else if (x >= g_mbound.x)
    {
      x = g_mbound.x - 1;
    }

  if (y < 0)
    {
      y = 0;
    }
  else if (y >= g_mbound.y)
    {
      y = g_mbound.y - 1;
    }

 /* Look any change in values */

  if (x != g_mpos.x || y != g_mpos.y || button != g_mbutton)
    {
      /* Update the mouse value */

      g_mpos.x  = x;
      g_mpos.y  = y;
      b_mbutton = button;

      /* Pick the window to receive the mouse event */

      for (wnd = fe->be.topwnd; wnd; wnd = wnd->below)
        {
          nxsu_mousereport(wnd);
        }
    }
}

#endif /* CONFIG_NX_MOUSE */
