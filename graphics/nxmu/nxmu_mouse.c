/****************************************************************************
 * graphics/nxmu/nxmu_mouse.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nx.h>
#include "nxmu.h"

#ifdef CONFIG_NX_XYINPUT

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* REVISIT:  These globals will prevent this code from being used in an
 * environment with more than one display.  FIX ME!
 */

static struct nxgl_point_s       g_mpos;
static struct nxgl_point_s       g_mrange;
static uint8_t                   g_mbutton;
static FAR struct nxbe_window_s *g_mwnd;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmu_revalidate_g_mwnd
 *
 * Description:
 *   Check if the window pointed to by g_mwnd still exists. If it does,
 *   this function returns a pointer to it. Otherwise returns NULL.
 *
 ****************************************************************************/

static struct nxbe_window_s *
nxmu_revalidate_g_mwnd(FAR struct nxbe_window_s *wnd)
{
  if (!g_mwnd)
    {
      return NULL;
    }

  while (wnd)
    {
      if (wnd == g_mwnd)
        {
          return wnd;
        }

      wnd = wnd->below;
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmu_mouseinit
 *
 * Description:
 *   Initialize with the mouse in the center of the display
 *
 ****************************************************************************/

void nxmu_mouseinit(int x, int y)
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
 * Input Parameters:
 *   wnd - The window to receive the mouse report
 *
 * Returned Value:
 *   0: Mouse report sent; >0: Mouse report not sent; <0: An error occurred
 *
 ****************************************************************************/

int nxmu_mousereport(struct nxbe_window_s *wnd)
{
  struct nxclimsg_mousein_s outmsg;

  /* Does this window support mouse callbacks? */

  if (wnd->cb->mousein)
    {
      /* Yes.. Is the mouse position visible in this window? */

      if (nxbe_isvisible(wnd, &g_mpos))
        {
          /* Yes... Convert the mouse position to window relative
           * coordinates and send it to the client
           */

          outmsg.msgid   = NX_CLIMSG_MOUSEIN;
          outmsg.wnd     = wnd;
          outmsg.buttons = g_mbutton;
          nxgl_vectsubtract(&outmsg.pos, &g_mpos, &wnd->bounds.pt1);

          return nxmu_sendclientwindow(wnd, &outmsg,
                                       sizeof(struct nxclimsg_mousein_s));
        }
    }

  /* No error occurred, but the mouse report was not sent */

  return 1;
}

/****************************************************************************
 * Name: nxmu_mousein
 *
 * Description:
 *   New positional data has been received from the thread or interrupt
 *   handler that manages some kind of pointing hardware.  Route that
 *   positional data to the appropriate window client.
 *
 ****************************************************************************/

int nxmu_mousein(FAR struct nxmu_state_s *nxmu,
                 FAR const struct nxgl_point_s *pos, int buttons)
{
  FAR struct nxbe_window_s *wnd;
  nxgl_coord_t x = pos->x;
  nxgl_coord_t y = pos->y;
  uint8_t oldbuttons;
  int ret;

  /* Clip x and y to within the bounding rectangle */

  if (x < 0)
    {
      x = 0;
    }
  else if (x >= g_mrange.x)
    {
      x = g_mrange.x - 1;
    }

  if (y < 0)
    {
      y = 0;
    }
  else if (y >= g_mrange.y)
    {
      y = g_mrange.y - 1;
    }

  /* Look for any change in values */

  if (x != g_mpos.x || y != g_mpos.y || buttons != g_mbutton)
    {
      /* Update the mouse value */

      oldbuttons = g_mbutton;
      g_mpos.x   = x;
      g_mpos.y   = y;
      g_mbutton  = buttons;

      /* If a button is already down, regard this as part of a mouse drag
       * event. Pass all the following events to the window where the drag
       * started in, including the final button release event.
       */

      if (oldbuttons != 0)
        {
          g_mwnd = nxmu_revalidate_g_mwnd(nxmu->be.topwnd);
          if (g_mwnd != NULL && g_mwnd->cb->mousein)
            {
              struct nxclimsg_mousein_s outmsg;
              outmsg.msgid   = NX_CLIMSG_MOUSEIN;
              outmsg.wnd     = g_mwnd;
              outmsg.buttons = g_mbutton;
              nxgl_vectsubtract(&outmsg.pos, &g_mpos, &g_mwnd->bounds.pt1);

              return nxmu_sendclientwindow(
                                g_mwnd,
                                &outmsg,
                                sizeof(struct nxclimsg_mousein_s));
            }
          else
            {
              /* Ignore events until the button is released */

              return OK;
            }
        }

      /* Pick the window to receive the mouse event.  Start with the top
       * window and go down.  Stop with the first window that gets the mouse
       * report
       */

      for (wnd = nxmu->be.topwnd; wnd; wnd = wnd->below)
        {
          /* The background window normally has no callback structure
           * (unlessa client has taken control of the background via
           * nx_requestbkgd()).
           */

          if (wnd->cb)
            {
              ret = nxmu_mousereport(wnd);
              if (ret == 0)
                {
                  break;
                }
            }
        }

      g_mwnd = wnd;
    }

  return OK;
}

#endif /* CONFIG_NX_XYINPUT */
