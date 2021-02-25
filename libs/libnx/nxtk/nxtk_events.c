/****************************************************************************
 * libs/libnx/nxtk/nxtk_events.c
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
#include <stdbool.h>
#include <stdlib.h>
#include <debug.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/nx/nx.h>
#include "nxtk.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nxtk_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                        bool morem, FAR void *arg);
static void nxtk_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                          FAR const struct nxgl_point_s *pos,
                          FAR const struct nxgl_rect_s *bounds,
                          FAR void *arg);
#ifdef CONFIG_NX_XYINPUT
static void nxtk_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                         uint8_t buttons, FAR void *arg);
#endif
#ifdef CONFIG_NX_KBD
static void nxtk_kbdin(NXWINDOW hwnd, uint8_t nch, const uint8_t *ch,
                       FAR void *arg);
#endif
static void nxtk_event(NXWINDOW hwnd, enum nx_event_e event,
                       FAR void *arg1, FAR void *arg2);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct nx_callback_s g_nxtkcb =
{
  nxtk_redraw,    /* redraw */
  nxtk_position   /* position */
#ifdef CONFIG_NX_XYINPUT
  , nxtk_mousein  /* mousein */
#endif
#ifdef CONFIG_NX_KBD
  , nxtk_kbdin    /* kbdin */
#endif
  , nxtk_event    /* event */
};

/****************************************************************************
 * Name: nxtk_redraw
 ****************************************************************************/

static void nxtk_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                        bool more, FAR void *arg)
{
  FAR struct nxtk_framedwindow_s *fwnd;
  struct nxgl_rect_s intersection;

  DEBUGASSERT(hwnd != NULL && rect != NULL);
  fwnd = (FAR struct nxtk_framedwindow_s *)hwnd;

  ginfo("hwnd=%p rect={(%d,%d),(%d,%d)} more=%d\n",
        hwnd, rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y, more);

  /* The incoming rectangle (rect) is relative to the containing window
   * (i.e., (0,0) is the top left corner of the outer, containing window).
   * If any part of the rectangle overlaps the client sub-window region, then
   * forward the redraw callback.
   */

  DEBUGASSERT(fwnd->fwcb != NULL);
  if (fwnd->fwcb->redraw)
    {
      /* Clip the redraw rectangle so that it lies within the client
       * sub-window bounds and move the rectangle to that it is relative to
       * the client sub-window
       * (i.e., (0,0) is the top left corner of the client sub-window).
       */

      nxtk_containerclip(fwnd, &intersection, rect, &fwnd->fwrect);

      ginfo("fwrect intersection={(%d,%d),(%d,%d)}\n",
           intersection.pt1.x, intersection.pt1.y,
           intersection.pt2.x, intersection.pt2.y);

      if (!nxgl_nullrect(&intersection))
        {
          fwnd->fwcb->redraw((NXTKWINDOW)fwnd,
              &intersection, false, fwnd->fwarg);
        }
    }

  /* If any part of the rectangle overlaps the client toolbar region, then
   * forward the redraw callback.
   */

  if (fwnd->tbcb && fwnd->tbcb->redraw)
    {
      /* Clip the redraw rectangle so that it lies within the toolbar
       * sub-window bounds and move the rectangle to that it is relative to
       * the toolbar sub-window
       * (i.e., (0,0) is the top left corner of the client sub-window).
       */

      nxtk_containerclip(fwnd, &intersection, rect, &fwnd->tbrect);

      ginfo("tbrect intersection={(%d,%d),(%d,%d)}\n",
           intersection.pt1.x, intersection.pt1.y,
           intersection.pt2.x, intersection.pt2.y);

      if (!nxgl_nullrect(&intersection))
        {
          fwnd->tbcb->redraw((NXTKWINDOW)fwnd,
               &intersection, false, fwnd->tbarg);
        }
    }

  /* Then draw the frame */

  nxtk_drawframe(fwnd, rect);
}

/****************************************************************************
 * Name: nxtk_position
 ****************************************************************************/

static void nxtk_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                           FAR const struct nxgl_point_s *pos,
                           FAR const struct nxgl_rect_s *bounds,
                           FAR void *arg)
{
  FAR struct nxtk_framedwindow_s *fwnd =
                          (FAR struct nxtk_framedwindow_s *)hwnd;
  struct nxgl_size_s subwindowsize;

  ginfo("hwnd=%p size=(%d,%d) pos=(%d,%d) bounds={(%d,%d),(%d,%d)}\n",
        hwnd, size->w, size->h, pos->x, pos->y,
        bounds->pt1.x, bounds->pt1.y, bounds->pt2.x, bounds->pt2.y);

  /* Recalculate the dimensions of the toolbar and client windows */

  nxtk_setsubwindows(fwnd);

  /* Report the size / position of the client sub-window */

  if (fwnd->fwcb->position)
    {
      nxgl_rectsize(&subwindowsize, &fwnd->fwrect);
      fwnd->fwcb->position((NXTKWINDOW)fwnd, &subwindowsize,
                           &fwnd->fwrect.pt1, bounds, fwnd->fwarg);
    }

  /* Report the size / position of the toolbar sub-window */

  if (fwnd->tbcb && fwnd->tbcb->position)
    {
      nxgl_rectsize(&subwindowsize, &fwnd->tbrect);
      fwnd->tbcb->position((NXTKWINDOW)fwnd, &subwindowsize,
                           &fwnd->tbrect.pt1, bounds, fwnd->tbarg);
    }
}

/****************************************************************************
 * Name: nxtk_mousein
 ****************************************************************************/

#ifdef CONFIG_NX_XYINPUT
static void nxtk_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                         uint8_t buttons, FAR void *arg)
{
  FAR struct nxtk_framedwindow_s *fwnd =
                               (FAR struct nxtk_framedwindow_s *)hwnd;
  struct nxgl_point_s abspos;
  struct nxgl_point_s relpos;

  /* Raise the window to the top if any mouse button was pressed or if
   * auto-raise is configured.  Do this before reporting the mouse event
   * (because processing of the mouse event could change the ordering again).
   */

  /* REVISIT:
   * This does not work correctly.  In a scenario where (1) there are
   * multiple queued touchscreen events and (2) the result of the first input
   * was to switch windows, then this autoraise implementation will cause the
   * window to revert to the previous window.  Not good behavior.
   */

#ifdef CONFIG_NXTK_AUTORAISE
  if (fwnd->wnd.above != NULL)
#else
  if (buttons != 0 && fwnd->wnd.above != NULL)
#endif
    {
       nx_raise((NXWINDOW)&fwnd->wnd);
    }

  /* When we get here, the mouse position that we receive has already been
   * offset by the window origin.  Here we need to detect mouse events in
   * the various regions of the windows:  The toolbar, the client window,
   * or the frame.  And then offset the position accordingly.
   */

  /* The fwrect and tbrect boxes are both in absolute display coordinates. So
   * the easiest thing to do is to restore the mouse position to absolute
   * display coordiantes before making the comparisons and adjustments.
   */

  nxgl_vectoradd(&abspos, pos, &fwnd->wnd.bounds.pt1);

  /* In order to deliver mouse release events to the same window where the
   * mouse down event happened, we store the initial mouse down location.
   */

  if (fwnd->mbutton == 0 && buttons != 0)
    {
      fwnd->mpos = abspos;
    }

  fwnd->mbutton = buttons;

  /* Is the mouse position inside of the client window region? */

  if (fwnd->fwcb->mousein != NULL &&
      nxgl_rectinside(&fwnd->fwrect, &fwnd->mpos))
    {
      nxgl_vectsubtract(&relpos, &abspos, &fwnd->fwrect.pt1);
      fwnd->fwcb->mousein((NXTKWINDOW)fwnd, &relpos, buttons, fwnd->fwarg);
    }

  /* If the mouse position inside the toolbar region? */

  else if (fwnd->tbcb->mousein != NULL &&
           nxgl_rectinside(&fwnd->tbrect, &fwnd->mpos))
    {
      nxgl_vectsubtract(&relpos, &abspos, &fwnd->tbrect.pt1);
      fwnd->tbcb->mousein((NXTKWINDOW)fwnd, &relpos, buttons, fwnd->tbarg);
    }

  /* REVISIT: Missing logic to detect border events.
   *
   * NOTE that the following logic is completely redundant!  All of the
   * above tests could be eliminated and only the following performed with
   * the same result.  These tests are separated from the above only to
   * guarantee a spot for future border mouse event event detection which
   * would go about here.
   */

  /* As a complexity, when dragging mouse events outside of the containing
   * raw window will be sent to this function as well.  We will need to pass
   * these outside-the-window reports as well.
   *
   * If the mouse report is below the toolbar, then we will report this
   * as a main window event.
   */

  else if (fwnd->fwcb->mousein != NULL &&
           pos->y >= fwnd->tbheight + CONFIG_NXTK_BORDERWIDTH)
    {
      nxgl_vectsubtract(&relpos, &abspos, &fwnd->fwrect.pt1);
      fwnd->fwcb->mousein((NXTKWINDOW)fwnd, &relpos, buttons, fwnd->fwarg);
    }

  /* Otherwise, we will report this as a toolbar event. */

  else if (fwnd->tbcb->mousein != NULL &&
           pos->y < fwnd->tbheight + CONFIG_NXTK_BORDERWIDTH)
    {
      nxgl_vectsubtract(&relpos, &abspos, &fwnd->tbrect.pt1);
      fwnd->tbcb->mousein((NXTKWINDOW)fwnd, &relpos, buttons, fwnd->tbarg);
    }
}
#endif

/****************************************************************************
 * Name: nxtk_kbdin
 ****************************************************************************/

#ifdef CONFIG_NX_KBD
static void nxtk_kbdin(NXWINDOW hwnd, uint8_t nch, const uint8_t *ch,
                       FAR void *arg)
{
  FAR struct nxtk_framedwindow_s *fwnd =
                              (FAR struct nxtk_framedwindow_s *)hwnd;

  /* Only the client window gets keyboard input */

  if (fwnd->fwcb->kbdin)
    {
      fwnd->fwcb->kbdin((NXTKWINDOW)fwnd, nch, ch, fwnd->fwarg);
    }
}
#endif

/****************************************************************************
 * Name: nxtk_event
 ****************************************************************************/

static void nxtk_event(NXWINDOW hwnd, enum nx_event_e event,
                       FAR void *arg1, FAR void *arg2)
{
  FAR struct nxtk_framedwindow_s *fwnd =
                             (FAR struct nxtk_framedwindow_s *)hwnd;

  /* Forward the event to the window client */

  if (fwnd->fwcb->event != NULL)
    {
      fwnd->fwcb->event((NXTKWINDOW)fwnd, event, fwnd->fwarg, arg2);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
