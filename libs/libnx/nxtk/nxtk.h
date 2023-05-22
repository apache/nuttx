/****************************************************************************
 * libs/libnx/nxtk/nxtk.h
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

#ifndef __LIBS_LIBNX_NXTK_NXTK_H
#define __LIBS_LIBNX_NXTK_NXTK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/nx/nxbe.h>
#include <nuttx/nx/nxtk.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the internal representation of the framed window object */

struct nxtk_framedwindow_s
{
  struct nxbe_window_s wnd;      /* The raw NX window */

  /* The toolbar region and callbacks */

  nxgl_coord_t tbheight;
  struct nxgl_rect_s tbrect;
  FAR const struct nx_callback_s *tbcb;
  FAR void *tbarg;

  /* Window data region and callbacks */

  struct nxgl_rect_s fwrect;
  FAR const struct nx_callback_s *fwcb;
  FAR void *fwarg;

  /* Initial mouse down location */

  uint8_t mbutton;
  struct nxgl_point_s mpos;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/* That is the callback for the framed window */

EXTERN FAR const struct nx_callback_s g_nxtkcb;

/* Frame border colors */

EXTERN nxgl_mxpixel_t g_bordercolor1[CONFIG_NX_NPLANES];
EXTERN nxgl_mxpixel_t g_bordercolor2[CONFIG_NX_NPLANES];
EXTERN nxgl_mxpixel_t g_bordercolor3[CONFIG_NX_NPLANES];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_setsubwindows
 *
 * Description:
 *   Give the window dimensions, border width, and toolbar height,
 *   calculate the new dimensions of the toolbar region and client window
 *   region
 *
 ****************************************************************************/

void nxtk_setsubwindows(FAR struct nxtk_framedwindow_s *fwnd);

/****************************************************************************
 * Name: nxtk_subwindowclip
 *
 * Description:
 *   Clip the src rectangle so that it lies within the sub-window bounds
 *   then move the rectangle to that it is relative to the containing
 *   window.
 *
 * Input Parameters:
 *   fwnd   - The framed window to be used
 *   dest   - The locaton to put the result
 *   src    - The src rectangle in relative sub-window coordinates
 *   bounds - The subwindow bounds in absolute screen coordinates.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxtk_subwindowclip(FAR struct nxtk_framedwindow_s *fwnd,
                        FAR struct nxgl_rect_s *dest,
                        FAR const struct nxgl_rect_s *src,
                        FAR const struct nxgl_rect_s *bounds);

/****************************************************************************
 * Name: nxtk_containerclip
 *
 * Description:
 *   We are given a 'src' rectangle in containing window, relative
 *   coordinates (i.e., (0,0) is the top left corner of the outer,
 *   containing window).
 *   This function will (1) clip that src rectangle so that it lies within
 *   the sub-window bounds, and then (2) move the rectangle to that it is
 *   relative to the sub-window (i.e., (0,0) is the top left corner of the
 *   sub-window).
 *
 * Input Parameters:
 *   fwnd   - The framed window to be used
 *   dest   - The location to put the result
 *   src    - The src rectangle in relative container-window coordinates
 *   bounds - The subwindow bounds in absolute screen coordinates.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxtk_containerclip(FAR struct nxtk_framedwindow_s *fwnd,
                        FAR struct nxgl_rect_s *dest,
                        FAR const struct nxgl_rect_s *src,
                        FAR const struct nxgl_rect_s *bounds);

/****************************************************************************
 * Name: nxtk_subwindowmove
 *
 * Description:
 *   Perform common clipping operations in preparation for calling nx_move()
 *
 * Input Parameters:
 *   fwnd       - The framed window within which the move is to be done.
 *                This must have been previously created by
 *                nxtk_openwindow().
 *   destrect   - The location to receive the clipped rectangle relative
 *                to containing window
 *   destoffset - The location to received the clipped offset.
 *   srcrect    - Describes the rectangular region relative to the client
 *                sub-window to move relative to the sub-window
 *   srcoffset  - The offset to move the region
 *   bounds     - The subwindow bounds in absolute screen coordinates.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

void nxtk_subwindowmove(FAR struct nxtk_framedwindow_s *fwnd,
                        FAR struct nxgl_rect_s *destrect,
                        FAR struct nxgl_point_s *destoffset,
                        FAR const struct nxgl_rect_s *srcrect,
                        FAR const struct nxgl_point_s *srcoffset,
                        FAR const struct nxgl_rect_s *bounds);

/****************************************************************************
 * Name: nxtk_drawframe
 *
 * Description:
 *   Redraw the window frame.
 *
 * Input Parameters:
 *   fwnd   - the framed window whose frame needs to be re-drawn.  This must
 *            have been previously created by nxtk_openwindow().
 *   bounds - Only draw the ports of the frame within this bounding box.
 *            (window relative coordinates).
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_drawframe(FAR struct nxtk_framedwindow_s *fwnd,
                   FAR const struct nxgl_rect_s *bounds);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __LIBS_LIBNX_NXTK_NXTK_H */
