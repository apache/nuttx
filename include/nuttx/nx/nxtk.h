/****************************************************************************
 * include/nuttx/nx/nxtk.h
 *
 *   Copyright (C) 2008-2012, 2015, 2019 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_NX_NXTK_H
#define __INCLUDE_NUTTX_NX_NXTK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/nx/nx.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_NXTK_BORDERWIDTH
#  define CONFIG_NXTK_BORDERWIDTH 4
#endif

#ifndef CONFIG_NXTK_BORDERCOLOR1
#  if !defined(CONFIG_NX_DISABLE_32BPP) || !defined(CONFIG_NX_DISABLE_24BPP)
#    define CONFIG_NXTK_BORDERCOLOR1 0x00a9a9a9
#  elif !defined(CONFIG_NX_DISABLE_16BPP)
#    define CONFIG_NXTK_BORDERCOLOR1 0xad55
#  elif !defined(CONFIG_NX_DISABLE_4BPP)
#    define CONFIG_NXTK_BORDERCOLOR1 6
#  else
#    define CONFIG_NXTK_BORDERCOLOR1 'B'
#  endif
#endif

#ifndef CONFIG_NXTK_BORDERCOLOR2
#  if !defined(CONFIG_NX_DISABLE_32BPP) || !defined(CONFIG_NX_DISABLE_24BPP)
#    define CONFIG_NXTK_BORDERCOLOR2 0x00696969
#  elif !defined(CONFIG_NX_DISABLE_16BPP)
#    define CONFIG_NXTK_BORDERCOLOR2 0x6b4d
#  elif !defined(CONFIG_NX_DISABLE_4BPP)
#    define CONFIG_NXTK_BORDERCOLOR2 4
#  else
#    define CONFIG_NXTK_BORDERCOLOR2 'b'
#  endif
#endif

#ifndef CONFIG_NXTK_BORDERCOLOR3
#  if !defined(CONFIG_NX_DISABLE_32BPP) || !defined(CONFIG_NX_DISABLE_24BPP)
#    define CONFIG_NXTK_BORDERCOLOR3 0x00d9d9d9
#  elif !defined(CONFIG_NX_DISABLE_16BPP)
#    define CONFIG_NXTK_BORDERCOLOR3 0xdedb
#  elif !defined(CONFIG_NX_DISABLE_4BPP)
#    define CONFIG_NXTK_BORDERCOLOR3 8
#  else
#    define CONFIG_NXTK_BORDERCOLOR3 'S'
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the handle that can be used to access the window data region */

typedef FAR void *NXTKWINDOW;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
# define EXTERN extern "C"
extern "C"
{
#else
# define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_openwindow
 *
 * Description:
 *   Create a new, framed window.
 *
 * Input Parameters:
 *   handle - The handle returned by nx_connect
 *   flags  - Optional flags.  These include:
 *            NXBE_WINDOW_RAMBACKED:  Creates a RAM backed window.  This
 *              option is only valid if CONFIG_NX_RAMBACKED is enabled.
 *            NXBE_WINDOW_HIDDEN:  The window is create in the HIDDEN state
 *             and can be made visible later with nxtk_setvisibility().
 *   cb     - Callbacks used to process window events
 *   arg    - User provided value that will be returned with NXTK callbacks.
 *
 * Returned Value:
 *   Success: A non-NULL handle used with subsequent NXTK window accesses
 *   Failure:  NULL is returned and errno is set appropriately
 *
 ****************************************************************************/

NXTKWINDOW nxtk_openwindow(NXHANDLE handle, uint8_t flags,
                           FAR const struct nx_callback_s *cb,
                           FAR void *arg);

/****************************************************************************
 * Name: nxtk_closewindow
 *
 * Description:
 *   Close the window opened by nxtk_openwindow
 *
 * Input Parameters:
 *   hfwnd - The handle returned by nxtk_openwindow
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_closewindow(NXTKWINDOW hfwnd);

/****************************************************************************
 * Name: nxtk_block
 *
 * Description:
 *   The response to this function call is two things:  (1) any queued
 *   callback messages to the window are 'blocked' and then (2) also
 *   subsequent window messaging is blocked.
 *
 *   The 'event' callback with the NXEVENT_BLOCKED event is the response
 *   from nx_block (or nxtk_block).  Those blocking interfaces are used to
 *   assure that no further messages are are directed to the window. Receipt
 *   of the NXEVENT_BLOCKED event signifies that (1) there are no further
 *    pending callbacks and (2) that the window is now 'defunct' and will
 *   receive no further callbacks.
 *
 *   This callback supports coordinated destruction of a window.  The client
 *   window logic must stay intact until all of the queued callbacks are
 *   processed.  Then the window may be safely closed.  Closing the window
 *   prior with pending callbacks can lead to bad behavior when the callback
 *   is executed.
 *
 * Input Parameters:
 *   hfwnd - The window to be blocked
 *   arg   - An argument that will accompany the block messages (This is arg2
 *           in the blocked callback).
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_block(NXTKWINDOW hfwnd, FAR void *arg);

/****************************************************************************
 * Name: nxtk_synch
 *
 * Description:
 *   This interface can be used to sychronize the window client with the
 *   NX server.  It really just implements an 'echo':  A synch message is
 *   sent from the window client to the server which then responds
 *   immediately by sending the NXEVENT_SYNCHED back to the windows client.
 *
 *   Due to the highly asynchronous nature of client-server communications,
 *   nxtk_synch() is sometimes necessary to assure that the client and server
 *   are fully synchronized in time.
 *
 *   Usage by the window client might be something like this:
 *
 *     extern bool g_synched;
 *     extern sem_t g_synch_sem;
 *
 *     g_synched = false;
 *     ret = nxtk_synch(hwnd, handle);
 *     if (ret < 0)
 *       {
 *          -- Handle the error --
 *       }
 *
 *     while (!g_synched)
 *       {
 *         ret = sem_wait(&g_sync_sem);
 *         if (ret < 0)
 *           {
 *              -- Handle the error --
 *           }
 *       }
 *
 *   When the window listener thread receives the NXEVENT_SYNCHED event, it
 *   would set g_synched to true and post g_synch_sem, waking up the above
 *   loop.
 *
 * Input Parameters:
 *   hfwnd - The window to be synched
 *   arg   - An argument that will accompany the block messages (This is arg2
 *           in the event callback).
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_synch(NXTKWINDOW hfwnd, FAR void *arg);

/****************************************************************************
 * Name: nxtk_getposition
 *
 * Description:
 *  Request the position and size information for the selected framed window.
 *  The size/position for the client window and toolbar will be returned
 *  asynchronously through the client callback function pointer.
 *
 * Input Parameters:
 *   hfwnd - The window handle returned by nxtk_openwindow.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_getposition(NXTKWINDOW hfwnd);

/****************************************************************************
 * Name: nxtk_setposition
 *
 * Description:
 *  Set the position for the selected client window.  This position does not
 *  include the offsets for the borders nor for any toolbar.  Those offsets
 *  will be added in to set the full window position.
 *
 * Input Parameters:
 *   hfwnd - The window handle returned by nxtk_openwindow
 *   pos   - The new position of the client sub-window
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_setposition(NXTKWINDOW hfwnd, FAR const struct nxgl_point_s *pos);

/****************************************************************************
 * Name: nxtk_setsize
 *
 * Description:
 *  Set the size for the selected client window.  This size does not
 *  include the sizes of the borders nor for any toolbar.  Those sizes
 *  will be added in to set the full window size.
 *
 * Input Parameters:
 *   hfwnd - The window handle returned by nxtk_openwindow
 *   size  - The new size of the client sub-window.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_setsize(NXTKWINDOW hfwnd, FAR const struct nxgl_size_s *size);

/****************************************************************************
 * Name: nxtk_raise
 *
 * Description:
 *   Bring the window containing the specified client sub-window to the top
 *   of the display.
 *
 * Input Parameters:
 *   hfwnd - the window to be raised.  This must have been previously created
 *           by nxtk_openwindow().
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_raise(NXTKWINDOW hfwnd);

/****************************************************************************
 * Name: nxtk_lower
 *
 * Description:
 *   Lower the window containing the specified client sub-window to the
 *   bottom of the display.
 *
 * Input Parameters:
 *   hfwnd - the window to be lowered.  This must have been previously created
 *           by nxtk_openwindow().
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_lower(NXTKWINDOW hfwnd);

/****************************************************************************
 * Name: nxtk_modal
 *
 * Description:
 *   May be used to either (1) raise a window to the top of the display and
 *   select modal behavior, or (2) disable modal behavior.
 *
 * Input Parameters:
 *   hfwnd - The window to be modified
 *   modal - True: enter modal state; False: leave modal state
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_modal(NXTKWINDOW hfwnd, bool modal);

/****************************************************************************
 * Name: nxtk_setvisibility
 *
 * Description:
 *   Select if the window is visible or hidden.  A hidden window is still
 *   present and will update normally, but will not be on visible on the
 *   display until it is unhidden.
 *
 * Input Parameters:
 *   hfwnd - The window to be modified
 *   hide  - True: Window will be hidden; false: Window will be visible
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_setvisibility(NXTKWINDOW hfwnd, bool hide);

/****************************************************************************
 * Name: nxtk_ishidden
 *
 * Description:
 *   Return true if the window is hidden.
 *
 *   NOTE:  There will be a delay between the time that the visibility of
 *   the window is changed via nxtk_setvisibily() before that new setting is
 *   reported by nxtk_ishidden().  nxtk_synch() may be used if temporal
 *   synchronization is required.
 *
 * Input Parameters:
 *   hfwnd - The window to be queried
 *
 * Returned Value:
 *   True: the window is hidden, false: the window is visible
 *
 ****************************************************************************/

bool nxtk_ishidden(NXTKWINDOW hfwnd);

/****************************************************************************
 * Name: nxtk_fillwindow
 *
 * Description:
 *  Fill the specified rectangle in the client window with the specified color
 *
 * Input Parameters:
 *   hfwnd - The window handle returned by nxtk_openwindow
 *   rect  - The location within the client window to be filled
 *   color - The color to use in the fill
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_fillwindow(NXTKWINDOW hfwnd, FAR const struct nxgl_rect_s *rect,
                    nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nxtk_getwindow
 *
 * Description:
 *  Get the raw contents of graphic memory within a rectangular region. NOTE:
 *  Since raw graphic memory is returned, the returned memory content may be
 *  the memory of windows above this one and may not necessarily belong to
 *  this window unless you assure that this is the top window.
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   rect - The location to be copied
 *   plane - Specifies the color plane to get from.
 *   dest - The location to copy the memory region
 *   deststride - The width, in bytes, of the dest memory
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_getwindow(NXTKWINDOW hfwnd, FAR const struct nxgl_rect_s *rect,
                   unsigned int plane, FAR uint8_t *dest,
                   unsigned int deststride);

/****************************************************************************
 * Name: nxtk_filltrapwindow
 *
 * Description:
 *  Fill the specified trapezoid in the client window with the specified color
 *
 * Input Parameters:
 *   hfwnd - The window handle returned by nxtk_openwindow
 *   trap  - The trapezoidal region to be filled
 *   color - The color to use in the fill
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_filltrapwindow(NXTKWINDOW hfwnd,
                        FAR const struct nxgl_trapezoid_s *trap,
                        nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nxtk_drawlinewindow
 *
 * Description:
 *  Fill the specified line in the window with the specified color.  This
 *  is simply a wrapper that uses nxgl_splitline() to break the line into
 *  trapezoids and then calls nxtk_filltrapwindow() to render the line.
 *
 * Input Parameters:
 *   hfwnd - The window handle returned by nxtk_openwindow
 *   vector - Describes the line to be drawn
 *   width  - The width of the line
 *   color  - The color to use to fill the line
 *   caps   - Draw a circular cap the ends of the line to support better
 *            line joins
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_drawlinewindow(NXTKWINDOW hfwnd, FAR struct nxgl_vector_s *vector,
                        nxgl_coord_t width,
                        nxgl_mxpixel_t color[CONFIG_NX_NPLANES], uint8_t caps);

/****************************************************************************
 * Name: nxtk_drawcirclewindow
 *
 * Description:
 *  Draw a circular outline using the specified line thickness and color.
 *
 * Input Parameters:
 *   hfwnd  - The window handle returned by nxtk_openwindow()
 *   center - A pointer to the point that is the center of the circle
 *   radius - The radius of the circle in pixels.
 *   width  - The width of the line
 *   color  - The color to use to fill the line
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_drawcirclewindow(NXTKWINDOW hfwnd,
                          FAR const struct nxgl_point_s *center,
                          nxgl_coord_t radius, nxgl_coord_t width,
                          nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nxtk_fillcirclewindow
 *
 * Description:
 *  Fill a circular region using the specified color.
 *
 * Input Parameters:
 *   hfwnd  - The window handle returned by nxtk_openwindow()
 *   center - A pointer to the point that is the center of the circle
 *   radius - The radius of the circle in pixels.
 *   color  - The color to use to fill the circle
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_fillcirclewindow(NXWINDOW hfwnd,
                          FAR const struct nxgl_point_s *center,
                          nxgl_coord_t radius,
                          nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nxtk_movewindow
 *
 * Description:
 *   Move a rectangular region within the client sub-window of a framed window
 *
 * Input Parameters:
 *   hfwnd   - The client sub-window within which the move is to be done.
 *            This must have been previously created by nxtk_openwindow().
 *   rect   - Describes the rectangular region relative to the client
 *            sub-window to move
 *   offset - The offset to move the region.  The  rectangular region will be
 *            moved so that the origin is translated by this amount.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_movewindow(NXTKWINDOW hfwnd, FAR const struct nxgl_rect_s *rect,
                    FAR const struct nxgl_point_s *offset);

/****************************************************************************
 * Name: nxtk_bitmapwindow
 *
 * Description:
 *   Copy a rectangular region of a larger image into the rectangle in the
 *   specified client sub-window.
 *
 * Input Parameters:
 *   hfwnd    The client sub0window that will receive the bitmap image
 *   dest   - Describes the rectangular region on in the client sub-window
 *            will receive the bit map.
 *   src    - The start of the source image(s). This is an array source
 *            images of size CONFIG_NX_NPLANES.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in sub-window coordinates, however, the
 *            origin may lie outside of the sub-window display.
 *   stride - The width of the full source image in pixels.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_bitmapwindow(NXTKWINDOW hfwnd, FAR const struct nxgl_rect_s *dest,
                      FAR const void **src,
                      FAR const struct nxgl_point_s *origin,
                      unsigned int stride);

/****************************************************************************
 * Name: nxtk_opentoolbar
 *
 * Description:
 *   Create a tool bar at the top of the specified framed window
 *
 * Input Parameters:
 *   hfwnd  - The handle returned by nxtk_openwindow
 *   height - The requested height of the toolbar in pixels
 *   cb     - Callbacks used to process toolbar events
 *   arg    - User provided value that will be returned with toolbar callbacks.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_opentoolbar(NXTKWINDOW hfwnd, nxgl_coord_t height,
                     FAR const struct nx_callback_s *cb, FAR void *arg);

/****************************************************************************
 * Name: nxtk_closetoolbar
 *
 * Description:
 *   Remove the tool bar at the top of the specified framed window
 *
 * Input Parameters:
 *   hfwnd - The handle returned by nxtk_openwindow
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_closetoolbar(NXTKWINDOW hfwnd);

/****************************************************************************
 * Name: nxtk_toolbarbounds
 *
 * Description:
 *   Return a bounding box that contains the toolbar in the coordinates of
 *   the containing, framed window.  For example, the returned  origin
 *  (rect.pt1) is the offset toolbar in the framed window.
 *
 *   NOTE: This function is unsafe in the case of the multi-user NX server
 *   where the width of the window may be being changed asynchronously!  It
 *   may return the old size in this case.
 *
 * Input Parameters:
 *   hfwnd  - The handle returned by nxtk_openwindow
 *   bounds - User provided location in which to return the bounding box.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_toolbarbounds(NXTKWINDOW hfwnd, FAR struct nxgl_rect_s *bounds);

/****************************************************************************
 * Name: nxtk_filltoolbar
 *
 * Description:
 *  Fill the specified rectangle in the toolbar sub-window with the specified color
 *
 * Input Parameters:
 *   hfwnd - The handle returned by nxtk_openwindow
 *   rect  - The location within the toolbar window to be filled
 *   color - The color to use in the fill
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_filltoolbar(NXTKWINDOW hfwnd, FAR const struct nxgl_rect_s *rect,
                     nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nxtk_gettoolbar
 *
 * Description:
 *  Get the raw contents of graphic memory within a rectangular region. NOTE:
 *  Since raw graphic memory is returned, the returned memory content may be
 *  the memory of windows above this one and may not necessarily belong to
 *  this window unless you assure that this is the top window.
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   rect - The location to be copied
 *   plane - Specifies the color plane to get from.
 *   dest - The location to copy the memory region
 *   deststride - The width, in bytes, of the dest memory
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_gettoolbar(NXTKWINDOW hfwnd, FAR const struct nxgl_rect_s *rect,
                    unsigned int plane, FAR uint8_t *dest,
                    unsigned int deststride);

/****************************************************************************
 * Name: nxtk_filltraptoolbar
 *
 * Description:
 *  Fill the specified trapezoid in the toolbar sub-window with the specified color
 *
 * Input Parameters:
 *   hfwnd - The handle returned by nxtk_openwindow
 *   trap  - The trapezoidal region to be filled
 *   color - The color to use in the fill
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_filltraptoolbar(NXTKWINDOW hfwnd,
                         FAR const struct nxgl_trapezoid_s *trap,
                         nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nxtk_drawlinetoolbar
 *
 * Description:
 *  Fill the specified line in the toolbar sub-window with the specified
 *  color.  This is simply a wrapper that uses nxgl_splitline() to break the
 *  line into trapezoids and then calls nxtk_filltraptoolbar() to render the
 *  lines.
 *
 * Input Parameters:
 *   hfwnd - The window handle returned by nxtk_openwindow
 *   vector - Describes the line to be drawn
 *   width  - The width of the line
 *   color  - The color to use to fill the line
 *   caps   - Draw a circular cap on the ends of the line to support better
 *            line joins
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_drawlinetoolbar(NXTKWINDOW hfwnd, FAR struct nxgl_vector_s *vector,
                         nxgl_coord_t width,
                         nxgl_mxpixel_t color[CONFIG_NX_NPLANES], uint8_t caps);

/****************************************************************************
 * Name: nxtk_drawcircletoolbar
 *
 * Description:
 *  Draw a circular outline using the specified line thickness and color.
 *
 * Input Parameters:
 *   hfwnd  - The window handle returned by nxtk_openwindow()
 *   center - A pointer to the point that is the center of the circle
 *   radius - The radius of the circle in pixels.
 *   width  - The width of the line
 *   color  - The color to use to fill the line
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_drawcircletoolbar(NXTKWINDOW hfwnd,
                           FAR const struct nxgl_point_s *center,
                           nxgl_coord_t radius, nxgl_coord_t width,
                           nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nxtk_fillcircletoolbar
 *
 * Description:
 *  Fill a circular region using the specified color.
 *
 * Input Parameters:
 *   hfwnd  - The window handle returned by nxtk_openwindow()
 *   center - A pointer to the point that is the center of the circle
 *   radius - The radius of the circle in pixels.
 *   color  - The color to use to fill the circle
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_fillcircletoolbar(NXWINDOW hfwnd,
                           FAR const struct nxgl_point_s *center,
                           nxgl_coord_t radius,
                           nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nxtk_movetoolbar
 *
 * Description:
 *   Move a rectangular region within the toolbar sub-window of a framed window
 *
 * Input Parameters:
 *   hfwnd  - The sub-window containing the toolbar within which the move is
 *            to be done. This must have been previously created by
 *            nxtk_openwindow().
 *   rect   - Describes the rectangular region relative to the toolbar
 *            sub-window to move
 *   offset - The offset to move the region.  The  rectangular region will be
 *            moved so that the origin is translated by this amount.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_movetoolbar(NXTKWINDOW hfwnd, FAR const struct nxgl_rect_s *rect,
                     FAR const struct nxgl_point_s *offset);

/****************************************************************************
 * Name: nxtk_bitmaptoolbar
 *
 * Description:
 *   Copy a rectangular region of a larger image into the rectangle in the
 *   specified toolbar sub-window.
 *
 * Input Parameters:
 *   hfwnd  - The sub-window whose toolbar will receive the bitmap image
 *   dest   - Describes the rectangular region on in the toolbar sub-window
 *            will receive the bit map.
 *   src    - The start of the source image.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in sub-window coordinates, however, the
 *            origin may lie outside of the sub-window display.
 *   stride - The width of the full source image in bytes.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_bitmaptoolbar(NXTKWINDOW hfwnd, FAR const struct nxgl_rect_s *dest,
                       FAR const void *src[CONFIG_NX_NPLANES],
                       FAR const struct nxgl_point_s *origin,
                       unsigned int stride);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_NX_NXTK_H */
