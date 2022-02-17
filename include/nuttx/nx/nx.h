/****************************************************************************
 * include/nuttx/nx/nx.h
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

#ifndef __INCLUDE_NUTTX_NX_NX_H
#define __INCLUDE_NUTTX_NX_NX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/video/vnc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_NX_NPLANES
#  define CONFIG_NX_NPLANES   1  /* Max number of color planes supported */
#endif

/* Check if the underlying graphic device supports read operations */

#if !defined(CONFIG_NX_WRITEONLY) && defined(CONFIG_NX_LCDDRIVER) && defined(CONFIG_LCD_NOGETRUN)
#  define CONFIG_NX_WRITEONLY 1
#endif

/* Default server MQ name used by nx_run() macro */

#define NX_DEFAULT_SERVER_MQNAME "nxs"

/* Mouse button bits */

#define NX_MOUSE_NOBUTTONS    0x00
#define NX_MOUSE_LEFTBUTTON   0x01
#define NX_MOUSE_CENTERBUTTON 0x02
#define NX_MOUSE_RIGHTBUTTON  0x04

/* Line caps */

#define NX_LINECAP_NONE       0x00  /* No line caps */
#define NX_LINECAP_PT1        0x01  /* Line cap on pt1 of the vector only */
#define NX_LINECAP_PT2        0x02  /* Line cap on pt2 of the vector only */
#define NX_LINECAP_BOTH       0x03  /* Line cap on both ends of the vector only */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Handles ******************************************************************/

/* The interface to the NX server is managed using a opaque handle: */

typedef FAR void *NXHANDLE;

/* The interface to a specific window is managed using an opaque handle: */

typedef FAR void *NXWINDOW;

/* NX server callbacks ******************************************************/

/* Event callbacks */

enum nx_event_e
{
  NXEVENT_BLOCKED = 0,   /* Window messages are blocked */
  NXEVENT_SYNCHED,       /* Synchronization handshake */
};

/* These define callbacks that must be provided to nx_openwindow.  These
 * callbacks will be invoked as part of the processing performed by
 * nx_eventhandler()
 */

struct nx_callback_s
{
  /**************************************************************************
   * Name: redraw
   *
   * Description:
   *   NX requests that the client re-draw the portion of the window within
   *   with rectangle.
   *
   * Input Parameters:
   *   hwnd - Window handle
   *   rect - The rectangle that needs to be re-drawn (in window relative
   *          coordinates)
   *   more - true:  More re-draw requests will follow
   *   arg  - User provided argument (see nx_openwindow, nx_requestbkgd,
   *          nxtk_openwindow, or nxtk_opentoolbar)
   *
   * Returned Value:
   *   None
   *
   **************************************************************************/

  void (*redraw)(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                 bool more, FAR void *arg);

  /**************************************************************************
   * Name: position
   *
   * Description:
   *   The size or position of the window has changed (or the window was
   *   just created with zero size.
   *
   * Input Parameters:
   *   hwnd   - Window handle
   *   size   - The size of the window
   *   pos    - The position of the upper left hand corner of the window on
   *            the overall display
   *   bounds - The bounding rectangle that the describes the entire
   *            display
   *   arg    - User provided argument (see nx_openwindow, nx_requestbkgd,
   *            nxtk_openwindow, or nxtk_opentoolbar)
   *
   * Returned Value:
   *   None
   *
   **************************************************************************/

  void (*position)(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                   FAR const struct nxgl_point_s *pos,
                   FAR const struct nxgl_rect_s *bounds,
                   FAR void *arg);

  /**************************************************************************
   * Name: mousein
   *
   * Description:
   *   New mouse data is available for the window.
   *
   * Input Parameters:
   *   hwnd    - Window handle
   *   pos     - The (x,y) position of the mouse
   *   buttons - See NX_MOUSE_* definitions
   *   arg     - User provided argument (see nx_openwindow, nx_requestbkgd,
   *             nxtk_openwindow, or nxtk_opentoolbar)
   *
   * Returned Value:
   *   None
   *
   **************************************************************************/

#ifdef CONFIG_NX_XYINPUT
  void (*mousein)(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                  uint8_t buttons, FAR void *arg);
#endif

  /**************************************************************************
   * Name: kbdin
   *
   * Description:
   *   New keyboard/keypad data is available for the window
   *
   * Input Parameters:
   *   hwnd - Window handle
   *   nch  - The number of characters that are available in ch[]
   *   ch   - The array of characters
   *   arg  - User provided argument (see nx_openwindow, nx_requestbkgd,
   *          nxtk_openwindow, or nxtk_opentoolbar)
   *
   * Returned Value:
   *   None
   *
   **************************************************************************/

#ifdef CONFIG_NX_KBD
  void (*kbdin)(NXWINDOW hwnd, uint8_t nch,
                FAR const uint8_t *ch, FAR void *arg);
#endif

  /**************************************************************************
   * Name: event
   *
   * Description:
   *   This callback is used to communicate server events to the window
   *   listener.
   *
   *   NXEVENT_BLOCKED - Window messages are blocked.
   *
   *     This callback is the response from nx_block (or nxtk_block). Those
   *     blocking interfaces are used to assure that no further messages are
   *     directed to the window. Receipt of the blocked callback signifies
   *     that (1) there are no further pending callbacks and (2) that the
   *     window is now 'defunct' and will receive no further callbacks.
   *
   *     This callback supports coordinated destruction of a window.  In
   *     the multi-user mode, the client window logic must stay intact until
   *     all of the queued callbacks are processed.  Then the window may be
   *     safely closed.  Closing the window prior with pending callbacks can
   *     lead to bad behavior when the callback is executed.
   *
   *   NXEVENT_SYNCHED - Synchronization handshake
   *
   *     This completes the handshake started by nx_synch().  nx_synch()
   *     sends a syncrhonization messages to the NX server which responds
   *     with this event.  The sleeping client is awakened and continues
   *     graphics processing, completing the handshake.
   *
   *     Due to the highly asynchronous nature of client-server
   *     communications, nx_synch() is sometimes necessary to assure that
   *     the client and server are fully synchronized.
   *
   * Input Parameters:
   *   hwnd  - Window handle of window receiving the event
   *   event - The server event
   *   arg1  - User provided argument (see nx_openwindow, nx_requestbkgd,
   *           nxtk_openwindow, or nxtk_opentoolbar)
   *   arg2  - User provided argument (see nx[tk]_block or nx[tk]_synch)
   *
   * Returned Value:
   *   None
   *
   **************************************************************************/

  void (*event)(NXWINDOW hwnd, enum nx_event_e event, FAR void *arg1,
                FAR void *arg2);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nx_vnc_fbinitialize
 *
 * Description:
 *   This is just a wrapper around vnc_fbinitialize() that will establish
 *   the default mouse and keyboard callout functions.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *   handle - And instance of NXHANDLE returned from initialization of the
 *     NX graphics system for that display.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

/* int nx_vnc_fbinitialize(int display, NXHANDLE handle); */

#if defined(CONFIG_NX_KBD) && defined(CONFIG_NX_XYINPUT)

#  define nx_vnc_fbinitialize(d,h) \
  vnc_fbinitialize((d), nx_kbdin, nx_mousein, (FAR void *)(h))

#elif defined(CONFIG_NX_KBD)

#  define nx_vnc_fbinitialize(d,h) \
  vnc_fbinitialize((d), nx_kbdin, NULL, (FAR void *)(h))

#elif defined(CONFIG_NX_XYINPUT)

#  define nx_vnc_fbinitialize(d,h) \
  vnc_fbinitialize((d), NULL, nx_mousein, (FAR void *)(h))

#else

#  define nx_vnc_fbinitialize(d,h) \
  vnc_fbinitialize((d), NULL, NULL, NULL)

#endif

/****************************************************************************
 * Name: nx_runinstance (and nx_run macro)
 *
 * Description:
 *   This is the server entry point.  It does not return; the calling thread
 *   is dedicated to supporting NX server.
 *
 *   NOTE that multiple instances of the NX server may run at the same time,
 *   with different callback and message queue names.  nx_run() is simply
 *   a macro that can be used when only one server instance is required.  In
 *   that case, a default server name is used.
 *
 * Input Parameters:
 *   mqname - The name for the server incoming message queue
 *   dev     - Vtable "object" of the framebuffer "driver" to use
 *
 * Returned Value:
 *   This function usually does not return.  If it does return, it will
 *   return ERROR and errno will be set appropriately.
 *
 ****************************************************************************/

int nx_runinstance(FAR const char *mqname, FAR NX_DRIVERTYPE *dev);
#define nx_run(dev) nx_runinstance(NX_DEFAULT_SERVER_MQNAME, dev)

/****************************************************************************
 * Name: nx_connectinstance (and nx_connect macro)
 *
 * Description:
 *   Open a connection from a client to the NX server.  One one client
 *   connection is normally needed per thread as each connection can host
 *   multiple windows.
 *
 *   NOTES:
 *   - This function returns before the connection is fully instantiated.
 *     it is necessary to wait for the connection event before using the
 *     returned handle.
 *   - Multiple instances of the NX server may run at the same time,
 *     each with different message queue names.
 *   - nx_connect() is simply a macro that can be used when only one
 *     server instance is required.  In that case, a default server name
 *     is used.
 *
 * Input Parameters:
 *   svrmqname - The name for the server incoming message queue
 *
 * Returned Value:
 *   Success: A non-NULL handle used with subsequent NX accesses
 *   Failure:  NULL is returned and errno is set appropriately
 *
 ****************************************************************************/

NXHANDLE nx_connectinstance(FAR const char *svrmqname);
#define nx_connect(cb) nx_connectinstance(NX_DEFAULT_SERVER_MQNAME)

/****************************************************************************
 * Name: nx_disconnect
 *
 * Description:
 *   Disconnect a client from the NX server and/or free resources reserved
 *   by nx_connect/nx_connectinstance.
 *
 * Input Parameters:
 *   handle - the handle returned by nx_connect
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nx_disconnect(NXHANDLE handle);

/****************************************************************************
 * Name: nx_eventhandler
 *
 * Description:
 *   The client code must call this function periodically to process
 *   incoming messages from the server.  If CONFIG_NX_BLOCKING is defined,
 *   then this function not return until a server message is received.
 *
 *   When CONFIG_NX_BLOCKING is not defined, the client must exercise
 *   caution in the looping to assure that it does not eat up all of
 *   the CPU bandwidth calling nx_eventhandler repeatedly.  nx_eventnotify
 *   may be called to get a signal event whenever a new incoming server
 *   event is available.
 *
 * Input Parameters:
 *   handle - the handle returned by nx_connect
 *
 * Returned Value:
 *     OK: No errors occurred.  If CONFIG_NX_BLOCKING is defined, then
 *         one or more server messages were processed.
 *  ERROR: An error occurred and errno has been set appropriately.  Of
 *         particular interest, it will return errno == EHOSTDOWN when the
 *         server is disconnected.  After that event, the handle can no
 *         longer be used.
 *
 ****************************************************************************/

int nx_eventhandler(NXHANDLE handle);

/****************************************************************************
 * Name: nx_eventnotify
 *
 * Description:
 *   Rather than calling nx_eventhandler periodically, the client may
 *   register to receive a signal when a server event is available.  The
 *   client can then call nv_eventhandler() only when incoming events are
 *   available.
 *
 *   Only one such event is issued.  Upon receipt of the signal, if the
 *   client wishes further notifications, it must call nx_eventnotify again.
 *
 * Input Parameters:
 *   handle - the handle returned by nx_connect
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_eventnotify(NXHANDLE handle, int signo);

/****************************************************************************
 * Name: nx_openwindow
 *
 * Description:
 *   Create a new window.
 *
 * Input Parameters:
 *   handle - The handle returned by nx_connect()
 *   flags  - Optional flags.  These include:
 *            NXBE_WINDOW_RAMBACKED:  Creates a RAM backed window.  This
 *              option is only valid if CONFIG_NX_RAMBACKED is enabled.
 *            NXBE_WINDOW_HIDDEN:  The window is create in the HIDDEN state
 *             and can be made visible later with nx_setvisibility().
 *   cb     - Callbacks used to process window events
 *   arg    - User provided value that will be returned with NX callbacks.
 *
 * Returned Value:
 *   Success: A non-NULL handle used with subsequent NX accesses
 *   Failure:  NULL is returned and errno is set appropriately
 *
 ****************************************************************************/

NXWINDOW nx_openwindow(NXHANDLE handle, uint8_t flags,
                       FAR const struct nx_callback_s *cb, FAR void *arg);

/****************************************************************************
 * Name: nx_closewindow
 *
 * Description:
 *   Destroy a window created by nx_openwindow.
 *
 * Input Parameters:
 *   wnd - The window to be destroyed
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_closewindow(NXWINDOW hwnd);

/****************************************************************************
 * Name: nx_block
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
 *   wnd - The window to be blocked
 *   arg - An argument that will accompany the block messages (This is arg2
 *         in the event callback).
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_block(NXWINDOW hwnd, FAR void *arg);

/****************************************************************************
 * Name: nx_synch
 *
 * Description:
 *   This interface can be used to synchronize the window client with the
 *   NX server.  It really just implements an 'echo':  A synch message is
 *   sent from the window client to the server which then responds
 *   immediately by sending the NXEVENT_SYNCHED back to the windows client.
 *
 *   Due to the highly asynchronous nature of client-server communications,
 *   nx_synch() is sometimes necessary to assure that the client and server
 *   are fully synchronized in time.
 *
 *   Usage by the window client might be something like this:
 *
 *     extern bool g_synched;
 *     extern sem_t g_synch_sem;
 *
 *     g_synched = false;
 *     ret = nx_synch(hwnd, handle);
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
 *   wnd - The window to be synched
 *   arg - An argument that will accompany the synch messages (This is arg2
 *         in the event callback).
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_synch(NXWINDOW hwnd, FAR void *arg);

/****************************************************************************
 * Name: nx_requestbkgd
 *
 * Description:
 *   NX normally controls a separate window called the background window.
 *   It repaints the window as necessary using only a solid color fill.  The
 *   background window always represents the entire screen and is always
 *   below other windows.  It is useful for an application to control the
 *   background window in the following conditions:
 *
 *   - If you want to implement a windowless solution.  The single screen
 *     can be used to create a truly simple graphic environment.
 *   - When you want more on the background than a solid color.  For
 *     example, if you want an image in the background, or animations in the
 *     background, or live video, etc.
 *
 *   This API only requests the handle of the background window.  That
 *   handle will be returned asynchronously in a subsequent position and
 *   redraw callbacks.
 *
 *   Cautions:
 *   - The following should never be called using the background window.
 *     They are guaranteed to cause severe crashes:
 *
 *       nx_setposition, nx_setsize, nx_raise, nx_lower.
 *
 *   - Neither nx_requestbkgd or nx_releasebkgd should be called more than
 *     once.  Multiple instances of the background window are not supported.
 *
 * Input Parameters:
 *   handle - The handle returned by nx_connect
 *   cb     - Callbacks to use for processing background window events
 *   arg    - User provided value that will be returned with NX callbacks.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_requestbkgd(NXHANDLE handle, FAR const struct nx_callback_s *cb,
                   FAR void *arg);

/****************************************************************************
 * Name: nx_releasebkgd
 *
 * Description:
 *   Release the background window previously acquired using nx_openbgwindow
 *   and return control of the background to NX.
 *
 * Input Parameters:
 *   hwnd - The handle returned (indirectly) by nx_requestbkgd
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_releasebkgd(NXWINDOW hwnd);

/****************************************************************************
 * Name: nx_getposition
 *
 * Description:
 *  Request the position and size information for the selected window.  The
 *  values will be return asynchronously through the client callback function
 *  pointer.
 *
 * Input Parameters:
 *   hwnd   - The window handle
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_getposition(NXWINDOW hwnd);

/****************************************************************************
 * Name: nx_setposition
 *
 * Description:
 *  Set the position and size for the selected window
 *
 * Input Parameters:
 *   hwnd  - The window handle
 *   pos   - The new position of the window
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_setposition(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos);

/****************************************************************************
 * Name: nx_setsize
 *
 * Description:
 *  Set the size of the selected window
 *
 * Input Parameters:
 *   hwnd   - The window handle
 *   size   - The new size of the window.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_setsize(NXWINDOW hwnd, FAR const struct nxgl_size_s *size);

/****************************************************************************
 * Name: nx_raise
 *
 * Description:
 *   Bring the specified window to the top of the display.
 *
 * Input Parameters:
 *   hwnd - the window to be raised
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_raise(NXWINDOW hwnd);

/****************************************************************************
 * Name: nx_lower
 *
 * Description:
 *   Lower the specified window to the bottom of the display.
 *
 * Input Parameters:
 *   hwnd - The window to be lowered
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_lower(NXWINDOW hwnd);

/****************************************************************************
 * Name: nx_modal
 *
 * Description:
 *   May be used to either (1) raise a window to the top of the display and
 *   select modal behavior, or (2) disable modal behavior.
 *
 * Input Parameters:
 *   hwnd  - The window to be modified
 *   modal - True: enter modal state; False: leave modal state
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_modal(NXWINDOW hwnd, bool modal);

/****************************************************************************
 * Name: nx_setvisibility
 *
 * Description:
 *   Select if the window is visible or hidden.  A hidden window is still
 *   present and will update normally, but will not be visible on the
 *   display until it is unhidden.
 *
 * Input Parameters:
 *   hwnd - The window to be modified
 *   hide - True: Window will be hidden; false: Window will be visible
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_setvisibility(NXWINDOW hwnd, bool hide);

/****************************************************************************
 * Name: nx_ishidden
 *
 * Description:
 *   Return true if the window is hidden.
 *
 *   NOTE:  There will be a delay between the time that the visibility of
 *   the window is changed via nx_setvisibily() before that new setting is
 *   reported by nx_ishidden().  nx_synch() may be used if temporal
 *   synchronization is required.
 *
 * Input Parameters:
 *   hwnd - The window to be queried
 *
 * Returned Value:
 *   True: the window is hidden, false: the window is visible
 *
 ****************************************************************************/

bool nx_ishidden(NXWINDOW hwnd);

/****************************************************************************
 * Name: nx_setpixel
 *
 * Description:
 *  Set a single pixel in the window to the specified color.  This is simply
 *  a degenerate case of nx_fill but may be optimized in some architectures.
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   pos  - The pixel location to be set
 *   col  - The color to use in the set
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_setpixel(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nx_fill
 *
 * Description:
 *  Fill the specified rectangle in the window with the specified color
 *
 * Input Parameters:
 *   hwnd  - The window handle
 *   rect  - The location to be filled
 *   color - The color to use in the fill
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_fill(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
            nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nx_getrectangle
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
 * Input Parameters:
 *   hwnd  - The window handle
 *   rect  - The location to be filled
 *   color - The color to use in the fill
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_getrectangle(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                    unsigned int plane, FAR uint8_t *dest,
                    unsigned int deststride);

/****************************************************************************
 * Name: nx_filltrapezoid
 *
 * Description:
 *  Fill the specified trapezoidal region in the window with the specified
 *  color
 *
 * Input Parameters:
 *   hwnd  - The window handle
 *   clip  - Clipping rectangle relative to window (may be null)
 *   trap  - The trapezoidal region to be filled
 *   color - The color to use in the fill
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_filltrapezoid(NXWINDOW hwnd, FAR const struct nxgl_rect_s *clip,
                     FAR const struct nxgl_trapezoid_s *trap,
                     nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nx_drawline
 *
 * Description:
 *  Fill the specified line in the window with the specified color.  This
 *  is simply a wrapper that uses nxgl_splitline() to break the line into
 *  trapezoids and then calls nx_filltrapezoid() to render the line.
 *
 * Input Parameters:
 *   hwnd   - The window handle
 *   vector - Describes the line to be drawn
 *   width  - The width of the line
 *   color  - The color to use to fill the line
 *   caps   - Draw a circular on the both ends of the line to support better
 *            line joins
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_drawline(NXWINDOW hwnd, FAR struct nxgl_vector_s *vector,
                nxgl_coord_t width, nxgl_mxpixel_t color[CONFIG_NX_NPLANES],
                uint8_t caps);

/****************************************************************************
 * Name: nx_drawcircle
 *
 * Description:
 *  Draw a circular outline using the specified line thickness and color.
 *
 * Input Parameters:
 *   hwnd   - The window handle
 *   center - A pointer to the point that is the center of the circle
 *   radius - The radius of the circle in pixels.
 *   width  - The width of the line
 *   color  - The color to use to fill the line
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_drawcircle(NXWINDOW hwnd, FAR const struct nxgl_point_s *center,
                  nxgl_coord_t radius, nxgl_coord_t width,
                  nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nx_fillcircle
 *
 * Description:
 *  Fill a circular region using the specified color.
 *
 * Input Parameters:
 *   hwnd   - The window handle
 *   center - A pointer to the point that is the center of the circle
 *   radius - The radius of the circle in pixels.
 *   color  - The color to use to fill the circle
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_fillcircle(NXWINDOW hwnd, FAR const struct nxgl_point_s *center,
                  nxgl_coord_t radius,
                  nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nx_setbgcolor
 *
 * Description:
 *  Set the color of the background
 *
 * Input Parameters:
 *   handle  - The connection handle
 *   color - The color to use in the background
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_setbgcolor(NXHANDLE handle, nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nx_move
 *
 * Description:
 *   Move a rectangular region within the window
 *
 * Input Parameters:
 *   hwnd   - The window within which the move is to be done
 *   rect   - Describes the rectangular region to move
 *   offset - The offset to move the region.  The  rectangular region will be
 *            moved so that the origin is translated by this amount.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_move(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
            FAR const struct nxgl_point_s *offset);

/****************************************************************************
 * Name: nx_bitmap
 *
 * Description:
 *   Copy a rectangular region of a larger image into the rectangle in the
 *   specified window.
 *
 * Input Parameters:
 *   hwnd   - The window that will receive the bitmap image
 *   dest   - Describes the rectangular region on the display that will
 *            receive the bit map.
 *   src    - The start of the source image.  This is an array source
 *            images of size CONFIG_NX_NPLANES.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in window coordinates, however, origin
 *            may lie outside of the display.
 *   stride - The width of the full source image in bytes.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_bitmap(NXWINDOW hwnd, FAR const struct nxgl_rect_s *dest,
              FAR const void *src[CONFIG_NX_NPLANES],
              FAR const struct nxgl_point_s *origin, unsigned int stride);

/****************************************************************************
 * Name: nx_kbdin
 *
 * Description:
 *   Used by a thread or interrupt handler that manages some kind of keypad
 *   hardware to report text information to the NX server.  That text
 *   data will be routed by the NX server to the appropriate window client.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_KBD
int nx_kbdchin(NXHANDLE handle, uint8_t ch);
int nx_kbdin(NXHANDLE handle, uint8_t nch, FAR const uint8_t *ch);
#endif

/****************************************************************************
 * Name: nx_mousein
 *
 * Description:
 *   Used by a thread or interrupt handler that manages some kind of pointing
 *   hardware to report new positional data to the NX server. That positional
 *   data will be routed by the NX server to the appropriate window client.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_XYINPUT
int nx_mousein(NXHANDLE handle, nxgl_coord_t x,
               nxgl_coord_t y, uint8_t buttons);
#endif

/****************************************************************************
 * NX-Internal Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_redrawreq
 *
 * Description:
 *   This will cause a NX re-draw callback to the client that owns the
 *   window.  This is not normally called from user code, but may be
 *   used within middle-ware layers when redrawing is needed.
 *
 *   NXTK uses this function, for example, when a change in the main window
 *   necessitates redrawing of the toolbar window.
 *
 * Input Parameters:
 *   hwnd - Window handle
 *   rect - The rectangle that needs to be re-drawn (in window relative
 *          coordinates)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nx_redrawreq(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect);

/****************************************************************************
 * Name: nx_constructwindow
 *
 * Description:
 *   This function is the same as nx_openwindow EXCEPT the client provides
 *   the window structure instance.  nx_constructwindow will initialize the
 *   the pre-allocated window structure for use by NX.  This function is
 *   provided in addition to nx_openwindow in order to support a kind of
 *   inheritance:  The caller's window structure may include extensions that
 *   are not visible to NX.
 *
 *   NOTE:  hwnd must have been allocated using a user-space allocator that
 *   permits user access to the window.  Once provided to nx_constructwindow
 *   that memory is owned and managed by NX.  On certain error conditions or
 *   when the window is closed, NX will free the window.
 *
 * Input Parameters:
 *   handle - The handle returned by nx_connect
 *   hwnd   - The pre-allocated window structure.
 *   flags  - Optional flags.  Must be zero unless CONFIG_NX_RAMBACKED is
 *            enabled.  In that case, it may be zero or
 *            NXBE_WINDOW_RAMBACKED
 *   cb     - Callbacks used to process window events
 *   arg    - User provided value that will be returned with NX callbacks.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately.  In the
 *   case of ERROR, NX will have deallocated the pre-allocated window.
 *
 ****************************************************************************/

int nx_constructwindow(NXHANDLE handle, NXWINDOW hwnd, uint8_t flags,
                       FAR const struct nx_callback_s *cb, FAR void *arg);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_NX_NX_H */
