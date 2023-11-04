==
NX
==

Overview
========

NX provides a tiny windowing system in the spirit of X, but greatly scaled
down and appropriate for most resource-limited embedded environments.
The current NX implementation supports the general following, high-level
features:

* **Virtual Vertical Graphics Space** Windows that reside in a virtual,
  vertical space so that it makes sense to talk about one window being
  on top of another and obscuring the window below it.

* **Client/Server Model** A standard client server/model was adopted.
  NX may be considered a server and other logic that presents the windows
  are NX clients.

* **Multi-User Support** NX includes front-end logic to an NX server
  daemon that can serve multiple NX client threads. The NX sever
  thread/daemon serializes graphics operations from multiple clients.
  
* **Minimal Graphics Toolset** The actual implementation of the graphics
  operations is performed by common, back-end logic. This back-end supports
  only a primitive set of graphic and rendering operations.

* **Device Interface** NX supports any graphics device either of two
  device interfaces:

  #. Any device with random accesss video memory using the NuttX framebuffer
     driver interface (see include/nuttx/video/fb.h).
  #. Any LCD-like device than can accept raster line runs through a parallel
     or serial interface (see include/nuttx/lcd/lcd.h). By default, NX is
     configured to use the frame buffer driver unless CONFIG_NX_LCDDRIVER
     is defined =y in your NuttX configuration file.

* **Transparent to NX Client** The window client on "sees" the sub-window
  that is operates in and does not need to be concerned with the virtual,
  vertical space (other that to respond to redraw requests from NX when needed).

* **Framed Windows and Toolbars** NX also adds the capability to support
  windows with frames and toolbars on top of the basic windowing support.
  These are windows such as those shown in the screenshot above. These framed
  windows sub-divide one one window into three relatively independent
  subwindows: A frame, the contained window and an (optional) toolbar window.

* **Mouse Support** NX provides support for a mouse or other X/Y pointing
  devices. APIs are provided to allow external devices to give X/Y position
  information and mouse button presses to NX. NX will then provide the mouse
  input to the relevant window clients via callbacks. Client windows only
  receive the mouse input callback if the mouse is positioned over a visible
  portion of the client window; X/Y position is provided to the client in the
  relative coordinate system of the client window.

* **Keyboard input** NX also supports keyboard/keypad devices. APIs are provided
  to allow external devices to give keypad information to NX. NX will then
  provide the mouse input to the top window on the display (the window that
  has the focus) via a callback function.

Pre-Processor Definitions
=========================

The default server message queue name used by the :c:macro:`nx_run` macro:

.. code-block:: c

  #define NX_DEFAULT_SERVER_MQNAME "/dev/nxs"

Mouse button bits:

.. code-block:: c

  #define NX_MOUSE_NOBUTTONS    0x00
  #define NX_MOUSE_LEFTBUTTON   0x01
  #define NX_MOUSE_CENTERBUTTON 0x02
  #define NX_MOUSE_RIGHTBUTTON  0x04

NX Types
========

The interface to the NX server is managed using a opaque handle:

.. c:type:: FAR void *NXHANDLE

The interface to a specific window is managed using an opaque handle:

.. c:type:: FAR void *NXWINDOW

These define callbacks that must be provided to :c:func:`nx_openwindow`.
These callbacks will be invoked as part of the processing performed by
:c:func:`nx_eventhandler`.

.. c:struct:: nx_callback_s

  .. code-block:: c

    struct nx_callback_s
    {
      void (*redraw)(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                     bool more, FAR void *arg);
      void (*position)(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                       FAR const struct nxgl_point_s *pos,
                       FAR const struct nxgl_rect_s *bounds,
                       FAR void *arg);
    #ifdef CONFIG_NX_XYINPUT
      void (*mousein)(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                      uint8_t buttons, FAR void *arg);
    #endif
    #ifdef CONFIG_NX_KBD
      void (*kbdin)(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch, FAR void *arg);
    #endif
    };

Starting the NX Server
======================

The *NX Server* is a kernel daemon that receives and serializes graphic
commands. Before you can use the NX graphics system, you must first
start this daemon. There are two ways that this can be done:

#. The NX server may be started in your board startup logic by simply
   calling the function ``nxmu_start()``. The board startup logic
   usually resides the the ``boards/arch/chip/board/src`` directory. The
   board startup logic can run automatically during the early system if
   ``CONFIG_BOARD_LATE_INITIALIZE`` is defined in the configuration. Or,
   the board startup logic can execute under control of the application
   by calling :c:func:`boardctl` as:

   .. code-block:: c

     boardctl(BOARDIOC_INIT, arg)

   The board initialization logic will run in either case and the simple
   call to ``nxmu_start()`` will start the NX server.

#. The NX server may also be started later by the application via
   :c:func:`boardctl` as:

   .. code-block:: c

     boardctl(BOARDIOC_NX_START, arg)

.. c:function:: int nxmu_start(int display, int plane);

  Provides a wrapper function to
  simplify and standardize the starting of the NX server.

  :param display: The display number to be served by this new NXMU instance.
  :param plane: The plane number to use to get information about the display geometry and color format.

  :return: Zero (``OK``) is returned on success. This indicates
    that the NX server has been successfully started, is running, and
    waiting to accept connections from NX clients.
    A negated ``errno`` value is returned on failure. The ``errno`` value
    indicates the nature of the failure.

NX Server Callbacks
===================

.. c:function:: void redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect, bool more, FAR void *arg);

  NX requests that the client re-draw the portion of the
  window within with rectangle.

  :param hwnd:
     The handle created by :c:func:`nx_openwindow` or :c:func:`nx_requestbkgd`
  :param rect:
     The rectangle that needs to be re-drawn (in window relative
     coordinates)
  :param more:
     true: More re-draw requests will follow
  :param arg:
     User provided argument (see :c:func:`nx_openwindow`)

.. c:function:: void position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size, \
              FAR const struct nxgl_point_s *pos, \
              FAR const struct nxgl_rect_s *bounds, \
              FAR void *arg);

  The size or position of the window has changed (or the
  window was just created with zero size.

  :param hwnd:
     The handle created by :c:func:`nx_openwindow` or :c:func:`nx_requestbkgd`
  :param size:
     The size of the window
  :param pos:
     The position of the upper left hand corner of the window on the
     overall display
  :param bounds:
     The bounding rectangle that the describes the entire display
  :param arg:
     User provided argument (see :c:func:`nx_openwindow`)

.. c:function:: void mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos, \
             uint8_t buttons, FAR void *arg);

  New mouse data is available for the window

  :param hwnd:
     The handle created by :c:func:`nx_openwindow` or :c:func:`nx_requestbkgd`
  :param pos:
     The (x,y) position of the mouse
  :param buttons:
     See ``NX_MOUSE_*`` definitions
  :param arg:
     User provided argument (see :c:func:`nx_openwindow`)

.. c:var:: void (*kbdin)(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch, FAR void *arg);

  New keyboard/keypad data is available for the window.

  :param hwnd:
       The handle created by :c:func:`nx_openwindow` or :c:func:`nx_requestbkgd`
  :param nch:
     The number of characters that are available in ch[]
  :param ch:
     The array of characters
  :param arg:
     User provided argument (see :c:func:`nx_openwindow`)

.. c:var:: void (*event)(NXWINDOW hwnd, enum nx_event_e event, FAR void *arg1, FAR void *arg2);

  This callback is used to communicate server events to the window listener.

  - ``NXEVENT_BLOCKED``: Window messages are blocked.
     This callback is the response from :c:func:`nx_block`,
     :c:func:`nxtk_block`. Those blocking interfaces are used
     to assure that no further messages are directed to the window.
     Receipt of the blocked callback signifies that (1) there are no
     further pending callbacks and (2) that the window is now *defunct*
     and will receive no further callbacks. This callback supports
     coordinated destruction of a window. In the multi-user mode, the
     client window logic must stay intact until all of the queued
     callbacks are processed. Then the window may be safely closed.
     Closing the window prior with pending callbacks can lead to bad
     behavior when the callback is executed.
  - ``NXEVENT_SYNCHED``: Synchronization handshake
     This completes the handshake started by
     :c:func:`nx_synch`, or :c:func:`nxtk_synch`.
     Those interfaces send a synchronization messages to the NX server
     which responds with this event. The sleeping client is awakened and
     continues graphics processing, completing the handshake. Due to the
     highly asynchronous nature of client-server communications,
     synchronization is sometimes necessary to assure that the client and
     server are working together properly.

  :param hwnd:
     TWindow handle of window receiving the event
  :param event:
     The server event
  :param arg1:
     User provided argument (see :c:func:`nx_openwindow`,
     :c:func:`nx_requestbkgd`, or :c:func:`nxtk_opentoolbar`)
  :param arg2:
     TUser provided argument (see :c:func:`nx_block`, :c:func:`nxtk_block`,
     :c:func:`nx_synch`, or :c:func:`nxtk_synch`)

.. c:macro:: nx_run(fb)

  .. code-block:: c

    #define nx_run(fb) nx_runinstance(NX_DEFAULT_SERVER_MQNAME, dev)

.. c:function:: int nx_runinstance(FAR const char *mqname, FAR struct fb_vtable_s *fb)

  This is the server entry point. It does not return; the
  calling thread is dedicated to supporting NX server.

  NOTE that multiple instances of the NX server may run at the same time,
  with different callback and message queue names. ``nx_run()`` is simply
  a macro that can be used when only one server instance is required. In
  that case, a default server name is used.

  :param mqname: The name for the server incoming message queue
  :param dev: Framebuffer or LCD driver "object" to be used

  :return: This function usually does not return. If it does
    return, it will return ``ERROR`` and ``errno`` will be set
    appropriately.

.. c:macro:: nx_connect(cb)

  .. code-block:: c

    #define nx_connect(cb) nx_connectinstance(NX_DEFAULT_SERVER_MQNAME)

.. c:function:: NXHANDLE nx_connectinstance(FAR const char *svrmqname);

  Open a connection from a client to the NX server. One
  one client connection is normally needed per thread as each connection
  can host multiple windows.

  NOTES:

  -  This function returns before the connection is fully instantiated. it
     is necessary to wait for the connection event before using the
     returned handle.
  -  Multiple instances of the NX server may run at the same time, each
     with different message queue names.
  -  ``nx_connect()`` is simply a macro that can be used when only one
     server instance is required. In that case, a default server name is
     used.

  :param svrmqname: The name for the server incoming message queue

  :return: Success: A non-NULL handle used with subsequent NX accesses
    Failure: NULL is returned and errno is set appropriately.

.. c:function:: void nx_disconnect(NXHANDLE handle)

  Disconnect a client from the NX server and/or free
  resources reserved by :c:func:`nx_connect`/c:func:`nx_connectinstance`.

  :param handle: The handle returned by :c:func:`nx_connectinstance`.

.. c:function:: int nx_eventhandler(NXHANDLE handle);

  The client code must call this function periodically to
  process incoming messages from the server. If ``CONFIG_NX_BLOCKING`` is
  defined, then this function not return until a server message is
  received.

  When ``CONFIG_NX_BLOCKING`` is not defined, the client must exercise
  caution in the looping to assure that it does not eat up all of the CPU
  bandwidth calling nx_eventhandler repeatedly.
  ```nx_eventnotify()`` <#nxeventnotify>`__ may be called to get a signal
  event whenever a new incoming server event is available.

  :param handle: The handle returned by ```nx_connect()`` <#nxconnectinstance>`__.

  :return:
    -  ``OK``: No errors occurred. If ``CONFIG_NX_BLOCKING`` is defined,
       then one or more server messages were processed.
    -  ``ERROR``: An error occurred and ``errno`` has been set
       appropriately. Of particular interest, it will return
       ``errno == EHOSTDOWN`` when the server is disconnected. After that
       event, the handle can no longer be used.

.. c:function:: int nx_eventnotify(NXHANDLE handle, int signo);

  Rather than calling :c:func:`nx_eventhandler` periodically, the client may
  register to receive a signal when a server event is available. The
  client can then call :c:func:nx_eventhandler` only
  when incoming events are available.

  The underlying implementation used ``mq_notifiy()`` and, as a result,
  the client must observe the rules for using ``mq_notifiy()``:

  -  Only one event is signalled. Upon receipt of the signal, if the
     client wishes further notifications, it must call
     ``nx_eventnotify()`` again.
  -  The signal will only be issued when the message queue transitions
     from empty to not empty.

  :param handle: The handle returned by ```nx_connect()`` <#nxconnectinstance>`__.
  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nx_block(NXWINDOW hwnd, FAR void *arg);

  The response to this function call is two things: (1)
  any queued callback messages to the window are 'blocked' and then (2)
  also subsequent window messaging is blocked.

  The ``event`` callback with the ``NXEVENT_BLOCKED`` event is the
  response from ``nx_block()``. This blocking interface is used to assure
  that no further messages are are directed to the window. Receipt of the
  ``NXEVENT_BLOCKED`` event signifies that (1) there are no further
  pending callbacks and (2) that the window is now *defunct* and will
  receive no further callbacks.

  This callback supports coordinated destruction of a window. The client
  window logic must stay intact until all of the queued callbacks are
  processed. Then the window may be safely closed. Closing the window
  prior with pending callbacks can lead to bad behavior when the callback
  is executed.

  :param wnd: The window to be blocked
  :param arg: An argument that will accompany the block messages (This is ``arg2`` in
    the event callback).

  :return: OK on success; ERROR on failure with errno set
    appropriately.

.. c:function:: int nx_synch(NXWINDOW hwnd, FAR void *arg);

  This interface can be used to synchronize the window
  client with the NX server. It really just implements an *echo*: A synch
  message is sent from the window client to the server which then responds
  immediately by sending the ``NXEVENT_SYNCHED`` back to the windows
  client.

  Due to the highly asynchronous nature of client-server communications,
  ``nx_synch()`` is sometimes necessary to assure that the client and
  server are fully synchronized in time.

  Usage by the window client might be something like this:

  .. code-block:: c

    extern bool g_synched;
    extern sem_t g_synch_sem;

    g_synched = false;
    ret = nx_synch(hwnd, handle);
    if (ret < 0)
      {
         -- Handle the error --
      }

    while (!g_synched)
      {
        ret = sem_wait(&g_sync_sem);
        if (ret < 0)
          {
             -- Handle the error --
          }
      }

  When the window listener thread receives the ``NXEVENT_SYNCHED`` event,
  it would set ``g_synched`` to ``true`` and post ``g_synch_sem``, waking
  up the above loop.

  :param wnd: The window to be synched
  :param arg: An argument that will accompany the synch messages (This is ``arg2`` in the event callback).

  :return: OK on success; ERROR on failure with errno set
    appropriately

.. c:function:: NXWINDOW nx_openwindow(NXHANDLE handle, uint8_t flags, \
                       FAR const struct nx_callback_s *cb, \
                       FAR void *arg);

  Create a new window.

  :param handle: The handle returned by ```nx_connect()`` <#nxconnectinstance>`__.
  :param flags: Optional flags. These include:
    - ``NXBE_WINDOW_RAMBACKED``: Creates a RAM backed window. This option is only valid if ``CONFIG_NX_RAMBACKED`` is enabled.
    - ``NXBE_WINDOW_HIDDEN``: The window is create in the HIDDEN state and can be made visible later with ``nx_setvisibility()``.

  :param cb: Callbacks used to process window events
  :param arg: User provided value that will be returned with NX callbacks.

  :return: Success: A non-NULL handle used with subsequent NX accesses
    Failure: NULL is returned and errno is set appropriately.

.. c:function:: int nx_closewindow(NXWINDOW hwnd)

  Destroy a window created by :c:func:`nx_openwindow` window.

  :param hwnd: The handle returned by ```nx_openwindow()`` <#nxopenwindow>`__ that
    identifies the window to be destroyed. This handle must not have been
    one returned by ```nx_requestbkgd()`` <#nxrequestbkgd>`__.

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nx_requestbkgd(NXHANDLE handle, \
                   FAR const struct nx_callback_s *cb, \
                   FAR void *arg);

  NX normally controls a separate window called the
  background window. It repaints the window as necessary using only a
  solid color fill. The background window always represents the entire
  screen and is always below other windows. It is useful for an
  application to control the background window in the following
  conditions:

  -  If you want to implement a windowless solution. The single screen can
     be used to create a truly simple graphic environment.
  -  When you want more on the background than a solid color. For example,
     if you want an image in the background, or animations in the
     background, or live video, etc.

  This API only requests the handle of the background window. That handle
  will be returned asynchronously in a subsequent position and redraw
  callbacks.

  Cautions:

  -  The following should never be called using the background window.
     They are guaranteed to cause severe crashes: :c:func:`nx_setposition`,
     :c:func:`nx_setsize`, :c:func:`nx_raise`, or :c:func:`nx_lower`,
     :c:func:`nx_modal`, :c:func:`nx_setvisibility`.
  -  Neither :c:func:`nx_requestbkgd` nor :c:func:`nx_releasebkgd`
     should be called more than once. Multiple instances of the
     background window are not supported.

  :param handle: The handle returned by ```nx_connect()`` <#nxconnectinstance>`__.
  :param cb: Callbacks to use for processing background window events
  :param arg: User provided argument (see ```nx_openwindow()`` <#nxopenwindow>`__)

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nx_releasebkgd(NXWINDOW hwnd)

  Release the background window previously acquired using
  :c:func:`nx_requestbkgd` and return control of the background to NX.

  :param handle: The handle returned indirectly by :c:func:`nx_requestbkgd`.
    This handle must not have been one created by :c:func:`nx_openwindow`.

  :return: ``OK`` on success; ``ERROR`` on failure with ``errno`` set appropriately

.. c:function:: int nx_getposition(NXWINDOW hwnd)

  Request the position and size information for the
  selected window. The values will be return asynchronously through the
  client callback function pointer.

  :param hwnd: The handle returned by :c:func:`nx_openwindow` or
    :c:func:`nx_requestbkgd`.

  :return: ``OK`` on success; ``ERROR`` on failure with ``errno`` set appropriately

.. c:function:: int nx_setposition(NXWINDOW hwnd, FAR struct nxgl_point_s *pos)

  Set the position and size for the selected window.

  :param hwnd: The handle returned by :c:func:`nx_openwindow`. This
    handle must not have been created by :c:func:`nx_requestbkgd`.
  :param pos: The new position of the window

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nx_setsize(NXWINDOW hwnd, FAR struct nxgl_size_s *size)

  Set the size of the selected window.

  :param hwnd: The handle returned by :c:func:`nx_openwindow`. This
    handle must not have been created by :c:func:`nx_requestbkgd`.
  :param size: The new size of the window (in pixels).

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nx_raise(NXWINDOW hwnd)

  Bring the specified window to the top of the display.

  :param hwnd: The handle returned by :c:func:`nx_openwindow`. This
    handle must not have been created by :c:func:`nx_requestbkgd`.

  :return: ``OK`` on success; ``ERROR`` on failure with ``errno`` set appropriately

.. c:function:: int nx_lower(NXWINDOW hwnd);

  Lower the specified window to the bottom of the display.

  :param hwnd: The handle returned by :c:func:`nx_openwindow`. This
    handle must not have been created by :c:func:`nx_requestbkgd`.

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nx_modal(NXWINDOW hwnd, bool modal)

  May be used to either (1) raise a window to the top of
  the display and select modal behavior, or (2) disable modal behavior.

  :param hwnd: The handle returned by :c:func:`nx_openwindow`. This
    handle must not have been created by :c:func:`nx_requestbkgd`.
  :param modal: True: enter modal state; False: leave modal state

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nx_setvisibility(NXWINDOW hwnd, bool hide);

  Select if the window is visible or hidden. A hidden
  window is still present and will update normally, but will not be
  visible on the display until it is unhidden.

  :param hwnd: The handle returned by :c:func:`nx_openwindow`. This
    handle must not have been created by :c:func:`nx_requestbkgd`.
  :param hide: True: Window will be hidden; false: Window will be visible

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: bool nx_ishidden(NXWINDOW hwnd);

  Return true if the window is hidden.

  **NOTE**: There will be a delay between the time that the visibility of
  the window is changed via :c:func:`nx_setvisibily`
  before that new setting is reported by :c:func:`nx_ishidden`. ``nx_synch()``
  may be used if temporal synchronization is required.

  :param hwnd: The handle returned by :c:func:`nx_openwindow` that
    identifies the window to be queried.

  :return: *True*: the window is hidden, *false*: the window is
    visible

.. c:function:: int nx_fill(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect, \
                   nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

  Fill the specified rectangle in the window with the
  specified color.

  :param hwnd: The handle returned by ```nx_openwindow()`` <#nxopenwindow>`__ or
    ```nx_requestbkgd()`` <#nxrequestbkgd>`__
  :param rect: The location to be filled
  :param color: The color to use in the fill

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: void nx_getrectangle(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect, \
                     unsigned int plane, FAR uint8_t *dest, \
                     unsigned int deststride);

  Get the raw contents of graphic memory within a
  rectangular region. NOTE: Since raw graphic memory is returned, the
  returned memory content may be the memory of windows above this one and
  may not necessarily belong to this window unless you assure that this is
  the top window.

  :param hwnd: The handle returned by :c:func:`nx_openwindow` or
    :c:func:`nx_requestbkgd`
  :param rect: The location to be copied
  :param plane: Specifies the color plane to get from
  :param dest: The location to copy the memory region
  :param deststride: The width, in bytes, of the dest memory

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nx_filltrapezoid(NXWINDOW hwnd, FAR const struct nxgl_rect_s *clip, \
                            FAR const struct nxgl_trapezoid_s *trap, \
                            nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

  Fill the specified trapezoidal region in the window
  with the specified color.

  :param hwnd: The handle returned by :c:func:`nx_openwindow` or
    :c:func:`nx_requestbkgd`
  :param clip: Clipping rectangle relative to window (may be null)
  :param trap: The trapezoidal region to be filled
  :param color: The color to use in the fill

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nx_drawline(NXWINDOW hwnd, FAR struct nxgl_vector_s *vector, \
               nxgl_coord_t width, nxgl_mxpixel_t color[CONFIG_NX_NPLANES], \
               uint8_t caps);

  Fill the specified trapezoidal region in the window
  with the specified color. Fill the specified line in the window with the
  specified color. This is simply a wrapper that uses :c:func:`nxgl_splitline`
  to break the line into trapezoids and then calls :c:func:`nx_filltrapezoid`
  to render the line.

  :param hwnd: The handle returned by :c:func:`nx_openwindow` or
    :c:func:`nx_requestbkgd`
  :param vector: Describes the line to be drawn.
  :param width: The width of the line
  :param color: The color to use to fill the line
  :param caps: Draw a circular cap on the ends of the line to support better line
    joins. One of::

      /* Line caps */

      #define NX_LINECAP_NONE  0x00, /* No line caps */
      #define NX_LINECAP_PT1   0x01  /* Line cap on pt1 on of the vector only */
      #define NX_LINECAP_PT2   0x02  /* Line cap on pt2 on of the vector only */
      #define NX_LINECAP_BOTH  0x03  /* Line cap on both ends of the vector only */

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nx_drawcircle(NXWINDOW hwnd, FAR const struct nxgl_point_s *center, \
                  nxgl_coord_t radius, nxgl_coord_t width, \
                  nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

  Draw a circular outline using the specified line
  thickness and color.

  :param hwnd: The handle returned by :c:func:`nx_openwindow` or
    :c:func:`nx_requestbkgd`
  :param center: A pointer to the point that is the center of the circle.
  :param radius: The radius of the circle in pixels.
  :param width: The width of the line
  :param color: The color to use to fill the line

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nx_fillcircle(NXWINDOW hwnd, FAR const struct nxgl_point_s *center, \
                  nxgl_coord_t radius, nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

  Fill a circular region using the specified color.

  :param hwnd: The handle returned by :c:func:`nx_openwindow` or
    :c:func:`nx_requestbkgd`
  :param center: A pointer to the point that is the center of the circle.
  :param radius: The width of the line
  :param color: The color to use to fill the circle

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nx_setbgcolor(NXHANDLE handle, \
                  nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

Set the color of the background.

:param handle: The handle created by :c:func:`nx_openwindow` or
  :c:func:`nx_requestbkgd`
:param color: The color to use in the background

:return: ``OK`` on success; ``ERROR`` on failure with
  ``errno`` set appropriately

.. c:function:: int nx_move(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,  \
                           FAR const struct nxgl_point_s *offset);

Move a rectangular region within the window.

:param hwnd: The handle returned by :c:func:`nx_openwindow` or
  :c:func:`nx_requestbkgd` that specifies the window within which the move is to be done
:param rect: Describes the (source) rectangular region to move
:param offset: The offset to move the region

:return: ``OK`` on success; ``ERROR`` on failure with ``errno`` set appropriately

.. c:function:: int nx_bitmap(NXWINDOW hwnd, FAR const struct nxgl_rect_s *dest, \
                     FAR const void *src[CONFIG_NX_NPLANES], \
                     FAR const struct nxgl_point_s *origin, \
                     unsigned int stride);

  Copy a rectangular region of a larger image into the
  rectangle in the specified window.

  :param hwnd: The handle returned by :c:func:`nx_openwindow` or
    :c:func:`nx_requestbkgd` that specifies the window that will receive the bitmap image.
  :param dest: Describes the rectangular on the display that will receive the bit map.
  :param src: The start of the source image. This is an array source images of size ``CONFIG_NX_NPLANES`` (probably 1).
  :param origin: The origin of the upper, left-most corner of the full bitmap. Both
    dest and origin are in window coordinates, however, the origin may
    lie outside of the display.
  :param stride: The width of the full source image in bytes.

  :return: ``OK`` on success; ``ERROR`` on failure with ``errno`` set appropriately

.. c:function:: int nx_kbdchin(NXHANDLE handle, uint8_t ch);
.. c:function:: int nx_kbdin(NXHANDLE handle, uint8_t nch, FAR const uint8_t *ch);

  Used by a thread or interrupt handler that manages some
  kind of keypad hardware to report text information to the NX server.
  That text data will be routed by the NX server to the appropriate window
  client.

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nx_mousein(NXHANDLE handle, nxgl_coord_t x, nxgl_coord_t y, uint8_t buttons)

  Used by a thread or interrupt handler that manages some
  kind of pointing hardware to report new positional data to the NX
  server. That positional data will be routed by the NX server to the
  appropriate window client.

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. _nx-tool-kit-nxtk-1:

