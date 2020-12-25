======================
NX Tool Kit (``NXTK``)
======================

NXTK implements where the *framed window*. NX framed windows consist of
three components within one NX window:

  #. The window *border*,
  #. The main *client window* area, and
  #. A *toolbar* area

Each sub-window represents a region within one window. `Figure
1 <#screenshot>`__ shows some simple NX framed windows. NXTK allows
these sub-windows to be managed more-or-less independently:

  -  Each component has its own callbacks for redraw and position events
     as well as mouse and keyboard inputs. The client sub-window callbacks
     are registered when the framed window is created with a call to
     :c:func:`nxtk_openwindow`; Separate toolbar
     sub-window callbacks are reigistered when the toolbar is added using
     :c:func:`nxtk_opentoolbar`. (NOTES: (1) only the
     client sub-window receives keyboard input and, (2) border callbacks
     are not currently accessible by the user).
  -  All position informational provided within the callback is relative
     to the specific sub-window. That is, the origin (0,0) of the
     coordinate system for each sub-window begins at the top left
     corner of the subwindow. This means that toolbar logic need not
     be concerned about client window geometry (and vice versa) and,
     for example, common toolbar logic can be used with different windows.

.. c:type:: FAR void *NXTKWINDOW

  This is the handle that can be used to access the window data region.

.. c:function:: int nxtk_block(NXWINDOW hwnd, FAR void *arg)

  The response to this function call is two things: (1)
  any queued callback messages to the window are 'blocked' and then (2)
  also subsequent window messaging is blocked.

  The ``event`` callback with the ``NXEVENT_BLOCKED`` event is the
  response from ``nxtk_block()``. This blocking interface is used to
  assure that no further messages are are directed to the window. Receipt
  of the ``NXEVENT_BLOCKED`` event signifies that (1) there are no further
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

.. c:function:: int nxtk_synch(NXWINDOW hwnd, FAR void *arg);

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
    ret = nxtk_synch(hfwnd, handle);
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

  :param wnd:
     The window to be synched
  :param arg:
     An argument that will accompany the synch messages (This is ``arg2``
     in the event callback).

  :return: OK on success; ERROR on failure with errno set
    appropriately

.. c:function:: NXTKWINDOW nxtk_openwindow(NXHANDLE handle, uint8_t flags, \
                           FAR const struct nx_callback_s *cb, \
                           FAR void *arg);

  Create a new, framed window.

  :param handle:
     The handle returned by ```nx_connect()`` <#nxconnectinstance>`__.
  :param flags:
     Optional flags. These include:

     -  ``NXBE_WINDOW_RAMBACKED``: Creates a RAM backed window. This
        option is only valid if ``CONFIG_NX_RAMBACKED`` is enabled.
     -  ``NXBE_WINDOW_HIDDEN``: The window is create in the HIDDEN state
        and can be made visible later with ``nxtk_setvisibility()``.

  :param cb:
     Callbacks used to process window events
  :param arg:
     User provided argument (see ```nx_openwindow()`` <#nxopenwindow>`__)

  :return: Success: A non-NULL handle used with subsequent NXTK window accesses
    Failure: NULL is returned and errno is set appropriately.

.. c:function:: int nxtk_closewindow(NXTKWINDOW hfwnd);

  Close the window opened by
  ```nxtk_openwindow()`` <#nxtkopenwindow>`__.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__.

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_getposition(NXTKWINDOW hfwnd);

  Request the position and size information for the
  selected framed window. The size/position for the client window and
  toolbar will be return asynchronously through the client callback
  function pointer.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__.

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_setposition(NXTKWINDOW hfwnd, FAR struct nxgl_point_s *pos);

  Set the position for the selected client window. This
  position does not include the offsets for the borders nor for any
  toolbar. Those offsets will be added in to set the full window position.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
  :param pos:
     The new position of the client sub-window

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_setsize(NXTKWINDOW hfwnd, FAR struct nxgl_size_s *size);

  Set the size for the selected client window. This size
  does not include the sizes of the borders nor for any toolbar. Those
  sizes will be added in to set the full window size.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
  :param size:
     The new size of the client sub-window.

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_raise(NXTKWINDOW hfwnd);

  Bring the window containing the specified client
  sub-window to the top of the display.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__ specifying the window to
     be raised.

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_lower(NXTKWINDOW hfwnd);

  Lower the window containing the specified client
  sub-window to the bottom of the display.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__ specifying the window to
     be lowered.

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_modal(NXWINDOW hwnd, bool modal);

  May be used to either (1) raise a window to the top of
  the display and select modal behavior, or (2) disable modal behavior.

  :param hwnd:
     The handle returned by ```nxtk_openwindow()`` <#nxtkopenwindow>`__
     specifying the window to be modified.
  :param modal:
     True: enter modal state; False: leave modal state

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_setvisibility(NXWINDOW hwnd, bool hide);

  Select if the window is visible or hidden. A hidden
  window is still present and will update normally, but will not be
  visible on the display until it is unhidden.

  :param hwnd:
     The handle returned by ```nxtk_openwindow()`` <#nxtkopenwindow>`__
     specifying the window to be modified.
  :param hide:
     True: Window will be hidden; false: Window will be visible

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: bool nxtk_ishidden(NXTKWINDOW hfwnd);

  Return true if the window is hidden.

  **NOTE**: There will be a delay between the time that the visibility of
  the window is changed via
  ```nxtk_setvisibily()`` <#nxtksetvisibility>`__ before that new setting
  is reported by ``nxtk_ishidden()``. ``nxtk_synch()`` may be used if
  temporal synchronization is required.

  :param hfwnd:
     The handle returned by ```nxtk_openwindow()`` <#nxtkopenwindow>`__
     that identifies the window to be queried.

  :return: *True*: the window is hidden, *false*: the window is
    visible

.. c:function:: int nxtk_fillwindow(NXTKWINDOW hfwnd, FAR const struct nxgl_rect_s *rect, \
                    nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

  Fill the specified rectangle in the client window with
  the specified color.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
  :param rect:
     The location within the client window to be filled
  :param color:
     The color to use in the fill

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: void nxtk_getwindow(NXTKWINDOW hfwnd, FAR const struct nxgl_rect_s *rect, \
                    unsigned int plane, FAR uint8_t *dest, \
                    unsigned int deststride);

  Get the raw contents of graphic memory within a
  rectangular region. NOTE: Since raw graphic memory is returned, the
  returned memory content may be the memory of windows above this one and
  may not necessarily belong to this window unless you assure that this is
  the top window.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
  :param rect:
     The location within the client window to be retrieved.
  :param plane:
     Specifies the color plane to get from.
  :param dest:
     The location to copy the memory region
  :param deststride:
     The width, in bytes, of the dest memory

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_filltrapwindow(NXTKWINDOW hfwnd, \
                        FAR const struct nxgl_trapezoid_s *trap, \
                        nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

  Fill the specified trapezoid in the client window with
  the specified color

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
  :param trap:
     The trapezoidal region to be filled.
  :param color:
     The color to use in the fill.

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_drawlinewindow(NXTKWINDOW hfwnd, FAR struct nxgl_vector_s *vector, \
                        nxgl_coord_t width, nxgl_mxpixel_t color[CONFIG_NX_NPLANES], \
                        uint8_t caps);

  Fill the specified trapezoidal region in the window
  with the specified color. Fill the specified line in the window with the
  specified color. This is simply a wrapper that uses ``nxgl_splitline()``
  to break the line into trapezoids and then calls
  ``nxtk_filltrapwindow()`` to render the line.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
  :param vector:
     Describes the line to be drawn.
  :param width:
     The width of the line
  :param color:
     The color to use to fill the line
  :param caps:
     Draw a circular cap on the ends of the line to support better line
     joins. One of:

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_drawcirclewindow(NXTKWINDOW hfwnd, FAR const struct nxgl_point_s *center, \
                          nxgl_coord_t radius, nxgl_coord_t width, \
                          nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

  Draw a circular outline using the specified line
  thickness and color.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
  :param center:
     A pointer to the point that is the center of the circle.
  :param radius:
     The radius of the circle in pixels.
  :param width:
     The width of the line
  :param color:
     The color to use to fill the line

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_fillcirclewindow(NXWINDOW hfwnd, FAR const struct nxgl_point_s *center, \
                          nxgl_coord_t radius, nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

  Fill a circular region using the specified color.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
  :param center:
     A pointer to the point that is the center of the circle.
  :param radius:
     The width of the line
  :param color:
     The color to use to fill the circle

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_movewindow(NXTKWINDOW hfwnd, FAR const struct nxgl_rect_s *rect, \
                    FAR const struct nxgl_point_s *offset);

  Move a rectangular region within the client sub-window
  of a framed window.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__ specifying the client
     sub-window within which the move is to be done.
  :param rect:
     Describes the rectangular region relative to the client sub-window to
     move.
  :param offset:
     The offset to move the region

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_bitmapwindow(NXTKWINDOW hfwnd, \
                      FAR const struct nxgl_rect_s *dest, \
                      FAR const void *src[CONFIG_NX_NPLANES], \
                      FAR const struct nxgl_point_s *origin, \
                      unsigned int stride);

  Copy a rectangular region of a larger image into the
  rectangle in the specified client sub-window.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__ specifying the client
     sub-window that will receive the bitmap.
  :param dest:
     Describes the rectangular region on in the client sub-window will
     receive the bit map.
  :param src:
     The start of the source image(s). This is an array source images of
     size ``CONFIG_NX_NPLANES`` (probably 1).
  :param origin:
     The origin of the upper, left-most corner of the full bitmap. Both
     dest and origin are in sub-window coordinates, however, the origin
     may lie outside of the sub-window display.
  :param stride:
     The width of the full source image in pixels.

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_opentoolbar(NXTKWINDOW hfwnd, nxgl_coord_t height, \
                     FAR const struct nx_callback_s *cb, \
                     FAR void *arg);


  Create a tool bar at the top of the specified framed
  window.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
  :param height:
     The requested height of the toolbar in pixels.
  :param cb:
     Callbacks used to process toolbar events.
  :param arg:
     User provided value that will be returned with toolbar callbacks.

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_closetoolbar(NXTKWINDOW hfwnd);

  Remove the tool bar at the top of the specified framed
  window.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__.

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_filltoolbar(NXTKWINDOW hfwnd, FAR const struct nxgl_rect_s *rect, \
                     nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);


  Fill the specified rectangle in the toolbar sub-window
  with the specified color.

  :param hfwnd:
    A handle previously returned by
    ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
  :param rect:
    The location within the toolbar window to be filled.
  :param color:
    The color to use in the fill.

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_gettoolbar(NXTKWINDOW hfwnd, FAR const struct nxgl_rect_s *rect, \
                    unsigned int plane, FAR uint8_t *dest, \
                    unsigned int deststride);


  Get the raw contents of graphic memory within a
  rectangular region. NOTE: Since raw graphic memory is returned, the
  returned memory content may be the memory of windows above this one and
  may not necessarily belong to this window unless you assure that this is
  the top window.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
  :param rect:
     The location within the toolbar window to be retrieved.
  :param plane:
     TSpecifies the color plane to get from.
  :param dest:
     TThe location to copy the memory region.
  :param deststride:
     The width, in bytes, of the dest memory.

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_filltraptoolbar(NXTKWINDOW hfwnd, FAR const struct nxgl_trapezoid_s *trap, \
                         nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

  Fill the specified trapezoid in the toolbar sub-window
  with the specified color.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
  :param trap:
     The trapezoidal region to be filled
  :param color:
     The color to use in the fill

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_drawlinetoolbar(NXTKWINDOW hfwnd, FAR struct nxgl_vector_s *vector, \
                         nxgl_coord_t width, nxgl_mxpixel_t color[CONFIG_NX_NPLANES], \
                         uint8_t caps);


  Fill the specified line in the toolbar sub-window with
  the specified color. This is simply a wrapper that uses
  ``nxgl_splitline()`` to break the line into trapezoids and then calls
  ``nxtk_filltraptoolbar()`` to render the line.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
  :param vector:
     Describes the line to be drawn.
  :param width:
     The width of the line
  :param color:
     The color to use to fill the line
  :param caps:
     Draw a circular cap on the ends of the line to support better line
     joins. One of:

     .. code-block:: c

      /* Line caps */

      #define NX_LINECAP_NONE  0x00, /* No line caps */
      #define NX_LINECAP_PT1   0x01  /* Line cap on pt1 on of the vector only */
      #define NX_LINECAP_PT2   0x02  /* Line cap on pt2 on of the vector only */
      #define NX_LINECAP_BOTH  0x03  /* Line cap on both ends of the vector only */


  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_drawcircletoolbar(NXTKWINDOW hfwnd, FAR const struct nxgl_point_s *center, \
                           nxgl_coord_t radius, nxgl_coord_t width, \
                           nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

  Draw a circular outline using the specified line
  thickness and color.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
  :param center:
     A pointer to the point that is the center of the circle.
  :param radius:
     The radius of the circle in pixels.
  :param width:
     The width of the line
  :param color:
     The color to use to fill the line

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_fillcircletoolbar(NXWINDOW hfwnd, FAR const struct nxgl_point_s *center, \
                           nxgl_coord_t radius, nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

  Fill a circular region using the specified color.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
  :param center:
     A pointer to the point that is the center of the circle.
  :param radius:
     The width of the line
  :param color:
     The color to use to fill the circle

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_movetoolbar(NXTKWINDOW hfwnd, FAR const struct nxgl_rect_s *rect, \
                     FAR const struct nxgl_point_s *offset);

  Move a rectangular region within the toolbar sub-window
  of a framed window.

  :param hfwnd:
     A handle identifying sub-window containing the toolbar within which
     the move is to be done. This handle must have previously been
     returned by ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
  :param rect:
     Describes the rectangular region relative to the toolbar sub-window
     to move.
  :param offset:
     The offset to move the region

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. c:function:: int nxtk_bitmaptoolbar(NXTKWINDOW hfwnd, \
                       FAR const struct nxgl_rect_s *dest, \
                       FAR const void *src[CONFIG_NX_NPLANES], \
                       FAR const struct nxgl_point_s *origin, \
                       unsigned int stride);

  Copy a rectangular region of a larger image into the
  rectangle in the specified toolbar sub-window.

  :param hfwnd:
     A handle previously returned by
     ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
  :param dest:
     Describes the rectangular region on in the toolbar sub-window will
     receive the bit map.
  :param src:
     The start of the source image.
  :param origin:
     The origin of the upper, left-most corner of the full bitmap. Both
     dest and origin are in sub-window coordinates, however, the origin
     may lie outside of the sub-window display.
  :param stride:
     The width of the full source image in bytes.

  :return: ``OK`` on success; ``ERROR`` on failure with
    ``errno`` set appropriately

.. _nx-fonts-support-nxfonts-1:


