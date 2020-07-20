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

2.4.3 ``nxtk_synch()``
~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** This interface can be used to synchronize the window
client with the NX server. It really just implements an *echo*: A synch
message is sent from the window client to the server which then responds
immediately by sending the ``NXEVENT_SYNCHED`` back to the windows
client.

Due to the highly asynchronous nature of client-server communications,
``nx_synch()`` is sometimes necessary to assure that the client and
server are fully synchronized in time.

Usage by the window client might be something like this:

When the window listener thread receives the ``NXEVENT_SYNCHED`` event,
it would set ``g_synched`` to ``true`` and post ``g_synch_sem``, waking
up the above loop.

**Input Parameters:**

``wnd`` 
   The window to be synched
``arg`` 
   An argument that will accompany the synch messages (This is ``arg2``
   in the event callback).

**Returned Value:** OK on success; ERROR on failure with errno set
appropriately

2.4.4 ``nxtk_openwindow()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Create a new, framed window.

**Input Parameters:**

``handle`` 
   The handle returned by ```nx_connect()`` <#nxconnectinstance>`__.
``flags`` 
   Optional flags. These include:

   -  ``NXBE_WINDOW_RAMBACKED``: Creates a RAM backed window. This
      option is only valid if ``CONFIG_NX_RAMBACKED`` is enabled.
   -  ``NXBE_WINDOW_HIDDEN``: The window is create in the HIDDEN state
      and can be made visible later with ``nxtk_setvisibility()``.

``cb`` 
   Callbacks used to process window events
``arg`` 
   User provided argument (see ```nx_openwindow()`` <#nxopenwindow>`__)

**Returned Value:**

2.4.5 ``nxtk_closewindow()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Close the window opened by
```nxtk_openwindow()`` <#nxtkopenwindow>`__.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__.

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.6 ``nxtk_getposition()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Request the position and size information for the
selected framed window. The size/position for the client window and
toolbar will be return asynchronously through the client callback
function pointer.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__.

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.7 ``nxtk_setposition()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Set the position for the selected client window. This
position does not include the offsets for the borders nor for any
toolbar. Those offsets will be added in to set the full window position.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
``pos`` 
   The new position of the client sub-window

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.8 ``nxtk_setsize()``
~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Set the size for the selected client window. This size
does not include the sizes of the borders nor for any toolbar. Those
sizes will be added in to set the full window size.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
``size`` 
   The new size of the client sub-window.

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.9 ``nxtk_raise()``
~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Bring the window containing the specified client
sub-window to the top of the display.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__ specifying the window to
   be raised.
```` 

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.10 ``nxtk_lower()``
~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Lower the window containing the specified client
sub-window to the bottom of the display.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__ specifying the window to
   be lowered.
```` 

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.11 ``nxtk_modal()``
~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** May be used to either (1) raise a window to the top of
the display and select modal behavior, or (2) disable modal behavior.

**Input Parameters:**

``hwnd`` 
   The handle returned by ```nxtk_openwindow()`` <#nxtkopenwindow>`__
   specifying the window to be modified.
``modal`` 
   True: enter modal state; False: leave modal state

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.12 ``nxtk_setvisibility()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Select if the window is visible or hidden. A hidden
window is still present and will update normally, but will not be
visible on the display until it is unhidden.

**Input Parameters:**

``hwnd`` 
   The handle returned by ```nxtk_openwindow()`` <#nxtkopenwindow>`__
   specifying the window to be modified.
``hide`` 
   True: Window will be hidden; false: Window will be visible

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.13 ``nxtk_ishidden()``
~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Return true if the window is hidden.

**NOTE**: There will be a delay between the time that the visibility of
the window is changed via
```nxtk_setvisibily()`` <#nxtksetvisibility>`__ before that new setting
is reported by ``nxtk_ishidden()``. ``nxtk_synch()`` may be used if
temporal synchronization is required.

**Input Parameters:**

``hfwnd`` 
   The handle returned by ```nxtk_openwindow()`` <#nxtkopenwindow>`__
   that identifies the window to be queried.

**Returned Value:** *True*: the window is hidden, *false*: the window is
visible

2.4.14 ``nxtk_fillwindow()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Fill the specified rectangle in the client window with
the specified color.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
``rect`` 
   The location within the client window to be filled
``color`` 
   The color to use in the fill

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.15 ``nxtk_getwindow()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Get the raw contents of graphic memory within a
rectangular region. NOTE: Since raw graphic memory is returned, the
returned memory content may be the memory of windows above this one and
may not necessarily belong to this window unless you assure that this is
the top window.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
``rect`` 
   The location within the client window to be retrieved.
``plane`` 
   Specifies the color plane to get from.
``dest`` 
   The location to copy the memory region
``deststride`` 
   The width, in bytes, of the dest memory

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.16 ``nxtk_filltrapwindow()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Fill the specified trapezoid in the client window with
the specified color

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
``trap`` 
   The trapezoidal region to be filled.
``color`` 
   The color to use in the fill.

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.17 ``nxtk_drawlinewindow()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Fill the specified trapezoidal region in the window
with the specified color. Fill the specified line in the window with the
specified color. This is simply a wrapper that uses ``nxgl_splitline()``
to break the line into trapezoids and then calls
``nxtk_filltrapwindow()`` to render the line.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
``vector`` 
   Describes the line to be drawn.
``width`` 
   The width of the line
``color`` 
   The color to use to fill the line
``caps`` 
   Draw a circular cap on the ends of the line to support better line
   joins. One of:

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.18 ``nxtk_drawcirclewindow()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Draw a circular outline using the specified line
thickness and color.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
``center`` 
   A pointer to the point that is the center of the circle.
``radius`` 
   The radius of the circle in pixels.
``width`` 
   The width of the line
``color`` 
   The color to use to fill the line

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.19 ``nxtk_fillcirclewindow()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Fill a circular region using the specified color.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
``center`` 
   A pointer to the point that is the center of the circle.
``radius`` 
   The width of the line
``color`` 
   The color to use to fill the circle

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.20 ``nxtk_movewindow()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Move a rectangular region within the client sub-window
of a framed window.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__ specifying the client
   sub-window within which the move is to be done.
``rect`` 
   Describes the rectangular region relative to the client sub-window to
   move.
``offset`` 
   The offset to move the region

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.21 ``nxtk_bitmapwindow()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Copy a rectangular region of a larger image into the
rectangle in the specified client sub-window.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__ specifying the client
   sub-window that will receive the bitmap.
``dest`` 
   Describes the rectangular region on in the client sub-window will
   receive the bit map.
``src`` 
   The start of the source image(s). This is an array source images of
   size ``CONFIG_NX_NPLANES`` (probably 1).
``origin`` 
   The origin of the upper, left-most corner of the full bitmap. Both
   dest and origin are in sub-window coordinates, however, the origin
   may lie outside of the sub-window display.
``stride`` 
   The width of the full source image in pixels.

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.22 ``nxtk_opentoolbar()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Create a tool bar at the top of the specified framed
window.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
``height`` 
   The requested height of the toolbar in pixels.
``cb`` 
   Callbacks used to process toolbar events.
``arg`` 
   User provided value that will be returned with toolbar callbacks.

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.23 ``nxtk_closetoolbar()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Remove the tool bar at the top of the specified framed
window.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
```` 

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.24 ``nxtk_filltoolbar()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Fill the specified rectangle in the toolbar sub-window
with the specified color.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
``rect`` 
   The location within the toolbar window to be filled.
``color`` 
   The color to use in the fill.

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.25 ``nxtk_gettoolbar()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Get the raw contents of graphic memory within a
rectangular region. NOTE: Since raw graphic memory is returned, the
returned memory content may be the memory of windows above this one and
may not necessarily belong to this window unless you assure that this is
the top window.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
``rect`` 
   The location within the toolbar window to be retrieved.
``plane`` 
   TSpecifies the color plane to get from.
``dest`` 
   TThe location to copy the memory region.
``deststride`` 
   The width, in bytes, of the dest memory.

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.26 ``nxtk_filltraptoolbar()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Fill the specified trapezoid in the toolbar sub-window
with the specified color.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
``trap`` 
   The trapezoidal region to be filled
``color`` 
   The color to use in the fill

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.27 ``nxtk_drawlinetoolbar()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Fill the specified line in the toolbar sub-window with
the specified color. This is simply a wrapper that uses
``nxgl_splitline()`` to break the line into trapezoids and then calls
``nxtk_filltraptoolbar()`` to render the line.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
``vector`` 
   Describes the line to be drawn.
``width`` 
   The width of the line
``color`` 
   The color to use to fill the line
``caps`` 
   Draw a circular cap on the ends of the line to support better line
   joins. One of:

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.28 ``nxtk_drawcircletoolbar()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Draw a circular outline using the specified line
thickness and color.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
``center`` 
   A pointer to the point that is the center of the circle.
``radius`` 
   The radius of the circle in pixels.
``width`` 
   The width of the line
``color`` 
   The color to use to fill the line

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.29 ``nxtk_fillcircletoolbar()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Fill a circular region using the specified color.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
``center`` 
   A pointer to the point that is the center of the circle.
``radius`` 
   The width of the line
``color`` 
   The color to use to fill the circle

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.30 ``nxtk_movetoolbar()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Move a rectangular region within the toolbar sub-window
of a framed window.

**Input Parameters:**

``hfwnd`` 
   A handle identifying sub-window containing the toolbar within which
   the move is to be done. This handle must have previously been
   returned by ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
``rect`` 
   Describes the rectangular region relative to the toolbar sub-window
   to move.
``offset`` 
   The offset to move the region

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

2.4.31 ``nxtk_bitmaptoolbar()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Function Prototype:**

**Description:** Copy a rectangular region of a larger image into the
rectangle in the specified toolbar sub-window.

**Input Parameters:**

``hfwnd`` 
   A handle previously returned by
   ```nxtk_openwindow()`` <#nxtkopenwindow>`__.
``dest`` 
   Describes the rectangular region on in the toolbar sub-window will
   receive the bit map.
``src`` 
   The start of the source image.
``origin`` 
   The origin of the upper, left-most corner of the full bitmap. Both
   dest and origin are in sub-window coordinates, however, the origin
   may lie outside of the sub-window display.
``stride`` 
   The width of the full source image in bytes.

**Returned Value:** ``OK`` on success; ``ERROR`` on failure with
``errno`` set appropriately

.. _nx-fonts-support-nxfonts-1:


