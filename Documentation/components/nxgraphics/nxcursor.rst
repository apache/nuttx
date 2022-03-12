================================
NX Cursor Support (``NXCURSOR``)
================================

.. c:function:: int nxcursor_enable(NXHANDLE hnd, bool enable)

  Enable/disable presentation of the cursor. The disabled
  cursor still exits and still may be controlled, but is not visible on
  the display.

  :param hnd:
     The server handle returned by :c:func:`nx_connect`.
  :param enable: The new cursor position

  :return: OK on success; ERROR on failure with errno set appropriately.

.. c:function:: int nxcursor_setimage(NXHANDLE hnd, FAR const struct nx_cursorimage_s *image)

  Set the cursor image.

  The image is provided a a 2-bits-per-pixel image. The two bit incoding
  is as following:

  - 00: The transparent background.
  - 01: Color1: The main color of the cursor.
  - 10: Color2: The color of any border.
  - 11: Color3: A blend color for better imaging (fake anti-aliasing).

  **NOTE:** The NX logic will reference the user image buffer repeatedly.
  That image buffer must persist for as long as the NX server connection
  persists.

  :param hnd: The server handle returned by :c:func:`nx_connect`
  :param image:
    An instance of ``struct struct nx_cursorimage_s`` that describes the
    cursor image. See ``<nuttx/nx/nxcursor.h>`` for the full description
    of this structure.

  :return: OK on success; ERROR on failure with errno set appropriately.

.. c:function:: int nxcursor_setposition(NXHANDLE hnd, FAR const struct nxgl_point_s *pos)

  Move the cursor to the specified position.

  :param hnd: The server handle returned by :c:func:`nx_connect`
  :param pos: The new cursor position

  :return: OK on success; ERROR on failure with errno set appropriately.

