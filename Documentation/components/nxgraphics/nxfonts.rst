==============================
NX Fonts Support (``NXFONTS``)
==============================

NXFONTS types
=============

.. c:struct:: nx_fontmetric_s

  This structures provides the metrics for one glyph:

  .. code-block:: c

    struct nx_fontmetric_s
    {
      uint32_t stride   : 2;      /* Width of one font row in bytes */
      uint32_t width    : 6;      /* Width of the font in bits */
      uint32_t height   : 6;      /* Height of the font in rows */
      uint32_t xoffset  : 6;      /* Top, left-hand corner X-offset in pixels */
      uint32_t yoffset  : 6;      /* Top, left-hand corner y-offset in pixels */
      uint32_t unused   : 6;
    };

.. c:struct:: nx_fontbitmap_s

  This structure binds the glyph metrics to the glyph bitmap:

  .. code-block:: c

    struct nx_fontbitmap_s
    {
      struct nx_fontmetric_s metric; /* Character metrics */
      FAR const uint8_t *bitmap;     /* Pointer to the character bitmap */
    };


.. c:struct:: nx_fontset_s

  This structure describes one contiguous grouping of glyphs that can be
  described by an array starting with encoding ``first`` and extending
  through (``first`` + ``nchars`` - 1).

  .. code-block:: c

    struct nx_fontset_s
    {
      uint8_t  first;             /* First bitmap character code */
      uint8_t  nchars;            /* Number of bitmap character codes */
      FAR const struct nx_fontbitmap_s *bitmap;
    };

.. c:struct:: nx_font_s

  This structure describes the overall fontset.

  .. code-block:: c

    struct nx_font_s
    {
      uint8_t  mxheight;          /* Max height of one glyph in rows */
      uint8_t  mxwidth;           /* Max width of any glyph in pixels */
      uint8_t  mxbits;            /* Max number of bits per character code */
      uint8_t  spwidth;           /* The width of a space in pixels */
    };

.. c:function:: NXHANDLE nxf_getfonthandle(enum nx_fontid_e fontid);

  Given a numeric font ID, return a handle that may be
  subsequently be used to access the font data sets.

  :param fontid: Identifies the font set to use

  :return: A handle that may be subsequently be used to access the font data sets.

.. c:function:: FAR const struct nx_font_s *nxf_getfontset(NXHANDLE handle);

  Return information about the current font set.

  :param handle: A font handle previously returned by :c:func:`nxf_getfonthandle`.
  :return: An instance of ``struct nx_font_s`` describing the font set.

.. c:function:: FAR const struct nx_fontbitmap_s *nxf_getbitmap(NXHANDLE handle, uint16_t ch)

  Return font bitmap information for the selected
  character encoding.

  :param ch: The char code for the requested bitmap.
  :param handle: A font handle previously returned by :c:func:`nxf_getfonthandle`.
  :return: An instance of :c:struct:`nx_fontbitmap_s` describing the glyph.

.. c:function:: int nxf_convert_2bpp(FAR uint8_t *dest, uint16_t height, \
                     uint16_t width, uint16_t stride, \
                     FAR const struct nx_fontbitmap_s *bm, \
                     nxgl_mxpixel_t color);

.. c:function:: int nxf_convert_4bpp(FAR uint8_t *dest, uint16_t height, \
                     uint16_t width, uint16_t stride, \
                     FAR const struct nx_fontbitmap_s *bm, \
                     nxgl_mxpixel_t color);
.. c:function:: int nxf_convert_8bpp(FAR uint8_t *dest, uint16_t height, \
                     uint16_t width, uint16_t stride, \
                     FAR const struct nx_fontbitmap_s *bm, \
                     nxgl_mxpixel_t color);
.. c:function:: int nxf_convert_16bpp(FAR uint16_t *dest, uint16_t height, \
                      uint16_t width, uint16_t stride, \
                      FAR const struct nx_fontbitmap_s *bm, \
                      nxgl_mxpixel_t color);
.. c:function:: int nxf_convert_24bpp(FAR uint32_t *dest, uint16_t height, \
                      uint16_t width, uint16_t stride, \
                      FAR const struct nx_fontbitmap_s *bm, \
                      nxgl_mxpixel_t color);
.. c:function:: int nxf_convert_32bpp(FAR uint32_t *dest, uint16_t height, \
                      uint16_t width, uint16_t stride, \
                      FAR const struct nx_fontbitmap_s *bm, \
                      nxgl_mxpixel_t color);

  Convert the 1BPP font to a new pixel depth.

  :param dest: The destination buffer provided by the caller.
  :param height: The max height of the returned char in rows.
  :param width: The max width of the returned char in pixels.
  :param stride: The width of the destination buffer in bytes.
  :param bm: Describes the character glyph to convert
  :param color: The color to use for '1' bits in the font bitmap (0 bits are transparent).

  :return: ``OK`` on success; ``ERROR`` on failure with ``errno`` set appropriately.

