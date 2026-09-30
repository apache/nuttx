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

Wide Font Support
=================

Question::

  > My team is trying the nuttx graphics with chinese fonts, but nx seems not
  > support fonts quantity more than 256 chars, right?

Answer::

  NuttX currently only uses fonts with 7-bit and 8-bit character sets. But
  I believe that that limitation is mostly arbitrary. It should be a simple
  extension to the font subsystem to use 16-bit fonts.

Adding 16-Bit Font support
--------------------------

Current 7/8-bit Font Implementation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

All of critical font interfaces allow for 16-bit character sets:

.. code-block:: C

   FAR const struct nx_fontbitmap_s *nxf_getbitmap(NXHANDLE handle, uint16_t ch)

The character code is only used to look-up of a glyph in a table. There is a
definition that controls the width of the character set: CONFIG_NXFONTS_CHARBITS.
This currently defaults to 7 but all existing fonts support 8-bits.

My first guess is that the only thing that would have to change is that single
file nxfonts_bitmaps.c (and the function nxf_getglyphset() in the file
nxfonts_getfont.c) . nxfonts_bitmaps.c is used to auto-generate 7/8-bit font
data sents. Here is how that works:

* Each 7-8 bit file is described by a header file like, for example,
  nxfonts_sans17x22.h.

* At build time each of these header files is used to create a C file,
  like, nxfonts_bitmaps_sans17x22.c.

* It creates the C file (like nxfonts_bitmaps_sans17x22.c) by compiling
  nxfonts_bitmaps.c and including nxfonts_sans17x22.h to create the font
  dataset at build time.

The function nxf_getglyphset() in the file nxfonts_getfont.c selects the 7-bit
font range (codes < 128) or the 8-bit range (code >= 128 > 256). The fonts are
kept in simple arrays splitting the data up into ranges of values lets you above
the non-printable codes at the beginning and end of each range. There is even a
comment in the code there "Someday, perhaps 16-bit fonts will go here".

Adding Wide Fonts
~~~~~~~~~~~~~~~~~

To add a single wide font, the easiest way would be to simply add the final
.C file without going through the C auto-generation step. That should be VERY
easy. (But since it has never been used with larger character sets, I am sure
that there are bugs and things that need to be fixed).

If you want to add many wide fonts, then perhaps you would have to create a new
version of the C auto-generation logic. That would require more effort.

I am willing to help and advise. Having good wide character support in the NuttX
graphics would be an important improvement to NuttX. This is not a lot of code
nor is it very difficult code so you should not let it be an obstacle for you.

Font Storage Issues
-------------------

One potential problem may be the amount of memory required by fonts with
thousands of characters. If you have a lot of flash, it may not be a problem,
but on many microcontrollers it will be quite limiting.

Options are:

* **Font Compression** Addition of some font compression algorithm in NuttX.
  However, Chinese character bitmaps do not compress well: Many of them contain
  so much data that there is not much of anything to compress. Some actually
  expand under certain compression algorithms.

* **Mass Storage** A better option would be put the wide the fonts in file
  system, in NAND or serial FLASH or on an SD card. In this case, additional
  logic would be required to (1) format a font binary file and to (2) access
  the font binary from the file system as needed.
