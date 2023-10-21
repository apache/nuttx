``nximage`` Display NuttX Logo
==============================

This is a simple example that just puts the NuttX logo image in the center of
the display. This only works for ``RGB23`` (``888``), ``RGB16`` (``656``), ``RGB8``
(``332``), and 8-bit greyscale for now.

- ``CONFIG_NSH_BUILTIN_APPS`` – Build the ``NXIMAGE`` example as a built-in that
  can be executed from the NSH command line.
- ``CONFIG_EXAMPLES_NXIMAGE_VPLANE`` – The plane to select from the frame- buffer
  driver for use in the test. Default: ``0``.
- ``CONFIG_EXAMPLES_NXIMAGE_DEVNO`` – The LCD device to select from the LCD driver
  for use in the test: Default: ``0``.
- ``CONFIG_EXAMPLES_NXIMAGE_BPP`` – Pixels per pixel to use. Valid options include
  ``8``, ``16`` and ``24``. Default is ``16``.
- ``CONFIG_EXAMPLES_NXIMAGE_XSCALEp5``, ``CONFIG_EXAMPLES_NXIMAGE_XSCALE1p5`` or
  ``CONFIG_EXAMPLES_NXIMAGE_XSCALE2p0`` – The logo image width is 160 columns. One
  of these may be defined to rescale the image horizontally by .5, 1.5 or 2.0.
- ``CONFIG_EXAMPLES_NXIMAGE_YSCALEp5``, ``CONFIG_EXAMPLES_NXIMAGE_YSCALE1p5`` or
  ``CONFIG_EXAMPLES_NXIMAGE_YSCALE2p0`` – The logo image height is 160 rows. One
  of these may be defined to rescale the image vertically by .5, 1.5 or 2.0.
- ``CONFIG_EXAMPLES_NXIMAGE_GREYSCALE`` – Grey scale image. Default: ``RGB``.

How was that run-length encoded image produced?

1. I used GIMP output the image as a ``.c`` file.
2. I added some C logic to palette-ize the RGB image in the GIMP ``.c`` file.
3. Then I add some simple run-length encoding to palette-ized image.

But now there is a tool that can be found in the NxWidgets package at
``NxWidgets/tools/bitmap_converter.py`` that can be used to convert any graphics
format to the NuttX RLE format.

**Note**: As of this writing, most of the pixel depth, scaling options, and
combinations thereof have not been tested.
