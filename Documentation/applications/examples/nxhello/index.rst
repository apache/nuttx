``nxhello``
===========

A very simple graphics example that just says Hello, World! in the center of
the display.

The following configuration options can be selected:

- ``CONFIG_NSH_BUILTIN_APPS`` – Build the ``NXHELLO`` example as a built-in that
  can be executed from the NSH command line
- ``CONFIG_EXAMPLES_NXHELLO_VPLANE`` – The plane to select from the frame- buffer
  driver for use in the test. Default: ``0``.
- ``CONFIG_EXAMPLES_NXHELLO_DEVNO`` – The LCD device to select from the LCD driver
  for use in the test. Default: ``0``.
- ``CONFIG_EXAMPLES_NXHELLO_BGCOLOR`` – The color of the background. Default
  depends on ``CONFIG_EXAMPLES_NXHELLO_BPP``.
- ``CONFIG_EXAMPLES_NXHELLO_FONTID`` – Selects the font (see font ID numbers in
  include/nuttx/nx/nxfonts.h).
- ``CONFIG_EXAMPLES_NXHELLO_FONTCOLOR`` – The color of the fonts used in the
  background window. Default depends on ``CONFIG_EXAMPLES_NXHELLO_BPP``.
- ``CONFIG_EXAMPLES_NXHELLO_BPP`` – Pixels per pixel to use. Valid options include
  ``2``, ``4``, ``8``, ``16``, ``24`` and ``32``. Default: ``32``.
