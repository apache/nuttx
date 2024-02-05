===========================
``nxlines`` NX Line Drawing
===========================

A very simple graphics example that just exercised the NX line drawing logic.

The following configuration options can be selected:

- ``CONFIG_EXAMPLES_NXLINES_VPLANE`` – The plane to select from the frame- buffer
  driver for use in the test. Default: ``0``.
- ``CONFIG_EXAMPLES_NXLINES_DEVNO`` – The LCD device to select from the LCD driver
  for use in the test: Default: ``0``.
- ``CONFIG_EXAMPLES_NXLINES_BGCOLOR`` – The color of the background. Default
  depends on ``CONFIG_EXAMPLES_NXLINES_BPP``.
- ``CONFIG_EXAMPLES_NXLINES_LINEWIDTH`` – Selects the width of the lines in pixels
  (default: ``16``).
- ``CONFIG_EXAMPLES_NXLINES_LINECOLOR`` – The color of the central lines drawn in
  the background window. Default depends on ``CONFIG_EXAMPLES_NXLINES_BPP`` (there
  really is no meaningful default).
- ``CONFIG_EXAMPLES_NXLINES_BORDERWIDTH`` – The width of the circular border drawn
  in the background window. (default: ``16``).
- ``CONFIG_EXAMPLES_NXLINES_BORDERCOLOR`` – The color of the circular border drawn
  in the background window. Default depends on ``CONFIG_EXAMPLES_NXLINES_BPP``
  (there really is no meaningful default).
- ``CONFIG_EXAMPLES_NXLINES_CIRCLECOLOR`` – The color of the circular region
  filled in the background window. Default depends on
  ``CONFIG_EXAMPLES_NXLINES_BPP`` (there really is no meaningful default).
- ``CONFIG_EXAMPLES_NXLINES_BORDERCOLOR`` – The color of the lines drawn in the
  background window. Default depends on ``CONFIG_EXAMPLES_NXLINES_BPP`` (there
  really is no meaningful default).
- ``CONFIG_EXAMPLES_NXLINES_BPP`` – Pixels per pixel to use. Valid options include
  ``2``, ``4``, ``8``, ``16``, ``24``, and ``32``. Default is ``16``.
- ``CONFIG_NSH_BUILTIN_APPS`` – Build the NX lines examples as an NSH built-in
  function.
