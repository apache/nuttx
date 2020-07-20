===========
Sample Code
===========

``apps/examples/nx*``. No sample code is provided in this document.
However, examples can be found in the NuttX source tree at the follow
locations: That example code is intended to test NX. Since it is test
code, it is designed to exercise functionality and does not necessarily
represent best NX coding practices.

- ``apps/examples/nx``. This is a test of windows, optionally with
  toolbars. Two windows are created, re-sized, moved, raise lowered.
  Simulated mouse and keyboard input is provided.
- ``apps/examples/nxhello``. This is intended to be simplest NX test:
  It simply displays the words "Hello, World!" centered on the display.
- ``apps/examples/nxtext``. This illustrates how fonts may be managed
  to provide scrolling text windows. Pop-up windows are included to
  verify the clipping and re-drawing of the text display.

In its current form, the NX graphics system provides a low level of
graphics and window support. Most of the complexity of manage redrawing
and handling mouse and keyboard events must be implemented by the NX
client code.

**Building** ``apps/examples/nx``. Testing was performed using the
Linux/Cygwin-based NuttX simulator. Instructions are provided for
building that simulation are provided in `Appendix C <#testcoverage>`__
of this document.


