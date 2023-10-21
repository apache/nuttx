``pwfb``
========

A graphics example using pre-window frame buffers. The example shows three
windows containing text moving around, crossing each other from above and from
_below_. The example application is NOT updating the windows any anyway! The
application is only changing the window position. The windows are being updated
from the per-winidow framebuffers automatically.

This example is reminiscent of Pong: Each window travels in straight line until
it hits an edge, then it bounces off. The window is also raised when it hits the
edge (gets focus). This tests all combinations of overap.

**Note**: A significant amount of RAM, usually external SDRAM, is required to
run this demo. At 16bpp and a 480x272 display, each window requires about 70Kb
of RAM for its framebuffer.
