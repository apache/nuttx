README.txt
==========

STM32F429I-DISCO LTDC Framebuffer demo example

Configure and build
-------------------

cd tools
./configure -a <appdir> stm32f429i-disco/fb
cd ..
make

Framebuffer calculation
----------------------

Use the helper script boards/stm32f429i-disco/tools/fbcalc.sh for calculating
the heap2 and framebuffer memory region. The script assumes that all overlay
buffers (LTDC and DMA2D) located in heap2 memory region starting at address
0xD0000000. When changing the display size (when using a custom display), DMA2D
overlay size or the pixel format you have to recalculate the heap2 settings.
In this configuration all overlays (LTDC and DMA2D) positioned at the end of
heap2.

LTDC hardware acceleration
--------------------------

The LTDC driver provides two 2 LTDC overlays and supports the following hardware
acceleration and features:

Configured at build time
- background color
- default color (outside visible screen)

Configurable by nuttx framebuffer interface
- cmap support (color table is shared by both LTDC overlays and DMA2D when
  enabled)

Configurable via the nuttx framebuffer interface (for each layer separately)
- chromakey
- transparency (const alpha and pixel alpha)
- blank
- color (if DMA2D is enabled and cmap is disabled)
- blit (if DMA2D is enabled)
- blend (if DMA2D is enabled and cmap is disabled)

LTDC overlays are similar to a non-destructive overlay. Both LTDC overlays will
be permanently blended in the order (background -> overlay 0 -> overlay 1) and
converted to a resulting video signal by the LTDC controller. That means each
operation with a LTDC overlay (Overlay 0 and Overlay 1) via nuttx framebuffer
interface will be visible immediately.
Think about continuous blending between both overlays.

DMA2D hardware acceleration
---------------------------

The DMA2D driver implements the following hardware acceleration:

Configurable via the nuttx framebuffer interface
- cmap support (color table is shared by all DMA2D overlays and LTDC overlays)

Configurable via the nuttx framebuffer interface (for each layer separately)

- color (fill memory region with a specific ARGB8888 color immediately), if
  cmap is disabled
- blit (copy memory region to another memory region with pixel format
  conversion if necessary)
- blend (blend two memory regions and copy the result to a third memory region
  with pixel format conversion if necessary), if cmap is disabled

Blit and blend operation using a fixes memory size defined by the background
layer. DMA2D controller doesn't support scaling.

DMA2D overlays are similar to destructive overlays. They are invisible. They can
be used for image preprocessing. The memory region affected by the operations
(color, blit, blend) can be addressed by the area control command before. The
configured overlay transparency of DMA2D overlays will be used for subsequently
blend operation and is valid for the whole overlay.

Configuration
------------

This configuration provides 2 LTDC (visible overlays) and 2 DMA2D overlays with
pixel format RGB565 and a resolution of 240x320.

Loading
-------

st-flash write nuttx.bin 0x8000000

Executing
---------

The ltdc is initialized during boot up.  Interaction with NSH is via the serial
console at 115200 8N1 baud.  From the nsh comandline execute the fb example:

  nsh> fb

The test will put a pattern of concentric squares in the framebuffer and
terminate.

You can also test overlay hardware acceleration functionality by executing the
following command (shows a commandline help):

  nsh> fboverlay
