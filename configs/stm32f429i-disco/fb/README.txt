README.txt
==========

STM32F429I-DISCO LTDC Framebuffer demo example

Configure and build
-----------------------------------------------
cd tools
./configure -a <appdir> stm32f429i-disco/ltdc
cd ..
make

Note!
In the current implementation the DMA2D driver only supports clut pixel format
if the LTDC driver it does. Otherwise it will not be compatible with the nx
framework. If CONFIG_FB_CMAP is configured, nx expects that any pixel format
supports color lookup tables. This is also the case for non CLUT formats e.g.
FB_FMT_RGB16_565. This may result in wrong color representation by nx if the
pixel format is unequal to FB_FMT_RGB8.

On the other hand layers with CLUT pixel format are not supported by the DMA2D
controller, in the case they will be used as destination layer for the following
operations:
- blit
- blend
- fillarea

To enable clut support in both LTDC and DMA2D driver the following
configurations are valid:

1.
- Enable LTDC_INTERFACE and LAYER1/LAYER2
- Layer1 FB_FMT_RGB8
- Layer2 any non clut format

But Layer2 can only be used as destination layer for dma2d operations above.
This configuration is not compatibly to nx because LAYER2 will be referenced
by up_fbgetvplane and is an invalid CLUT pixel format.

2.
- Enable LTDC_INTERFACE and LAYER1/LAYER2
- Layer2 FB_FMT_RGB8
- Layer1 any non clut format

But Layer1 can only be used as destination layer for dma2d operations above.
This configuration should be compatibly to nx because LAYER2 will be referenced
by up_fbgetvplane and is an valid CLUT pixel format.

All other non clut configuration work fine.

If using the DMA2D controller without the LTDC controller e.g. camera interface
than enable CONFIG_FB_CMAP and optional CONFIG_FB_TRANSPARENCY in your board
specific configuration.


Loading
-----------------------------------------------
st-flash write nuttx.bin 0x8000000

Executing
-----------------------------------------------
The ltdc is initialized during boot up.
Interaction with NSH is via the serial console at 115200 8N1 baud.
From the nsh comandline execute one (or both) of the examples:
- nx (default nx example)
- ltdc (trivial ltdc interface test)

Note! The ltdc example ends in an infinite loop. To get control of the nsh
start this example in the background with 'ltdc &'.

