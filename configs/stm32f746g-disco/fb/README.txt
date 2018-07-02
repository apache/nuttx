README.txt
==========

STM32F746G-DISCO LTDC Framebuffer demo example

Configure and build
-------------------

tools/configure.sh stm32f746g-disco/fb
make


Configuration
------------

This configuration provides 1 LTDC with
16bpp pixel format and a resolution of 480x272.


Loading
-------

st-flash write nuttx.bin 0x8000000


Executing
---------

The ltdc is initialized during boot up.  Interaction with NSH is via the serial
console provided by ST-LINK USB at 115200 8N1 baud.
From the nsh comandline execute the fb example:

  nsh> fb

The test will put a pattern of concentric squares in the framebuffer and
terminate.
