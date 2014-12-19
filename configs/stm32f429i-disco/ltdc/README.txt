README.txt
==========

STM32F429I-DISCO LTDC Framebuffer demo example

Configure and build
-----------------------------------------------
cd tools
./configure -a <appdir> stm32f429i-disco/ltdc
cd ..
make

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
