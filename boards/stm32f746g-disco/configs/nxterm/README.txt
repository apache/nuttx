README.txt
==========

STM32F746G-DISCO NX Terminal example

Configure and build
-------------------

tools/configure.sh stm32f746g-disco/nxterm
make


Configuration
------------

This configuration provides 1 LTDC with
16bpp pixel format and a resolution of 480x272.

Trickiest part of config is increasing max message size (CONFIG_MQ_MAXMSGSIZE=256).
NX server - client communication cannot be established with default value 8 bytes.


Loading
-------

st-flash write nuttx.bin 0x8000000

or

openocd -f interface/stlink.cfg -f target/stm32f7x.cfg
telnet localhost 4444
> program nuttx verify reset


Executing
---------

The ltdc is initialized during boot up.  Interaction with NSH is via the serial
console provided by ST-LINK USB at 115200 8N1 baud.

From the nsh comandline execute the example:

  nsh> nxterm

The test will show terminal window on the screen.
