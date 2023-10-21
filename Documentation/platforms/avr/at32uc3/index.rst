=================
Microchip AT32UC3
=================

AV32DEV1. This port uses the www.mcuzone.com AVRDEV1 board based on the
Microchip AT32UC3B0256 MCU. This port requires a special GNU avr32 toolchain
available from atmel.com website. This is a windows native toolchain and
so can be used only under Cygwin on Windows.

**STATUS:** This port is has completed all basic development, but there
is more that needs to be done. All code is complete for the basic NuttX
port including header files for all AT32UC3\* peripherals. The untested
AVR32 code was present in the 5.12 release of NuttX. Since then, the
basic RTOS port has solidified:

-  The port successfully passes the NuttX OS test
   (apps/examples/ostest).
-  A NuttShell (NSH) configuration is in place (see :ref:`NSH <nsh>`).
   Testing of that configuration has been postponed (because it got
   bumped by the Olimex LPC1766-STK port). Current Status: I think I
   have a hardware problem with my serial port setup. There is a good
   chance that the NSH port is complete and functional, but I am not yet
   able to demonstrate that. At present, I get nothing coming in the
   serial RXD line (probably because the pins are configured wrong or I
   have the MAX232 connected wrong).

The basic, port was be released in NuttX-5.13. A complete port will
include drivers for additional AVR32 UC3 devices -- like SPI and USB ---
and will be available in a later release, time permitting. Refer to the
NuttX board
`README <https://github.com/apache/nuttx/blob/master/boards/avr/at32uc3/avr32dev1/README.txt>`__
file for further information.

