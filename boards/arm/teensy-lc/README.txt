Teensy LC README
================

  This is the README file for NuttX on the PJRC Teensy LC.  The Teensy LC
  is a DIP style breakout board for the MKL25Z64 and comes with a USB
  based bootloader.  Contributed by Michael Hope.

Development Environment
=======================

  All testing was done with the GNU ARM Embedded 4.9 toolchain on
  Linux.  See https://developer.arm.com/open-source/gnu-toolchain/gnu-rm to download.

  Once you've configured and built NuttX, flash the resulting
  nuttx.hex file to the board using the Teensy Loader Application.

LEDs
====

  The Teensy LC has a single LED. If CONFIG_ARCH_LEDS is defined, then
  NuttX will update the LED as the board boots.  The summary is:

  * LED off: board booting
  * LED on: initial stack created
  * LED flashing: panic.

  See `include/board.h` for details.

Serial Console
==============

  The serial console is mapped to UART0 and appears on pins 0 (RX) and
  1 (TX). Consider using a 3.3 V USB to serial adapter such as the
  Sparkfun #9717 FTDI cable.
