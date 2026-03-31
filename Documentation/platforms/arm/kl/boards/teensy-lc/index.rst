=========
teensy-lc
=========

This is the NuttX port for the PJRC Teensy LC board. The Teensy LC is a
DIP-style breakout board for the MKL25Z64 and includes a USB-based
bootloader.

Development Environment
=======================

Testing was done with the GNU ARM Embedded 4.9 toolchain on Linux. See
https://developer.arm.com/open-source/gnu-toolchain/gnu-rm for toolchain
downloads.

Once you have configured and built NuttX, flash the resulting ``nuttx.hex``
image to the board with the Teensy Loader application.

LEDs
====

The Teensy LC provides a single LED. If ``CONFIG_ARCH_LEDS`` is enabled, NuttX
updates that LED during boot:

* LED off: board booting
* LED on: initial stack created
* LED flashing: panic

See ``include/board.h`` for the board-specific definitions.

Serial Console
==============

The serial console is mapped to UART0 and appears on pins 0 (RX) and 1 (TX).
Use a 3.3 V USB-to-serial adapter, such as the Sparkfun ``#9717`` FTDI cable.
