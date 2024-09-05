README
======

This directory contains the port of NuttX to the Raspberry Pi Pico.
See https://www.raspberrypi.org/products/raspberry-pi-pico-2/ for information
about Raspberry Pi Pico 2.

NuttX supports the following RP2530 capabilities:
  - UART  (console port)
    - GPIO 0 (UART0 TX) and GPIO 1 (UART0 RX) are used for the console.
  - ADC
  - USB device
    - CDC/ACM serial device can be used for the console.
  - Flash ROM Boot
  - SRAM Boot

Installation
============

1. Configure and build NuttX

  $ git clone https://github.com/apache/nuttx.git nuttx
  $ git clone https://github.com/apache/nuttx-apps.git apps
  $ cd nuttx
  $ make distclean
  $ ./tools/configure.sh raspberrypi-pico-2:nsh
  $ make V=1

4. Connect Raspberry Pi Pico 2 board to USB port while pressing BOOTSEL.
   The board will be detected as USB Mass Storage Device.
   Then copy "nuttx.uf2" into the device.
   (Same manner as the standard Pico SDK applications installation.)

5. To access the console, GPIO 0 and 1 pins must be connected to the
   device such as USB-serial converter.

   `usbnsh` configuration provides the console access by USB CDC/ACM serial
   devcice.  The console is available by using a terminal software on the USB
   host.

Defconfigs
==========

- nsh
    Minimum configuration with NuttShell

- usbnsh
    USB CDC/ACM serial console with NuttShell

License exceptions
==================

The following files are originated from the files in Pico SDK.
So, the files are licensed under 3-Clause BSD same as Pico SDK.

- arch/arm/src/rp23xx/rp23xx_clock.c
- arch/arm/src/rp23xx/rp23xx_pll.c
- arch/arm/src/rp23xx/rp23xx_xosc.c
  - These are created by referring the Pico SDK clock initialization.

- arch/arm/src/rp23xx/hardware/*.h
- arch/arm/src/rp23xx/pico/*.h
  - These are originally provided in Pico SDK.
