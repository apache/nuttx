README
======

This directory contains the porting of NuttX to the Raspberry Pi Pico 2.
See https://www.raspberrypi.org/products/raspberry-pi-pico-2/ for information
about Raspberry Pi Pico 2.

NuttX supports the following RP2350 capabilities:
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
  $ make -j

4. Connect Raspberry Pi Pico 2 board to the USB port while pressing BOOTSEL.
   The board will be detected as USB Mass Storage Device.
   Then copy "nuttx.uf2" into the device.
   (Same manner as the standard Pico SDK applications installation.)

5. To access the console, GPIO 0 and 1 pins must be connected to the
   device such as a USB-serial converter.

   `usbnsh` configuration provides the console access by USB CDC/ACM serial
   device. The console is available by using a terminal software on the USB
   host.

Defconfigs
==========

- nsh
    Minimum configuration with NuttShell

- usbnsh
    USB CDC/ACM serial console with NuttShell
