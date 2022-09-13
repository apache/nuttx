README
======

This directory contains the port of NuttX to the Adafruit QT Py RP2040.
See https://learn.adafruit.com/adafruit-qt-py-2040 for information 
about Adafruit QT Py RP2040.

NuttX supports the following RP2040 capabilities:
  - UART  (console port)
    - GPIO 5 (UART1 RX) and GPIO 20 (UART1 TX) are used for the console.
  - I2C
  - SPI (master only)
  - DMAC
  - PWM
  - ADC
  - Watchdog
  - USB device
    - MSC, CDC/ACM serial and these composite device are supported.
    - CDC/ACM serial device can be used for the console.
  - PIO (RP2040 Programmable I/O)
  - Flash ROM Boot
  - SRAM Boot
    - If Pico SDK is available, nuttx.uf2 file which can be used in
      BOOTSEL mode will be created.
  - Persistent flash filesystem in unused flash ROM

NuttX also provide support for these external devices:

  - WS2812 smart pixel support

There is currently no direct user mode access to these RP2040 hardware features:
  - SPI Slave Mode
  - SSI
  - RTC
  - Timers

Installation
============

1. Download Raspberry Pi Pico SDK

  $ git clone -b 1.1.2 https://github.com/raspberrypi/pico-sdk.git

2. Set PICO_SDK_PATH environment variable

  $ export PICO_SDK_PATH=<absolute_path_to_pico-sdk_directory>

3. Configure and build NuttX

  $ git clone https://github.com/apache/incubator-nuttx.git nuttx
  $ git clone https://github.com/apache/incubator-nuttx-apps.git apps
  $ cd nuttx
  $ make distclean
  $ ./tools/configure.sh raspberrypi-pico:nsh
  $ make V=1

4. Connect Adafruit KB2040 board to USB port while pressing BOOTSEL.
   The board will be detected as USB Mass Storage Device.
   Then copy "nuttx.uf2" into the device.
   (Same manner as the standard Pico SDK applications installation.)

5. To access the console, GPIO 20 (TX) and 5 (RX) pins must be connected to a
   device such as USB-serial converter.

   `usbnsh` configuration provides the console access by USB CDC/ACM serial
   devcice.  The console is available by using a terminal software on the USB
   host.

Defconfigs
==========

- nsh
    Minimum configuration with NuttShell

- nsh-flash
    NuttX shell with SMART flash filesystem.

- nshsram
    Load NuttX binary to SRAM
  
- smp
    Enable SMP mode. Both Core 0 and Core 1 are used by NuttX.

- usbnsh
    USB CDC/ACM serial console with NuttShell

- composite
    USB composite device (MSC + CDC/ACM) support
    `conn` command enables the composite device.

License exceptions
==================

The following files are originated from the files in Pico SDK.
So, the files are licensed under 3-Clause BSD same as Pico SDK.

- arch/arm/src/rp2040/rp2040_clock.c
- arch/arm/src/rp2040/rp2040_pll.c
- arch/arm/src/rp2040/rp2040_xosc.c
  - These are created by referring the Pico SDK clock initialization.

- arch/arm/src/rp2040/rp2040_pio.c
- arch/arm/src/rp2040/rp2040_pio.h
- arch/arm/src/rp2040/rp2040_pio_instructions.h
  - These provide the similar APIs to Pico SDK's hardware_pio APIs.

- arch/arm/src/rp2040/hardware/*.h
  - These are generated from rp2040.svd originally provided in Pico SDK.
