README
======

This directory contains the port of NuttX to the Seeed Studio Xiao RP2040.
See https://wiki.seeedstudio.com/XIAO-RP2040/ for information about Seeed
Studio Xiao RP2040.

NuttX supports the following RP2040 capabilities:
  - UART  (console port)
    - GPIO 0 (UART0 TX) and GPIO 1 (UART0 RX) are used for the console.
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

NuttX also provides support for these external devices:

  - BMP180 sensor at I2C0 (don't forget to define I2C0 GPIOs at "I2C0 GPIO pin assign" in Board Selection menu)
  - INA219 sensor / module (don't forget to define I2C0 GPIOs at "I2C0 GPIO pin assign" in Board Selection menu)
  - WS2812 smart pixel support

There is currently no direct user mode access to these RP2040 hardware features:
  - SPI Slave Mode
  - SSI
  - RTC
  - Timers

Installation
============

1. Download Raspberry Pi Pico SDK and update submodule(cyw43-driver)

  $ git clone -b 1.4.0 https://github.com/raspberrypi/pico-sdk.git
  $ cd pico-sdk
  $ git submodule update --init --recursive lib/cyw43-driver

2. Set PICO_SDK_PATH environment variable

  $ export PICO_SDK_PATH=<absolute_path_to_pico-sdk_directory>

3. Configure and build NuttX

  $ git clone https://github.com/apache/nuttx.git nuttx
  $ git clone https://github.com/apache/nuttx-apps.git apps
  $ cd nuttx
  $ make distclean
  $ ./tools/configure.sh seeed-xiao-rp2040:nsh
  $ make V=1

4. Connect Seeed Studio Xiao RP2040 board to USB port while pressing BOOTSEL.
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
