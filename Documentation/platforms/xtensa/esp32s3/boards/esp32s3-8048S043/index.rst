================
ESP32S3-8048S043
================

.. tags:: chip:esp32, chip:esp32s3

The `ESP32S3-8048S343 <https://www.openhasp.com/0.7.0/hardware/sunton/esp32-8048s0xx/>` is a dual-core MCU, integrated WI-FI and Bluetooth functions, the main frequency can reach 240MHz, 512KB SRAM, 384KB ROM, 8M PSRAM, Flash size is 16MB.

.. list-table::
   :align: center

   * - .. figure:: esp32s3-8048S043.png
          :align: center

Features
========

  - ESP32-S3 WROOM-1 Module
  - USB Type-C ports
  - Power LED
  - LCD Display
  - 8MB Octal PSRAM
  - 16MB SPI Flash
  - TF card slot
  - RST and BOOT buttons (BOOT accessible to user)

Serial Console
==============

UART0 is, by default, the serial console.  It connects to the on-board
CP2102 converter and is available on the USB connector USB CON8 (J1).

It will show up as /dev/ttyUSB[n] where [n] will probably be 0.

Buttons and I/Os
================

The board has two buttons: Boot and EN. The EN button functions only as 
reset and is not available to software. The BOOT button (connected to IO0)
determines boot mode during reset and can be used as software input after
initialization. Most I/O pins are available on the back of the board through 
two JST 1.28mm 4P connectors.

Configurations
==============

All of the configurations presented below can be tested by running the following commands::

    $ ./tools/configure.sh esp32s3-8048S043:<config_name>
    $ make flash ESPTOOL_PORT=/dev/ttyUSB0 -j

Where <config_name> is the name of board configuration you want to use, i.e.: nsh, buttons, tc...
Then use a serial console terminal like ``picocom`` configured to 115200 8N1.

buttons
-------

This configuration shows the use of the buttons subsystem. It can be used by executing
the ``buttons`` application and pressing on any of the available board buttons::

    nsh> buttons
    buttons_main: Starting the button_daemon
    buttons_main: button_daemon started
    button_daemon: Running
    button_daemon: Opening /dev/buttons
    button_daemon: Supported BUTTONs 0x01
    nsh> Sample = 1
    Sample = 0

nsh
---

Basic NuttShell configuration (console enabled in UART0, exposed via
USB connection by means of CP2102 converter, at 115200 bps).

i2c
------

This configuration can be used to scan and manipulate I2C devices.
You can scan for all I2C devices using the following command::

    nsh> i2c dev 0x00 0x7f

touchscreen
------------

The LCD panel comes with the integrated capacitive touchscreen sensor 
GT911 connected to the pins 20 (SCL), 19 (SDA) and address 0x5D::

    nsh> tc
    tc_main: nsamples: 0
    tc_main: Opening /dev/input0
    Sample     :
       npoints : 1
    Point 1    :
            id : 0
         flags : 31
             x : 149
             y : 140
             h : 0
             w : 0
      pressure : 45
     timestamp : 0
