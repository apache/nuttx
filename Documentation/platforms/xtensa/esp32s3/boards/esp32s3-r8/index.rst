========================
Espressif ESP32-S3-R8
========================

.. tags:: arch:xtensa, chip:esp32s3, vendor:espressif

The ESP32-S3-R8 is an ESP32-S3 development board based on the
ESP32-S3-WROOM-2-N32R8V module. It is designed as a variant of the
ESP32-S3-EYE and provides an LCD and 32 MB SPI flash.

.. list-table::
   :align: center

   * - .. figure:: esp32s3-r8.png
          :align: center

Features
========

  - ESP32-S3-WROOM-2-N32R8V module
  - 32 MB SPI flash
  - 8 MB octal PSRAM
  - ST7789 LCD
  - User button connected to GPIO0
  - USB-Serial/JTAG console

Configurations
==============

The following configurations are provided:

  - ``nsh``: basic NuttShell configuration
  - ``gpio``: GPIO and button example
  - ``i2c``: I2C example
  - ``lcd``: LCD configuration
  - ``usbnsh``: NSH over USB CDC/ACM
  - ``wifi``: Wi-Fi configuration
  - ``openvela``: OpenVela configuration

To build a configuration:

.. code:: console

   $ ./tools/configure.sh esp32s3-r8:<config_name>
   $ make -j

For example:

.. code:: console

   $ ./tools/configure.sh esp32s3-r8:nsh
   $ make -j

Flashing
========

Connect the board over USB-Serial/JTAG and run:

.. code:: console

   $ make flash ESPTOOL_PORT=/dev/ttyACM0 ESPTOOL_BINDIR=./

Buttons and LEDs
================

Board Buttons
-------------

The board exposes a user button connected to GPIO0.

Board LEDs
----------

One LED is available through the GPIO subsystem and can be controlled from
NuttX.

Pin Mapping
===========

===== ========== ==========
Pin   Signal     Notes
===== ========== ==========
0     GPIO0      User button
46    GPIO46     ES8311 PA control
===== ========== ==========

Power Supply
============

Power is provided through the USB connector.

Serial Console
==============

The ESP32-S3 internal USB-Serial/JTAG interface is used as the default
serial console. On Linux it is typically detected as ``/dev/ttyACM0``.
