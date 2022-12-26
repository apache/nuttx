==============
ESP32S3-DevKit
==============

The `ESP32S3 DevKit <https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html>`_ is a development board for the ESP32-S3 SoC from Espressif, based on a ESP32-S3-WROOM-1 module.

.. list-table::
   :align: center

   * - .. figure:: esp32-s3-devkitc-1.png
          :align: center

Features
========

  - ESP32-S3-WROOM-1 Module
  - USB-to-UART bridge via micro USB port
  - Power LED
  - EN and BOOT buttons (BOOT accessible to user)
  - SPI FLASH (size varies according to model

Serial Console
==============

UART0 is, by default, the serial console.  It connects to the on-board
CP2102 converter and is available on the USB connector USB CON8 (J1).

It will show up as /dev/ttyUSB[n] where [n] will probably be 0.

Buttons and LEDs
================

Buttons
-------

There are two buttons labeled Boot and EN.  The EN button is not available
to software.  It pulls the chip enable line that doubles as a reset line.

The BOOT button is connected to IO0.  On reset it is used as a strapping
pin to determine whether the chip boots normally or into the serial
bootloader.  After reset, however, the BOOT button can be used for software
input.

LEDs
----

There are several on-board LEDs for that indicate the presence of power
and USB activity.  None of these are available for use by software.

Configurations
==============

nsh
---

Basic NuttShell configuration (console enabled in UART0, exposed via
USB connection by means of CP2102 converter, at 115200 bps).

mcuboot_nsh
---

Similar configuration as nsh, except that it enables booting from
MCUboot and the experimental features configuration.
