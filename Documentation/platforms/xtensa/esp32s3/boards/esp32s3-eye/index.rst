=============
ESP32-S3-EYE
=============

The `ESP32-S3-EYE <https://github.com/espressif/esp-who/blob/master/docs/en/get-started/ESP32-S3-EYE_Getting_Started_Guide.md>`_ is a small-sized AI development board from Espressif featuring the ESP32-S3 CPU with a 2-Megapixel camera, an LCD display, a microphone, an 8 MB Octal PSRAM and an 8 MB flash.

.. list-table::
   :align: center

   * - .. figure:: esp32s3_eye.png
          :align: center

Features
========

  - ESP32-S3 WROOM-1 Module
  - USB micro USB port (it uses internal USB JTAG SERIAL device for flashing)
  - Power LED
  - LCD Display
  - 2-MP Camera
  - MEMS Microphone
  - 8MB Octal PSRAM
  - 8MB SPI Flash
  - RST and BOOT buttons (BOOT accessible to user)

Configurations
==============

nsh
---

Basic NuttShell configuration (console enabled in USB JTAG SERIAL Device, exposed via
USB connection at 9600 bps).

You can configure this board profile this way:

  $ cd nuttxspace/nuttx
  $ ./tools/configure.sh esp32s3-eye:nsh

Flashing
========

Because ESP32-S3 doesn't use an external USB/Serial chip like others ESP32
boards you should put it in programming mode this way:

  1) Press and hold BOOT and RESET (RST) buttons at same time;
  2) Release the RESET button and keep BOOT button pressed;
  3) After one or more seconds release the BOOT button;
  4) Run the flashing command: make flash ESPTOOL_PORT=/dev/ttyACM0

Serial Console
==============

The internal USB JTAG SERIAL Device, by default, is used as serial console.
It is normally detected by Linux host as a USB CDC/ACM serial device.

It will show up as /dev/ttyACM[n] where [n] will probably be 0.

You can use minicom with /dev/ttyACM0 port at 9600 8n1 or picocom this way:

  $ picocom -b9600 /dev/ttyACM0

Buttons and LEDs
================

Buttons
-------

There are two buttons labeled BOOT and RST.  The RST button is not available
to software.  It pulls the chip enable line that doubles as a reset line.

The BOOT button is connected to IO0.  On reset it is used as a strapping
pin to determine whether the chip boots normally or into the serial
bootloader.  After reset, however, the BOOT button can be used for software
input.

LEDs
----

There are several on-board LEDs for that indicate the presence of power
and USB activity.  None of these are available for use by software.

Pin Mapping
===========

.. todo:: To be updated

===== ========== ==========
Pin   Signal     Notes
===== ========== ==========
?     ?          ?
===== ========== ==========

