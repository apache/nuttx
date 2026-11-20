=====================
Espressif ESP32-S3-R8
=====================

.. tags:: arch:xtensa, chip:esp32s3, vendor:espressif

The ESP32-S3-R8 is an ESP32-S3-EYE variant based on the ESP32-S3-WROOM-2-N32R8V
module with 32 MB octal flash and 8 MB octal PSRAM. It provides an ST7789 LCD,
SDMMC and SPI flash MTD.

Configurations
==============

The following configurations are provided:

  - gpio
  - i2c
  - lcd
  - nsh
  - openvela
  - usbnsh
  - wifi

They can be built with::

    $ ./tools/configure.sh esp32s3-r8:<config_name>
    $ make -j

Serial Console
==============

The console is available on the ESP32-S3 internal USB JTAG serial device,
typically enumerated as ``/dev/ttyACM0``.
