===============
Espressif ESP32
===============

ESP32 series of SoCs from Espressif are single and dual-core chips based on
Xtensa architecture, with support for Bluetooth and WiFi. 

Toolchain
=========

You can use the prebuilt `compiler <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-tools.html#xtensa-esp32-elf>`__ for Xtensa architecture and `OpenOCD <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-tools.html#openocd-esp32>`__ for ESP32 by Espressif. For flashing firmware, you will need to install ``esptool.py`` by running::

    pip install esptool


Flashing
========

Firmware for ESP32 is flashed via the USB/UART interface using the ``esptool.py`` tool. To flash your NuttX firmware simply run::

    make download ESPTOOL_PORT=<port>

where ``<port>`` is typically ``/dev/ttyUSB0`` or similar. You can change the baudrate by passing ``ESPTOOL_BAUD``.

Bootloader and partitions
-------------------------

ESP32 requires a bootloader to be flashed as well as a set of FLASH partitions. This is only needed the first time
(or any time you which to modify either of these). An easy way is to use prebuilt binaries for NuttX from `here <https://github.com/espressif/esp-nuttx-bootloader>`_. In there you will find instructions to rebuild these if necessary. 
Once you downloaded both binaries, you can flash them by adding an ``ESPTOOL_BINDIR`` parameter, pointing to the directiry where these binaries were downloaded::

    make download ESPTOOL_PORT=<port> ESPTOOL_BINDIR=<dir>

.. note:: It is recommended that if this is the first time you are using the board with NuttX that you perform a complete
   SPI FLASH erase::
       
       esptool.py erase_flash

Peripheral Support
==================

.. todo:: To be updated

========== ======= =====
Peripheral Support NOTES
========== ======= =====
?          ?       ?
========== ======= =====

WiFi
====

A standard network interface will be configured and can be initialized such as::

    ifup wlan0
    wapi psk wlan0 mypasswd 1
    wapi essid wlan0 myssid 1
    renew wlan0

In this case a connection to AP with SSID ``myssid`` is done, using ``mypasswd`` as
password. IP address is obtained via DHCP using ``renew`` command. You can check
the result by running ``ifconfig`` afterwards.

.. tip:: Boards usually expose a ``wapi`` defconfig which enables WiFi

Bluetooth
=========

Bluetooth is not currently supported.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
