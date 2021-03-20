=============
ESP32 DevKitC
=============

The `ESP32 DevKitC <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/modules-and-boards.html#esp32-devkitc-v4>`_ is a development board for the ESP32 SoC from Espressif, based on a ESP-WROOM-32 module. You can find the original V2 version and the newer V4 variant. They are
pin compatible.

.. list-table::
   :align: center

   * - .. figure:: esp32-core-board-v2.jpg
          :align: center

          ESP32 DevKitC/Core V2

     - .. figure:: esp32-devkitc-v4-front.jpg
          :align: center

          ESP32 DevKitC V4

Features
========

  - ESP32 WROOM Module
  - USB-to-UART bridge via micro USB port
  - Power LED
  - EN and BOOT buttons (BOOT accessible to user)
  - SPI FLASH (size varies according to model

Pin Mapping
===========

.. todo:: To be updated

===== ========== ==========
Pin   Signal     Notes
===== ========== ==========
?     ?          ?
===== ========== ==========

Configurations
==============

nsh
---

Basic NuttShell configuration (console enabled in UART0, exposed via USB connection, at 115200 bps).

wapi
----

Enables WiFi support
