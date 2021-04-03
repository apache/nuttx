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

mqttc
-----

This configuration tests the MQTT-C publisher example.

From the host, start the broker and subscribe to the :code:`test` topic.  Using
`mosquitto` this should be::

    mosquitto&
    mosquitto_sub -t test

From the NSH, connect to an access point::

    wapi psk wlan0 mypasswd 1
    wapi essid wlan0 myssid 1
    renew wlan0

Publish to the broker::

    nsh> mqttc_pub -h 192.168.1.11

The default behavior is to publish the message :code:`test`.  The following should be
outputted::

    nsh> mqttc_pub -h 192.168.1.11
         Success: Connected to broker!
         Success: Published to broker!

         Disconnecting from 192.168.1.11

From the host the message :code:`test` should be outputted.

