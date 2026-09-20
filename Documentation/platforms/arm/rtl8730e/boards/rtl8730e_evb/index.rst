============
RTL8730E_EVB
============

.. tags:: chip:rtl8730e, arch:arm, vendor:realtek

.. image:: img/rtl8730e_evb.png
   :align: center
   :alt: PKE8730EAH-VD3-F32 evaluation board

The RTL8730E_EVB is a Realtek RTL8730E evaluation board built around the
RTL8730EH (DDR2 64 MB, 16 MB NOR flash).  NuttX runs on the dual-core Arm
Cortex-A32 application processor at up to 1.2 GHz as BL33 in the ATF boot
chain.  See the :doc:`RTL8730E chip documentation <../../index>` for the full
SoC specifications and vendor-SDK dependency.

Features
========

* RTL8730EH: dual-core Arm Cortex-A32 up to 1.2 GHz, 64 MB DDR2, 16 MB NOR
  flash
* Symmetric Multi-Processing (SMP) on both Cortex-A32 cores
* Wi-Fi 6 (802.11ax) dual-band (2.4 / 5 GHz) station and SoftAP
* Bluetooth 5 dual-mode (BR/EDR + LE)
* SPI NOR flash with littlefs at ``/data``
* LOG-UART console at 1500000 8N1

Supported in this NuttX port:

* NSH shell over the LOG-UART console
* SMP on both Cortex-A32 cores; tasks are dispatched across CPU0 and CPU1
  by the NuttX SMP scheduler
* littlefs persistent storage mounted at ``/data`` (a dedicated SPI NOR flash
  partition), backing the Wi-Fi key-value store
* Wi-Fi station and SoftAP through the ``wapi`` tool
* DHCP client (STA) and DHCP server (SoftAP)
* ``/etc`` read-only ROMFS populated from the board's ``src/etc/`` tree

Configurations
==============

Build and flash any of these per the :doc:`RTL8730E build instructions
<../../index>`.

.. code:: console

   $ ./tools/configure.sh rtl8730e_evb:<config-name>

nsh
---

Networking-enabled NSH with SMP, littlefs at ``/data``, and the ``wapi``
Wi-Fi tool.  The console is the LOG-UART at 1500000 8N1.

Wi-Fi
=====

Station (connect to an AP)::

    nsh> wapi mode  wlan0 2
    nsh> wapi psk   wlan0 <password> 3
    nsh> wapi essid wlan0 <ssid> 1
    nsh> renew wlan0

SoftAP (become an access point, with a DHCP server for clients)::

    nsh> wapi mode   wlan0 3
    nsh> wapi psk    wlan0 <password> 3
    nsh> wapi essid  wlan0 <ssid> 1
    nsh> dhcpd_start wlan0

Stop the SoftAP with ``wapi essid wlan0 <ssid> 0``.

License Exceptions
==================

This board depends on Realtek vendor code that is not part of NuttX and is
subject to its own license:

* The prebuilt Wi-Fi / Bluetooth firmware image and the Realtek ``ameba-rtos``
  SDK libraries/headers linked into the image.  See the SDK's own license; the
  SDK is auto-fetched and is not redistributed in the NuttX tree.
* The prebuilt ATF BL2 and BL32 binaries under ``prebuilt/`` are provided by
  Realtek and are subject to the Realtek binary license included in the SDK.
