============
RTL8720F EVB
============

.. tags:: chip:rtl8720f, arch:arm, vendor:realtek

.. todo::

   Add a photo of the RTL8720F EVB board here as ``rtl8720f_evb.png`` in this
   directory, referenced with a ``.. figure::`` directive.

The RTL8720F EVB is a Realtek RTL8720F evaluation board built around the
RTL8720FBF (QFN40, 4 MB NOR flash). NuttX runs on the KM4TZ application core —
an Arm Cortex-M55-compatible core running at up to 320 MHz. See the
:doc:`RTL8720F chip documentation <../../index>` for the full SoC
specifications and the vendor-SDK dependency.

Features
========

* RTL8720FBF: Arm Cortex-M55-compatible KM4TZ core up to 320 MHz, 512 KB SRAM,
  4 MB NOR flash
* Wi-Fi 6 (802.11 b/g/n/ax), 2.4 GHz station and SoftAP
* SPI NOR flash (XIP)
* LOG-UART console

Supported in this NuttX port:

* NSH shell over the LOG-UART console
* littlefs persistent storage mounted at ``/data`` (a dedicated SPI NOR flash
  partition), backing the Wi-Fi key-value store
* Wi-Fi station and SoftAP through the ``wapi`` tool
* DHCP client (STA) and DHCP server (SoftAP)

Buttons and LEDs
================

This NuttX port does not wire any user buttons or LEDs.

Configurations
==============

.. code:: console

   $ ./tools/configure.sh rtl8720f_evb:<config-name>

nsh
---

Networking-enabled NSH with littlefs at ``/data`` and the ``wapi`` Wi-Fi tool.
The console is the LOG-UART at 1500000 8N1 (the rate is configured by the
bootloader and inherited by NuttX). The Wi-Fi examples below are available from
this configuration.

Wi-Fi
=====

Station (connect to an AP)::

    nsh> wapi psk    wlan0 <password> 3
    nsh> wapi essid  wlan0 <ssid> 1
    nsh> renew wlan0

SoftAP (become an access point, with a DHCP server for clients)::

    nsh> wapi mode   wlan0 3
    nsh> wapi psk    wlan0 <password> 3
    nsh> wapi essid  wlan0 <ssid> 1
    nsh> ifconfig    wlan0 192.168.4.1 netmask 255.255.255.0
    nsh> dhcpd_start wlan0

Stop the SoftAP with ``wapi essid wlan0 <ssid> 0``.

Building and Flashing
=====================

The build auto-fetches the Realtek ``ameba-rtos`` SDK on first use and a
Realtek ``arm-none-eabi`` toolchain must be on ``PATH``; see the
:doc:`chip documentation <../../index>` for both.

.. code:: console

   $ ./tools/configure.sh rtl8720f_evb:nsh
   $ make

This produces the two flashable images in the build directory: ``app.bin``
(the NuttX application image, which also bundles the prebuilt Wi-Fi firmware)
and ``boot.bin`` (the bootloader).

After a successful build, flash via one of these methods:

**CLI (Linux/macOS)** — connect a USB-UART adapter (PL2303) and use the
built-in ``make flash`` target:

.. code:: console

   $ make flash AMEBA_PORT=/dev/ttyUSB0

The baud rate defaults to 1500000; override with ``AMEBA_BAUD`` if needed.

**GUI (Windows)** — use the Realtek AmebaImageTool (``AmebaImageTool.exe``
under ``tools/ameba/ImageTool/`` in the SDK tree) to select ``boot.bin`` and
``app.bin``.

See the `Realtek Ameba ImageTool guide
<https://aiot.realmcu.com/en/latest/tools/image_tool/index.html>`_ for the
Windows GUI tool and download-mode entry (hold the download button /
power-cycle with the ``UART_LOG_TX`` line asserted).

**Serial console** — after flashing, connect to the LOG-UART at 1500000 8N1::

    $ picocom -b 1500000 /dev/ttyUSB0

Other tools: ``screen /dev/ttyUSB0 1500000`` or ``minicom -b 1500000 -D /dev/ttyUSB0``.

License Exceptions
==================

This board depends on Realtek vendor code that is not part of NuttX and is
subject to its own license:

* The prebuilt Wi-Fi / Bluetooth firmware image and the Realtek ``ameba-rtos``
  SDK libraries/headers linked into the image. See the SDK's own license; the
  SDK is auto-fetched and is not redistributed in the NuttX tree.
