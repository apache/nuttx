============
RTL8720F EVB
============

.. tags:: chip:rtl8720f, arch:arm, vendor:realtek

.. todo::

   Add a photo of the RTL8720F EVB board here as ``rtl8720f_evb.png`` in this
   directory, referenced with a ``.. figure::`` directive.

The RTL8720F EVB is a Realtek RTL8720F (dual-core Wi-Fi Host Controller)
evaluation board. NuttX runs on the KM4TZ (Cortex-M33, secure world)
application core as the Wi-Fi host; the KM4NS network processor runs the
prebuilt vendor Wi-Fi firmware. See the
:doc:`RTL8720F chip documentation <../../index>` for the SoC architecture,
memory map and vendor-SDK dependency.

Features
========

* RTL8720F dual-core Wi-Fi Host Controller (KM4TZ Cortex-M33 secure host +
  KM4NS NP)
* 2.4 GHz Wi-Fi (station and SoftAP)
* SPI NOR flash (shared with the NP, XIP)
* LOG-UART console

Supported in this NuttX port:

* NSH shell over the LOG-UART console (NuttX owns LOG-UART RX directly; the NP
  shell is disabled)
* littlefs persistent storage mounted at ``/data`` (a dedicated SPI NOR flash
  partition), backing the Wi-Fi key-value store
* Wi-Fi station and SoftAP through the ``wapi`` tool
* DHCP client (STA) and DHCP server (SoftAP)

.. note::

   The Wi-Fi MAC/PHY is driven by the prebuilt vendor firmware on the KM4NS
   network processor. NuttX is the Wi-Fi host and exchanges frames with the NP
   over the on-chip IPC transport; see the chip documentation.

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

This produces ``nuttx/app.bin`` (the NuttX KM4TZ image2), ``boot.bin`` and the
NP (KM4NS) image in the build directory.

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

* The prebuilt KM4NS (NP) Wi-Fi firmware image and the Realtek ``ameba-rtos``
  SDK libraries/headers linked into the image. See the SDK's own license; the
  SDK is auto-fetched and is not redistributed in the NuttX tree.
