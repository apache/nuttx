================
PINE64 PinePhone
================

`PinePhone <https://wiki.pine64.org/index.php/PinePhone>`_ is an ARM64 smartphone created by PINE64.

Features
========

- **System on Chip:** Allwinner A64
    - **CPU:** Quad-Core ARM Cortex-A53
    - **GPU:** ARM Mali400 MP2
    - **Interrupt Controller:** ARM GIC PL400 (Generic Interrupt Controller v2)
    - **Display Engine:** Allwinner Display Engine 2.0 (MIPI DSI with DMA)
- **Display:** Xingbangda XBD599 HD IPS Capacitive Touchscreen (5.95 inches, 1440x720 resolution, 16M colors)
- **LCD Controller:** Sitronix ST7703 (MIPI DSI)
- **RAM:** 2GB or 3GB LPDDR3 SDRAM
- **Internal Storage:** 16GB or 32GB eMMC, extendable up to 2TB via microSD
- **Back Camera:** Single 5MP, 1/4", LED Flash
- **Front Camera:** Single 2MP, f/2.8, 1/5"
- **Sound:** Loudspeaker, 3.5mm jack & mic (jack doubles as Hardware UART if Privacy Switch 6 is Off)
- Modem: Quectel EG25-G
    - **LTE-FDD, LTE-TDD, WCDMA, GSM**
    - **GNSS:** GPS / GLONASS / BeiDou / Galileo / QZSS, with A-GPS
- 2.4 GHz Wireless: Realtek RTL8723CS
    - **WLAN:** WiFi 802.11 b/g/n, single-band, hotspot
    - **Bluetooth:** 4.0, A2DP
- **Sensors:** Accelerometer, Gyroscope, Proximity, Ambient Light, Compass
- **Privacy Switches:** Modem, WiFi & Bluetooth, Microphone, Cameras, Headphone
- **Battery:** Lithium-ion, rated capacity 2800mAh (10.64Wh), typical capacity 3000mAh (11.40Wh)
- **I/O:** USB Type-C, USB Host, DisplayPort Alternate Mode output, 15W 5V 3A Quick Charge, follows USB PD specification

Serial Console
==============

A `PinePhone Serial Debug Cable <https://wiki.pine64.org/index.php/PinePhone#Serial_console>`_
is required to run NuttX on PinePhone.

On PinePhone, set the `Privacy Switch 6 (Headphone) <https://wiki.pine64.org/index.php/PinePhone#Privacy_switch_configuration>`_
to **Off**.

Connect PinePhone to our computer with the Serial Debug Cable.
On our computer, start a Serial Terminal and connect to the USB Serial Port
at **115.2 kbps**.

NuttX will appear in the Serial Console when it boots on PinePhone.

ARM64 Toolchain
===============

Before building NuttX for PinePhone, download the ARM64 Toolchain for
**AArch64 Bare-Metal Target** ``aarch64-none-elf`` from
`Arm GNU Toolchain Downloads <https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads>`_.
(Skip the section for Beta Releases)

Add the downloaded toolchain ``gcc-arm-...-aarch64-none-elf/bin``
to the ``PATH`` Environment Variable.

Check the ARM64 Toolchain:

.. code:: console

   $ aarch64-none-elf-gcc -v

Building
========

To build NuttX for PinePhone, :doc:`install the prerequisites </quickstart/install>` and
:doc:`clone the git repositories </quickstart/install>` for ``nuttx`` and ``apps``.

Configure the NuttX project and build the project:

.. code:: console

   $ cd nuttx
   $ tools/configure.sh pinephone:nsh
   $ make
   $ cp nuttx.bin Image
   $ rm -f Image.gz
   $ gzip Image

This produces the file ``Image.gz``, which will be copied to PinePhone in the next step.

If the build fails with the error ``token "@" is not valid in preprocessor``,
`apply this patch <https://github.com/apache/nuttx/pull/7284/commits/518b0eb31cb66f25b590ae9a79ab16c319b96b94#diff-12291efd8a0ded1bc38bad733d99e4840ae5112b465c04287f91ba5169612c73>`_
to ``gcc-arm-none-eabi/arm-none-eabi/include/_newlib_version.h``
in the ARM64 Toolchain.

Booting
=======

NuttX boots on PinePhone via a microSD Card. To prepare the microSD Card, download the
**PinePhone Jumpdrive Image** ``pine64-pinephone.img.xz`` from
`dreemurrs-embedded/Jumpdrive <https://github.com/dreemurrs-embedded/Jumpdrive/releases>`_.

Write the downloaded image to a microSD Card with
`Balena Etcher <https://www.balena.io/etcher/>`_.

Copy the file ``Image.gz`` from the previous section
and overwrite the file on the microSD Card.

Check that PinePhone is connected to our computer via a
`Serial Debug Cable <https://wiki.pine64.org/index.php/PinePhone#Serial_console>`_ at 115.2 kbps.
`Privacy Switch 6 (Headphone) <https://wiki.pine64.org/index.php/PinePhone#Privacy_switch_configuration>`_
should be set to **Off**.

Insert the microSD Card into PinePhone and power up PinePhone.
NuttX boots on PinePhone and NuttShell (nsh) appears in the Serial Console.

To see the available commands in NuttShell:

.. code:: console

   $ help

LEDs
====

The supported PinePhone LEDs are:

===== ========= ===
Index LED       PIO
===== ========= ===
LED1  Green LED PD18
LED2  Red LED   PD19
LED3  Blue LED  PD20
===== ========= ===

Configurations
==============

nsh
---

Basic configuration that runs NuttShell (nsh).
This configuration is focused on low level, command-line driver testing.
Built-in applications are supported, but none are enabled.
Serial Console is enabled on UART0 at 115.2 kbps.

Peripheral Support
==================

NuttX for PinePhone supports these peripherals:

=========== ======= =====
Peripheral  Support NOTES
=========== ======= =====
MIPI D-PHY   Yes
MIPI DSI     Yes
PIO          Yes
UART         Yes    Only UART0 is supported
=========== ======= =====
