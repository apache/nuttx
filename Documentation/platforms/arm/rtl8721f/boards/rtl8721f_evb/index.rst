============
RTL8721F EVB
============

.. tags:: chip:rtl8721f, arch:arm, vendor:realtek

.. figure:: rtl8721f_evb.png
   :scale: 50 %
   :align: center
   :alt: Realtek RTL8721F EVB development board

   The RTL8721F EVB development board.

The RTL8721F EVB is a Realtek RTL8721F evaluation board. NuttX runs on the
KM4TZ application core — an Arm Cortex-M55-compatible core running at up to
320 MHz. See the :doc:`RTL8721F chip documentation <../../index>` for the
full SoC specifications and the vendor-SDK dependency.

Features
========

* RTL8721F: Arm Cortex-M55-compatible KM4TZ core up to 320 MHz,
  512 KB SRAM, 4 MB NOR flash
* Wi-Fi 6 (802.11 a/b/g/n/ax), dual-band 2.4/5 GHz station and SoftAP
* SPI NOR flash (XIP)
* LOG-UART console

Supported in this NuttX port:

* NSH shell over the LOG-UART console
* littlefs persistent storage mounted at ``/data`` (a dedicated SPI NOR flash
  partition)
* Wi-Fi station and SoftAP through the ``wapi`` tool
* DHCP client (STA) and DHCP server (SoftAP)
* GPIO pins exposed as ``/dev/gpioN`` character devices (input, output and
  interrupt), driven directly on the SDK fwlib register layer
* General-purpose UARTs exposed as ``/dev/ttySN`` serial devices, driven
  directly on the SDK fwlib register layer

Buttons and LEDs
================

This NuttX port does not wire any user buttons or LEDs.

Configurations
==============

Build and flash any of these per the :doc:`RTL8721F build instructions
<../../index>`; for the CMake build, source ``. tools/ameba/env.sh
rtl8721f_evb`` first (the make build needs no sourcing).

.. code:: console

   $ ./tools/configure.sh rtl8721f_evb:<config-name>

gpio
----

Minimal NSH with the GPIO driver and the ``gpio`` example enabled (no Wi-Fi).
The board registers three pins from its pin table (see
``boards/arm/rtl8721f/rtl8721f_evb/src/rtl8721f_gpio.c``): an output at
``/dev/gpio0``, an input at ``/dev/gpio1`` and an interrupt pin at
``/dev/gpio2``. Edit that table to match a board's wiring. Exercise them with
the example::

    nsh> gpio -o 1 /dev/gpio0     # drive the output high
    nsh> gpio /dev/gpio1          # read the input
    nsh> gpio -w 1 /dev/gpio2     # wait for a falling-edge interrupt

Pins are encoded with the ``AMEBA_PA()`` / ``AMEBA_PB()`` helpers from
``arch/arm/src/common/ameba/ameba_gpio.h`` (port A/B, pin 0-31), matching the
Ameba SDK ``PinName`` layout.

uart
----

Minimal NSH with the general-purpose UART driver and the ``serialrx`` /
``serialblaster`` examples enabled (no Wi-Fi). The LOG-UART owns the console
and ``/dev/ttyS0``, so the board registers UART0 from its table (see
``boards/arm/rtl8721f/rtl8721f_evb/src/rtl8721f_uart.c``) as ``/dev/ttyS1`` at
115200 8N1. Edit that table -- controller, TX/RX pads and baud -- to match a
board's wiring. The TX/RX pads use the same ``AMEBA_PA()`` / ``AMEBA_PB()``
encoding as the GPIO table; the driver muxes them to the UART function and
pulls RX high through the SDK ROM. Exercise the port with the examples (loop TX
back to RX, or wire it to a host serial adapter)::

    nsh> serialrx /dev/ttyS1 2600 &     # start the receiver first
    nsh> serialblaster /dev/ttyS1 2600  # then loop TX back to RX

The line format can be changed at runtime through ``tcsetattr()`` (the config
enables ``CONFIG_SERIAL_TERMIOS``). UART3 is shared with Bluetooth and is not
exposed by the driver.

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

License Exceptions
==================

This board depends on Realtek vendor code that is not part of NuttX and is
subject to its own license:

* The prebuilt Wi-Fi / Bluetooth firmware image and the Realtek ``ameba-rtos``
  SDK libraries/headers linked into the image. See the SDK's own license; the
  SDK is auto-fetched and is not redistributed in the NuttX tree.
