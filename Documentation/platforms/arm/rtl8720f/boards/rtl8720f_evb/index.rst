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
* GPIO pins exposed as ``/dev/gpioN`` character devices (input, output and
  interrupt), driven directly on the SDK fwlib register layer
* General-purpose UARTs exposed as ``/dev/ttySN`` serial devices, driven
  directly on the SDK fwlib register layer
* I2C master buses exposed as ``/dev/i2cN`` character devices, driven directly
  on the SDK fwlib register layer
* SPI master buses exposed as ``/dev/spiN`` character devices, driven directly
  on the SDK fwlib register layer

Buttons and LEDs
================

This NuttX port does not wire any user buttons or LEDs.

Configurations
==============

Build and flash any of these per the :doc:`RTL8720F build instructions
<../../index>`; for the CMake build, source ``. tools/ameba/env.sh
rtl8720f_evb`` first (the make build needs no sourcing).

.. code:: console

   $ ./tools/configure.sh rtl8720f_evb:<config-name>

gpio
----

Minimal NSH with the GPIO driver and the ``gpio`` example enabled (no Wi-Fi).
The board registers three pins from its pin table (see
``boards/arm/rtl8720f/rtl8720f_evb/src/rtl8720f_gpio.c``): an output at
``/dev/gpio0``, an input at ``/dev/gpio1`` and an interrupt pin at
``/dev/gpio2``. Edit that table to match a board's wiring. Exercise them with
the example::

    nsh> gpio -o 1 /dev/gpio0     # drive the output high
    nsh> gpio /dev/gpio1          # read the input
    nsh> gpio -w 1 /dev/gpio2     # wait for a falling-edge interrupt

RTL8720F drives all GPIO through a single port A controller, so pins are
encoded with the ``AMEBA_PA()`` helper from
``arch/arm/src/common/ameba/ameba_gpio.h`` (pin 0-31), matching the Ameba SDK
``PinName`` layout.

uart
----

Minimal NSH with the general-purpose UART driver and the ``serialrx`` /
``serialblaster`` examples enabled (no Wi-Fi). The LOG-UART owns the console
and ``/dev/ttyS0``, so the board registers UART0 from its table (see
``boards/arm/rtl8720f/rtl8720f_evb/src/rtl8720f_uart.c``) as ``/dev/ttyS1`` at
115200 8N1. Edit that table -- controller, TX/RX pads and baud -- to match a
board's wiring. The TX/RX pads use the same ``AMEBA_PA()`` encoding as the
GPIO table; the driver muxes them to the UART function and pulls RX high
through the SDK ROM. Exercise the port with the examples (loop TX back to RX,
or wire it to a host serial adapter)::

    nsh> serialrx /dev/ttyS1 2600 &     # start the receiver first
    nsh> serialblaster /dev/ttyS1 2600  # then loop TX back to RX

The line format can be changed at runtime through ``tcsetattr()`` (the config
enables ``CONFIG_SERIAL_TERMIOS``). UART2 is not exposed by the driver.

i2c
---

Minimal NSH with the I2C master driver and the ``i2ctool`` (``system/i2c``)
enabled (no Wi-Fi). The board registers its I2C controllers from a table (see
``boards/arm/rtl8720f/rtl8720f_evb/src/rtl8720f_i2c.c``): I2C0 at ``/dev/i2c0``
on PA22/PA23 and I2C1 at ``/dev/i2c1`` on PA24/PA25. Edit that table --
controller and SCL/SDA pads -- to match a board's wiring; the pads use the
same ``AMEBA_PA()`` encoding as the GPIO table and are muxed to the I2C
function through the SDK ROM. The I2C bus is open-drain, so fit external
pull-ups on SCL/SDA. Probe a bus with the tool::

    nsh> i2c dev -b 0 0x03 0x77     # scan /dev/i2c0 for devices

spi
---

Minimal NSH with the SPI master driver and the ``spi`` tool
(``system/spi``) enabled (no Wi-Fi). The board registers SPI0 at
``/dev/spi0`` from its table (see
``boards/arm/rtl8720f/rtl8720f_evb/src/rtl8720f_spi.c``) with CLK/MOSI/MISO on
PA14/PA15/PA16 and a software chip-select on PA17. Edit that table --
controller, CLK/MOSI/MISO pads and CS pad -- to match a board's wiring; the
pads use the same ``AMEBA_PA()`` encoding as the GPIO table (RTL8720F drives
all GPIO through a single port A controller) and are muxed to the SPI function
through the SDK ROM, while the chip-select is driven as a plain GPIO. Note
that not every pad can carry every SPI signal -- pick pads the SDK pin mux
actually routes to the controller. Exercise a bus with the tool::

    nsh> spi exch -b 0 -x 4 deadbeef     # full-duplex transfer on /dev/spi0

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
