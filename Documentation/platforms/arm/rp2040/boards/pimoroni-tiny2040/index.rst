=================
Pimoroni Tiny2040
=================

.. tags:: chip:rp2040

The Tiny2040 is a general purpose RP2040 board supplied by Pimoroni. 

.. figure:: Tiny2040.png
   :align: center

The Pimoroni Tiny 2040 has two buttons (RESET and BOOT) allowing to boot
from ROM without disconnecting the device.

See the `Pimoroni website
<https://shop.pimoroni.com/products/tiny-2040?variant=39560012234835/>`_ for
information about the Pimoroni Tiny 2040.

Features
========

* RP2040 microcontroller chip
* Dual-core ARM Cortex M0+ processor, flexible clock running up to 133 MHz
* 264kB of SRAM, and 2MB or 8MB of on-board Flash memory
* Castellated module allows soldering direct to carrier boards
* USB Host and Device support via type C connector
* Low-power sleep and dormant modes
* Drag & drop programming using mass storage over USB
* 12 multi-function GPIO pins
* 2× SPI, 2× I2C, 2× UART, 3× 12-bit ADC, 16× controllable PWM channels
* Accurate clock and timer on-chip
* Temperature sensor
* Accelerated floating point libraries on-chip
* 8 × Programmable IO (PIO) state machines for custom peripheral support

Buttons and LEDs
================

There is a single onboard RGB LED controlled by pins
GPIO18 (red), GPIO19 (green), and GPIO20 (blue).

The is a User/BOOT button readable as GPIO23. If held down when power
is first applied the RP2040 will boot into program mode and appear to
a computer connected via USB as a storage device.  Saving
a .UF2 file to this device will replace the Flash ROM contents 
on the RP2040.

Pin Mapping
===========
Pads numbered anticlockwise from USB connector.

===== ========== ==========
Pad   Signal     Notes
===== ========== ==========
1     VBUS       Connected to USB +5V
2     Ground
3     3V3        Out to peripherals
4     GPIO29     ADC3
5     GPIO28     ADC2
6     GPIO27     ADC1
7     GPIO26     ADC0
8     Ground
9     GPIO7
10    GPIO6
11    GPIO5
12    GPIO4
13    GPIO3
14    GPIO2
15    GPIO1      Default RX for UART0 serial console
16    GPIO0      Default TX for UART0 serial console
===== ========== ==========

Power Supply 
============

The Raspberry Pi Pico can be powered via the USB connector,
or by supplying +5V to pin 1. 

The Raspberry Pi Pico chip run on 3.3 volts.  This is supplied
by an onboard voltage regulator.

Installation & Build
====================

For instructions on how to to install the build dependencies and create a NuttX
image for this board, consult the main :doc:`RP2040 documentation
<../../index>`.

Configurations
==============

All configurations listed below can be configured using the following command in
the ``nuttx`` directory (again, consult the main :doc:`RP2040 documentation
<../../index>`):

.. code:: console

   $ ./tools/configure.sh pimoroni-tiny2040:<configname>

composite
---------

NuttShell configuration (console enabled in UART0, at 115200 bps) with support for
CDC/ACM with MSC USB composite driver. ``conn`` command enables the composite
device.

gpio
--------

NuttShell configuration (console enabled in UART0, at 115200 bps) with GPIO examples.

.. list-table:: GPIO pin options
   :widths: auto
   :header-rows: 1

   * - GPIO
     - Function
   * - GPIO18
     - Onboard RGB LED (red, out)
   * - GPIO19
     - Onboard RGB LED (green, out)
   * - GPIO20
     - Onboard RGB LED (blue, out)
   * - GPIO23
     - Onboard BOOT button (user)

No interrupt pin configured.

nsh
---

Basic NuttShell configuration (console enabled in UART0, at 115200 bps).

nsh-flash
---------

Basic NuttShell configuration (console enabled in UART0, at 115200 bps
with SMART flash filesystem.

nshsram
-------

NuttShell configuration (console enabled in UART0, at 115200 bps) with interrupt
vectors in RAM.

smp
---

Basic NuttShell configuration (console enabled in UART0, at 115200 bps) with
both ARM cores enabled.

spisd
-----

NuttShell configuration (console enabled in UART0, at 115200 bps) with SPI SD
card support enabled.

.. list-table:: spisd connections
   :widths: auto
   :header-rows: 1

   * - SD card slot
     - Pimoroni Tiny 2040
   * - DAT2
     - Not connected
   * - DAT3/CS
     - GP5 (SPI0 CSn) (Pin 11)
   * - CMD /DI
     - GP7 (SPI0 TX)  (Pin 9)
   * - VDD
     - 3V3 OUT (Pin 3)
   * - CLK/SCK
     - GP6 (SPI0 SCK) (Pin 10)
   * - VSS
     - GND (Pin 2 or 8)
   * - DAT0/DO
     - GP4 (SPI0 RX)  (Pin 12)
   * - DAT1          
     - Not connected

Card hot swapping is not supported.

usbmsc
------

NuttShell configuration (console enabled in UART0, at 115200 bps) with support for
USB MSC and CDC/ACM.

``msconn`` and ``sercon`` commands enable the MSC and CDC/ACM devices. The MSC
support provides the interface to the SD card with SPI, so the SD card slot
connection like spisd configuration is required.

usbnsh
------

Basic NuttShell configuration using CDC/ACM serial (console enabled in USB Port,
at 115200 bps).
