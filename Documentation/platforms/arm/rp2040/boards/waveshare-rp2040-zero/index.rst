===============================
Waveshare RP2040 Zero
===============================

.. tags:: chip:rp2040

The `Waveshare RP2040 Zero <https://www.waveshare.com/wiki/RP2040-Zero>`_ is a general purpose board supplied by
Waveshare.

.. figure:: WaveshareRP2040Zero.png
   :align: center

Features
========

* RP2040 microcontroller chip
* Dual-core ARM Cortex M0+ processor, flexible clock running up to 133 MHz
* 264kB of SRAM, and 2MB of on-board Flash memory
* Castellated module allows soldering direct to carrier boards
* USB 1.1 Host and Device support
* Low-power sleep and dormant modes
* Drag & drop programming using mass storage over USB
* 20 multi-function GPIO pins
* 2× SPI, 2× I2C, 2× UART, 3× 12-bit ADC, 16× controllable PWM channels
* Accurate clock and timer on-chip
* Temperature sensor
* Accelerated floating point libraries on-chip
* 8 × Programmable IO (PIO) state machines for custom peripheral support

Buttons and LEDs
================

A ws2812 (NeoPixel) smart RGB LED controlled by GPIO16 (data).

There is a BOOT button which if held down when power is first
applied or the RESET button is pressed will cause the RP2040 to
boot into program mode and appear as a storage device to
a USB connecter.  Saving a .UF2 file to this device will
replace the Flash ROM contents on the RP2040.

Pin Mapping
===========
Pads numbered anticlockwise from USB connector.

===== ========== ==========
Pad   Signal     Notes
===== ========== ==========
1     5V
2     Ground
3     3.3V
4     GPIO29
5     GPIO28
6     GPIO27
7     GPIO26
8     GPIO15
9     GPIO14
10    GPIO13
11    GPIO12
12    GPIO11     Default TX for SPI1
13    GPIO10     Default SCK for SPI1
14    GPIO9      Default CSn for SPI1
15    GPIO8      Default RX for SPI1
16    GPIO7      Default SCL for I2C1
17    GPIO6      Default SDA for I2C1
18    GPIO5      Default SCL for I2C0
19    GPIO4      Default SDA for I2C0
20    GPIO3
21    GPIO2
22    GPIO1      Default RX for UART0 serial console
23    GPIO0      Default TX for UART0 serial console
===== ========== ==========


Power Supply
============

The Waveshare RP2040 Zero can be powered via the USB connector,
or by supplying +5V to pin 1.

The RP2040 runs on 3.3 volts.  This is supplied
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

   $ ./tools/configure.sh waveshare-rp2040-zero:<configname>

gpio
--------

NuttShell configuration (console enabled in UART0, at 115200 bps) with GPIO examples.

nsh
---

Basic NuttShell configuration (console enabled in UART0, at 115200 bps).


usbnsh
------

Basic NuttShell configuration using CDC/ACM serial (console enabled in USB Port,
at 115200 bps).

ws2812
------

Basic NuttShell configuration with WS2812 driver and example enabled. Console is enabled over USB at 115200 bps.
