============
Moteino-Mega
============

.. tags:: arch:avr, chip:atmega1284p

This port was contributed by jeditekunum.

This is the README file for the port of NuttX to the MoteinoMEGA from
LowPowerLab (http://www.lowpowerlab.com). The MoteinoMEGA is based on an Atmel
ATMega1284P. As of this writing, documentation for the MoteinoMEGA board is
available `here
<https://lowpowerlab.com/shop/index.php?_route_=Moteino/moteinomega>`_.

MoteinoMEGA Features
====================

* 16MHz ATmega1284P Atmel 8bit AVR RISC Processor
* 128Kbyte Flash
* 16Kbyte RAM
* 4Kbyte EEPROM
* 2 High Speed Serial Ports
* 8Ch 10bit Analog Input port

Pinout
======

==================== =============================
ATMega1284P Pinout   MoteinoMEGA board connection
==================== =============================
 1 AREF              AR
 2 PA7               A7
 3 PA6               A6
 4 PA5               A5
 5 PA4               A4
 6 PA3               A3
 7 PA2               A2
 8 PA1               A1
 9 PA0               A0
10 PB0               0
11 PB1               1
12 PB2 (INT2)        2/i2  (used by optional radio)
13 PB3 (PWM0)        3
14 PB4 (PWM1/SS)     4/SS  (used by optional radio)
15 PB5 (MOSI)        5/MO  (used by optional radio/flash)
16 PB6 (MISO/PWM2)   6/MI  (used by optional radio/flash)
17 PB7 (SCK/PWM3)    7/SCK (used by optional radio/flash)
18 VOUT              3v3
19 VIN               VIN
20 GND               GND
21 DTR/RTS           DTR
22 TX0               v
23 RX0               ^
24 VIN
25
26 GND               GND
27 GND               GND
28 VIN               VIN
29 VOUT              3v3
30 RESET             RST
31 PD0 (RX0)         8/Serial 0 ^
32 PD1 (TX0)         9/Serial 0 v
33 PD2 (RX1/INT0)    10/Serial 0 ^/i0
34 PD3 (TX1/INT1)    11/Serial 1 v/i1
35 PD4 (PWM4)        12
36 PD5 (PWM5)        13
37 PD6 (PWM6)        14
38 PD7 (PWM7)        15/LED
39 PC0 (SCL)         16/SCL
40 PC1 (SDA)         17/SDA
41 PC2 (TCK)         18
42 PC3 (TMS)         19
43 PC4 (TDO)         20
44 PC5 (TDI)         21
45 PC6               22
46 PC7               23 (used by optional flash)
==================== =============================

Installation
============

DualOptiboot Bootloader
-----------------------

* FTDI (or similar) USB-To-Serial converter with compatible connector configured
  for DTR (AdaFruit, SparkFun, etc)
* Obtain ``ard-reset-arduino`` Python script (`one source
  <https://github.com/mikaelpatel/Cosa/blob/master/build/Arduino-Makefile/bin/ard-reset-arduino>`_)
  This script triggers the DTR pin to enter bootloader mode.
* Obtain `avrdude` for your platform.

Bootloader operates at 115200 baud. It would be useful to create a short script
that invokes ``ard-reset-arduino`` and then avrdude to load program. This script
could then also, optionally, invoke miniterm.py or some other serial interface
program for console.

Example:

.. code:: bash

   APP=nuttx
   CPU=atmega1284p
   BAUD=115200
   PORT=/dev/tty.usbserial-A703X8PQ
   avr-size --mcu=$CPU -C --format=avr $APP
   ard-reset-arduino --verbose $PORT
   avrdude -q -V -p $CPU -C {location-of-avrdude.conf} -D -c arduino -b $BAUD \
     -P $PORT -U flash:w:${APP}.hex:i
   miniterm.py --port=$PORT --baud=$BAUD -q --lf

Toolchains
----------

The toolchain may be selected in Kconfig by editing the existing configuration
file (defconfig), or by overriding the toolchain on the make commandline with
``CONFIG_AVR_TOOLCHAIN=<toolchain>``.

The valid values for ``<toolchain>`` are BUILDROOT, CROSSPACK, LINUXGCC and
WINAVR.

This port was tested using the macOS / CROSSPACK tool chain, GCC version 4.8.1.

Configurations
==============

Each MoteinoMEGA configuration can be selected as follows:

.. code:: console

   $ tools/configure.sh moteino-mega:<config>

Where ``<config>`` is one of the configurations listed below.

hello
-----

The simple "Hello, World!" example.

nsh
---

Configures the NuttShell (nsh). The configuration enables only the serial NSH
interfaces.
