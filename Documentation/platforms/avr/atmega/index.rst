===========
Atmega1284P
===========

The Atmega1284P is a chip from Microchip ("inherited" from Atmel) with the
following features:

  - 16MHz 8bit AVR RISC Processor
  - 128Kbyte Flash
  - 16Kbyte RAM
  - 4Kbyte EEPROM
  - 2 High Speed Serial Ports
  - 2 8-bit Timer/Counters
  - 2 16-bit Timer/Counter
  - 1 Master/Slave SPI
  - 1 I2C controller (aka Two-Wire interface)
  - 8Ch 10bit Analog Input port
  - up to 32 GPIOs
  - Watchdog timer
  - Real Time Clock
  - JTAG/OCD Interface

Clock Configuration
===================

System Timer
============

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  ======= =====
Peripheral  Support Notes
==========  ======= =====
GPIO        Yes
PWM         No
ADC         No
RTC         No
WTD         No
I2C         No
SPI         No
TIMER       Yes
UART        Yes
==========  ======= =====

UART
----

UART is implemented using interrupts. The chip doesn't support DMA.

TIMER
-----

The TIMER peripheral is exposed as standard timer.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
