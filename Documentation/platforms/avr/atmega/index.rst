================
Microchip Atmega
================

ATMega128
---------

This port of NuttX to the Amber Web Server from
`SoC Robotics <http://www.soc-robotics.com/index.htm>`__ is partially
completed. The Amber Web Server is based on an Microchip ATMega128.

Atmega1284P
-----------

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

AVR ATMega2560
--------------

Extension of the AVR architecture to support the
ATMega2560 and specifi support for the Arduion MEGA2560 board were
contributed by Dimitry Kloper and first released in NuttX-7.14.


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
