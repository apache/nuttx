======================
Microchip DA/DB family
======================

This is an attempt to support AVR DA/DB family of MCUs

Clock Configuration
===================

The chip features internal high frequency oscillator, its frequency
can be set in the Kconfig-based configuration. Clock speed in MHz
is then provided in CONFIG_AVRDX_HFO_CLOCK_FREQ

System Timer
============

System timer is provided by RTC peripheral and takes over it. (In theory,
the Periodic Interrupt - PIT - component of it can be still used
but they share single clock.)

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
