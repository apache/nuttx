=============
Kendryte K210
=============

System Controller (sysctl)
==========================

The K210 System Controller (sysctl) driver provides essential clock and reset
control functionality for the K210 SoC. It is built unconditionally for all
K210 boards.

Clock Frequency Configuration
-----------------------------

The driver supports querying clock frequencies for:

* PLL frequencies (PLL0, PLL1, PLL2)
* CPU clock frequency
* APB bus frequencies (APB0, APB1, APB2)
* Individual peripheral clock frequencies

CPU frequency can be configured at build time using the ``K210_CPU_FREQ``
Kconfig option (default: 400 MHz, range: 40-600 MHz).

Watchdog Timers
===============

The K210 has two independent watchdog timers (WDT0 and WDT1) for system
reliability. Both are accessible as character drivers via the standard
NuttX watchdog interface.

* **WDT0**: Base address ``0x50400000``, IRQ 21
* **WDT1**: Base address ``0x50410000``, IRQ 22
* **Timeout range**: Programmable based on 16-bit counter

Enable via Kconfig: ``CONFIG_K210_WDT`` (automatically selects
``CONFIG_WATCHDOG``). Devices are ``/dev/watchdog0`` and ``/dev/watchdog1``.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
