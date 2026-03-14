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

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
