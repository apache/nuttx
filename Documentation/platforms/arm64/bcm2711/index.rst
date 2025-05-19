=======
BCM2711
=======

.. tags:: chip:bcm2711, experimental

.. warning::

   The support for this chip is experimental. Not all features are
   implemented and they have not been extensively tested by many users.

   Help is wanted if you are interested in supporting a feature or if you've
   found an issue with any of the implementation! See :doc:`the contributing
   guidelines </contributing/index>`.

The `BCM2711
<https://www.raspberrypi.com/documentation/computers/processors.html#bcm2711>`_
is a Broadcom SoC used for the Raspberry Pi 4B board.

- **CPU:** Quad-core ARM Cortex-A72
- **Interrupt Controller:** GIC400

Supported Peripherals
=====================

======================== =======
Peripheral               Support
======================== =======
I2C                      Partial (able to read, that's it)
UART                     Mini UART yes, PL011 no
GPIO                     Partial
PWM                      No
SPI                      Interrupt-based driver (no DMA) for all SPI except 1 & 2 (auxiliary)
PCM                      No
======================== =======

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
