=======
BCM2711
=======

.. tags:: arch:arm64, chip:bcm2711, vendor:broadcom, experimental

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
I2C                      Full interrupt-based support. No 10b addressing.
UART                     Mini UART yes, PL011 no
GPIO                     Partial
MAILBOX                  Partial (polled method, only commands used by firmware are implemented)
EMMC2                    Interrupt-based support, no DMA.
EMMC                     Supported alongside EMMC2 in theory, but untested.
PWM                      No
SPI                      Interrupt-based driver (no DMA) for all SPI except 1 & 2 (auxiliary)
PCM                      No
======================== =======

.. warning::

   The SPI driver implemented for the :doc:`BCM2711 <../../index>` has only been
   tested on SPI0. It appears that even using the special `overlays
   <https://github.com/raspberrypi/linux/blob/stable/arch/arm/boot/dts/overlays/README>`_
   for the device tree passed to the proprietary firmware does not properly
   initialize the remaining SPI interfaces, and thus they have not been working
   properly. More effort is required to reverse engineer the magic incantations
   required to initialize these interfaces, at which point it is assumed that
   the driver implementation should extend to SPI3-6.

.. warning::

   The EMMC2 peripheral connects to the microSD card slot on the :doc:`Raspberry
   Pi 4B </platforms/arm64/bcm2711/boards/raspberrypi-4b/index>`. Currently, it
   has passed testing with a few different uSD cards. The only quirks are:

   * No card insertion/removal interrupts work, so hotswapping isn't possible
   * Multi-block transfers must be restricted to 1 block at a time
     (``CONFIG_MMCSD_MULTIBLOCK_LIMIT=1``), since the upper-half driver's method
     of doing block setup is not immediately compatible with the BCM2711 EMMC
     controller. Changes to the common-upperhalf would require extensive
     testing, so this performance sacrifice is done for correct behaviour in the
     short-term.

   All in all, be aware of issues with the SD card implementation.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
