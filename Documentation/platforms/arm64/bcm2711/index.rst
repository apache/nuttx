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
   has been tested using a 32GB Samsung microSD card and it has passed testing
   with that. The only quirks are:

   * No card insertion/removal interrupts work, so hotswapping isn't possible
   * The :doc:`sdstress </applications/testing/sd_stress/index>` example works
     unless the 'number of bytes' option is greater than 1023. I suspect this is
     something to do with the FIFO depth being only 1023 bytes.

   However, the 64GB microSD card exhibits very strange behaviour. There are
   often data CRC errors that prevent the boot filesystem from mounting. When
   that somehow passes (intermittent), running ``ls`` on the filesystem repeatedly
   sometimes causes certain files to disappear from the listing (they are not
   deleted and appear again on next boot), or causes their filenames to be shown
   in all caps. Writing to the card with ``echo`` often fails with data CRC
   error or timeout, and then the card is buggy for the remainder of the
   session. **It is not recommended to use 64GB cards with this implementation
   for the time being.**

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
