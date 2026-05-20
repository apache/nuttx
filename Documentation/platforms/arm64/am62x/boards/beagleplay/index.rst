==========
BeaglePlay
==========

.. tags:: chip:am62x, arch:arm64, vendor:beagleboard

`BeaglePlay <https://www.beagleboard.org/boards/beagleplay>`_ is an
open-hardware single-board computer from BeagleBoard.org based on the
TI AM6254 SoC.

Features
========

* TI AM6254 quad-core Arm Cortex-A53 SoC
* 2 GiB DDR4
* microSD and on-board eMMC boot options
* USB-C debug port with serial console access
* Four user LEDs
* Gigabit Ethernet, USB, Wi-Fi, and Bluetooth hardware on the board

Current NuttX support is limited to early boot, the 16550 serial console,
interactive NSH, and ``procfs`` mount during board bring-up. GPIO, LEDs,
I2C, SPI, MMC/SD, Ethernet, Wi-Fi, Bluetooth, and USB runtime support are
not implemented yet.

Buttons and LEDs
================

BeaglePlay exposes four user LEDs connected to the AM62x main GPIO block:

======  ========  ==================
LED     GPIO      Notes
======  ========  ==================
USR0    GPIO1_22  Green user LED
USR1    GPIO1_23  Yellow user LED
USR2    GPIO1_24  Red user LED
USR3    GPIO1_25  Blue user LED
======  ========  ==================

The current NuttX port does not yet provide functional GPIO-backed LED
control. The ``CONFIG_ARCH_LEDS`` hooks are present as placeholders only.

Serial Console
==============

Use the BeaglePlay **DEBUG** USB-C port and a host serial terminal at
**115200 8N1**.

Pin Mapping
===========

The current bring-up uses the on-board debug interfaces rather than the
expansion headers:

===========  ======================  =======================================
Interface    SoC signal              Notes
===========  ======================  =======================================
DEBUG USB-C  UART0                   Default NuttX serial console
microSD      MMC1                    Validated boot media for ``nuttx.bin``
USR0-3       GPIO1_22 through 1_25   User LEDs, not yet driven by NuttX
===========  ======================  =======================================

Power Supply
============

Follow the BeagleBoard.org board documentation for the supported USB-C power
inputs and peripherals. For NuttX bring-up, use the dedicated **DEBUG**
USB-C port for the serial console and the normal power path for board power.

Installation
============

Install an ``aarch64-none-elf`` bare-metal toolchain and make sure its
``bin`` directory is on your ``PATH``.

One option is the
`Arm GNU Toolchain <https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads>`_.

Building NuttX
==============

.. code:: console

   $ ./tools/configure.sh beagleplay:nsh
   $ make -j$(nproc) CROSS_COMPILE=aarch64-none-elf-

A successful build produces ``nuttx`` and ``nuttx.bin``.

Flashing
========

The currently validated boot path loads ``nuttx.bin`` from a FAT partition on
microSD using the BeaglePlay U-Boot prompt.

1. Format a microSD card with a single FAT32 partition.
2. Copy ``nuttx.bin`` to the root of the card.
3. Insert the card and connect the **DEBUG** USB-C serial cable.
4. Stop at the U-Boot prompt and run:

   .. code:: console

      => mmc dev 1 0
      => fatload mmc 1:1 0x82000000 nuttx.bin
      => go 0x82000000

5. Confirm the board reaches the NuttX prompt:

   .. code:: text

      NuttShell (NSH) NuttX-12.x
      nsh>

Configurations
==============

``beagleplay:nsh``
  Interactive NSH configuration for serial bring-up and shell access.

``beagleplay:ostest``
  Hardware validation configuration that boots directly into ``ostest_main``.
