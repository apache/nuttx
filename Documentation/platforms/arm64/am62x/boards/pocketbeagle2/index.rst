==============
PocketBeagle 2
==============

.. tags:: chip:am62x, arch:arm64, vendor:beagleboard

`PocketBeagle 2 <https://docs.beagleboard.org/boards/pocketbeagle-2/>`_ is a
compact open-hardware board from BeagleBoard.org based on the TI AM6254 SoC.

Features
========

* TI AM6254 quad-core Arm Cortex-A53 SoC
* 512 MiB DDR4
* microSD boot with on-board QSPI boot firmware
* USB-C debug/power connection with serial console access
* Expansion headers for GPIO, I2C, SPI, UART, PWM, and ADC
* User LEDs exposed by the board port

Current NuttX support includes early boot, the 16550 serial console,
interactive NSH, ``procfs`` mount during board bring-up, AM62x TISCI
initialization, main-domain GPIO, and I2C0/I2C2 controller support in the
``pocketbeagle2:i2c`` configuration. SPI, MMC/SD runtime support, Ethernet,
and USB runtime support are not implemented yet.

Buttons and LEDs
================

The current NuttX board port defines three logical user LEDs:

======  ========  =====================
LED     Colour    Notes
======  ========  =====================
USR0    Green     ``CONFIG_ARCH_LEDS`` placeholder
USR1    Amber     ``CONFIG_ARCH_LEDS`` placeholder
USR2    Red       ``CONFIG_ARCH_LEDS`` placeholder
======  ========  =====================

The AM62x GPIO controller is initialized by the board port. Automatic
``CONFIG_ARCH_LEDS`` LED hooks remain placeholders only.

Serial Console
==============

AM62x UART6 is routed to the USB-C debug interface. In the current NuttX
configuration it is exposed as the first 16550 console device. Connect the
USB-C cable to your host and open the enumerated serial port at **115200 8N1**.

Pin Mapping
===========

The current bring-up primarily relies on the on-board debug path:

===========  ======================  =======================================
Interface    SoC signal              Notes
===========  ======================  =======================================
USB-C debug  UART6                   Default NuttX serial console
microSD      MMC1                    Validated boot media for ``nuttx.bin``
USR0-2       Board user LEDs         Available through the main GPIO block
I2C0         I2C0                    Available in ``pocketbeagle2:i2c``
I2C2         Header I2C2             Available in ``pocketbeagle2:i2c``
===========  ======================  =======================================

Power Supply
============

PocketBeagle 2 can be powered through its USB-C connector. Follow the vendor
documentation for external peripherals and header power limits during bring-up.

Installation
============

Install an ``aarch64-none-elf`` bare-metal toolchain and make sure its
``bin`` directory is on your ``PATH``.

One option is the
`Arm GNU Toolchain <https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads>`_.

Building NuttX
==============

.. code:: console

   $ ./tools/configure.sh pocketbeagle2:nsh
   $ make -j$(nproc) CROSS_COMPILE=aarch64-none-elf-

A successful build produces ``nuttx`` and ``nuttx.bin``.

Flashing
========

PocketBeagle 2 boots NuttX from microSD using the U-Boot firmware already
present in on-board flash.

1. Format a microSD card with a single FAT32 partition.
2. Copy ``nuttx.bin`` and ``boards/arm64/am62x/pocketbeagle2/scripts/uEnv.txt``
   to the root of the card.
3. Insert the card and connect the USB-C debug cable.
4. Power on the board and let U-Boot load the image automatically.
5. Confirm the board reaches the NuttX prompt:

   .. code:: text

      NuttShell (NSH) NuttX-12.x
      nsh>

Configurations
==============

Configure NuttX with ``pocketbeagle2:<config-name>``.

.. code:: console

   $ ./tools/configure.sh pocketbeagle2:nsh

nsh
---

Interactive NSH configuration for serial bring-up and shell access.

i2c
---

NSH configuration with AM62x I2C0/I2C2 and the ``i2c`` tool enabled.

ostest
------

Hardware validation configuration that boots directly into ``ostest_main``.
