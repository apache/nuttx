=================
ST Nucleo H7S3L8
=================

.. tags:: chip:stm32, chip:stm32h7, chip:stm32h7s3

This page describes the NuttX port for the STMicro NUCLEO-H7S3L8
development board.  The board is based on the 600 MHz STM32H7S3L8
Cortex-M7 microcontroller.

XIP Boot
========

The STM32H7S3L8 has 64 KB of internal flash.  The board also carries a
32 MB MX25UW25645G NOR flash on XSPI2, mapped at ``0x70000000``.  Running
NuttX from the external flash uses two images:

* ``boot-xip`` runs from internal flash at ``0x08000000``.
* ``nsh-xip`` executes in place from XSPI2 flash at ``0x70000000``.

The bootloader configures the MX25UW25645G in octal STR memory-mapped mode,
validates the initial stack pointer and reset vector, and starts the external
image.  Image management, rollback, integrity checking, and authentication
are not currently provided.

Flashing
========

.. note::
   The ``XSPI2_HSLV`` option byte must be set to ``1`` before using the
   external flash.  The option byte setting is persistent.

Build the external image::

  cmake -B build/nsh-xip -DBOARD_CONFIG=nucleo-h7s3l8:nsh-xip -GNinja
  cmake --build build/nsh-xip

Build the internal bootloader::

  cmake -B build/boot-xip -DBOARD_CONFIG=nucleo-h7s3l8:boot-xip -GNinja
  cmake --build build/boot-xip

Install STM32CubeProgrammer and set ``CUBE_PROGRAMMER`` to the path of its
command-line executable.  The external loader is supplied by the
STM32CubeProgrammer installation::

  CUBE_PROGRAMMER=/path/to/STM32_Programmer_CLI
  EXTERNAL_LOADER="$(dirname "$CUBE_PROGRAMMER")/ExternalLoader/MX25UW25645G_NUCLEO-H7S3L8-OBL.stldr"

Erase the external flash, then program and verify the XIP image::

  "$CUBE_PROGRAMMER" --connect port=SWD mode=Normal --erase all \
    -el "$EXTERNAL_LOADER"
  "$CUBE_PROGRAMMER" --connect port=SWD mode=Normal \
    --download build/nsh-xip/nuttx.bin 0x70000000 --verify \
    --skipErase \
    -el "$EXTERNAL_LOADER"

Program and verify the internal bootloader, then reset the board::

  "$CUBE_PROGRAMMER" --connect port=SWD reset=HWrst \
    --download build/boot-xip/nuttx.bin 0x08000000 --verify --rst

The internal image initializes XSPI2 and transfers control to the external
image after reset.

Configurations
==============

Each configuration is maintained in a subdirectory of ``configs`` and can be
selected as follows::

  tools/configure.sh nucleo-h7s3l8:<subdir>

Where ``<subdir>`` is one of the following:

nsh
---

Provides a basic NuttShell configuration in internal flash.  The default
console is the ST-LINK virtual COM port on USART3.

nsh-xip
-------

Provides NuttShell, common diagnostic commands, and ``ostest``.  The image is
linked for execution from XSPI2 flash at ``0x70000000`` and requires an XIP
bootloader in internal flash.

jumbo-xip
---------

As ``nsh-xip``, with networking over the on-board LAN8742A Ethernet PHY.  The
address is obtained by DHCP.

boot-xip
--------

Provides the internal-flash NuttX bootloader used to start the ``nsh-xip``
image.
