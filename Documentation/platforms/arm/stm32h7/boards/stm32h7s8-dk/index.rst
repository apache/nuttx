===============
ST STM32H7S8-DK
===============

.. tags:: chip:stm32, chip:stm32h7, chip:stm32h7s3

.. figure:: stm32h7s8-dk-front.jpg
   :align: center
   :alt: STM32H7S8-DK Front

   STM32H8S8-DK Front

.. figure:: stm32h7s8-dk-back.jpg
   :align: center
   :alt: STM32H7S8-DK Back
          
   STM32H8S8-DK Back

This page describes the NuttX port for the `STMicro SMT32H7S8-DK <https://www.st.com/en/evaluation-tools/stm32h7s78-dk.html>`_
Discovery kit. The board is based on the 600 MHz STM32H7S3L8
Cortex-M7 microcontroller.

Tools to program internal/external flash
========================================

Install STM32CubeProgrammer and set ``CUBE_PROGRAMMER`` to the path of its
command-line executable. The external loader(needed to erase/program external
MX66UWw1G45G) is supplied by the STM32CubeProgrammer installation::

  CUBE_PROGRAMMER=/path/to/STM32_Programmer_CLI
  EXTERNAL_LOADER="$(dirname "$CUBE_PROGRAMMER")/ExternalLoader/MX66UW1G45G_STM32H7S78-DK.stldr"

Flashing Internal NSH image
===========================

Build the internal nsh image::

  cmake -B build/nsh -DBOARD_CONFIG=stm32h7s8-dk:nsh -GNinja
  cmake --build build/nsh

Program and verify the internal image, then reset the board::

  "$CUBE_PROGRAMMER" --connect port=SWD reset=HWrst \
    --download build/nsh/nuttx.bin 0x08000000 --verify --rst

The internal nsh image presents a shell on the ST-link console on startup.

XIP Boot
========

The STM32H7S3L8 has 64 KB of internal flash.  The board also carries a
32 MB MX66UW1G45G NOR flash on XSPI2, mapped at ``0x70000000``.  Running
NuttX from the external flash uses two images:

* ``boot-xip`` runs from internal flash at ``0x08000000``.
* ``nsh-xip`` executes in place from XSPI2 flash at ``0x70000000``.

The bootloader configures the MX66UW1G45G2 in octal STR memory-mapped mode,
validates the initial stack pointer and reset vector, and starts the external
image.  Image management, rollback, integrity checking, and authentication
are not currently provided.

Flashing External NSH image
===========================

.. note::
   The ``XSPI2_HSLV`` option byte must be set to ``1`` before using the
   external flash.  The option byte setting is persistent.

Build the internal bootloader::

  cmake -B build/boot-xip -DBOARD_CONFIG=stm32h7s8-dk:boot-xip -GNinja
  cmake --build build/boot-xip

Build the external NSH image::

  cmake -B build/nsh-xip -DBOARD_CONFIG=stm32h7s8-dk:nsh-xip -GNinja
  cmake --build build/nsh-xip

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

  
Configurations
==============

Each configuration is maintained in a subdirectory of ``configs`` and can be
selected as follows::

  tools/configure.sh stm32h7s8-dk:<subdir>

Where ``<subdir>`` is one of the following:

nsh
---

Provides a basic NuttShell configuration in internal flash and
includes support for on-board LEDs and user button.  The default
console is the ST-LINK virtual COM port on USART4.

nsh-xip
-------

Provides NuttShell, common diagnostic commands, and ``ostest``.  The image is
linked for execution from XSPI2 flash at ``0x70000000`` and requires an XIP
bootloader in internal flash.

boot-xip
--------

Provides the internal-flash NuttX bootloader used to start the ``nsh-xip``
image.
