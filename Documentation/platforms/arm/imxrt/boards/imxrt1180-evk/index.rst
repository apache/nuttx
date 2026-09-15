================
i.MX RT1180 EVK
================

`i.MX RT1180 EVK <https://www.nxp.com/design/design-center/development-boards-and-designs/i-mx-evaluation-and-development-boards/i-mx-rt1180-evaluation-kit:MIMXRT1180-EVK>`_
is an evaluation kit by NXP. This kit uses the i.MX RT1189 crossover MCU
with ARM Cortex-M33 and Cortex-M7 cores.

Features
========

- Processor
    - MIMXRT1189 processor
    - 300 MHz Cortex-M33
    - 800 MHz Cortex-M7
- Memory
    - On-chip TCM and OCRAM
    - External SDRAM
    - FlexSPI1 QSPI NOR flash
    - Octal SPI NOR flash
- Connectivity
    - MCU-Link debug interface
    - External JTAG/SWD connector
    - Ethernet connectors
    - USB connectors
    - Arduino interface

Serial Console
==============

The default serial console uses LPUART1 through the MCU-Link virtual COM
port.

.. code-block:: console

   115200 8N1

Connect a USB cable between the MCU-Link debug USB connector and the host
PC. The NSH console is normally available as ``/dev/ttyACM0`` on Linux.

Configurations
==============

nsh-m33
-------

Configures the NuttShell (nsh) to run on the Cortex-M33. The image is
booted directly by the RT1180 ROM from FlexSPI1 QSPI NOR flash.

bl
--

Builds a minimal Cortex-M33 bootloader. The bootloader starts from
FlexSPI1 QSPI NOR flash, sets the Cortex-M7 boot address, and releases
the Cortex-M7.

nsh
---

Configures the NuttShell (nsh) to run on the Cortex-M7. This image is a
raw XIP payload that is started by the ``bl`` configuration.

Flash Layout
============

The Cortex-M33 image is the bootable image. It contains the FlexSPI
Configuration Block (FCB) at offset ``0x400`` and the AHAB container at
offset ``0x1000``. The Cortex-M7 image is programmed separately as a raw
XIP payload.

============  ========  ==============================
Address       Offset    Contents
============  ========  ==============================
0x28000000    0x0       Cortex-M33 image, ``flash.bin``
0x28080000    0x80000   Cortex-M7 image, ``nuttx.bin``
============  ========  ==============================

Build and Flash nsh-m33
=======================

Configure and build the Cortex-M33 NSH image:

.. code-block:: console

   $ ./tools/configure.sh imxrt1180-evk:nsh-m33
   $ make -j$(nproc)

This produces ``flash.bin``. Program it at FlexSPI NOR offset ``0x0``:

.. code-block:: console

   $ boards/arm/imxrt/imxrt1180-evk/tools/flash-jlink.sh flash.bin 0x0

The ``flash-jlink.sh`` helper script is provided by the board port and
takes the image name and the FlexSPI NOR offset as arguments.

Build and Flash bl+nsh
======================

First configure and build the Cortex-M33 bootloader:

.. code-block:: console

   $ ./tools/configure.sh imxrt1180-evk:bl
   $ make -j$(nproc)
   $ cp flash.bin flash-bl.bin

Then configure and build the Cortex-M7 NSH payload:

.. code-block:: console

   $ ./tools/configure.sh imxrt1180-evk:nsh
   $ make -j$(nproc)
   $ cp nuttx.bin nuttx-m7.bin

Program both images:

.. code-block:: console

   $ boards/arm/imxrt/imxrt1180-evk/tools/flash-jlink.sh flash-bl.bin 0x0
   $ boards/arm/imxrt/imxrt1180-evk/tools/flash-jlink.sh nuttx-m7.bin 0x80000

The first command programs the Cortex-M33 bootloader image at the start of
FlexSPI NOR. The second command programs the Cortex-M7 payload at offset
``0x80000``.

J-Link External Debug Probe
===========================

Install the J-Link Debug Host Tools and make sure they are in your search
path. Set SW5 to ``0100`` for FlexSPI Quad SPI NOR boot.

The flash helper programs through the Cortex-M33 device profile because
the RT1180 ROM boots the Cortex-M33 first and initializes FlexSPI1 before
the SEGGER flash loader runs. Use the Cortex-M7 device profile for
debugging the running Cortex-M7 image.
