==================================================
Infineon AURIX TC375 Lite Kit (KIT_A2G_TC375_LITE)
==================================================

.. tags:: arch:tricore, chip:tc375, vendor:infineon

The `KIT_A2G_TC375_LITE <https://www.infineon.com/evaluation-board/KIT-A2G-TC375-LITE>`__ is a low cost evaluation board of the
Infineon AURIX TC375 with an on-board debugger.  The debugger also
provides a USB serial port, which is connected to ASCLIN0 (TX on P14.0, RX
on P14.1) and used as the NuttX console at 115200 baud.

Toolchains
==========

The AURIX GCC and the TASKING compiler are supported, with make and CMake.

Configurations
==============

Each configuration is maintained in a sub-directory and can be selected as
follows::

    tools/configure.sh a2g-tc375-lite:<subdir>

nsh
---

NuttShell with the ostest and hello examples::

    tools/configure.sh a2g-tc375-lite:nsh

Flashing
========

The image can be programmed with the Infineon AURIX OpenOCD and the board
configuration ``board/infineon/kit_a2g_tc375_lite.cfg``::

    openocd -f board/infineon/kit_a2g_tc375_lite.cfg -c init -c "reset halt" \
            -c "program nuttx.hex verify" -c "reset run" -c shutdown
