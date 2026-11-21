==================================================
Infineon AURIX TC4D7 Lite Kit (KIT_A3G_TC4D7_LITE)
==================================================

.. tags:: arch:tricore, chip:tc4d7, vendor:infineon

The `KIT_A3G_TC4D7_LITE <https://www.infineon.com/evaluation-board/KIT-A3G-TC4D7-LITE>`__ is a low cost evaluation board of the
Infineon AURIX TC4D7 with an on-board debugger.  The debugger also
provides a USB serial port, which is connected to ASCLIN0 (TX on P14.0, RX
on P14.1) and used as the NuttX console at 115200 baud.

Toolchains
==========

The AURIX GCC and the TASKING compiler are supported, with make and CMake.

Configurations
==============

Each configuration is maintained in a sub-directory and can be selected as
follows::

    tools/configure.sh a3g-tc4d7-lite:<subdir>

nsh
---

NuttShell with the ostest and hello examples::

    tools/configure.sh a3g-tc4d7-lite:nsh

Flashing
========

The image can be programmed with the Infineon AURIX OpenOCD and the board
configuration ``board/infineon/kit_a3g_tc4d7_lite.cfg``::

    openocd -f board/infineon/kit_a3g_tc4d7_lite.cfg -c init -c "reset halt" \
            -c "program nuttx.hex verify" -c "reset run" -c shutdown
