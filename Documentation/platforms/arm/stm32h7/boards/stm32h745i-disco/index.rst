===================
ST STM32H745I-DISCO
===================

Dual core support
=================

It is recommended to use CMake to build firmware consisting of multiple images.

Dual instances
--------------

The commands to build ``nsh_xxx`` configurations are as follows::

  cmake -B build_cm7 -DBOARD_CONFIG=stm32h745i-disco:nsh_cm7 -GNinja
  cmake -B build_cm4 -DBOARD_CONFIG=stm32h745i-disco:nsh_cm4 -GNinja

  cmake --build build_cm7
  cmake --build build_cm4

Dual core RPTun
---------------

The commands to build ``nsh_xxx_rptun`` configurations are as follows::

  cmake -B build_cm7_rptun -DBOARD_CONFIG=stm32h745i-disco:nsh_cm7_rptun -GNinja
  cmake -B build_cm4_rptun -DBOARD_CONFIG=stm32h745i-disco:nsh_cm4_rptun -GNinja

  cmake --build build_cm7_rptun
  cmake --build build_cm4_rptun

Serial console
==============

The STM32H745I-DISCO board's ST-LINK interface is connected to USART3, which
is used as the serial console for the Cortex-M7 core by default. The
connections for USART3 are as follows:

================= ====
USART3 Signal     Pin
================= ====
USART3_RX         PB11
USART3_TX         PB10
================= ====

When using the ``nsh_xxx`` configuration, UART7 is assigned to the Cortex-M4
core.
The UART7 connections can be accessed via the Arduino connector on the board:

================= =================
UART7 Signal      Pin
================= =================
UART7_RX          PA8 (Arduino D10)
UART7_TX          PB4 (Arduino D5)
================= =================

This allows the Cortex-M4 core to utilize a separate serial interface, making
it possible to debug or interact with both cores simultaneously through
different UART interfaces.

RPTun
-----

When using the ``nsh_xxx_rptun`` configuration, users can connect to the
CM4 core from the CM7 core by using the ``cu`` command::

  nsh-cm7> cu -l /dev/ttyproxy
  NuttShell (NSH) NuttX-10.4.0
  nsh-cm4>
