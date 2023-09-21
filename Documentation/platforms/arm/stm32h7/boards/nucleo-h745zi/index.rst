================
ST Nucleo H745ZI
================

Dual core support
-----------------

It is recommended to use CMake to build firmware consisting of multiple images.
The commands to build ``nsh_xxx_rptun`` configurations are as follows::

  cmake -B build_h7m7 -DBOARD_CONFIG=nucleo-h745zi:nsh_cm7_rptun -GNinja
  cmake -B build_h7m4 -DBOARD_CONFIG=nucleo-h745zi:nsh_cm4_rptun -GNinja

  cmake --build build_h7m7
  cmake --build build_h7m4

Serial Console
--------------

The STLINK virtual console uses Serial Port 3 (USART3) with TX on PD8
and RX on PD9 and is used by the Cortex-M7 core by default.

================= ===
VCOM Signal       Pin
================= ===
SERIAL_RX         PD9
SERIAL_TX         PD8
================= ===

Access to the Cortex-M4 core can be acheived using an additional UART port
or via RPMSG UART by setting ``CONFIG_RPMSG_UART_CONSOLE=y`` in CM4 configuration.

If the RPMSG UART console is enabled, we can connect to it from CM7 using ``cu``::

  nsh-cm7> cu -l /dev/ttyproxy
  NuttShell (NSH) NuttX-10.4.0
  nsh-cm4>

Tools
-----

Support for the board was tested using an external JLink interface.
Openocd with built-in ST-LINK didn't work well.

Image flashing was accomplished using ``JFlashLiteExe``, with the device
set to ``STM32H745ZI_M7``.
