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

Tools
-----

Support for the board was tested using an external JLink interface.
Openocd with built-in ST-LINK didn't work well.

Image flashing was accomplished using ``JFlashLiteExe``, with the device
set to ``STM32H745ZI_M7``.
