=============
Allwinner A10
=============

These following boards are based on the Allwinner A10
have are supported by NuttX:

-  **pcDuino v1**. A port of NuttX to the pcDuino v1 board was first
   released in NuttX-6.33. See http://www.pcduino.com/ for information
   about pcDuino Lite, v1, and v2 boards. These boards are based around
   the Allwinner A10 Cortex-A8 CPU. This port was developed on the v1
   board, but the others may be compatible:

   Refer to the NuttX board
   `README <https://github.com/apache/nuttx/blob/master/boards/arm/a1x/pcduino-a10/README.txt>`__
   file for further information.

   **STATUS**. This port was an experiment was was not completely
   developed. This configuration builds and runs an NuttShell (NSH), but
   only if a patch to work around some issues is applied. While not
   ready for "prime time", the pcDuino port is functional and could the
   basis for a more extensive development. There is, at present, no work
   in progress to extend this port, however.
