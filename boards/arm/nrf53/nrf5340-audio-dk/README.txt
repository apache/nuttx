nRF5340-AUDIO-DK
================

README for NuttX port to NRF5340 Audio DK (PCA10121) boards.

Tool Issues
===========

  OpenOCD
  -------------
  There is no support official support for Nordic Cortex M33 chips (nRF9160 or nRF5340).

  Segger J-Link
  -------------
  To start the GDB servers for the application core and the network core, use these commands:

    JLinkGDBServer -device nRF5340_xxAA_APP -autoconnect 1 -if SWD -speed 4000 -port 2331 -swoport 2332 -telnetport 2333
    JLinkGDBServer -device nRF5340_xxAA_NET -autoconnect 1 -if SWD -speed 4000 -port 2334 -swoport 2335 -telnetport 2336

  Then you can connect GDB to targets:

    (gdb_app) target remote localhost:2331
    (gdb_net) target remote localhost:2334
