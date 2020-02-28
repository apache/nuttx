README.txt
^^^^^^^^^^

OSTEST Project
--------------

ostest.zfpproj is a simple ZDS II - ZNEO 5.0.1 project that will allow you
  to use the ZDS-II debugger.  Before using, copy the following files from
  the toplevel directory:

    nuttx.hex, nuttx.map, nuttx.lod

  to this directory as:

    ostest.hex, ostest.map, ostest.lod

Loading and Executing Code
--------------------------

1. Copy the files to this directory as described above
2. Connect the ZiLOG XTools USB debugger.
3. Install the USB driver from the ZDS-II device_drivers directory
4. Start ZDS-II and load the ostest.zfpproj project
5. In the debug tab, connect to the debugger
6. In the debug tab, load code, reset, and go

Hmmm... it appears that the code does not run if started by a hardware reset.
It runs only when started via the debugger.  What is up with that?

Console Output
--------------

OS test results will be provided on the serial console at 57600 8N1 baud.

STATUS
------

This example works fine when started from ZDS-II.  But I have seen problems
when starting from a hardware reset.
