README.txt
==========

Contents
--------

  o NSH Project
  o Loading and Executing Code
  o Console Output
  o STATUS

NSH Project
-----------

nsh.zfpproj is a simple ZDS II - ZNEO 5.0.1 project that will allow you
  to use the ZDS-II debugger.  Before using, copy the following files from
  the toplevel directory:

    nuttx.hex, nuttx.map, nuttx.lod

  to this directory as:

    nsh.hex, nsh.map, nsh.lod

Loading and Executing Code
--------------------------

1. Copy the files to this directory as described above
2. Connect the ZiLOG XTools USB debugger.
3. Install the USB driver from the ZDS-II device_drivers directory
4. Start ZDS-II and load the nsh.zfpproj project
5. In the debug tab, connect to the debugger
6. In the debug tab, load code, reset, and go

Console Output
--------------

Interaction with NSH is via the serial console at 57600 8N1 baud.

STATUS
------
1. This configuration does not run correctly.  There is a problem with the
   serial driver.  When started, some garbled characters appear on the
   console.  I suspect (a) the UART is not being configured correctly, and
   (2) UART interrupts are not be set up correctly.

2. I bet that this code, like ostest, will not run if started by a hardware
   reset.  It may only run when started via the debugger.

