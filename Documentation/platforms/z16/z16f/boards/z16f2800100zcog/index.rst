===============
z16f2800100zcog
===============

z16f Microcontroller.  This port use the Zilog z16f2800100zcog
development kit and the Zilog ZDS-II Windows command line tools.  The
development environment is Cygwin under WinXP.


Configurations
==============

nsh
---

**Loading and Executing Code**

1. Copy the files to this directory as described above
2. Connect the ZiLOG XTools USB debugger.
3. Install the USB driver from the ZDS-II device_drivers directory
4. Start ZDS-II and load the nsh.zfpproj project
5. In the debug tab, connect to the debugger
6. In the debug tab, load code, reset, and go

**Console Output**

Interaction with NSH is via the serial console at 57600 8N1 baud.

**STATUS**

1. Note that you must apply the ZNEO patch if you are using ZDS-II 5.0.1.
   See the README.txt file in the parent directory for more information.
   This configuration does run correctly with the path applied.

2. I bet that this code, like ostest, will not run if started by a hardware
   reset.  It may only run when started via the debugger.


ostest
------

**OSTEST Project**

``ostest.zfpproj``
  is a simple ZDS II - ZNEO 5.0.1 project that will allow you
  to use the ZDS-II debugger.  Before using, copy the following files from
  the toplevel directory::

    nuttx.hex, nuttx.map, nuttx.lod

  to this directory as::

    ostest.hex, ostest.map, ostest.lod

**Loading and Executing Code**

1. Copy the files to this directory as described above
2. Connect the ZiLOG XTools USB debugger.
3. Install the USB driver from the ZDS-II device_drivers directory
4. Start ZDS-II and load the ostest.zfpproj project
5. In the debug tab, connect to the debugger
6. In the debug tab, load code, reset, and go

Hmmm... it appears that the code does not run if started by a hardware reset.
It runs only when started via the debugger.  What is up with that?

**Console Output**

OS test results will be provided on the serial console at 57600 8N1 baud.

**STATUS**

This example works fine when started from ZDS-II.  But I have seen problems
when starting from a hardware reset.
