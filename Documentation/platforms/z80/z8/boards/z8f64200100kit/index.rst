==============
z8f64200100kit
==============

z8Encore! Microcontroller.  This port use the Zilog z8f64200100kit
development kit, Z8F6423 part, and the Zilog ZDS-II Windows command line
tools.  The development environment is Cygwin under WinXP.

Configurations
==============

ostest
------

``ostest.zfpproj``
  is a simple ZDS-II project that will allow you
  to use the ZDS-II debugger.  Before using, copy the following
  files from the toplevel directory::

    nuttx.hex, nuttx.map, nuttx.lod

  to this directory as::

    ostest.hex, ostest.map, ostest.lod
