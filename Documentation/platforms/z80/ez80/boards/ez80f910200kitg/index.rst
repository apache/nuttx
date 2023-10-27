================
ez80f0910200kitg
================

ez80Acclaim! Microcontroller.  This port use the Zilog ez80f0910200kitg
development kit, eZ80F091 part, and the Zilog ZDS-II Windows command line
tools.  The development environment is Cygwin under WinXP.


Configurations
==============

ostest
------

``ostest.zdsproj``
  is a simple ZDS-II project that will allow you to use the ZDS-II debugger.
  Before using, copy the following files from the toplevel directory::

    nuttx.hex, nuttx.map, nuttx.lod

  to this directory as::

    ostest.hex, ostest.map, ostest.lod
