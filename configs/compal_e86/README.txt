compal_e86
==========

This directory contains the board support for compal e86 phones. This port
is tested on the following phone:

* motorola c139 (compal e86) with flash configuration

This port is based on patches contributed by Denis Carikli for both the
compal e99 and e88. At the time of initial check-in, the following phones
were tested:

* motorolla c155 (compal e99) with the compalram and highram configuration
* motorolla W220 (compal e88)
* The openmoko freerunner baseband(compal e88)

The patches were made by Alan Carvalho de Assis and Denis Carikli using
the Stefan Richter's patches that can be found here:

http://cgit.osmocom.org/cgit/nuttx-bb/log/?h=lputt%2Ftesting

Osmocom-BB Dependencies and Sercomm
===================================

The build environment assumes that you have the osmocom-bb project
directory at same level as the nuttx project:

  |- nuttx
  |- apps
  `- osmocom-bb

If you attempt to build this configuration without osmocom-bb, and that
you added support for sercomm in your configuration(CONFIG_SERCOMM_CONSOLE=y)
you will get compilation errors in drivers/sercomm due to header files that
are needed from the osmocom-bb directory.

By default, NuttX will not use sercomm (HDLC protocol) to communicate with
the host system. Sercomm is the transport used by osmocom-bb that runs on top
of serial.  See http://bb.osmocom.org/trac/wiki/nuttx-bb/run for detailed
the usage of nuttx with sercomm.

Running NuttX From Flash
========================

Flash layout:

0x00000 - 0x02000 - original compal loader
0x02000 - 0x10000 - simple binary to jump to 0x10000 (jumper.e86loader.bin)
0x10000 - ???     - NuttX binary (nuttx.bin)

Using osmocon/osmoload, retrieve the compal loader, flash it and the
jumper.e86loader.bin as well as nuttx.bin.

The jumper app is a modified version of the menu app in osmocom-bb, branch
jolly/menu. The app disabled irqs (setup by compal loader?) and jumps to
0x10000. This app is submitted as a patch to osmocom-bb mailing list.

Loading NuttX (highram)
=======================

The osmocom-bb wiki describes how to load NuttX.  See
http://bb.osmocom.org/trac/wiki/nuttx-bb for detailed information.
The way that nuttx is loaded depends on the configuration (highram/compalram)
and phone:

o compalram is for the ramloader(for phone having a bootloader on flash)
o highram is for phones having the romloader(if the phone has a bootrom)
  or for loading in the ram trough a special loader(loaded first on ram
  by talking to the ramloader) when having a ramloader(which can only
  load 64k).
