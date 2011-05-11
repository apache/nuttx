configs/pic32mx README
=====================

This README file discusses the port of NuttX to the PIC32MX board from
PCB Logic Design Co.  This board features the MicroChip PIC32MX460F512L.
The board is a very simple -- little more than a carrier for the PIC32
MCU plus voltage regulation, debug interface, and an OTG connector.

Contents
========

  Toolchains

Toolchains
==========

  I am using the free, LITE version of the PIC32MX toolchain available
  for download from the microchip.com web site.  I am using the Windows
  version.  This is only toolchain currently supported in these
  configurations, but it should be a simple matter to adapt to other
  toolchains by modifying the Make.defs file include in each configuration.
