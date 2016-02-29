README.txt
==========

This directory holds a port of NuttX to the NXP/Freescale Sabre board
featuring the iMX 6Quad CPU.

Contents
========

  - Status
  - Platform Features
  - Serial Console
  - Configurations

Status
======

2016-02-28: The i.MX6Q port is just beginning. A few files have been
populated with the port is a long way from being complete or even ready to
begin any kind of testing.

Platform Features
=================

Processor:
  - i.MX 6Quad or 6DualLite 1 GHz ARM Cortex-A9 processor
Memory/storage:
  - 1 GB DDR3 SDRAM up to 533 MHz (1066 MTPS) memory
  - 8 GB eMMC flash
  - 4 MB SPI NOR flash
Display:
  - 10.1" 1024 x 768 LVDS display with integrated P-cap sensing
  - HDMI connector
  - LVDS connector (for optional second display)
  - LCD expansion connector (parallel, 24-bit)
  - EPDC expansion connector (for 6DualLite only)
  - MIPI DSI connector (two data lanes, 1 GHz each)
User Interface:
  - 10.1" capacitive multitouch display
  - Buttons: power, reset, volume
Power Management:
  - Proprietary PF0100 PMIC
Audio:
  - Audio codec
  - 2x digital microphones
  - 2x 3.5 mm audio ports
  - Dual 1 watt speakers
Expansion Connector:
  - Camera MIPI CSI port
  - I2C, SPI signals
Connectivity:
  - 2x full-size SD/MMC card slots
  - 7-pin SATA data connector
  - 10/100/1000 Ethernet port
  - 1x USB 2.0 OTG port (micro USB)
Debug:
  - JTAG connector (20-pin)
  - 1x Serial-to-USB connector (for JTAG)
OS Support:
  - Linux® and Android™ from our company
  - Others supported via third party (QNX, Windows Embedded)
Tools Support:
  - Manufacturing tool from our company
  - IOMUX tool from our company
  - Lauterbach, ARM (DS-5), IAR and Macraigor
Additional Features:
  - Proprietary 3-axis accelerometer
  - Proprietary 3D magnetometer
  - Ambient light sensor
  - GPS receiver module
  - 2x 5MP cameras
  - Battery charger
  - Battery connectors (battery not included)

Serial Console
==============

A DEBUG VCOM is available MICRO USB AB 5 J509.  This corresponds to UART1
from the i.MX6.

Configurations
==============

Information Common to All Configurations
----------------------------------------
Each Sabre-6Quad configuration is maintained in a sub-directory and
can be selected as follow:

  cd tools
  ./configure.sh sabre-6quad/<subdir>
  cd -
  . ./setenv.sh

Before sourcing the setenv.sh file above, you should examine it and perform
edits as necessary so that TOOLCHAIN_BIN is the correct path to the directory
than holds your toolchain binaries.

And then build NuttX by simply typing the following.  At the conclusion of
the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

  make oldconfig
  make

The <subdir> that is provided above as an argument to the tools/configure.sh
must be is one of the following.

NOTES:

  1. These configurations use the mconf-based configuration tool.  To
    change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output on TBD.

  3. All of these configurations are set up to build under Windows using the
     "GNU Tools for ARM Embedded Processors" that is maintained by ARM
     (unless stated otherwise in the description of the configuration).

       https://launchpad.net/gcc-arm-embedded

     That toolchain selection can easily be reconfigured using
     'make menuconfig'.  Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_WINDOWS=y               : Window environment
       CONFIG_WINDOWS_CYGWIN=y             : Cywin under Windows

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y : GNU ARM EABI toolchain

Configuration sub-directories
-----------------------------
