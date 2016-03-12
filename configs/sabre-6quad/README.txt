README.txt
==========

This directory holds a port of NuttX to the NXP/Freescale Sabre board
featuring the iMX 6Quad CPU.

Contents
========

  - Status
  - Platform Features
  - Serial Console
  - LEDs and Buttons
  - Configurations

Status
======

2016-02-28: The i.MX6Q port is just beginning. A few files have been
populated with the port is a long way from being complete or even ready to
begin any kind of testing.

2016-03-12: The i.MX6Q port is code complete including initial
implementation of logic needed for CONFIG_SMP=y  .  There is no clock
configuration logic.  This is probably not an issue if we are loaded into
SDRAM by a bootloader (because we cannot change the clocking anyway in
that case).

There is a lot of testing that could be done but, unfortunately, I still
have no i.MX6 hardware to test on.

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
from the i.MX6.  UART1 connects to J509 via the CSIO_DAT10 and CSIO_DAT11
pins

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
     output on UART1 which is a available to the host PC from the USB
     micro AB as a VCOM part.

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

LEDs and Buttons
----------------

LEDs
----
A single LED is available driven GPIO1_IO02.  On the schematic this is
USR_DEF_RED_LED signal to pin T1 (GPIO_2).  This signal is shared with
KEY_ROW6 (ALT2).  A low value illuminates the LED.

This LED is not used by the board port unless CONFIG_ARCH_LEDS is
defined.  In that case, the usage by the board port is defined in
include/board.h and src/sam_autoleds.c. The LED is used to encode
OS-related events as follows:

  ------------------- ----------------------- ------
  SYMBOL              Meaning                 LED
  ------------------- ----------------------- ------
  LED_STARTED         NuttX has been started  OFF
  LED_HEAPALLOCATE    Heap has been allocated OFF
  LED_IRQSENABLED     Interrupts enabled      OFF
  LED_STACKCREATED    Idle stack created      ON
  LED_INIRQ           In an interrupt         N/C
  LED_SIGNAL          In a signal handler     N/C
  LED_ASSERTION       An assertion failed     N/C
  LED_PANIC           The system has crashed  FLASH

Thus if the LED is statically on, NuttX has successfully  booted and is,
apparently, running normally.  If the LED is flashing at approximately
2Hz, then a fatal error has been detected and the system has halted.

Buttons
-------


Configuration sub-directories
-----------------------------

  nsh
  ---
    This is a NuttShell (NSH) configuration that uses the NSH library
    at apps/nshlib with the start logic at apps/examples/nsh.

    NOTES:
