pirelli_dpl10
=============

This directory contains the board support for Pirelli "Discus" DP-L10
phones.

This port is a variant of the compal_e88 configuration with the small
change of enabling the IrDA serial console:

  * CONFIG_SERIAL_IRDA_CONSOLE=y

This port is based on patches contributed by Denis Carikli for both the
compal e99 and e88. At the time of initial check-in, the following phones
were tested:

  * Pirelli DPL-10 nsh_highram loaded via romload in osmocon

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

Loading NuttX
=============

The osmocom-bb wiki describes how to load NuttX.  See
http://bb.osmocom.org/trac/wiki/nuttx-bb for detailed information.
The way that nuttx is loaded depends on the configuration (highram/compalram)
and phone:

o compalram is for the ramloader(for phone having a bootloader on flash)
o highram is for phones having the romloader(if the phone has a bootrom)
  or for loading in the ram trough a special loader(loaded first on ram
  by talking to the ramloader) when having a ramloader(which can only
  load 64k).

Memory Map
=========

Calypso has 256KB of internal SRAM (0x800000-0x83ffff).  Only this internal SRAM
is used by these configurations.  The internal SRAM is broken up into three
logic banks.

    LRAM (rw) : ORIGIN = 0x00800000, LENGTH = 0x00020000
    TRAM (rw) : ORIGIN = 0x00820000, LENGTH = 0x00010000
    IRAM (rw) : ORIGIN = 0x00830000, LENGTH = 0x00010000

Code can be loaded by the bootloader only into TRAM and, hence, is restricted
to 64KB.  The additional 64KB if IRAM may be used for uninitialized data and
for the NuttX heap only.

JTAG and Alternative Serial Console
===================================

JTAG
  All JTAG lines, as well as the second uart (UART_MODEM), go to the
  unpopulated connector next to the display connector.

  --- ---------------------------
  PIN SIGNAL 
  --- ---------------------------
    1 Vcc
    2 RX_MODEM
    3 TESTRSTz (Iota)
    4 TDI
    5 TMS
    6 TCK
    7 TX_MODEM
    8 TDO
    9 N/C
   10 GND
   11 N/C
   12 N/C
  --- ---------------------------

JTAG Apapter:

  ------- ----------- --------------- --------------------------------------
  JTAG    20-PIN      DESCRIPTION     NOTES
  SIGNAL  CONNECTOR
  ------- ----------- --------------- --------------------------------------
   Vcc    1, 2        Vcc
   nTRST  3           Reset           Connect this pin to the (active
                                      low) reset input of the target MCU.
                                      Some JTAG adapters driver nTRST (high
                                      and low). Others can can configure
                                      nTRST as open collector (only drive
                                      low).
   GND    4, 6, 8,    Ground
          10, 12, 14,
          16, 20
   TDI    5           JTAG Test Data  Use 10K-100K Ohm pull-up resistor to
                      Input           VCC
   TMS    7           JTAG Test Mode  Use 10K-100K Ohm pull-up resistor to
                      Select          VCC
   TCK    9           Clock into the  Use 10K-100K Ohm pull-down resistor to
                      core            GND
   RTCK   11          Return clock    Some JTAG adapters have adaptive clocking
                                      using an RTCK signal.
   DBGSEL 11          Debug Select    Some devices have special pins that
                                      enable the JTAG interface. For example,
                                      on the NXP LPC2129 the signal RTCK must
                                      be driven low during RESET to enable the
                                      JTAG interface.
   TDO    13          JTAG Test Data  Use 10K-100K Ohm pull-up resistor to VCC
                      Output
   DBGRQ  17          N/C
   DGBACK 19          N/C
   ISP    ??          ISP             Most NXP MCU's have an ISP pin which
                                      (when pulled low) can be used to cause
                                      the MCU to enter a bootloader on reset.
                                      Use 10K-100K Ohm pull up resistor.
  ------- ----------- --------------- --------------------------------------

NuttX OABI "buildroot" Toolchain
================================

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the ARM GCC toolchain (if
  different from the default in your PATH variable).

  If you have no ARMtoolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh pirelli_dpl10/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/arm7tdmi-defconfig-4.3.3 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  details PLUS some special instructions that you will need to follow if you are
  building a Cortex-M3 toolchain for Cygwin under Windows.

Generic OABI Toolchain
======================

  The NuttX OABI toolchain is selected with:

    CONFIG_ARM_TOOLCHAIN_BUILDROOT=y
    CONFIG_ARM_OABI_TOOLCHAIN=y

  In most cases, OsmocomBB is built with a different OABI toolchain with a 
  prefix of arm-elf-.  To use that toolchain, change the configuration as
  follows:

    CONFIG_ARM_TOOLCHAIN_GNU_OABI=y
