README
=====

  This README file describes the port of NuttX to the SAMA5D3x-EK
  development boards. These boards feature the Atmel SAMA5D3
  microprocessors.  Four different SAMA5D3x-EK kits are available

    - SAMA5D31-EK with the ATSAMA5D31 (http://www.atmel.com/devices/sama5d31.aspx)
    - SAMA5D33-EK with the ATSAMA5D33 (http://www.atmel.com/devices/sama5d33.aspx)
    - SAMA5D34-EK with the ATSAMA5D34 (http://www.atmel.com/devices/sama5d34.aspx)
    - SAMA5D35-EK with the ATSAMA5D35 (http://www.atmel.com/devices/sama5d35.aspx)

  The each consist of an identical base board with different plug-in
  modules for each CPU.  I also have a 7 inch LCD for my SAMA5D3x-EK, but this
  is not yet generally available..

    SAMA5D3 Family

                              ATSAMA5D31    ATSAMA5D33    ATSAMA5D34    ATSAMA5D35
    ------------------------- ------------- ------------- ------------- -------------
    Pin Count                 324           324           324           324
    Max. Operating Frequency  536           536           536           536
    CPU                       Cortex-A5     Cortex-A5     Cortex-A5     Cortex-A5
    Max I/O Pins              160           160           160           160
    Ext Interrupts            160           160           160           160
    USB Transceiver           3             3             3             3
    USB Speed                 Hi-Speed      Hi-Speed      Hi-Speed      Hi-Speed
    USB Interface             Host, Device  Host, Device  Host, Device  Host, Device
    SPI                       6             6             6             6
    TWI (I2C)                 3             3             3             3
    UART                      7             5             5             7
    CAN                       -             -             2             2
    LIN                       4             4             4             4
    SSC                       2             2             2             2
    Ethernet                  1             1             1             2
    SD / eMMC                 3             2             3             3
    Graphic LCD               Yes           Yes           Yes           -
    Camera Interface          Yes           Yes           Yes           Yes
    ADC channels              12            12            12            12
    ADC Resolution (bits)     12            12            12            12
    ADC Speed (ksps)          440           440           440           440
    Resistive Touch Screen    Yes           Yes           Yes           Yes
    Crypto Engine             AES/DES/      AES/DES/      AES/DES/      AES/DES/
                              SHA/TRNG      SHA/TRNG      SHA/TRNG      SHA/TRNG
    SRAM (Kbytes)             128           128           128           128
    External Bus Interface    1             1             1             1
    DRAM Memory               DDR2/LPDDR,   DDR2/LPDDR,   DDR2/LPDDR,   DDR2/LPDDR,
                              SDRAM/LPSDR   SDRAM/LPSDR   DDR2/LPDDR,   DDR2/LPDDR,
    NAND Interface            Yes           Yes           Yes           Yes
    Temp. Range (deg C)       -40 to 85     -40 to 85     -40 to 85     -40 to 85
    I/O Supply Class          1.8/3.3       1.8/3.3       1.8/3.3       1.8/3.3
    Operating Voltage (Vcc)   1.08 to 1.32  1.08 to 1.32  1.08 to 1.32  1.08 to 1.32
    FPU                       Yes           Yes           Yes           Yes
    MPU / MMU                 No/Yes        No/Yes        No/Yes        No/Yes
    Timers                    5             5             5             6
    Output Compare channels   6             6             6             6
    Input Capture Channels    6             6             6             6
    PWM Channels              4             4             4             4
    32kHz RTC                 Yes           Yes           Yes           Yes
    Packages                  LFBGA324_A    LFBGA324_A    LFBGA324_A    LFBGA324_A

Contents
========

  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI "buildroot" Toolchain
  - NuttX OABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - Loading Code into SRAM with J-Link
  - Writing to FLASH using SAM-BA
  - Creating and Using NORBOOT
  - Buttons and LEDs
  - Serial Consoles
  - Serial FLASH
  - HSMCI Card Slots
  - USB Ports
  - AT24 Serial EEPROM
  - SAMA5D3x-EK Configuration Options
  - Configurations

Development Environment
=======================

  Several possibile development enviorments may be use:

  - Linux or OSX native
  - Cygwin unders Windows
  - MinGW + MSYS under Windows
  - Windows native (with GNUMake from GNUWin32).

  All testing has been performed using Cygwin under Windows.

  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems.

GNU Toolchain Options
=====================

  The NuttX make system will support the several different toolchain options.

  All testing has been conducted using the CodeSourcery GCC toolchain.  To use
  a different toolchain, you simply need to add change to one of the following
  configuration options to your .config (or defconfig) file:

    CONFIG_ARMV7A_TOOLCHAIN_CODESOURCERYW=y  : CodeSourcery under Windows
    CONFIG_ARMV7A_TOOLCHAIN_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_ARMV7A_TOOLCHAIN_ATOLLIC=y        : Atollic toolchain for Windos
    CONFIG_ARMV7A_TOOLCHAIN_DEVKITARM=y      : devkitARM under Windows
    CONFIG_ARMV7A_TOOLCHAIN_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin (default)
    CONFIG_ARMV7A_TOOLCHAIN_GNU_EABIL=y      : Generic GCC ARM EABI toolchain for Linux
    CONFIG_ARMV7A_TOOLCHAIN_GNU_EABIW=y      : Generic GCC ARM EABI toolchain for Windows

  The CodeSourcery GCC toolchain is selected with
  CONFIG_ARMV7A_TOOLCHAIN_GNU_EABIW=y and setting the PATH variable
  appropriately.

  If you are not using AtmelStudio GCC toolchain, then you may also have to
  modify the PATH in the setenv.h file if your make cannot find the tools.

  NOTE about Windows native toolchains
  ------------------------------------

  There are several limitations to using a Windows based toolchain in a
  Cygwin environment.  The three biggest are:

  1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
     performed automatically in the Cygwin makefiles using the 'cygpath'
     utility but you might easily find some new path problems.  If so, check
     out 'cygpath -w'

  2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic
     links are used in Nuttx (e.g., include/arch).  The make system works
     around these problems for the Windows tools by copying directories
     instead of linking them.  But this can also cause some confusion for
     you:  For example, you may edit a file in a "linked" directory and find
     that your changes had no effect.  That is because you are building the
     copy of the file in the "fake" symbolic directory.  If you use a\
     Windows toolchain, you should get in the habit of making like this:

       make clean_context all

     An alias in your .bashrc file might make that less painful.

  3. Dependencies are not made when using Windows versions of the GCC.  This is
     because the dependencies are generated using Windows pathes which do not
     work with the Cygwin make.

       MKDEP                = $(TOPDIR)/tools/mknulldeps.sh

  NOTE 1: Older CodeSourcery toolchains (2009q1) do not work with default
  optimization level of -Os (See Make.defs).  It will work with -O0, -O1, or
  -O2, but not with -Os.

  NOTE 2: The devkitARM toolchain includes a version of MSYS make.  Make sure that
  the paths to Cygwin's /bin and /usr/bin directories appear BEFORE the devkitARM
  path or will get the wrong version of make.

IDEs
====

  NuttX is built using command-line make.  It can be used with an IDE, but some
  effort will be required to create the project (There is a simple RIDE project
  in the RIDE subdirectory).

  Makefile Build
  --------------
  Under Eclipse, it is pretty easy to set up an "empty makefile project" and
  simply use the NuttX makefile to build the system.  That is almost for free
  under Linux.  Under Windows, you will need to set up the "Cygwin GCC" empty
  makefile project in order to work with Windows (Google for "Eclipse Cygwin" -
  there is a lot of help on the internet).

  Native Build
  ------------
  Here are a few tips before you start that effort:

  1) Select the toolchain that you will be using in your .config file
  2) Start the NuttX build at least one time from the Cygwin command line
     before trying to create your project.  This is necessary to create
     certain auto-generated files and directories that will be needed.
  3) Set up include pathes:  You will need include/, arch/arm/src/sam34,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/sam34/sam_vectors.S.  You may need to build NuttX
  one time from the Cygwin command line in order to obtain the pre-built
  startup object needed by RIDE.

NuttX EABI "buildroot" Toolchain
================================

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh sama5d3x-ek/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-eabi-defconfig-4.6.3 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  details PLUS some special instructions that you will need to follow if you are
  building a Cortex-M3 toolchain for Cygwin under Windows.

  NOTE:  Unfortunately, the 4.6.3 EABI toolchain is not compatible with the
  the NXFLAT tools.  See the top-level TODO file (under "Binary loaders") for
  more information about this problem. If you plan to use NXFLAT, please do not
  use the GCC 4.6.3 EABI toochain; instead use the GCC 4.3.3 OABI toolchain.
  See instructions below.

NuttX OABI "buildroot" Toolchain
================================

  The older, OABI buildroot toolchain is also available.  To use the OABI
  toolchain, use the build instructtions above, but (1) modify the
  cortexm3-eabi-defconfig-4.6.3 configuration to use OABI (using 'make
  menuconfig'), or (2) use an exising OABI configuration such as
  cortexm3-defconfig-4.3.3

NXFLAT Toolchain
================

  If you are *not* using the NuttX buildroot toolchain and you want to use
  the NXFLAT tools, then you will still have to build a portion of the buildroot
  tools -- just the NXFLAT tools.  The buildroot with the NXFLAT tools can
  be downloaded from the NuttX SourceForge download site
  (https://sourceforge.net/projects/nuttx/files/).

  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh sama5d3x-ek/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-defconfig-nxflat .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built NXFLAT binaries.

Loading Code into SRAM with J-Link
==================================

  Loading code with the Segger tools and GDB
  ------------------------------------------

    1) Change directories into the directory where you built NuttX.
    2) Start the GDB server and wait until it is ready to accept GDB
       connections.
    3) Then run GDB like this:

         $ arm-none-eabi-gdb
         (gdb) target remote localhost:2331
         (gdb) mon reset
         (gdb) load nuttx
         (gdb) ... start debugging ...

  Loading code using J-Link Commander
  ----------------------------------

    J-Link> r
    J-Link> loadbin <file> <address>
    J-Link> setpc <address of __start>
    J-Link> ... start debugging ...

Writing to FLASH using SAM-BA
=============================

  Assumed starting configuration:

    1. You have installed the J-Lnk CDC USB driver (Windows only, there is
        no need to install a driver on any regular Linux distribution),
    2. You have the USB connected to DBGU poort (J14)
    3. Terminal configuration:  115200 8N1

  Using SAM-BA to write to FLASH:

    1. Exit the terminal emulation program and remove the USB cable from
       the DBGU port (J14)
    2. Connect the USB cable to the device USB port (J20)
    3. JP9 must open (BMS == 1) to boot from on-chip Boot ROM.
    4. Press and maintain PB4 CS_BOOT button and power up the board.  PB4
       CS_BOOT button prevents booting from Nand or serial Flash by
       disabling Flash Chip Selects after having powered the board, you can
       release the PB4 BS_BOOT button.
    5. On Windows you may need to wait for a device driver to be installed.
    6. Start the SAM-BA application, selecting (1) the correct USB serial
       port, and (2) board = at91sama5d3x-ek.
    7. The SAM-BA menu should appear.
    8. Select the FLASH bank that you want to use and the address to write
       to and "Execute"
    9. When you are finished writing to FLASH, remove the USB cable from J20
       and re-connect the serial link on USB CDC / DBGU connector (J14) and
       re-open the terminal emulator program.
    10. If you loaded code in NOR flash (CS0), then you will need to close
        JP9 (BMS == 0) to force booting out of NOR flash (see NOTE).
    11. Power cycle the board.

  NOTES:  By closing JP9 (BMS == 0), you can force the board to boot
  directly to NOR FLASH.  Executing from other memories will require that
  you provide a special code header so that you code can be recognized as a
  boot-able image by the ROM bootloader.

Creating and Using NORBOOT
==========================

  In order to have more control of debugging code that runs out of NOR FLASH,
  I created the sama5d3x-ek/norboot configuration.  That configuration is
  described below under "Configurations."

  Here are some general instructions on how to build an use norboot:

  Building:
  1. Remove any old configurations (if applicable).

       cd <nuttx>
       make distclean

  2. Install and build the norboot configuration:

       cd tools
       ./configure.sh sama5d3x-ek/<subdir>
       cd -
       . ./setenv.sh

     Before sourcing the setenv.sh file above, you should examine it and
     perform edits as necessary so that TOOLCHAIN_BIN is the correct path
     to the directory than holds your toolchain binaries.

  3. Rename the binaries.  Since you will need two versions of NuttX:  this
     norboot version that runs in internal SRAM and another under test in
     NOR FLASH, I rename the resulting binary files so that they can be
     distinguished:

       mv nuttx norboot
       mv nuttx.hex norboot.hex
       mv nuttx.bin norboot.bin

  4. Build your NOR configuration and write this into NOR FLASH.  Here, for
     example, is how you would create the NSH NOR configuration:

       cd <nuttx>
       make distclean                 # Remove the norboot configuration
       cd tools
       ./configure.sh sama5d3x-ek/nsh # Establish the NSH configuration
       cd -
       make                           # Build the NSH configuration

     Then use SAM-BA to write the nuttx.bin binary into NOR FLASH.  This
     will involve holding the CS_BOOT button and power cycling to start
     the ROM loader.  The SAM-BA serial connection will be on the device
     USB port, not the debug USB port.  Follow the SAM-BA instruction to
     write the nuttx.bin binary to NOR FLASH.

   5. Restart the system without holding CS_BOOT to get back to the normal
      debug setup.

   6. Then start the J-Link GDB server and GDB.  In GDB, I do the following:

       (gdb) mon reset                # Reset and halt the CPU
       (gdb) load norboot             # Load norboot into internal SRAM
       (gdb) mon go                   # Start norboot
       (gdb) mon halt                 # Break in
       (gdb) mon reg pc = 0x10000040  # Set the PC to NOR flash entry point
       (gdb) mon go                   # And jump into NOR flash

      The norboot program can also be configured to jump directly into
      NOR FLASH without requiring the final halt and go, but since I
      have been debugging the early boot sequence, the above sequence has
      been most convenient for me.

    STATUS:
      2013-7-30:  I have been unable to execute this configuration from NOR
        FLASH by closing the BMS jumper (J9).  As far as I can tell, this
        jumper does nothing on my board???  So I have been using the norboot
        configuration exclusively to start the program-under-test in NOR FLASH.

Buttons and LEDs
================

  Buttons
  -------
  There are five push button switches on the SAMA5D3X-EK base board:

    1. One Reset, board reset (BP1)
    2. One Wake up, push button to bring the processor out of low power mode
      (BP2)
    3. One User momentary Push Button
    4. One Disable CS Push Button

  Only the momentary push button is controllable by software (labeled
  "PB_USER1" on the board):

    - PE27.  Pressing the switch connect PE27 to grounded.  Therefore, PE27
      must be pulled high internally.  When the button is pressed the SAMA5
      will sense "0" is on PE27.

  LEDs
  ----
  There are two LEDs on the SAMA5D3 series-CM board that can be controlled
  by software.  A  blue LED is controlled via PIO pins.  A red LED normally
  provides an indication that power is supplied to the board but can also
  be controlled via software.

    PE25.  This blue LED is pulled high and is illuminated by pulling PE25
    low.

    PE24.  The red LED is also pulled high but is driven by a transistor so
    that it is illuminated when power is applied even if PE24 is not
    configured as an output.  If PE24 is configured as an output, then the
    LCD is illuminated by a high output.

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
  events as follows:

    SYMBOL                Meaning                     LED state
                                                    Blue     Red
    -------------------  -----------------------  -------- --------
    LED_STARTED          NuttX has been started     OFF      OFF
    LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF
    LED_IRQSENABLED      Interrupts enabled         OFF      OFF
    LED_STACKCREATED     Idle stack created         ON       OFF
    LED_INIRQ            In an interrupt              No change
    LED_SIGNAL           In a signal handler          No change
    LED_ASSERTION        An assertion failed          No change
    LED_PANIC            The system has crashed     OFF      Blinking
    LED_IDLE             MCU is is sleep mode         Not used

  Thus if the blue LED is statically on, NuttX has successfully booted and
  is, apparently, running normmally.  If the red is flashing at
  approximately 2Hz, then a fatal error has been detected and the system
  has halted.

Serial Consoles
===============

  USART1
  ------
  By default USART1 is used as the NuttX serial console in all
  configurations (unless otherwise noted).  USART1 is buffered with an
  RS-232 Transceiver (Analog Devices ADM3312EARU) and connected to the DB-9
  male socket (J8).

    USART1 Connector J8
    -------------------------------
    SAMA5 FUNCTION  NUTTX PIO
    PIO   NAME      CONFIGURATION
    ---- ---------- ---------------
    PB27 RTS1       PIO_USART1_RTS
    PB29 TXD1       PIO_USART1_TXD
    PB28 RXD1       PIO_USART1_RXD
    PB26 CTS1       PIO_USART1_CTS

    NOTE: Debug TX and RX pins also go to the ADM3312EARU, but I am
    uncertain of the functionality.

    -------------------------------
    SAMA5 FUNCTION  NUTTX PIO
    PIO   NAME      CONFIGURATION
    ---- ---------- ---------------
    PB31 DTXD       PIO_DBGU_DTXD
    PB30 DRXD       PIO_DBGU_DRXD

  Hardware UART via CDC
  ---------------------
  "J-Link-OB-ATSAM3U4C comes with an additional hardware UART that is
   accessible from a host via CDC which allows terminal communication with
   the target device. This feature is enabled only if a certain port (CDC
   disabled, PA25, pin 24 on J-Link-OB-ATSAM3U4C) is NOT connected to ground
   (open).

    - Jumper JP16 not fitted: CDC is enabled
    - Jumper JP16 fitted : CDC is disabled"

Serial FLASH
============

  Both the Ronetix and Embest versions of the SAMAD3x CPU modules include an
  Atmel AT25DF321A, 32-megabit, 2.7-volt SPI serial flash.  The SPI
  connection is as follows:

    AT25DF321A      SAMA5
    --------------- -----------------------------------------------
    SI              PD11 SPI0_MOSI
    SO              PD10 SPI0_MIS0
    SCK             PD12 SPI0_SPCK
    /CS             PD13 via NL17SZ126 if JP1 is closed (See below)

  JP1 and JP2 seem to related to /CS on the Ronetix board, but the usage is
  less clear.  For the Embest module, JP1 must be closed to connect /CS to
  PD13; on the Ronetix schematic, JP11 seems only to bypass a resistor (may
  not be populated?).  I think closing JP1 is correct in either case.

HSMCI Card Slots
================

  The SAMA5D3x-EK provides a two SD memory card slots:  (1) a full size SD
  card slot (J7 labeled MCI0), and (2) a microSD memory card slot (J6
  labeled MCI1).

  The full size SD card slot connects via HSMCI0.  The card detect discrete
  is available on PB17 (pulled high).  The write protect descrete is tied to
  ground (via PP6) and not available to software.  The slot supports 8-bit
  wide transfer mode, but the NuttX driver currently uses only the 4-bit
  wide transfer mode

    PD17 MCI0_CD
    PD1  MCI0_DA0
    PD2  MCI0_DA1
    PD3  MCI0_DA2
    PD4  MCI0_DA3
    PD5  MCI0_DA4
    PD6  MCI0_DA5
    PD7  MCI0_DA6
    PD8  MCI0_DA7
    PD9  MCI0_CK
    PD0  MCI0_CDA

  The microSD connects vi HSMCI1.  The card detect discrete is available on
  PB18 (pulled high):

    PD18  MCI1_CD
    PB20  MCI1_DA0
    PB21  MCI1_DA1
    PB22  MCI1_DA2
    PB23  MCI1_DA3
    PB24  MCI1_CK
    PB19  MCI1_CDA

USB Ports
=========

  The SAMA5D3 series-MB features three USB communication ports:

    * Port A Host High Speed (EHCI) and Full Speed (OHCI) multiplexed with
      USB Device High Speed Micro AB connector, J20

    * Port B Host High Speed (EHCI) and Full Speed (OHCI) standard type A
      connector, J19 upper port

    * Port C Host Full Speed (OHCI) only standard type A connector, J19
      lower port

  All three USB host ports are equipped with 500 mA high-side power switch
  for self-powered and buspowered applications. The USB device port feature
  VBUS inserts detection function.

  Port A
  ------

    PIO  Signal Name Function
    ---- ----------- -------------------------------------------------------
    PD29  VBUS_SENSE VBus detection
    PD25  EN5V_USBA  VBus power enable (via MN15 AIC1526 Dual USB High-Side
                     Power Switch.  The other channel of the switch is for
                     the LCD)

  Port B
  ------

    PIO  Signal Name Function
    ---- ----------- -------------------------------------------------------
    PD26 EN5V_USBB   VBus power enable (via MN14 AIC1526 Dual USB High-Side
                     Power Switch).  To the A1 pin of J19 Dual USB A
                     connector

  Port C
  ------

    PIO  Signal Name Function
    ---- ----------- -------------------------------------------------------
    PD27 EN5V_USBC   VBus power enable (via MN14 AIC1526 Dual USB High-Side
                     Power Switch).  To the B1 pin of J19 Dual USB A
                     connector

  Both Ports B and C
  ------------------

    PIO  Signal Name Function
    ---- ----------- -------------------------------------------------------
    PD28 OVCUR_USB   Combined overrcurrent indication from port A and B

AT24 Serial EEPROM
==================

  A AT24C512 Serial EEPPROM was used for tested I2C.  There are other I2C/TWI
  devices on-board, but the serial EEPROM is the simplest test.

  There is, however, no AT24 EEPROM on board the SAMA5D3x-EK:  The Serial
  EEPROM was mounted on an external adaptor board and connected to the
  SAMA5D3x-EK thusly:

    - VCC -- VCC
    - GND -- GND
    - TWCK0(PA31) -- SCL
    - TWD0(PA30)  -- SDA

  By default, PA30 and PA31 are SWJ-DP pins, it can be used as a pin for TWI
  peripheral in the end application.

SAMA5D3x-EK Configuration Options
=================================

  CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
  be set to:

    CONFIG_ARCH="arm"

  CONFIG_ARCH_family - For use in C code:

    CONFIG_ARCH_ARM=y

  CONFIG_ARCH_architecture - For use in C code:

    CONFIG_ARCH_CORTEXA5=y

  CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

    CONFIG_ARCH_CHIP="sama5"

  CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
  chip:

    CONFIG_ARCH_CHIP_SAMA5=y

  and one of:

   CONFIG_ARCH_CHIP_ATSAMA5D31=y
   CONFIG_ARCH_CHIP_ATSAMA5D33=y
   CONFIG_ARCH_CHIP_ATSAMA5D34=y
   CONFIG_ARCH_CHIP_ATSAMA5D35=y

  CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
  hence, the board that supports the particular chip or SoC.

    CONFIG_ARCH_BOARD="sama5d3x-ek" (for the SAMA5D3x-EK development board)

  CONFIG_ARCH_BOARD_name - For use in C code

    CONFIG_ARCH_BOARD_SAMA5D3X_EK=y

  CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
  of delay loops

  CONFIG_ENDIAN_BIG - define if big endian (default is little
  endian)

  CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

    CONFIG_RAM_SIZE=0x0002000 (128Kb)

  CONFIG_RAM_START - The physical start address of installed DRAM

    CONFIG_RAM_START=0x20000000

  CONFIG_RAM_VSTART - The virutal start address of installed DRAM

    CONFIG_RAM_VSTART=0x20000000

  CONFIG_ARCH_IRQPRIO - The SAM3UF103Z supports interrupt prioritization

    CONFIG_ARCH_IRQPRIO=y

  CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
  have LEDs

  CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
  stack. If defined, this symbol is the size of the interrupt
  stack in bytes.  If not defined, the user task stacks will be
  used during interrupt handling.

  CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

  CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

  CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
  cause a 100 second delay during boot-up.  This 100 second delay
  serves no purpose other than it allows you to calibratre
  CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
  the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
  the delay actually is 100 seconds.

  Individual subsystems can be enabled:

    CONFIG_SAMA5_DBGU        - Debug Unit Interrupt
    CONFIG_SAMA5_PIT         - Periodic Interval Timer Interrupt
    CONFIG_SAMA5_WDT         - Watchdog timer Interrupt
    CONFIG_SAMA5_HSMC        - Multi-bit ECC Interrupt
    CONFIG_SAMA5_SMD         - SMD Soft Modem
    CONFIG_SAMA5_USART0      - USART 0
    CONFIG_SAMA5_USART1      - USART 1
    CONFIG_SAMA5_USART2      - USART 2
    CONFIG_SAMA5_USART3      - USART 3
    CONFIG_SAMA5_UART0       - UART 0
    CONFIG_SAMA5_UART1       - UART 1
    CONFIG_SAMA5_TWI0        - Two-Wire Interface 0
    CONFIG_SAMA5_TWI1        - Two-Wire Interface 1
    CONFIG_SAMA5_TWI2        - Two-Wire Interface 2
    CONFIG_SAMA5_HSMCI0      - High Speed Multimedia Card Interface 0
    CONFIG_SAMA5_HSMCI1      - High Speed Multimedia Card Interface 1
    CONFIG_SAMA5_HSMCI2      - High Speed Multimedia Card Interface 2
    CONFIG_SAMA5_SPI0        - Serial Peripheral Interface 0
    CONFIG_SAMA5_SPI1        - Serial Peripheral Interface 1
    CONFIG_SAMA5_TC0         - Timer Counter 0 (ch. 0, 1, 2)
    CONFIG_SAMA5_TC1         - Timer Counter 1 (ch. 3, 4, 5)
    CONFIG_SAMA5_PWM         - Pulse Width Modulation Controller
    CONFIG_SAMA5_ADC         - Touch Screen ADC Controller
    CONFIG_SAMA5_DMAC0       - DMA Controller 0
    CONFIG_SAMA5_DMAC1       - DMA Controller 1
    CONFIG_SAMA5_UHPHS       - USB Host High Speed
    CONFIG_SAMA5_UDPHS       - USB Device High Speed
    CONFIG_SAMA5_GMAC        - Gigabit Ethernet MAC
    CONFIG_SAMA5_EMAC        - Ethernet MAC
    CONFIG_SAMA5_LCDC        - LCD Controller
    CONFIG_SAMA5_ISI         - Image Sensor Interface
    CONFIG_SAMA5_SSC0        - Synchronous Serial Controller 0
    CONFIG_SAMA5_SSC1        - Synchronous Serial Controller 1
    CONFIG_SAMA5_CAN0        - CAN controller 0
    CONFIG_SAMA5_CAN1        - CAN controller 1
    CONFIG_SAMA5_SHA         - Secure Hash Algorithm
    CONFIG_SAMA5_AES         - Advanced Encryption Standard
    CONFIG_SAMA5_TDES        - Triple Data Encryption Standard
    CONFIG_SAMA5_TRNG        - True Random Number Generator
    CONFIG_SAMA5_ARM         - Performance Monitor Unit
    CONFIG_SAMA5_FUSE        - Fuse Controller
    CONFIG_SAMA5_MPDDRC      - MPDDR controller

  Some subsystems can be configured to operate in different ways. The drivers
  need to know how to configure the subsystem.

    CONFIG_SAMA5_PIOA_IRQ    - Support PIOA interrupts
    CONFIG_SAMA5_PIOB_IRQ    - Support PIOB interrupts
    CONFIG_SAMA5_PIOC_IRQ    - Support PIOD interrupts
    CONFIG_SAMA5_PIOD_IRQ    - Support PIOD interrupts
    CONFIG_SAMA5_PIOE_IRQ    - Support PIOE interrupts

    CONFIG_USART0_ISUART     - USART0 is configured as a UART
    CONFIG_USART1_ISUART     - USART1 is configured as a UART
    CONFIG_USART2_ISUART     - USART2 is configured as a UART
    CONFIG_USART3_ISUART     - USART3 is configured as a UART

  ST91SAMA5 specific device driver settings

    CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the USARTn (n=0,1,2,3) or UART
           m (m=4,5) for the console and ttys0 (default is the USART1).
    CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_U[S]ARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_U[S]ARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_U[S]ARTn_2STOP - Two stop bits

  AT91SAMA5 USB Host Configuration
  Pre-requisites

    CONFIG_USBDEV          - Enable USB device support
    CONFIG_USBHOST         - Enable USB host support
    CONFIG_SAMA5_UHPHS     - Needed
    CONFIG_SAMA5_OHCI      - Enable the STM32 USB OTG FS block
    CONFIG_SCHED_WORKQUEUE - Worker thread support is required

  Options:

    CONFIG_SAMA5_OHCI_NEDS
      Number of endpoint descriptors
    CONFIG_SAMA5_OHCI_NTDS
      Number of transfer descriptors
    CONFIG_SAMA5_OHCI_TDBUFFERS
      Number of transfer descriptor buffers
    CONFIG_SAMA5_OHCI_TDBUFSIZE
      Size of one transfer descriptor buffer
    CONFIG_USBHOST_INT_DISABLE
      Disable interrupt endpoint support
    CONFIG_USBHOST_ISOC_DISABLE
      Disable isochronous endpoint support
    CONFIG_USBHOST_BULK_DISABLE
      Disable bulk endpoint support

config SAMA5_OHCI_REGDEBUG

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each SAM3U-EK configuration is maintained in a sub-directory and
  can be selected as follow:

    cd tools
    ./configure.sh sama5d3x-ek/<subdir>
    cd -
    . ./setenv.sh

  Before sourcing the setenv.sh file above, you should examine it and perform
  edits as necessary so that TOOLCHAIN_BIN is the correct path to the directory
  than holds your toolchain binaries.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

    make

  The <subdir> that is provided above as an argument to the tools/configure.sh
  must be is one of the following.

  NOTES:

  1. These configurations use the mconf-based configuration tool.  To
    change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       and misc/tools/

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output on UART0 (J3).

  3. Unless otherwise stated, the configurations are setup for
     Linux (or any other POSIX environment like Cygwin under Windows):

     Build Setup:
       CONFIG_HOST_LINUX=y   : Linux or other POSIX environment

  4. All of these configurations use the Code Sourcery for Windows toolchain
     (unless stated otherwise in the description of the configuration).  That
     toolchain selection can easily be reconfigured using 'make menuconfig'.
     Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_WINDOS=y                : Microsoft Windows
       CONFIG_WINDOWS_CYGWIN=y             : Using Cygwin or other POSIX environment

     System Type -> Toolchain:
       CONFIG_ARMV7A_TOOLCHAIN_GNU_EABIW=y : GNU EABI toolchain for windows

     That same configuration will work with Atmel GCC toolchain.  The only
     change required to use the Atmel GCC toolchain is to change the PATH
     variable so that those tools are selected instead of the CodeSourcery
     tools.  Try 'which arm-none-eabi-gcc' to make sure that you are
     selecting the right tool.

     The setenv.sh file is available for you to use to set the PATH
     variable.  The path in the that file may not, however, be correct
     for your installation.

     See also the "NOTE about Windows native toolchains" in the section call
     "GNU Toolchain Options" above.

  Configuration sub-directories
  -----------------------------
  Summary:  Some of the descriptions below are long and wordy. Here is the
  concise summary of the available SAMA5D3x-EK configurations:

    demo: This is an NSH configuration that supports as much functionality
      as possible.  That is why it gets its name:  It attempts to show as
      much as possible
    hello:  The tiniest configuration possible (almost).  It just says
      "Hello, World!"  On the serial console.  It is so tiny that it is
      able to run entirely out of internal SRAM (all of the other
      configurations except norboot use NOR FLASH for .text and internal
      SRAM for .data and .bass).  This configuration is only useful for
      bring-up.
    norboot:
      This is a little program to help debug of code in NOR flash.  I wrote
      it because I don't yet understand how to get the SAMA5 to boot from
      NOR FLASH.  See the description below and the section above entitled
      "Creating and Using NORBOOT" for more information
    nsh:  This is another NSH configuration, not too different from the
      demo configuration.  The nsh configuration is, however, bare bones.
      It is the simplest possible NSH configuration and is useful as a
      platform for debugging and integrating new features in isolation.
    ostest:  This is another configuration that is only useful for bring-up.
      It executes an exhaustive OS test to verify a correct port of NuttX
      to the SAMA5D3-EK.  Since it now passes that test, the configuration
      has little further use other than for reference.

  Now for the gory details:

  demo:
    This configuration directory provide the NuttShell (NSH).  There are
    two NSH configurations:  nsh and demo.  The difference is that nsh is
    intended to be a very simple NSH configuration upon which you can build
    further functionality.  The demo configuration, on the other hand, is
    intended to be a rich configuration that shows many features all working
    together.

    See also the NOTES associated with the nsh configuration for other hints
    about features that can be included with this configuration.

    NOTES:
    1. This configuration uses the default USART1 serial console.  That
       is easily changed by reconfiguring to (1) enable a different
       serial peripheral, and (2) selecting that serial peripheral as
       the console device.

    2. By default, this configuration is set up to build on Windows
       under either a Cygwin or MSYS environment using a recent, Windows-
       native, generic ARM EABI GCC toolchain (such as the CodeSourcery
       toolchain).  Both the build environment and the toolchain
       selection can easily be changed by reconfiguring:

       CONFIG_HOST_WINDOWS=y                   : Windows operating system
       CONFIG_WINDOWS_CYGWIN=y                 : POSIX environment under windows
       CONFIG_ARMV7A_TOOLCHAIN_CODESOURCERYW=y : CodeSourcery for Windows

    3. This configuration executes out of CS0 NOR flash and can only
       be loaded via SAM-BA.  These are the relevant configuration options
       the define the NOR FLASH configuration:

       CONFIG_SAMA5_BOOT_CS0FLASH=y            : Boot from FLASH on CS0
       CONFIG_BOOT_RUNFROMFLASH=y              : Run in place on FLASH (vs copying to RAM)

       CONFIG_SAMA5_EBICS0=y                   : Enable CS0 external memory
       CONFIG_SAMA5_EBICS0_SIZE=134217728      : Memory size is 128KB
       CONFIG_SAMA5_EBICS0_NOR=y               : Memory type is NOR FLASH

       CONFIG_FLASH_START=0x10000000           : Physical FLASH start address
       CONFIG_FLASH_VSTART=0x10000000          : Virtual FLASH start address
       CONFIG_FLASH_SIZE=134217728             : FLASH size (again)

       CONFIG_RAM_START=0x00300400             : Data stored after page table
       CONFIG_RAM_VSTART=0x00300400
       CONFIG_RAM_SIZE=114688                  : Available size of 128KB - 16KB for page table

       NOTE:  In order to boot in this configuration, you need to close the
       BMS jumper.

    The following features are pre-enabled in the demo configuration, but not
    in the nsh configuration:

    4. SDRAM is supported.  .data and .bss is still retained in ISRAM, but
       SDRAM is intialized and the SDRAM memory is included in the heap.
       Relevant configuration settings:

       System Type->ATSAMA5 Peripheral Support
         CONFIG_SAMA5_MPDDRC=y                 : Enable the DDR controller

       System Type->External Memory Configuration
         CONFIG_SAMA5_DDRCS=y                  : Tell the system that DRAM is at the DDR CS
         CONFIG_SAMA5_DDRCS_SIZE=268435456     : 2Gb DRAM -> 256GB
         CONFIG_SAMA5_DDRCS_LPDDR2=y           : Its DDR2
         CONFIG_SAMA5_MT47H128M16RT=y          : This is the type of DDR2

       System Type->Heap Configuration
         CONFIG_SAMA5_DDRCS_HEAP=y             : Add the SDRAM to the heap

       Memory Management
         CONFIG_MM_REGIONS=2                   : Two heap memory regions:  ISRAM and SDRAM

    5. The Embest or Ronetix CPU module includes an Atmel AT25DF321A,
       32-megabit, 2.7-volt SPI serial flash.  Support for that serial
       FLASH can is enabled in this configuration.  These are the relevant
       configuration settings:

       System Type -> SAMA5 Peripheral Support
         CONFIG_SAMA5_SPI0=y                   : Enable SPI0
         CONFIG_SAMA5_DMAC0=y                  : Enable DMA controller 0

       System Type -> SPI device driver options
         CONFIG_SAMA5_SPI_DMA=y                : Use DMA for SPI transfers
         CONFIG_SAMA5_SPI_DMATHRESHOLD=4       : Don't DMA for small transfers

       Device Drivers -> SPI Driver Support
         CONFIG_SPI=y                          : Enable SPI support
         CONFIG_SPI_EXCHANGE=y                 : Support the exchange method

       Device Drivers -> Memory Technology Device (MTD) Support
         CONFIG_MTD=y                          : Enable MTD support
         CONFIG_MTD_AT25=y                     : Enable the AT25 driver
         CONFIG_AT25_SPIMODE=0                 : Use SPI mode 0
         CONFIG_AT25_SPIFREQUENCY=20000000     : Use SPI frequency 20MHz

       Application Configuration -> NSH Library
         CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

       Board Selection
         CONFIG_SAMA5_AT25_AUTOMOUNT=y         : Mounts AT25 for NSH
         CONFIG_SAMA5_AT25_FTL=y               : Create block driver for FAT

       NOTE that you must close JP1 on the Embest/Ronetix board in
       order to enable the AT25 FLASH chip select.

       You can then format the AT25 FLASH for a FAT file system and mount
       the file system at /mnt/at25 using these NSH commands:

         nsh> mkfatfs /dev/mtdblock0
         nsh> mount -t vfat /dev/mtdblock0 /mnt/at25

       Then you an use the FLASH as a normal FAT file system:

         nsh> echo "This is a test" >/mnt/at25/atest.txt
         nsh> ls -l /mnt/at25
         /mnt/at25:
          -rw-rw-rw-      16 atest.txt
         nsh> cat /mnt/at25/atest.txt
         This is a test

       NOTE:  It appears that if Linux runs out of NAND, it will destroy the
       contents of the AT25.

    6. Support for HSMCI car slots. The SAMA5D3x-EK provides a two SD memory
       card slots:  (1) a full size SD card slot (J7 labeled MCI0), and (2)
       a microSD memory card slot (J6 labeled MCI1).  The full size SD card
       slot connects via HSMCI0; the microSD connects vi HSMCI1.  Relevant
       configuration settings include:

       System Type->ATSAMA5 Peripheral Support
         CONFIG_SAMA5_HSMCI0=y                 : Enable HSMCI0 support
         CONFIG_SAMA5_HSMCI1=y                 : Enable HSMCI1 support
         CONFIG_SAMA5_DMAC0=y                  : DMAC0 is needed by HSMCI0
         CONFIG_SAMA5_DMAC1=y                  : DMAC1 is needed by HSMCI1

       System Type
         CONFIG_SAMA5_PIO_IRQ=y                : PIO interrupts needed
         CONFIG_SAMA5_PIOD_IRQ=y               : Card detect pins are on PIOD

       Device Drivers -> MMC/SD Driver Support
         CONFIG_MMCSD=y                        : Enable MMC/SD support
         CONFIG_MMSCD_NSLOTS=1                 : One slot per driver instance
         CONFIG_MMCSD_HAVECARDDETECT=y         : Supports card-detect PIOs
         CONFIG_MMCSD_MMCSUPPORT=n             : Interferes with some SD cards
         CONFIG_MMCSD_SPI=n                    : No SPI-based MMC/SD support
         CONFIG_MMCSD_SDIO=y                   : SDIO-based MMC/SD support
         CONFIG_SDIO_DMA=y                     : Use SDIO DMA
         CONFIG_SDIO_BLOCKSETUP=y              : Needs to know block sizes

       Library Routines
         CONFIG_SCHED_WORKQUEUE=y              : Driver needs work queue support

       Application Configuration -> NSH Library
         CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

       Using the SD card:

       1) After booting, the HSCMI devices will appear as /dev/mmcsd0
          and /dev/mmcsd1.

       2) If you try mounting an SD card with nothing in the slot, the
          mount will fail:

            nsh> mount -t vfat /dev/mmcsd1 /mnt/sd1
            nsh: mount: mount failed: 19

          NSH can be configured to provide errors as strings instead of
          numbers.  But in this case, only the error number is reported.
          The  error numbers can be found in nuttx/include/errno.h:

            #define ENODEV              19
            #define ENODEV_STR          "No such device"

          So the mount command is saying that there is no device or, more
          correctly, that there is no card in the SD card slot.

       3) Inserted the SD card.  Then the mount should succeed.

           nsh> mount -t vfat /dev/mmcsd1 /mnt/sd1
           nsh> ls /mnt/sd1
           /mnt/sd1:
            atest.txt
           nsh> cat /mnt/sd1/atest.txt
           This is a test

       4) Before removing the card, you must umount the file system.  This
          is equivalent to "ejecting" or "safely removing" the card on
          Windows:  It flushes any cached data to the card and makes the SD
          card unavailable to the applications.

            nsh> umount -t /mnt/sd1

          It is now safe to remove the card.  NuttX provides into callbacks
          that can be used by an application to automatically unmount the
          volume when it is removed.  But those callbacks are not used in
          this configuration.

    7. Support the USB high-speed EHCI device (UDPHS) driver is enabled.
       These are the relevant NuttX configuration settings:

       Device Drivers -> USB Device Driver Support
         CONFIG_USBDEV=y                       : Enable USB device support
         CONFIG_USBDEV_DUALSPEED=y             : Device support High and Full Speed
         CONFIG_USBDEV_DMA=y                   : Device uses DMA

       System Type -> ATSAMA5 Peripheral Support
         CONFIG_SAMA5_UDPHS=y                  : Enable UDPHS High Speed USB device

       Application Configuration -> NSH Library
         CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

       The Mass Storage Class (MSC) class driver is seleced for use with
       UDPHS:

       Device Drivers -> USB Device Driver Support
         CONFIG_USBMSC=y                       : Enable the USB MSC class driver
         CONFIG_USBMSC_EPBULKOUT=1             : Use EP1 for the BULK OUT endpoint
         CONFIG_USBMSC_EPBULKIN=2              : Use EP2 for the BULK IN endpoint

       The following setting enables an example that can can be used to
       control the CDC/ACM device.  It will add two new NSH commands:

         a. msconn will connect the USB serial device and export the AT25
            to the the host, and
         b. msdis which will disconnect the USB serial device.

       Application Configuration -> Examples:
         CONFIG_EXAMPLES_USBMSC=y              : Enable the USBMSC example
         CONFIG_EXAMPLES_USBMSC_NLUNS=1        : One LUN
         CONFIG_EXAMPLES_USBMSC_DEVMINOR1=0    : Minor device zero
         CONFIG_EXAMPLES_USBMSC_DEVPATH1="/dev/mmcsd0"
                                               : Use a single, LUN:  The AT25
                                               : block driver.

       NOTE:  To prevent file system corruption, make sure that the AT25
       is un-mounted *before* exporting the mass storage device to the host:

         nsh> umount /mnt
         nsh> mscon

       The AT25 can be re-mount after the mass storage class is disconnected:

         nsh> msdis
         nsh> mount -t vfat /dev/mtdblock0 /mnt/at25

    8. The USB high-speed EHCI and the low-/full- OHCI host drivers are supported
       in this configuration.

       Here are the relevant configuration options that enable EHCI support:

       System Type -> ATSAMA5 Peripheral Support
         CONFIG_SAMA5_UHPHS=y                 : USB Host High Speed

       System Type -> USB High Speed Host driver options
         CONFIG_SAMA5_EHCI=y                  : High-speed EHCI support
         CONFIG_SAMA5_OHCI=y                  : Low/full-speed OHCI support
                                              : Defaults for values probably OK for both
       Device Drivers
         CONFIG_USBHOST=y                     : Enable USB host support
         CONFIG_USBHOST_ISOC_DISABLE=y        : Isochronous endpoints not needed

       Device Drivers -> USB Host Driver Support
         CONFIG_USBHOST_ISOC_DISABLE=y        : Isochronous endpoints not used
         CONFIG_USBHOST_MSC=y                 : Enable the mass storage class driver
         CONFIG_USBHOST_HIDKBD=y              : Enable the HID keybaord class driver
                                              : Defaults for values probably OK for both

       Library Routines
         CONFIG_SCHED_WORKQUEUE=y             : Worker thread support is required

       Application Configuration -> NSH Library
         CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

       Example Usage:

       NuttShell (NSH) NuttX-6.29
       nsh> ls /dev
        /dev:
        console
        mtdblock0
        null
        ttyS0

       Here a USB FLASH stick is inserted.  Nothing visible happens in the
       the shell.  But a new device will appear:

       nsh> ls /dev
       /dev:
        console
        mtdblock0
        null
        sda
        ttyS0
       nsh> mount -t vfat /dev/sda /mnt/sda
       nsh> ls -l /mnt/sda
       /mnt/sda:
        -rw-rw-rw-    8788 viminfo
        drw-rw-rw-       0 .Trash-1000/
        -rw-rw-rw-    3378 zmodem.patch
        -rw-rw-rw-    1503 sz-1.log
        -rw-rw-rw-     613 .bashrc

    The following features are *not* enabled in the demo configuration but
    might be of some use to you:

    9.  Debugging USB.  There is normal console debug output available that
        can be enabled with CONFIG_DEBUG + CONFIG_DEBUG_USB.  However, USB
        operation is very time critical and enabling this debug output WILL
        interfere with some operation.  USB tracing is a less invasive way
        to get debug information:  If tracing is enabled, the USB driver(s)
        will save encoded trace output in in-memory buffers; if the USB
        monitor is also enabled, those trace buffers will be periodically
        emptied and dumped to the system logging device (the serial console
        in this configuration):

        Either or both USB device or host controller driver tracing can
        be enabled:

        Device Drivers -> "USB Device Driver Support:
          CONFIG_USBDEV_TRACE=y                   : Enable USB device trace feature
          CONFIG_USBDEV_TRACE_NRECORDS=256        : Buffer 256 records in memory
          CONFIG_USBDEV_TRACE_STRINGS=y           : (optional)

        Device Drivers -> "USB Host Driver Support:
          CONFIG_USBHOST_TRACE=y                   : Enable USB host trace feature
          CONFIG_USBHOST_TRACE_NRECORDS=256        : Buffer 256 records in memory
          CONFIG_USBHOST_TRACE_VERBOSE=y           : Buffer everything

        These settings will configure the USB monitor thread which will dump the
        buffered USB debug data once every second:

        Application Configuration -> NSH LIbrary:
          CONFIG_NSH_USBDEV_TRACE=n               : No builtin tracing from NSH
          CONFIG_NSH_ARCHINIT=y                   : Automatically start the USB monitor

        Application Configuration -> System NSH Add-Ons:
          CONFIG_SYSTEM_USBMONITOR=y              : Enable the USB monitor daemon
          CONFIG_SYSTEM_USBMONITOR_STACKSIZE=2048 : USB monitor daemon stack size
          CONFIG_SYSTEM_USBMONITOR_PRIORITY=50    : USB monitor daemon priority
          CONFIG_SYSTEM_USBMONITOR_INTERVAL=1     : Dump trace data every second

          CONFIG_SYSTEM_USBMONITOR_TRACEINIT=y    : Enable TRACE output (USB device tracing only)
          CONFIG_SYSTEM_USBMONITOR_TRACECLASS=y
          CONFIG_SYSTEM_USBMONITOR_TRACETRANSFERS=y
          CONFIG_SYSTEM_USBMONITOR_TRACECONTROLLER=y
          CONFIG_SYSTEM_USBMONITOR_TRACEINTERRUPTS=y

       NOTE: If USB debug output is also enabled, both outpus will appear
       on the serial console.  However, the debug output will be
       asynchronous with the trace output and, hence, difficult to
       interpret.

    STATUS:
      AT25
      2013-9-6:  I have not confirmed this, but it appears that the AT25 does not
        retain its formatting across power cycles.  I think that the contents of
        the AT25 are destroyed (i.e., reformatted for different use) by Linux when
        it runs out of NAND.

      OHCI WITH EHCI
      2013-9-11:  OHCI does not work with EHCI.  At present, EHCI receives the
        full- or low-speed devices and correctly hands them off to OHCI.  But, for
        some unknown reason, the connection is lost and the port reverts to EHCI which
        returns the port to OHCI.  This sequence of connection events occurs
        indefinitiely.  OHCI does work without EHCI enabled, however.

  hello:
    This configuration directory, performs the (almost) simplest of all
    possible examples:  examples/hello.  This just comes up, says hello
    on the serial console and terminates.  This configuration is of
    value during bring-up because it is small and can run entirely out
    of internal SRAM.

    NOTES:
    1. This configuration uses the default USART1 serial console.  That
       is easily changed by reconfiguring to (1) enable a different
       serial peripheral, and (2) selecting that serial peripheral as
       the console device.

    2. By default, this configuration is set up to build on Windows
       under either a Cygwin or MSYS environment using a recent, Windows-
       native, generic ARM EABI GCC toolchain (such as the CodeSourcery
       toolchain).  Both the build environment and the toolchain
       selection can easily be changed by reconfiguring:

       CONFIG_HOST_WINDOWS=y                   : Windows operating system
       CONFIG_WINDOWS_CYGWIN=y                 : POSIX environment under windows
       CONFIG_ARMV7A_TOOLCHAIN_CODESOURCERYW=y : CodeSourcery for Windows

    3. This configuration executes out of internal SRAM and can only
       be loaded via JTAG.

       CONFIG_SAMA5_BOOT_ISRAM=y               : Boot into internal SRAM
       CONFIG_BOOT_RUNFROMISRAM=y              : Run from internal SRAM

    STATUS:
      2013-7-19:  This configuration (as do the others) run at 396MHz.
        The SAMA5D3 can run at 536MHz.  I still need to figure out the
        PLL settings to get that speed.

      2013-7-28:  This configuration was verified functional.

      2013-7-31:  Delay loop calibrated.

  norboot:
    This is a little program to help debug of code in NOR flash.  It
    does the following:

    - It enables and configures NOR FLASH, then
    - Waits for you to break in with GDB.

    At that point, you can set the PC and begin executing from NOR FLASH
    under debug control.

    NOTES:
    1. This program derives from the hello configuration.  All of the
       notes there apply to this configuration as well.

    STATUS:
      2013-7-19:  This configuration (as do the others) run at 396MHz.
        The SAMA5D3 can run at 536MHz.  I still need to figure out the
        PLL settings to get that speed.

      2013-7-31:  Delay loop calibrated.

  nsh:
    This configuration directory provide the NuttShell (NSH).  There are
    two NSH configurations:  nsh and demo.  The difference is that nsh is
    intended to be a very simple NSH configuration upon which you can build
    further functionality.  The demo configuration, on the other hand, is
    intended to be a rich configuration that shows many features all working
    together.

    NOTES:
    1. This configuration uses the default USART1 serial console.  That
       is easily changed by reconfiguring to (1) enable a different
       serial peripheral, and (2) selecting that serial peripheral as
       the console device.

    2. By default, this configuration is set up to build on Windows
       under either a Cygwin or MSYS environment using a recent, Windows-
       native, generic ARM EABI GCC toolchain (such as the CodeSourcery
       toolchain).  Both the build environment and the toolchain
       selection can easily be changed by reconfiguring:

       CONFIG_HOST_WINDOWS=y                   : Windows operating system
       CONFIG_WINDOWS_CYGWIN=y                 : POSIX environment under windows
       CONFIG_ARMV7A_TOOLCHAIN_CODESOURCERYW=y : CodeSourcery for Windows

    3. This configuration executes out of CS0 NOR flash and can only
       be loaded via SAM-BA.  These are the relevant configuration options
       the define the NOR FLASH configuration:

       CONFIG_SAMA5_BOOT_CS0FLASH=y            : Boot from FLASH on CS0
       CONFIG_BOOT_RUNFROMFLASH=y              : Run in place on FLASH (vs copying to RAM)

       CONFIG_SAMA5_EBICS0=y                   : Enable CS0 external memory
       CONFIG_SAMA5_EBICS0_SIZE=134217728      : Memory size is 128KB
       CONFIG_SAMA5_EBICS0_NOR=y               : Memory type is NOR FLASH

       CONFIG_FLASH_START=0x10000000           : Physical FLASH start address
       CONFIG_FLASH_VSTART=0x10000000          : Virtual FLASH start address
       CONFIG_FLASH_SIZE=134217728             : FLASH size (again)

       CONFIG_RAM_START=0x00300400             : Data stored after page table
       CONFIG_RAM_VSTART=0x00300400
       CONFIG_RAM_SIZE=114688                  : Available size of 128KB - 16KB for page table

       NOTE:  In order to boot in this configuration, you need to close the
       BMS jumper.

    4. This configuration has support for NSH built-in applications enabled.
       However, no built-in applications are selected in the base configuration.

    5. This configuration has support for the FAT file system built in.  However,
       by default, there are no block drivers intialized.  The FAT file system can
       still be used to create RAM disks.

    6. SDRAM support can be enabled by adding the following to your NuttX
       configuration file:

       System Type->ATSAMA5 Peripheral Support
       CONFIG_SAMA5_MPDDRC=y                   : Enable the DDR controller

       System Type->External Memory Configuration
       CONFIG_SAMA5_DDRCS=y                    : Tell the system that DRAM is at the DDR CS
       CONFIG_SAMA5_DDRCS_SIZE=268435456       : 2Gb DRAM -> 256GB
       CONFIG_SAMA5_DDRCS_LPDDR2=y             : Its DDR2
       CONFIG_SAMA5_MT47H128M16RT=y            : This is the type of DDR2

       Now that you have SDRAM enabled, what are you going to do with it?  One
       thing you can is add it to the heap

       System Type->Heap Configuration
       CONFIG_SAMA5_DDRCS_HEAP=y               : Add the SDRAM to the heap

       Memory Management
       CONFIG_MM_REGIONS=2                     : Two memory regions:  ISRAM and SDRAM

       Another thing you could do is to enable the RAM test built-in
       application:

    7. You can enable the NuttX RAM test that may be used to verify the
       external SDAM.  To do this, keep the SDRAM out of the heap so that
       it can be tested without crashing programs using the memory:

       System Type->Heap Configuration
       CONFIG_SAMA5_DDRCS_HEAP=n               : Don't add the SDRAM to the heap

       Memory Management
       CONFIG_MM_REGIONS=1                     : One memory regions:  ISRAM

       Then enable the RAM test built-in application:

       Application Configuration->System NSH Add-Ons->Ram Test
       CONFIG_SYSTEM_RAMTEST=y

       In this configuration, the SDRAM is not added to heap and so is not
       excessible to the applications.  So the RAM test can be freely
       executed against the SRAM memory beginning at address 0x2000:0000
       (DDR CS):

       nsh> ramtest -h
       Usage: <noname> [-w|h|b] <hex-address> <decimal-size>

       Where:
         <hex-address> starting address of the test.
         <decimal-size> number of memory locations (in bytes).
         -w Sets the width of a memory location to 32-bits.
         -h Sets the width of a memory location to 16-bits (default).
         -b Sets the width of a memory location to 8-bits.

       To test the entire external 256MB SRAM:

       nsh> ramtest -w 20000000 268435456
       RAMTest: Marching ones: 20000000 268435456
       RAMTest: Marching zeroes: 20000000 268435456
       RAMTest: Pattern test: 20000000 268435456 55555555 aaaaaaaa
       RAMTest: Pattern test: 20000000 268435456 66666666 99999999
       RAMTest: Pattern test: 20000000 268435456 33333333 cccccccc
       RAMTest: Address-in-address test: 20000000 268435456

    8. The Embest or Ronetix CPU module includes an Atmel AT25DF321A,
       32-megabit, 2.7-volt SPI serial flash.  Support for that serial
       FLASH can be enabled by modifying the NuttX configuration as
       follows:

       System Type -> SAMA5 Peripheral Support
         CONFIG_SAMA5_SPI0=y                   : Enable SPI0

       Device Drivers -> SPI Driver Support
         CONFIG_SPI=y                          : Enable SPI support
         CONFIG_SPI_EXCHANGE=y                 : Support the exchange method

       Device Drivers -> Memory Technology Device (MTD) Support
         CONFIG_MTD=y                          : Enable MTD support
         CONFIG_MTD_AT25=y                     : Enable the AT25 driver
         CONFIG_AT25_SPIMODE=0                 : Use SPI mode 0
         CONFIG_AT25_SPIFREQUENCY=20000000     : Use SPI frequency 20MHz

       Application Configuration -> NSH Library
         CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

       Board Selection
         CONFIG_SAMA5_AT25_AUTOMOUNT=y         : Mounts AT25 for NSH
         CONFIG_SAMA5_AT25_FTL=y               : Create block driver for FAT

       The SPI driver can be built to do polled or DMA SPI data transfers.
       The following additional changes will enable SPI DMA:

       System Type -> SAMA5 Peripheral Support
         CONFIG_SAMA5_DMAC0=y                   : Enable DMA controller 0

       System Type -> SPI device driver options
         CONFIG_SAMA5_SPI_DMA=y                 : Use DMA for SPI transfers
         CONFIG_SAMA5_SPI_DMATHRESHOLD=4        : Don't DMA for small transfers

       NOTE that you must close JP1 on the Embest/Ronetix board in
       order to enable the AT25 FLASH chip select.

       You can then format the AT25 FLASH for a FAT file system and mount
       the file system at /mnt/at25 using these NSH commands:

         nsh> mkfatfs /dev/mtdblock0
         nsh> mount -t vfat /dev/mtdblock0 /mnt/at25

       Then you an use the FLASH as a normal FAT file system:

         nsh> echo "This is a test" >/mnt/at25/atest.txt
         nsh> ls -l /mnt/at25
         /mnt/at25:
          -rw-rw-rw-      16 atest.txt
         nsh> cat /mnt/at25/atest.txt
         This is a test

    9. Enabling HSMCI support. The SAMA5D3x-EK provides a two SD memory card
       slots:  (1) a full size SD card slot (J7 labeled MCI0), and (2) a
       microSD memory card slot (J6 labeled MCI1).  The full size SD card
       slot connects via HSMCI0; the microSD connects vi HSMCI1.  Support
       for both SD slots can be enabled with the following settings:

       System Type->ATSAMA5 Peripheral Support
         CONFIG_SAMA5_HSMCI0=y                 : Enable HSMCI0 support
         CONFIG_SAMA5_HSMCI1=y                 : Enable HSMCI1 support
         CONFIG_SAMA5_DMAC0=y                  : DMAC0 is needed by HSMCI0
         CONFIG_SAMA5_DMAC1=y                  : DMAC1 is needed by HSMCI1

       System Type
         CONFIG_SAMA5_PIO_IRQ=y                : PIO interrupts needed
         CONFIG_SAMA5_PIOD_IRQ=y               : Card detect pins are on PIOD

       Device Drivers -> MMC/SD Driver Support
         CONFIG_MMCSD=y                        : Enable MMC/SD support
         CONFIG_MMSCD_NSLOTS=1                 : One slot per driver instance
         CONFIG_MMCSD_HAVECARDDETECT=y         : Supports card-detect PIOs
         CONFIG_MMCSD_MMCSUPPORT=n             : Interferes with some SD cards
         CONFIG_MMCSD_SPI=n                    : No SPI-based MMC/SD support
         CONFIG_MMCSD_SDIO=y                   : SDIO-based MMC/SD support
         CONFIG_SDIO_DMA=y                     : Use SDIO DMA
         CONFIG_SDIO_BLOCKSETUP=y              : Needs to know block sizes

       Library Routines
         CONFIG_SCHED_WORKQUEUE=y              : Driver needs work queue support

       Application Configuration -> NSH Library
         CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

       Using the SD card:

       1) After booting, the HSCMI devices will appear as /dev/mmcsd0
          and /dev/mmcsd1.

       2) If you try mounting an SD card with nothing in the slot, the
          mount will fail:

            nsh> mount -t vfat /dev/mmcsd1 /mnt/sd1
            nsh: mount: mount failed: 19

          NSH can be configured to provide errors as strings instead of
          numbers.  But in this case, only the error number is reported.
          The  error numbers can be found in nuttx/include/errno.h:

            #define ENODEV              19
            #define ENODEV_STR          "No such device"

          So the mount command is saying that there is no device or, more
          correctly, that there is no card in the SD card slot.

       3) Inserted the SD card.  Then the mount should succeed.

           nsh> mount -t vfat /dev/mmcsd1 /mnt/sd1
           nsh> ls /mnt/sd1
           /mnt/sd1:
            atest.txt
           nsh> cat /mnt/sd1/atest.txt
           This is a test

       3) Before removing the card, you must umount the file system.  This
          is equivalent to "ejecting" or "safely removing" the card on
          Windows:  It flushes any cached data to the card and makes the SD
          card unavailable to the applications.

            nsh> umount -t /mnt/sd1

          It is now safe to remove the card.  NuttX provides into callbacks
          that can be used by an application to automatically unmount the
          volume when it is removed.  But those callbacks are not used in
          this configuration.

    10. Support the USB low/full-speed OHCI host driver can be enabled by changing
        the NuttX configuration file as follows:

        System Type -> ATSAMA5 Peripheral Support
          CONFIG_SAMA5_UHPHS=y                 : USB Host High Speed

        System Type -> USB High Speed Host driver options
          CONFIG_SAMA5_OHCI=y                  : Low/full-speed OHCI support
                                               : Defaults for values probably OK
        Device Drivers
          CONFIG_USBHOST=y                     : Enable USB host support

        Device Drivers -> USB Host Driver Support
          CONFIG_USBHOST_ISOC_DISABLE=y        : Isochronous endpoints not used
          CONFIG_USBHOST_MSC=y                 : Enable the mass storage class driver

        Library Routines
          CONFIG_SCHED_WORKQUEUE=y             : Worker thread support is required

       Application Configuration -> NSH Library
         CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

       NOTE:  When OHCI is selected, the SAMA5 will operate at 384MHz instead
       of 396MHz.  This is so that the PLL generates a frequency which is a
       multiple of the 48MHz needed for OHCI.  The delay loop calibration
       values that are used will be off slightly because of this.

    11. Support the USB high-speed EHCI host driver can be enabled by changing
        the NuttX configuration file as follows.  If EHCI is enabled by itself,
        then only high-speed devices can be supported.  If OHCI is also enabled,
        then all low-, full-, and high speed devices should work.

        System Type -> ATSAMA5 Peripheral Support
          CONFIG_SAMA5_UHPHS=y                 : USB Host High Speed

        System Type -> USB High Speed Host driver options
          CONFIG_SAMA5_EHCI=y                  : High-speed EHCI support
          CONFIG_SAMA5_OHCI=y                  : Low/full-speed OHCI support
                                               : Defaults for values probably OK for both
        Device Drivers
          CONFIG_USBHOST=y                     : Enable USB host support
          CONFIG_USBHOST_INT_DISABLE=y         : Interrupt endpoints not needed
          CONFIG_USBHOST_ISOC_DISABLE=y        : Isochronous endpoints not needed

        Device Drivers -> USB Host Driver Support
          CONFIG_USBHOST_ISOC_DISABLE=y        : Isochronous endpoints not used
          CONFIG_USBHOST_MSC=y                 : Enable the mass storage class driver

        Library Routines
          CONFIG_SCHED_WORKQUEUE=y             : Worker thread support is required

        Application Configuration -> NSH Library
          CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

        Debugging USB Host.  There is normal console debug output available
        that can be enabled with CONFIG_DEBUG + CONFIG_DEBUG_USB.  However,
        USB host operation is very time critical and enabling this debug
        output might interfere with the operation of the UDPHS.  USB host
        tracing is a less invasive way to get debug information:  If tracing
        is enabled, the USB host will save encoded trace output in in-memory
        buffer; if the USB monitor is also enabled, that trace buffer will be
        periodically emptied and dumped to the system logging device (the
        serial console in this configuration):

        Device Drivers -> "USB Host Driver Support:
          CONFIG_USBHOST_TRACE=y                   : Enable USB host trace feature
          CONFIG_USBHOST_TRACE_NRECORDS=256        : Buffer 256 records in memory
          CONFIG_USBHOST_TRACE_VERBOSE=y           : Buffer everything

        Application Configuration -> NSH LIbrary:
          CONFIG_NSH_USBDEV_TRACE=n               : No builtin tracing from NSH
          CONFIG_NSH_ARCHINIT=y                   : Automatically start the USB monitor

        Application Configuration -> System NSH Add-Ons:
          CONFIG_SYSTEM_USBMONITOR=y              : Enable the USB monitor daemon
          CONFIG_SYSTEM_USBMONITOR_STACKSIZE=2048 : USB monitor daemon stack size
          CONFIG_SYSTEM_USBMONITOR_PRIORITY=50    : USB monitor daemon priority
          CONFIG_SYSTEM_USBMONITOR_INTERVAL=1     : Dump trace data every second
          CONFIG_SYSTEM_USBMONITOR_TRACEINIT=y    : Enable TRACE output
          CONFIG_SYSTEM_USBMONITOR_TRACECLASS=y
          CONFIG_SYSTEM_USBMONITOR_TRACETRANSFERS=y
          CONFIG_SYSTEM_USBMONITOR_TRACECONTROLLER=y
          CONFIG_SYSTEM_USBMONITOR_TRACEINTERRUPTS=y

       NOTE: If USB debug output is also enabled, both outpus will appear
       on the serial console.  However, the debug output will be
       asynchronous with the trace output and, hence, difficult to
       interpret.

    12. Support the USB high-speed USB device driver (UDPHS) can be enabled
        by changing the NuttX configuration file as follows:

        Device Drivers -> USB Device Driver Support
          CONFIG_USBDEV=y                       : Enable USB device support
          CONFIG_USBDEV_DMA=y                   : Device uses DMA
          CONFIG_USBDEV_DUALSPEED=y             : Device support High and Full Speed

        System Type -> ATSAMA5 Peripheral Support
          CONFIG_SAMA5_UDPHS=y                  : Enable UDPHS High Speed USB device

        Application Configuration -> NSH Library
          CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

        You also need to select a device-side class driver for the USB device,
        This will select the CDC/ACM serial device.  Defaults for the other
        options should be okay.

        Device Drivers -> USB Device Driver Support
          CONFIG_CDCACM=y                       : Enable the CDC/ACM device
          CONFIG_CDCACM_BULKIN_REQLEN=768       : Default too small for high-speed

        The following setting enables an example that can can be used to
        control the CDC/ACM device.  It will add two new NSH commands:
        (1) sercon will connect the USB serial device (creating /dev/ttyACM0),
        and (2) serdis which will disconnect the USB serial device (destroying
        /dev/ttyACM0).

        Application Configuration -> Examples:
          CONFIG_EXAMPLES_CDCACM=y              : Enable an CDC/ACM example

        Debugging USB Device.  There is normal console debug output available
        that can be enabled with CONFIG_DEBUG + CONFIG_DEBUG_USB.  However,
        USB device operation is very time critical and enabling this debug
        output WILL interfere with the operation of the UDPHS.  USB device
        tracing is a less invasive way to get debug information:  If tracing
        is enabled, the USB device will save encoded trace output in in-memory
        buffer; if the USB monitor is also enabled, that trace buffer will be
        periodically emptied and dumped to the system logging device (the
        serial console in this configuration):

        Device Drivers -> "USB Device Driver Support:
          CONFIG_USBDEV_TRACE=y                   : Enable USB trace feature
          CONFIG_USBDEV_TRACE_NRECORDS=256        : Buffer 256 records in memory
          CONFIG_USBDEV_TRACE_STRINGS=y           : (optional)

        Application Configuration -> NSH LIbrary:
          CONFIG_NSH_USBDEV_TRACE=n               : No builtin tracing from NSH
          CONFIG_NSH_ARCHINIT=y                   : Automatically start the USB monitor

        Application Configuration -> System NSH Add-Ons:
          CONFIG_SYSTEM_USBMONITOR=y              : Enable the USB monitor daemon
          CONFIG_SYSTEM_USBMONITOR_STACKSIZE=2048 : USB monitor daemon stack size
          CONFIG_SYSTEM_USBMONITOR_PRIORITY=50    : USB monitor daemon priority
          CONFIG_SYSTEM_USBMONITOR_INTERVAL=1     : Dump trace data every second
          CONFIG_SYSTEM_USBMONITOR_TRACEINIT=y    : Enable TRACE output
          CONFIG_SYSTEM_USBMONITOR_TRACECLASS=y
          CONFIG_SYSTEM_USBMONITOR_TRACETRANSFERS=y
          CONFIG_SYSTEM_USBMONITOR_TRACECONTROLLER=y
          CONFIG_SYSTEM_USBMONITOR_TRACEINTERRUPTS=y

       NOTE: If USB debug output is also enabled, both outpus will appear
       on the serial console.  However, the debug output will be
       asynchronous with the trace output and, hence, difficult to
       interpret.

    13. AT24 Serial EEPROM. A AT24C512 Serial EEPPROM was used for tested
        I2C.  There are other I2C/TWI devices on-board, but the serial
        EEPROM is the simplest test.

        There is, however, no AT24 EEPROM on board the SAMA5D3x-EK:  The
        serial EEPROM was mounted on an external adaptor board and
        connected to the SAMA5D3x-EK thusly:

        - VCC -- VCC
        - GND -- GND
        - TWCK0(PA31) -- SCL
        - TWD0(PA30)  -- SDA

        By default, PA30 and PA31 are SWJ-DP pins, it can be used as a pin
        for TWI peripheral in the end application.

        The following configuration settings were used:

       System Type -> SAMA5 Peripheral Support
         CONFIG_SAMA5_TWI0=y                   : Enable TWI0

       System Type -> TWI device driver options
         SAMA5_TWI0_FREQUENCY=100000           : Select a TWI frequency

       Device Drivers -> I2C Driver Support
         CONFIG_I2C=y                          : Enable I2C support
         CONFIG_I2C_TRANSFER=y                 : Driver supports the transfer() method
         CONFIG_I2C_WRITEREAD=y                : Driver supports the writeread() method

       Device Drivers -> Memory Technology Device (MTD) Support
         CONFIG_MTD=y                          : Enable MTD support
         CONFIG_MTD_AT24XX=y                   : Enable the AT24 driver
         CONFIG_AT24XX_SIZE=512                : Specifies the AT 24C512 part
         CONFIG_AT24XX_ADDR=0x53               : AT24 I2C address

       Application Configuration -> NSH Library
         CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

       File systems
         CONFIG_NXFFS=y                        : Enables the NXFFS file system
         CONFIG_NXFFS_PREALLOCATED=y           : Required
                                               : Other defaults are probably OK

       Board Selection
         CONFIG_SAMA5_AT24_AUTOMOUNT=y         : Mounts AT24 for NSH
         CONFIG_SAMA5_AT24_NXFFS=y             : Mount the AT24 using NXFFS

       You can then format the AT25 FLASH for a FAT file system and mount
       the file system at /mnt/at24 using these NSH commands:

         nsh> mkfatfs /dev/mtdblock0
         nsh> mount -t vfat /dev/mtdblock0 /mnt/at24

       Then you an use the FLASH as a normal FAT file system:

         nsh> echo "This is a test" >/mnt/at24/atest.txt
         nsh> ls -l /mnt/at24
         /mnt/at24:
          -rw-rw-rw-      16 atest.txt
         nsh> cat /mnt/at24/atest.txt
         This is a test

    13. I2C Tool. NuttX supports an I2C tool at apps/system/i2c that can be
        used to peek and poke I2C devices.  That tool cal be enabled by
        setting the following:

       System Type -> SAMA5 Peripheral Support
         CONFIG_SAMA5_TWI0=y                   : Enable TWI0
         CONFIG_SAMA5_TWI1=y                   : Enable TWI1
         CONFIG_SAMA5_TWI2=y                   : Enable TWI2

       System Type -> TWI device driver options
         SAMA5_TWI0_FREQUENCY=100000           : Select a TWI0 frequency
         SAMA5_TWI1_FREQUENCY=100000           : Select a TWI1 frequency
         SAMA5_TWI2_FREQUENCY=100000           : Select a TWI2 frequency

       Device Drivers -> I2C Driver Support
         CONFIG_I2C=y                          : Enable I2C support
         CONFIG_I2C_TRANSFER=y                 : Driver supports the transfer() method
         CONFIG_I2C_WRITEREAD=y                : Driver supports the writeread() method

       Application Configuration -> NSH Library
         CONFIG_SYSTEM_I2CTOOL=y               : Enable the I2C tool
         CONFIG_I2CTOOL_MINBUS=0               : TWI0 has the minimum bus number 0
         CONFIG_I2CTOOL_MAXBUS=2               : TWI2 has the maximum bus number 2
         CONFIG_I2CTOOL_DEFFREQ=100000         : Pick a consistent frequency

       The I2C tool has extensive help that can be accessed as follows:

       nsh> i2c help
       Usage: i2c <cmd> [arguments]
       Where <cmd> is one of:

         Show help     : ?
         List busses   : bus
         List devices  : dev [OPTIONS] <first> <last>
         Read register : get [OPTIONS] [<repititions>]
         Show help     : help
         Write register: set [OPTIONS] <value> [<repititions>]
         Verify access : verf [OPTIONS] [<value>] [<repititions>]

       Where common "sticky" OPTIONS include:
         [-a addr] is the I2C device address (hex).  Default: 03 Current: 03
         [-b bus] is the I2C bus number (decimal).  Default: 0 Current: 0
         [-r regaddr] is the I2C device register address (hex).  Default: 00 Current: 00
         [-w width] is the data width (8 or 16 decimal).  Default: 8 Current: 8
         [-s|n], send/don't send start between command and data.  Default: -n Current: -n
         [-i|j], Auto increment|don't increment regaddr on repititions.  Default: NO Current: NO
         [-f freq] I2C frequency.  Default: 100000 Current: 100000

       NOTES:
       o Arguments are "sticky".  For example, once the I2C address is
         specified, that address will be re-used until it is changed.

       WARNING:
       o The I2C dev command may have bad side effects on your I2C devices.
         Use only at your own risk.

       As an eample, the I2C dev comman can be used to list all devices
       responding on TWI0 (the default) like this:

         nsh> i2c dev 0x03 0x77
              0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
         00:          -- -- -- -- -- -- -- -- -- -- -- -- --
         10: -- -- -- -- -- -- -- -- -- -- 1a -- -- -- -- --
         20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
         30: -- -- -- -- -- -- -- -- -- 39 -- -- -- 3d -- --
         40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
         50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
         60: 60 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
         70: -- -- -- -- -- -- -- --
         nsh>

        Address 0x1a is the WM8904.  Address 0x39 is the SIL9022A. I am
        not sure what is at address 0x3d and 0x60

    14. Networking support via the EMAC 10/100Base-T peripheral can be
        added to NSH be selecting the following configuration options.
        Remember that only the SAMA5D31 and SAMAD35 support the EMAC
        peripheral!  This will add several new commands to NSH:  ifconfig,
        wget, put, get, ping, etc.

        System Type
          CONFIG_ARCH_CHIP_ATSAMA5D31=y       : SAMA5D31 or SAMAD35 support EMAC
          CONFIG_ARCH_CHIP_ATSAMA5D35=y       : (others do not)

        System Type -> SAMA5 Peripheral Support
          CONFIG_SAMA5_EMAC=y                 : Enable the EMAC peripheral

        System Type -> EMAC device driver options
          CONFIG_SAMA5_EMAC_NRXBUFFERS=16     : Set aside some RS and TX buffers
          CONFIG_SAMA5_EMAC_NTXBUFFERS=4
          CONFIG_SAMA5_EMAC_PHYADDR=1         : KSZ8051 PHY is at address 1
          CONFIG_SAMA5_EMAC_AUTONEG=y         : Use autonegotiation
          CONFIG_SAMA5_EMAC_RMII=y            : Either MII or RMII interface should work
          CONFIG_SAMA5_EMAC_PHYSR=30          : Address of PHY status register on KSZ8051
          CONFIG_SAMA5_EMAC_PHYSR_ALTCONFIG=y : Needed for KSZ8051
          CONFIG_SAMA5_EMAC_PHYSR_ALTMODE=0x7 : "    " " " "     "
          CONFIG_SAMA5_EMAC_PHYSR_10HD=0x1    : "    " " " "     "
          CONFIG_SAMA5_EMAC_PHYSR_100HD=0x2   : "    " " " "     "
          CONFIG_SAMA5_EMAC_PHYSR_10FD=0x5    : "    " " " "     "
          CONFIG_SAMA5_EMAC_PHYSR_100FD=0x6   : "    " " " "     "

        Device drivers -> Network Device/PHY Support
          CONFIG_NETDEVICES=y                 : Enabled PHY selection
          CONFIG_ETH0_PHY_KSZ8051=y           : Select the KSZ8051 PHY

        Networking Support
          CONFIG_NET=y                        : Enable Neworking
          CONFIG_NET_SOCKOPTS=y               : Enable socket operations
          CONFIG_NET_BUFSIZE=562              : Maximum packet size (MTD) 1518 is more standard
          CONFIG_NET_RECEIVE_WINDOW=562       : Should be the same as CONFIG_NET_BUFSIZE
          CONFIG_NET_TCP=y                    : Enable TCP/IP networking
          CONFIG_NET_UDP=y                    : Enable UDP networking
          CONFIG_NET_ICMP=y                   : Enable ICMP networking
          CONFIG_NET_ICMP_PING=y              : Needed for NSH ping command
                                             : Defaults should be okay for other options
        Application Configuration -> Network Utilities
          CONFIG_NETUTILS_RESOLV=y            : Enable host address resolution
          CONFIG_NETUTILS_TFTPC=y             : Enable TFTP data file transfers for get and put commands
          CONFIG_NETUTILS_TELNETD=y           : Enable the Telnet daemon
          CONFIG_NETUTILS_UIPLIB=y            : Network library support is needed
          CONFIG_NETUTILS_WEBCLIENT=y         : Needed for wget support
                                              : Defaults should be okay for other options
        Application Configuration -> NSH Library
          CONFIG_NSH_TELNET=y                 : Enable NSH session via Telnet
          CONFIG_NSH_IPADDR=0x0a000002        : Select an IP address
          CONFIG_NSH_DRIPADDR=0x0a000001      : IP address of gateway/host PC
          CONFIG_NSH_NETMASK=0xffffff00       : Netmask
          CONFIG_NSH_NOMAC=y                  : Need to make up a bogus MAC address

    STATUS:
      PCK FREQUENCY
      2013-7-19:  This configuration (as do the others) run at 396MHz.
        The SAMA5D3 can run at 536MHz.  I still need to figure out the
        PLL settings to get that speed.

        If the CPU speed changes, then so must the NOR and SDRAM
        initialization!

      BOOT FROM NOT FLASH
      2013-7-31:  I have been unable to execute this configuration from NOR
        FLASH by closing the BMS jumper (J9).  As far as I can tell, this
        jumper does nothing on my board???  I have been using the norboot
        configuration to start the program in NOR FLASH (see just above).
        See "Creating and Using NORBOOT" above.

      2013-7-31:  The basic NSH configuration appears to be fully functional.

      CALIBRATION
      2013-7-31:  Using delay loop calibration from the hello configuration.
        That configuration runs out of internal SRAM and, as a result, this
        configuration should be recalibrated.

      SDRAM
      2013-8-3:  SDRAM configuration and RAM test usage have been verified
        and are functional.  I note some issues; occassionally, SDRAM is
        not functional on initial boot or is initially not functional but
        improves with accesses.  Clearly, more work needs to be done.

      AT25 SERIAL FLASH
      2013-8-5:  The AT25 configuration has been verified to be functional.
      2013-8-9:  The AT25 configuration has been verified with DMA
        enabled.

      2013-9-11: Basic HSCMI0/1 functionality (with DMA) has been verified.

      OHCI
      2013-8-16: The OCHI configuration is now basically functional.
        Testing is not yet extensive, however:
        a) I have tested only control and bulk endpoints.  I still need
           to test interrupt endpoints.

      EHCI
      2013-8-26:
        The hand-off of full speed devices to OHCI does not work. In this
        case, OHCI gets the port, but the port is reset, lost by OHCI and
        returned to EHCI.  EHCI sees the full-speed port and hands it off to
        OHCI and this sequence continues forever.
      2013-8-28: EHCI is partially functional.

      UDPHS
      2013-9-5: The UDPHS driver is basically functional.

      I2C
      2013-9-12:  I have been unusuccessful getting the external serial
        AT24 EEPROM to work.  I am pretty sure that this is a problem with
        my external AT24 board (the TWI0 bus hangs when the AT24 is plugged
        in).  I will skip the AT24 integration since it is not on the critical
        path at the moment.
      2013-9-12:  The I2C tool, however, seems to work well.  It succesfully
        enumerates the devices on the bus and successfully exchanges a few
        commands.  The real test of the come later when a real I2C device is
        integrated.

      EMAC:
      2013-9-17:  Driver created, but not fully integrated yet.

  ostest:
    This configuration directory, performs a simple OS test using
    examples/ostest.

    NOTES:
    1. This configuration uses the default USART1 serial console.  That
       is easily changed by reconfiguring to (1) enable a different
       serial peripheral, and (2) selecting that serial peripheral as
       the console device.

    2. By default, this configuration is set up to build on Windows
       under either a Cygwin or MSYS environment using a recent, Windows-
       native, generic ARM EABI GCC toolchain (such as the CodeSourcery
       toolchain).  Both the build environment and the toolchain
       selection can easily be changed by reconfiguring:

       CONFIG_HOST_WINDOWS=y                   : Windows operating system
       CONFIG_WINDOWS_CYGWIN=y                 : POSIX environment under windows
       CONFIG_ARMV7A_TOOLCHAIN_CODESOURCERYW=y : CodeSourcery for Windows

    3. This configuration executes out of CS0 NOR flash and can only
       be loaded via SAM-BA.  These are the relevant configuration options
       the define the NOR FLASH configuration:

       CONFIG_SAMA5_BOOT_CS0FLASH=y            : Boot from FLASH on CS0
       CONFIG_BOOT_RUNFROMFLASH=y              : Run in place on FLASH (vs copying to RAM)

       CONFIG_SAMA5_EBICS0=y                   : Enable CS0 external memory
       CONFIG_SAMA5_EBICS0_SIZE=134217728      : Memory size is 128KB
       CONFIG_SAMA5_EBICS0_NOR=y               : Memory type is NOR FLASH

       CONFIG_FLASH_START=0x10000000           : Physical FLASH start address
       CONFIG_FLASH_VSTART=0x10000000          : Virtual FLASH start address
       CONFIG_FLASH_SIZE=134217728             : FLASH size (again)

       CONFIG_RAM_START=0x00300400             : Data stored after page table
       CONFIG_RAM_VSTART=0x00300400
       CONFIG_RAM_SIZE=114688                  : Available size of 128KB - 16KB for page table

       NOTE:  In order to boot in this configuration, you need to close the
       BMS jumper.

    STATUS:
      2013-7-19:  This configuration (as do the others) run at 396MHz.
        The SAMA5D3 can run at 536MHz.  I still need to figure out the
        PLL settings to get that speed.

        If the CPU speed changes, then so must the NOR and SDRAM
        initialization!

      2013-7-30:  I have been unable to execute this configuration from NOR
        FLASH by closing the BMS jumper (J9).  As far as I can tell, this
        jumper does nothing on my board???  I have been using the norboot
        configuration to start the program in NOR FLASH (see just above).
        See "Creating and Using NORBOOT" above.

      2013-7-31:  The OS test configuration is functional.

      2013-7-31:  Using delay loop calibration from the hello configuration.
        That configuration runs out of internal SRAM and, as a result, this
        configuration needs to be recalibrated.
