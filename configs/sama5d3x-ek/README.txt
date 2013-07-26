README
=====

  This README file describes the port of NuttX to the SAMA5D3x-EK
  development boards. These boards feature the Atmel SAMA5D3
  microprocessors.  Three different SAMA5D3x-EK kits are available

    - SAMA5D31-EK with the ATSAMA5D1 (http://www.atmel.com/devices/sama5d31.aspx)
    - SAMA5D33-EK with the ATSAMA5D3 (http://www.atmel.com/devices/sama5d31.aspx)
    - SAMA5D34-EK with the ATSAMA5D4 (http://www.atmel.com/devices/sama5d31.aspx)
    - SAMA5D35-EK with the ATSAMA5D5 (http://www.atmel.com/devices/sama5d31.aspx)

  The each consist of an identical base board with different plug-in
  modules for each CPU.  An option 7 inch LCD is also available..

  The SAMA5D3FAE-EK bundle includes everything:  The base board, all four
  CPU modules, and the LCD.

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

  - Configurations

Contents
========

  - PIO Muliplexing
  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI "buildroot" Toolchain
  - NuttX OABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - Loading Code
  - Buttons and LEDs
  - Serial Consoles
  - SAMA5D3x-EK Configuration Options
  - Configurations

PIO Muliplexing
===============

  To be provided

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

Loading Code
============

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
  by software.  A  blue LED is controlled via GPIO pins.  A red LED normally
  provides an indication that power is supplied to the board but can also
  be controlled via software.

    PE25.  This blue LED is pulled high and is illuminated by pulling PE25
    low.

    PE24.  The red LED is also pulled high but is driven by a transistor so
    that it is illuminated when power is applied even if PE24 is not
    configured as an output.  If PE24 is configured as an output, then the
    LCD is illuminated by a low output.

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
    SAMA5 FUNCTION  NUTTX GPIO
    PIO   NAME      CONFIGURATION
    ---- ---------- ---------------
    PB27 RTS1       GPIO_USART1_RTS
    PB29 TXD1       GPIO_USART1_TXD
    PB28 RXD1       GPIO_USART1_RXD
    PB26 CTS1       GPIO_USART1_CTS

    NOTE: Debug TX and RX pins also go the the ADM3312EARU, but I am
    uncertain of the functionality.

    -------------------------------
    SAMA5 FUNCTION  NUTTX GPIO
    PIO   NAME      CONFIGURATION
    ---- ---------- ---------------
    PB31 DTXD       GPIO_DBGU_DTXD
    PB30 DRXD       GPIO_DBGU_DRXD

  Hardware UART via CDC
  ---------------------
  "J-Link-OB-ATSAM3U4C comes with an additional hardware UART that is
   accessible from a host via CDC which allows terminal communication with
   the target device. This feature is enabled only if a certain port (CDC
   disabled, PA25, pin 24 on J-Link-OB-ATSAM3U4C) is NOT connected to ground
   (open).

    - Jumper JP16 not fitted: CDC is enabled
    - Jumper JP16 fitted : CDC is disabled"

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

    CONFIG_PIOA_IRQ          - Support PIOA interrupts
    CONFIG_PIOB_IRQ          - Support PIOB interrupts
    CONFIG_PIOC_IRQ          - Support PIOD interrupts
    CONFIG_PIOD_IRQ          - Support PIOD interrupts
    CONFIG_PIOE_IRQ          - Support PIOE interrupts

    CONFIG_USART0_ISUART     - USART0 is configured as a UART
    CONFIG_USART1_ISUART     - USART1 is configured as a UART
    CONFIG_USART2_ISUART     - USART2 is configured as a UART
    CONFIG_USART3_ISUART     - USART3 is configured as a UART

  ST91SAM4S specific device driver settings

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
  edits as necessary so that BUILDROOT_BIN is the correct path to the directory
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

    3. This configuration executes out of internal SRAM an can only
       be loaded via JTAG.

       CONFIG_SAMA5_BOOT_ISRAM=y               : Boot into internal SRAM
       CONFIG_BOOT_RUNFROMISRAM=y              : Run from internal SRAM

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

    3. This configuration executes out of internal SRAM an can only
       be loaded via JTAG.

       CONFIG_SAMA5_BOOT_ISRAM=y               : Boot into internal SRAM
       CONFIG_BOOT_RUNFROMISRAM=y              : Run from internal SRAM

    STATUS:
      2013-7-26:  This ostest configuration is too large to fit in SAMA5
        internal SRAM (along with a usable heap and a 16KB page table).
        I do not want to simplify this test because I will, eventually,
        need to do the entire OS test.  But I will need to configure this
        to run out of FLASH or SDRAM in this future.  For now, I created
        the smaller 'hello' configuration for the basic bringup.
