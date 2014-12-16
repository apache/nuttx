README.txt
==========

  This README file discuss discusses the port of NuttX to the Texas
  Instruments DK-TM4C129x Connected Development Kit.

  Description
  -----------
  The Tiva™ C Series TM4C129x Connected Development Kit highlights
  the 120-MHz Tiva C Series TM4C129XNCZAD ARM® Cortex™-M4 based
  microcontroller, including an integrated 10/100 Ethernet MAC +
  PHY as well as many other key features.

  Features
  --------

    - Color LCD interface 
    - USB 2.0 OTG | Host | Device port 
    - TI wireless EM connection 
    - BoosterPack and BoosterPack XL interfaces 
    - Quad SSI-supported 512-Mbit Flash memory 
    - MicroSD slot 
    - Expansion interface headers: MCU high-speed USB ULPI port,
      Ethernet RMII and MII ports External peripheral interface for
      memories, parallel peripherals, and other system functions. 
    - In-Circuit Debug Interface (ICDI)

On-Board GPIO Usage
===================

  [To be provided]

Using OpenOCD and GDB with an FT2232 JTAG emulator
==================================================

  Building OpenOCD under Cygwin:

    Refer to configs/olimex-lpc1766stk/README.txt

  Installing OpenOCD in Linux:

      sudo apt-get install openocd

    You can also build openocd from its source:

      git clone http://git.code.sf.net/p/openocd/code openocd
      cd openocd

  Helper Scripts:

    I have been using the on-board In-Circuit Debug Interface (ICDI) interface.
    OpenOCD requires a configuration file.  I keep the one I used last here:

      configs/dk-tm4c129x/tools/dk-tm4c129x.cfg

    However, the "correct" configuration script to use with OpenOCD may
    change as the features of OpenOCD evolve.  So you should at least
    compare that dk-tm4c129x.cfg file with configuration files in
    /usr/share/openocd/scripts.  As of this writing, the configuration
    files of interest were:

      /usr/local/share/openocd/scripts/board/dk-tm4c129x.cfg
      /usr/local/share/openocd/scripts/interface/ti-icdi.cfg
      /usr/local/share/openocd/scripts/target/stellaris_icdi.cfg

    There is also a script on the tools/ directory that I use to start
    the OpenOCD daemon on my system called oocd.sh.  That script will
    probably require some modifications to work in another environment:

    - Possibly the value of OPENOCD_PATH and TARGET_PATH
    - It assumes that the correct script to use is the one at
      configs/dk-tm4c129x/tools/dk-tm4c129x.cfg

  Starting OpenOCD

    If you are in the top-level NuttX build directlory then you should
    be able to start the OpenOCD daemon like:

      oocd.sh $PWD

    The relative path to the oocd.sh script is configs/dk-tm4c129x/tools,
    but that should have been added to your PATH variable when you sourced
    the setenv.sh script.

    Note that OpenOCD needs to be run with administrator privileges in
    some environments (sudo).

  Connecting GDB

    Once the OpenOCD daemon has been started, you can connect to it via
    GDB using the following GDB command:

      arm-nuttx-elf-gdb
      (gdb) target remote localhost:3333

    NOTE:  The name of your GDB program may differ.  For example, with the
    CodeSourcery toolchain, the ARM GDB would be called arm-none-eabi-gdb.

    After starting GDB, you can load the NuttX ELF file:

      (gdb) symbol-file nuttx
      (gdb) monitor reset
      (gdb) monitor halt
      (gdb) load nuttx

    NOTES:

    1. Loading the symbol-file is only useful if you have built NuttX to
       include debug symbols (by setting CONFIG_DEBUG_SYMBOLS=y in the
       .config file).
    2. The MCU must be halted prior to loading code using 'mon reset'
       as described below.

    OpenOCD will support several special 'monitor' commands.  These
    GDB commands will send comments to the OpenOCD monitor.  Here
    are a couple that you will need to use:

     (gdb) monitor reset
     (gdb) monitor halt

    NOTES:

    1. The MCU must be halted using 'mon halt' prior to loading code.
    2. Reset will restart the processor after loading code.
    3. The 'monitor' command can be abbreviated as just 'mon'.

Development Environment
=======================

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment.

GNU Toolchain Options
=====================

  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The NuttX buildroot Toolchain (default, see below),
  2. The CodeSourcery GNU toolchain,
  3. The devkitARM GNU toolchain,
  4. The Atollic toolchain, or
  5. The Code Red toolchain

  All testing has been conducted using the Buildroot toolchain for Cygwin/Linux.
  To use a different toolchain, you simply need to add one of the following
  configuration options to your .config (or defconfig) file:

    CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin (default)
    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y  : CodeSourcery under Windows or Cygwin
    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_ARMV7M_TOOLCHAIN_DEVKITARM=y      : The Atollic toolchain under Windows or Cygwin
    CONFIG_ARMV7M_TOOLCHAIN_CODEREDW=y       : The Code Red toolchain under Windows
    CONFIG_ARMV7M_TOOLCHAIN_CODEREDL=y       : The Code Red toolchain under Linux

    CONFIG_ARMV7M_OABI_TOOLCHAIN=y           : If you use an older, OABI buildroot toolchain

  If you change the default toolchain, then you may also have to modify the PATH in
  the setenv.h file if your make cannot find the tools.

  NOTE: the CodeSourcery (for Windows), Atollic, devkitARM, and Code Red (for Windows)
  toolchains are Windows native toolchains.  The CodeSourcey (for Linux) and NuttX
  buildroot toolchains are Cygwin and/or Linux native toolchains. There are several
  limitations to using a Windows based toolchain in a Cygwin environment.  The three
  biggest are:

  1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
     performed automatically in the Cygwin makefiles using the 'cygpath' utility
     but you might easily find some new path problems.  If so, check out 'cygpath -w'

  2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic links
     are used in Nuttx (e.g., include/arch).  The make system works around these
     problems for the Windows tools by copying directories instead of linking them.
     But this can also cause some confusion for you:  For example, you may edit
     a file in a "linked" directory and find that your changes had no effect.
     That is because you are building the copy of the file in the "fake" symbolic
     directory.  If you use a Windows toolchain, you should get in the habit of
     making like this:

       make clean_context all

     An alias in your .bashrc file might make that less painful.

  3. Dependencies are not made when using Windows versions of the GCC.  This is
     because the dependencies are generated using Windows pathes which do not
     work with the Cygwin make.

       MKDEP                = $(TOPDIR)/tools/mknulldeps.sh

  NOTE 1: The CodeSourcery toolchain (2009q1) did not work with default optimization
  level of -Os (See Make.defs).  It will work with -O0, -O1, or -O2, but not with
  -Os.  I have not seen this problem with current toolchains.

  NOTE 2: The devkitARM toolchain includes a version of MSYS make.  Make sure that
  the paths to Cygwin's /bin and /usr/bin directories appear BEFORE the devkitARM
  path or will get the wrong version of make.

IDEs
====

  NuttX is built using command-line make.  It can be used with an IDE, but some
  effort will be required to create the project.

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
  3) Set up include paths:  You will need include/, arch/arm/src/tiva,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/tiva/tiva_vectors.S.

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
     ./configure.sh dk-tm4c129x/<sub-dir>

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
  details PLUS some special instructions that you will need to follow if you
  are building a Cortex-M3 toolchain for Cygwin under Windows.

  NOTE:  Unfortunately, the 4.6.3 EABI toolchain is not compatible with the
  the NXFLAT tools.  See the top-level TODO file (under "Binary loaders") for
  more information about this problem. If you plan to use NXFLAT, please do not
  use the GCC 4.6.3 EABI toochain; instead use the GCC 4.3.3 OABI toolchain.
  See instructions below.

NuttX OABI "buildroot" Toolchain
================================

  The older, OABI buildroot toolchain is also available.  To use the OABI
  toolchain:

  1. When building the buildroot toolchain, either (1) modify the cortexm3-eabi-defconfig-4.6.3
     configuration to use EABI (using 'make menuconfig'), or (2) use an exising OABI
     configuration such as cortexm3-defconfig-4.3.3

  2. Modify the Make.defs file to use the OABI conventions:

    +CROSSDEV = arm-nuttx-elf-
    +ARCHCPUFLAGS = -mtune=cortex-m3 -march=armv7-m -mfloat-abi=soft
    +NXFLATLDFLAGS2 = $(NXFLATLDFLAGS1) -T$(TOPDIR)/binfmt/libnxflat/gnu-nxflat-gotoff.ld -no-check-sections
    -CROSSDEV = arm-nuttx-eabi-
    -ARCHCPUFLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
    -NXFLATLDFLAGS2 = $(NXFLATLDFLAGS1) -T$(TOPDIR)/binfmt/libnxflat/gnu-nxflat-pcrel.ld -no-check-sections

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
     ./configure.sh dk-tm4c129x/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-defconfig-nxflat .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly builtNXFLAT binaries.

LEDs
====

  [To be provided]

Serial Console
==============

  [To be provided]


DK-TM4129X Configuration Options
================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM4=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP="tiva"

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_TM4C129XNC

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=dk-tm4c129x (for the DK-TM4129X)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_DK_TM4C129X

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_RAM_SIZE=0x00008000 (32Kb)

    CONFIG_RAM_START - The start address of installed DRAM

       CONFIG_RAM_START=0x20000000

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

  There are configurations for disabling support for interrupts GPIO ports.
  GPIOJ must be disabled because it does not exist on the TM4C129x.
  Additional interrupt support can be disabled if desired to reduce memory
  footprint.

    CONFIG_TIVA_DISABLE_GPIOA_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOB_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOC_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOD_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOE_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOF_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOG_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOH_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOJ_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOK_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOL_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOM_IRQS=n
    CONFIG_TIVA_DISABLE_GPION_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOP_IRQS=n
    CONFIG_TIVA_DISABLE_GPIOQ_IRQS=n

  TM4C129x specific device driver settings

    CONFIG_UARTn_SERIAL_CONSOLE - selects the UARTn for the
       console and ttys0 (default is the UART0).
    CONFIG_UARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_UARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_UARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_UARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_UARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_UARTn_2STOP - Two stop bits

    CONFIG_SSI0_DISABLE - Select to disable support for SSI0
    CONFIG_SSI1_DISABLE - Select to disable support for SSI1
    CONFIG_SSI_POLLWAIT - Select to disable interrupt driven SSI support.
      Poll-waiting is recommended if the interrupt rate would be to
      high in the interrupt driven case.
    CONFIG_SSI_TXLIMIT - Write this many words to the Tx FIFO before
      emptying the Rx FIFO.  If the SPI frequency is high and this
      value is large, then larger values of this setting may cause
      Rx FIFO overrun errors.  Default: half of the Tx FIFO size (4).

    CONFIG_TIVA_ETHERNET - This must be set (along with CONFIG_NET)
      to build the Tiva Ethernet driver
    CONFIG_TIVA_ETHLEDS - Enable to use Ethernet LEDs on the board.
    CONFIG_TIVA_BOARDMAC - If the board-specific logic can provide
      a MAC address (via tiva_ethernetmac()), then this should be selected.
    CONFIG_TIVA_ETHHDUPLEX - Set to force half duplex operation
    CONFIG_TIVA_ETHNOAUTOCRC - Set to suppress auto-CRC generation
    CONFIG_TIVA_ETHNOPAD - Set to suppress Tx padding
    CONFIG_TIVA_MULTICAST - Set to enable multicast frames
    CONFIG_TIVA_PROMISCUOUS - Set to enable promiscuous mode
    CONFIG_TIVA_BADCRC - Set to enable bad CRC rejection.
    CONFIG_TIVA_DUMPPACKET - Dump each packet received/sent to the console.

Configurations
==============

Each DK-TM4129X configuration is maintained in a
sub-directory and can be selected as follow:

    cd tools
    ./configure.sh dk-tm4c129x/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    configuration enables the serial VCOM interfaces on UART0.  Support for
    builtin applications is enabled, but in the base configuration no
    builtin applications are selected.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. By default, this configuration uses the CodeSourcery toolchain
       for Windows and builds under Cygwin (or probably MSYS).  That
       can easily be reconfigured, of course.

       CONFIG_HOST_LINUX=y                 : Linux (Cygwin under Windows okay too).
       CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y : Buildroot (arm-nuttx-elf-gcc)
       CONFIG_RAW_BINARY=y                 : Output formats: ELF and raw binary
