README
^^^^^^

This README discusses issues unique to NuttX configurations for the
Atmel SAM4L Xplained Pro development board.  This board features the
ATSAM4LC4C MCU

Contents
^^^^^^^^

  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI "buildroot" Toolchain
  - NuttX OABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - LEDs
  - Serial Consoles
  - SAM4L Xplained Pro-specific Configuration Options
  - Configurations

Development Environment
^^^^^^^^^^^^^^^^^^^^^^^

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment.

GNU Toolchain Options
^^^^^^^^^^^^^^^^^^^^^

  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The CodeSourcery GNU toolchain,
  2. The devkitARM GNU toolchain, ok
  4. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the NuttX buildroot toolchain.  However,
  the make system is setup to default to use the devkitARM toolchain.  To use
  the CodeSourcery, devkitARM or Raisonance GNU toolchain, you simply need to
  add one of the following configuration options to your .config (or defconfig)
  file:

    CONFIG_SAM34_CODESOURCERYW=y  : CodeSourcery under Windows
    CONFIG_SAM34_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_SAM34_DEVKITARM=y      : devkitARM under Windows
    CONFIG_SAM34_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin (default)

  If you are not using CONFIG_SAM34_BUILDROOT, then you may also have to modify
  the PATH in the setenv.h file if your make cannot find the tools.

  NOTE: the CodeSourcery (for Windows), devkitARM, and Raisonance toolchains are
  Windows native toolchains.  The CodeSourcey (for Linux) and NuttX buildroot
  toolchains are Cygwin and/or Linux native toolchains. There are several limitations
  to using a Windows based toolchain in a Cygwin environment.  The three biggest are:

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

  NOTE 1: The CodeSourcery toolchain (2009q1) does not work with default optimization
  level of -Os (See Make.defs).  It will work with -O0, -O1, or -O2, but not with
  -Os.

  NOTE 2: The devkitARM toolchain includes a version of MSYS make.  Make sure that
  the paths to Cygwin's /bin and /usr/bin directories appear BEFORE the devkitARM
  path or will get the wrong version of make.

IDEs
^^^^

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
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.shsam4l-xplained/<sub-dir>

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
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^

  If you are *not* using the NuttX buildroot toolchain and you want to use
  the NXFLAT tools, then you will still have to build a portion of the buildroot
  tools -- just the NXFLAT tools.  The buildroot with the NXFLAT tools can
  be downloaded from the NuttX SourceForge download site
  (https://sourceforge.net/projects/nuttx/files/).
 
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh lpcxpresso-lpc1768/<sub-dir>

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
^^^^
  There are three LEDs on board the SAM4L Xplained Pro board:  The EDBG
  controls two of the LEDs, a power LED and a status LED.  There is only
  one user controllable LED, a yellow LED labeled LED0 near the SAM4L USB
  connector.

  This LED is controlled by PC07 and LED0 can be activated by driving the
  PC07 to GND.

  When CONFIG_ARCH_LEDS is defined in the NuttX configuration, NuttX will
  control LED0 as follows:

    SYMBOL              Meaning                 LED0
    ------------------- ----------------------- ------
    LED_STARTED         NuttX has been started  OFF
    LED_HEAPALLOCATE    Heap has been allocated OFF
    LED_IRQSENABLED     Interrupts enabled      OFF
    LED_STACKCREATED    Idle stack created      ON
    LED_INIRQ           In an interrupt         N/C
    LED_SIGNAL          In a signal handler     N/C
    LED_ASSERTION       An assertion failed     N/C
    LED_PANIC           The system has crashed  FLASH

  Thus is LED0 is statically on, NuttX has successfully  booted and is,
  apparently, running normmally.  If LED0 is flashing at approximately
  2Hz, then a fatal error has been detected and the system has halted.

Serial Consoles
^^^^^^^^^^^^^^^

  USART0
  ------

  USART is available on connectors EXT1 and EXT4

    EXT1  TXT4  GPIO  Function
    ----  ---- ------ -----------
     13    13   PB00  USART0_RXD
     14    14   PB01  USART0_TXD
     19    19         GND
     20    20         VCC

  If you have a TTL to RS-232 convertor then this is the most convenient
  serial console to use.  It is the default in all of these configurations.
  An option is to use the virtual COM port.

  Virtual COM Port
  ----------------

  The SAM4L Xplained Pro contains an Embedded Debugger (EDBG) that can be
  used to program and debug the ATSAM4LC4C using Serial Wire Debug (SWD).
  The Embedded debugger also include a Virtual Com port interface over
  USART1.  Virtual COM port connections:

    PC26 USART1 RXD
    PC27 USART1 TXD

SAM4L Xplained Pro-specific Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM4=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP="sam34"

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_SAM34
       CONFIG_ARCH_CHIP_SAM4L
       CONFIG_ARCH_CHIP_ATSAM4LC4C

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=sam4l-xplained (for the SAM4L Xplained Pro development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_SAM4L_XPLAINED=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_DRAM_SIZE=0x00008000 (32Kb)

    CONFIG_DRAM_START - The start address of installed DRAM

       CONFIG_DRAM_START=0x20000000

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

    CPU
    ---
    CONFIG_SAM34_OCD

    HSB
    ---
    CONFIG_SAM34_APBA
    CONFIG_SAM34_AESA

    PBA
    ---
    CONFIG_SAM34_IISC
    CONFIG_SAM34_SPI
    CONFIG_SAM34_TC0
    CONFIG_SAM34_TC1
    CONFIG_SAM34_TWIM0
    CONFIG_SAM34_TWIS0
    CONFIG_SAM34_TWIM1
    CONFIG_SAM34_TWIS1
    CONFIG_SAM34_USART0
    CONFIG_SAM34_USART1
    CONFIG_SAM34_USART2
    CONFIG_SAM34_USART3
    CONFIG_SAM34_ADC12B
    CONFIG_SAM34_DACC
    CONFIG_SAM34_ACC
    CONFIG_SAM34_GLOC
    CONFIG_SAM34_ABDACB
    CONFIG_SAM34_TRNG
    CONFIG_SAM34_PARC
    CONFIG_SAM34_CATB
    CONFIG_SAM34_TWIM2
    CONFIG_SAM34_TWIM3
    CONFIG_SAM34_LCDCA

    PBB
    ---
    CONFIG_SAM34_HRAMC1
    CONFIG_SAM34_HMATRIX
    CONFIG_SAM34_PDCA
    CONFIG_SAM34_CRCCU
    CONFIG_SAM34_USBC
    CONFIG_SAM34_PEVC

    PBC
    ---
    CONFIG_SAM34_CHIPID
    CONFIG_SAM34_FREQM

    PBD
    ---
    CONFIG_SAM34_AST
    CONFIG_SAM34_WDT
    CONFIG_SAM34_EIC
    CONFIG_SAM34_PICOUART

  Some subsystems can be configured to operate in different ways. The drivers
  need to know how to configure the subsystem.

    CONFIG_GPIOA_IRQ
    CONFIG_GPIOB_IRQ
    CONFIG_GPIOC_IRQ
    CONFIG_USART0_ISUART
    CONFIG_USART1_ISUART
    CONFIG_USART2_ISUART
    CONFIG_USART3_ISUART

  ST91SAM4L specific device driver settings

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
^^^^^^^^^^^^^^

  Each SAM4L Xplained Pro configuration is maintained in a sub-directory and
  can be selected as follow:

    cd tools
    ./configure.shsam4l-xplained/<subdir>
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

  NOTE:  These configurations use the mconf-based configuration tool.  To
  change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       and misc/tools/

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

Configuration sub-directories
-----------------------------

  ostest:
    This configuration directory performs a simple OS test using
    examples/ostest.

    NOTES:

    1. This configuration provides test output on USART0 which is available
       on EXT1 or EXT4 (see the section "Serial Consoles" above).  The
       virtual COM port could be used, instead, by reconfiguring to use
       USART1 instead of USART0:

       System Type -> AT91SAM3/4 Peripheral Support
         CONFIG_SAM_USART0=y
         CONFIG_SAM_USART1=n

       Device Drivers -> Serial Driver Support -> Serial Console
         CONFIG_USART0_SERIAL_CONSOLE=y

       Device Drivers -> Serial Driver Support -> USART0 Configuration
         CONFIG_USART0_2STOP=0
         CONFIG_USART0_BAUD=115200
         CONFIG_USART0_BITS=8
         CONFIG_USART0_PARITY=0
         CONFIG_USART0_RXBUFSIZE=256
         CONFIG_USART0_TXBUFSIZE=256

    2. This configuration is set up to use the NuttX OABI toolchain (see
       above). Of course this can be reconfigured if you prefer a different
       toolchain.

  nsh:
    This configuration directory will built the NuttShell.

    NOTES:

    1. This configuration provides test output on USART0 which is available
       on EXT1 or EXT4 (see the section "Serial Consoles" above).  The
       virtual COM port could be used, instead, by reconfiguring to use
       USART1 instead of USART0:

       System Type -> AT91SAM3/4 Peripheral Support
         CONFIG_SAM_USART0=y
         CONFIG_SAM_USART1=n

       Device Drivers -> Serial Driver Support -> Serial Console
         CONFIG_USART0_SERIAL_CONSOLE=y

       Device Drivers -> Serial Driver Support -> USART0 Configuration
         CONFIG_USART0_2STOP=0
         CONFIG_USART0_BAUD=115200
         CONFIG_USART0_BITS=8
         CONFIG_USART0_PARITY=0
         CONFIG_USART0_RXBUFSIZE=256
         CONFIG_USART0_TXBUFSIZE=256

    2. This configuration is set up to use the NuttX OABI toolchain (see
       above). Of course this can be reconfigured if you prefer a different
       toolchain.
