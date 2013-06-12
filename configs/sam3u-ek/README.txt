README
^^^^^^

This README discusses issues unique to NuttX configurations for the Atmel
SAM3U-EK development board featuring the ATAM3U

Contents
^^^^^^^^

  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI "buildroot" Toolchain
  - NuttX OABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - LEDs
  - SAM3U-EK-specific Configuration Options
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
     ./configure.sh sam3u-ek/<sub-dir>

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

The SAM3U-EK board has four LEDs labeled LD1, LD2, LD3 and LD4 on the
the board.  Usage of these LEDs is defined in include/board.h and src/up_leds.c.
They are encoded as follows:

    SYMBOL              Meaning                 LED0*   LED1    LED2
    ------------------- ----------------------- ------- ------- -------
    LED_STARTED         NuttX has been started  OFF     OFF     OFF
    LED_HEAPALLOCATE    Heap has been allocated OFF     OFF     ON
    LED_IRQSENABLED     Interrupts enabled      OFF     ON      OFF
    LED_STACKCREATED    Idle stack created      OFF     ON      ON
    LED_INIRQ           In an interrupt**       N/C     FLASH   N/C
    LED_SIGNAL          In a signal handler***  N/C     N/C     FLASH
    LED_ASSERTION       An assertion failed     FLASH   N/C     N/C
    LED_PANIC           The system has crashed  FLASH   N/C     N/C

  * If LED1 and LED2 are statically on, then NuttX probably failed to boot
    and these LEDs will give you some indication of where the failure was
 ** The normal state is LED0=OFF, LED2=ON and LED1 faintly glowing.  This faint
    glow is because of timer interupts that result in the LED being illuminated
    on a small proportion of the time.
*** LED2 may also flicker normally if signals are processed.

SAM3U-EK-specific Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM3=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP="sam34"

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_SAM34
       CONFIG_ARCH_CHIP_SAM3U
       CONFIG_ARCH_CHIP_AT91SAM3U4

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=sam3u-ek (for the SAM3U-EK development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_SAM3UEK=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_DRAM_SIZE=0x0000c000 (48Kb)

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

    CONFIG_SAM34_RTC           - Real Time Clock
    CONFIG_SAM34_RTT           - Real Time Timer
    CONFIG_SAM34_WDT           - Watchdog Timer
    CONFIG_SAM34_UART0         - UART 0
    CONFIG_SAM34_SMC           - Static Memory Controller
    CONFIG_SAM34_USART0        - USART 0
    CONFIG_SAM34_USART1        - USART 1
    CONFIG_SAM34_USART2        - USART 2
    CONFIG_SAM34_USART3        - USART 3
    CONFIG_SAM34_HSMCI         - High Speed Multimedia Card Interface
    CONFIG_SAM34_TWI0          - Two-Wire Interface 0
    CONFIG_SAM34_TWI1          - Two-Wire Interface 1
    CONFIG_SAM34_SPI           - Serial Peripheral Interface
    CONFIG_SAM34_SSC           - Synchronous Serial Controller
    CONFIG_SAM34_TC0           - Timer Counter 0
    CONFIG_SAM34_TC1           - Timer Counter 1
    CONFIG_SAM34_TC2           - Timer Counter 2
    CONFIG_SAM34_PWM           - Pulse Width Modulation Controller
    CONFIG_SAM34_ADC12B        - 12-bit ADC Controller
    CONFIG_SAM34_ADC           - 10-bit ADC Controller
    CONFIG_SAM34_DMA           - DMA Controller
    CONFIG_SAM34_UDPHS         - USB Device High Speed

  Some subsystems can be configured to operate in different ways. The drivers
  need to know how to configure the subsystem.

    CONFIG_GPIOA_IRQ
    CONFIG_GPIOB_IRQ
    CONFIG_GPIOC_IRQ
    CONFIG_USART0_ISUART
    CONFIG_USART1_ISUART
    CONFIG_USART2_ISUART
    CONFIG_USART3_ISUART
    CONFIG_SAM34_NAND          - NAND memory

  AT91SAM3U specific device driver settings

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

  LCD Options.  Other than the standard LCD configuration options
  (see configs/README.txt), the SAM3U-EK driver also supports:

    CONFIG_LCD_PORTRAIT - Present the display in the standard 240x320
       "Portrait" orientation.  Default:  The display is rotated to
       support a 320x240 "Landscape" orientation.

Configurations
^^^^^^^^^^^^^^

  Each SAM3U-EK configuration is maintained in a sub-directory and
  can be selected as follow:

    cd tools
    ./configure.sh sam3u-ek/<subdir>
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
     output of UART0 (J3).

Configuration sub-directories
-----------------------------

  knsh:
    This is identical to the nsh configuration below except that NuttX
    is built as a kernel-mode, monolithic module and the user applications
    are built separately.  It is recommends to use a special make command;
    not just 'make' but make with the following two arguments:

        make pass1 pass2

    In the normal case (just 'make'), make will attempt to build both user-
    and kernel-mode blobs more or less interleaved.  This actual works!
    However, for me it is very confusing so I prefer the above make command:
    Make the user-space binaries first (pass1), then make the the kernel-space
    binaries (pass2)

    NOTES:

    1. This configuration uses the older, OABI, buildroot toolchain.  But
       that is easily reconfigured:

       CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y : Buildroot toolchain
       CONFIG_ARMV7M_OABI_TOOLCHAIN=y      : Older, OABI toolchain

    2. At the end of the build, there will be several files in the top-level
       NuttX build directory:

       PASS1:
         nuttx_user.elf    - The pass1 user-space ELF file
         nuttx_user.hex    - The pass1 Intel HEX format file (selected in defconfig)
         User.map          - Symbols in the user-space ELF file

       PASS2:
         nuttx             - The pass2 kernel-space ELF file
         nuttx.hex         - The pass2 Intel HEX file (selected in defconfig)
         System.map        - Symbols in the kernel-space ELF file

       The J-Link programmer will except files in .hex, .mot, .srec, and .bin
       formats.

    3. Combining .hex files.  If you plan to use the .hex files with your
       debugger or FLASH utility, then you may need to combine the two hex
       files into a single .hex file.  Here is how you can do that.

       a. The 'tail' of the nuttx.hex file should look something like this
          (with my comments added):

            $ tail nuttx.hex
            # 00, data records
            ...
            :10 9DC0 00 01000000000800006400020100001F0004
            :10 9DD0 00 3B005A0078009700B500D400F300110151
            :08 9DE0 00 30014E016D0100008D
            # 05, Start Linear Address Record
            :04 0000 05 0800 0419 D2
            # 01, End Of File record
            :00 0000 01 FF

          Use an editor such as vi to remove the 05 and 01 records.

       b. The 'head' of the nuttx_user.hex file should look something like
          this (again with my comments added):

            $ head nuttx_user.hex
            # 04, Extended Linear Address Record
            :02 0000 04 0801 F1
            # 00, data records
            :10 8000 00 BD89 01084C800108C8110208D01102087E
            :10 8010 00 0010 00201C1000201C1000203C16002026
            :10 8020 00 4D80 01085D80010869800108ED83010829
            ...

          Nothing needs to be done here.  The nuttx_user.hex file should
          be fine.

       c. Combine the edited nuttx.hex and un-edited nuttx_user.hex
          file to produce a single combined hex file:

          $ cat nuttx.hex nuttx_user.hex >combined.hex

       Then use the combined.hex file with the to write the FLASH image.
       If you do this a lot, you will probably want to invest a little time
       to develop a tool to automate these steps.

  nsh:
    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables both the serial and telnetd NSH interfaces.

    NOTES:

    1. This configuration uses the older, OABI, buildroot toolchain.  But
       that is easily reconfigured:

       System Type:
         CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y : Buildroot toolchain
         CONFIG_ARMV7M_OABI_TOOLCHAIN=y      : Older, OABI toolchain

    2. NSH built-in applications are supported.  However, there are
       no built-in applications built with the default configuration.

       Binary Formats:
         CONFIG_BUILTIN=y                    : Enable support for built-in programs

       Applicaton Configuration:
         CONFIG_NSH_BUILTIN_APPS=y           : Enable starting apps from NSH command line

    3. This configuration has been used for verifying the touchscreen on
       on the SAM3U-EK LCD.  With these modifications, you can include the
       touchscreen test program at apps/examples/touchscreen as an NSH built-in
       application.  You can enable the touchscreen and test by modifying the
       default configuration in the following ways:

          Drivers:
            CONFIG_INPUT=y                    : Enable support for input devices
            CONFIG_INPUT_ADS7843E=y           : Enable support for the XPT2048
            CONFIG_ADS7843E_SPIDEV=0          : Use SPI for communication
            CONFIG_ADS7843E_SPIMODE=0         : Use SPI mode 0
            CONFIG_ADS7843E_THRESHX=39        : These will probably need to be tuned
            CONFIG_ADS7843E_THRESHY=51
            CONFIG_SPI=y                      : Enable SPI support
            CONFIG_SPI_EXCHANGE=n             : exchange() method is not supported

          System Type:
            CONFIG_GPIO_IRQ=y                 : GPIO interrupt support
            CONFIG_GPIOA_IRQ=y                : Enable GPIO interrupts from port A
            CONFIG_SAM34_SPI=y                : Enable support for SPI

          RTOS Features:
            CONFIG_DISABLE_SIGNALS=n          : Signals are required

          Library Support:
            CONFIG_SCHED_WORKQUEUE=y          : Work queue support required

          Applicaton Configuration:
            CONFIG_EXAMPLES_TOUCHSCREEN=y     : Enable the touchscreen built-int test

          Defaults should be okay for related touchscreen settings.  Touchscreen
          debug output on UART0 can be enabled with:

          Build Setup:
            CONFIG_DEBUG=y                    : Enable debug features
            CONFIG_DEBUG_VERBOSE=y            : Enable verbose debug output
            CONFIG_DEBUG_INPUT=y              : Enable debug output from input devices

          NOTE:
            As of this writing, the touchscreen is not functional (no
            interrupts).  More work is needed.

  nx:
    Configures to use examples/nx using the HX834x LCD hardware on
    the SAM3U-EK development board.

  ostest:
    This configuration directory, performs a simple OS test using
    examples/ostest.  By default, this project assumes that you are
    using the DFU bootloader.
