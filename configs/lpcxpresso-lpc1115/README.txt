README
^^^^^^

README for NuttX port to the NXP the LPCXpresso board.

Contents
^^^^^^^^

  LCPXpresso LPC1115 Board
  Development Environment
  GNU Toolchain Options
  NuttX EABI "buildroot" Toolchain
  NuttX OABI "buildroot" Toolchain
  NXFLAT Toolchain
  Code Red IDE
  LEDs
  LPCXpresso Configuration Options
  Configurations

LCPXpresso LPC1115 Board
^^^^^^^^^^^^^^^^^^^^^^^^

  Pin Description                  Connector
  -------------------------------- ---------

  P0[0]/RD1/TXD3/SDA1               J6-9
  P0[1]/TD1/RXD3/SCL                J6-10
  P0[2]/TXD0/AD0[7]                 J6-21
  P0[3]/RXD0/AD0[6]                 J6-22
  P0[4]/I2SRX-CLK/RD2/CAP2.0        J6-38
  P0[5]/I2SRX-WS/TD2/CAP2.1         J6-39
  P0[6]/I2SRX_SDA/SSEL1/MAT2[0]     J6-8
  P0[7]/I2STX_CLK/SCK1/MAT2[1]      J6-7
  P0[8]/I2STX_WS/MISO1/MAT2[2]      J6-6
  P0[9]/I2STX_SDA/MOSI1/MAT2[3]     J6-5
  P0[10]                            J6-40
  P0[11]                            J6-41

  P1[0]/ENET-TXD0                   J6-34?
  P1[1]/ENET_TXD1                   J6-35?
  P1[4]/ENET_TX_EN
  P1[8]/ENET_CRS
  P1[9]/ENET_RXD0
  P1[10]/ENET_RXD1

  P2[0]/PWM1.1/TXD1
  P2[1]/PWM1.2/RXD1                 J6-43
  P2[2]/PWM1.3/CTS1/TRACEDATA[3]    J6-44
  P2[3]/PWM1.4/DCD1/TRACEDATA[2]    J6-45
  P2[4]/PWM1.5/DSR1/TRACEDATA[1]    J6-46
  P2[5]/PWM1[6]/DTR1/TRACEDATA[0]   J6-47
  P2[6]/PCAP1[0]/RI1/TRACECLK       J6-48
  P2[7]/RD2/RTS1                    J6-49
  P2[8]/TD2/TXD2                    J6-50
  P2[9]/USB_CONNECT/RXD2            PAD19
  P2[10]/EINT0/NMI                  J6-51

  P3[25]/MAT0.0/PWM1.2              PAD13
  P3[26]/STCLK/MAT0.1/PWM1.3        PAD14

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

  1. The Code Red GNU toolchain
  2. The CodeSourcery GNU toolchain,
  3. The devkitARM GNU toolchain,
  4. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the Code Red toolchain and the
  make system is setup to default to use the Code Red Linux toolchain.  To use
  the other toolchain, you simply need add one of the following configuration
  options to your .config (or defconfig) file:

    CONFIG_ARMV6M_TOOLCHAIN_CODESOURCERYW=y   : CodeSourcery under Windows
    CONFIG_ARMV6M_TOOLCHAIN_CODESOURCERYL=y   : CodeSourcery under Linux
    CONFIG_ARMV6M_TOOLCHAIN_DEVKITARM=y       : devkitARM under Windows
    CONFIG_ARMV6M_TOOLCHAIN_BUILDROOT=y       : NuttX buildroot under Linux or Cygwin (default)
    CONFIG_ARMV6M_TOOLCHAIN_CODEREDW=n        : Code Red toolchain under Windows
    CONFIG_ARMV6M_TOOLCHAIN_CODEREDL=y        : Code Red toolchain under Linux

  You may also have to modify the PATH in the setenv.h file if your make cannot
  find the tools.

  NOTE: the CodeSourcery (for Windows), devkitARM, and Code Red (for Windoes)
  are Windows native toolchains.  The CodeSourcey (for Linux), Code Red (for Linux)
  and NuttX buildroot toolchains are Cygwin and/or Linux native toolchains. There
  are several limitations to using a Windows based toolchain in a Cygwin
  environment.  The three biggest are:

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

Code Red IDE
^^^^^^^^^^^^

  NuttX is built using command-line make.  It can be used with an IDE, but some
  effort will be required to create the project.

  Makefile Build
  --------------
  Under Linux Eclipse, it is pretty easy to set up an "empty makefile project" and
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
  3) Set up include pathes:  You will need include/, arch/arm/src/lpc11xx,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/lpc11x/lpc11_vectors.S.

  Using Code Red GNU Tools from Cygwin
  ------------------------------------

  Under Cygwin, the Code Red command line tools (e.g., arm-non-eabi-gcc) cannot
  be executed because the they only have execut privileges for Administrators.  I
  worked around this by:

  Opening a native Cygwin RXVT as Administrator (Right click, "Run as administrator"),
  then executing 'chmod 755 *.exe' in the following directories:

  /cygdrive/c/nxp/lpcxpreeso_3.6/bin, and
  /cygdrive/c/nxp/lpcxpreeso_3.6/Tools/bin

  Command Line Flash Programming
  ------------------------------

  During the port development was used a STLink-v2 SWD programmer with OpenOCD to
  write the firmware in the flash and GDB to debug NuttX initialization.

  If using LPCLink as your debug connection, first of all boot the LPC-Link using
  the script:

    bin\Scripts\bootLPCXpresso type

  where type = winusb for Windows XP, or type = hid for Windows Vista / 7.

  Now run the flash programming utility with the following options

    flash_utility wire -ptarget -flash-load[-exec]=filename [-load-base=base_address]

  Where flash_utility is one of:

    crt_emu_lpc11_13 (for LPC11xx or LPC13xx parts)
    crt_emu_cm3_nxp (for LPC11xx parts)
    crt_emu_a7_nxp (for LPC21/22/23/24 parts)
    crt_emu_a9_nxp (for LPC31/32 and LPC29xx parts)
    crt_emu_cm3_lmi (for TI Stellaris parts)

  wire is one of:

    (empty) (for Red Probe+, Red Probe, RDB1768v1, or TI Stellaris evaluation boards)
    -wire=hid (for RDB1768v2 without upgraded firmware)
    -wire=winusb (for RDB1768v2 with upgraded firmware)
    -wire=winusb (for LPC-Link on Windows XP)
    -wire=hid (for LPC-Link on Windows Vista/ Windows 7)

  target is the target chip name. For example LPC1343, LPC1114/301, LPC1115 etc.

  filename is the file to flash program. It may be an executable (axf) or a binary
  (bin) file. If using a binary file, the base_address must be specified.

  base_address is the base load address when flash programming a binary file. It
  should be specified as a hex value with a leading 0x.

  Note:
  - flash-load will leave the processor in a stopped state
  - flash-load-exec will start execution of application as soon as download has
    completed.

  Examples
    To load the executable file app.axf and start it executing on an LPC1158
    target using Red Probe, use the following command line:

      crt_emu_cm3_nxp -pLPC1158 -flash-load-exec=app.axf

    To load the binary file binary.bin to address 0x1000 to an LPC1343 target
    using LPC-Link on Windows XP, use the following command line:

      crt_emu_lpc11_13_nxp -wire=hid -pLPC1343 -flash-load=binary.bin -load-base=0x1000

  tools/flash.sh
  --------------

  All of the above steps are automated in the bash script flash.sh that can
  be found in the configs/lpcxpresso/tools directory.

NuttX EABI "buildroot" Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M0 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh lpcxpresso-lpc1115/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm0-eabi-defconfig-4.6.3 .config

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
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  The older, OABI buildroot toolchain is also available.  To use the OABI
  toolchain:

  1. When building the buildroot toolchain, either (1) modify the cortexm3-eabi-defconfig-4.6.3
     configuration to use EABI (using 'make menuconfig'), or (2) use an exising OABI
     configuration such as cortexm3-defconfig-4.3.3

  2. Modify the Make.defs file to use the OABI conventions:

    +CROSSDEV = arm-nuttx-elf-
    +ARCHCPUFLAGS = -mtune=cortex-m3 -march=armv6-m -mfloat-abi=soft
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
     ./configure.sh lpcxpresso-lpc1115/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm0-defconfig-nxflat .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly builtNXFLAT binaries.

LEDs
^^^^

  If CONFIG_ARCH_LEDS is defined, then support for the LPCXpresso LEDs will be
  included in the build.  See:

  - configs/lpcxpresso-lpc1115/include/board.h - Defines LED constants, types and
    prototypes the LED interface functions.

  - configs/lpcxpresso-lpc1115/src/lpcxpresso_internal.h - GPIO settings for the LEDs.

  - configs/lpcxpresso-lpc1115/src/up_leds.c - LED control logic.

  The LPCXpresso LPC1115 has a single LEDs.  Usage this single LED by NuttX
  is as follows:

  - The LED is not illuminated until the LPCXpresso completes initialization.

    If the LED is stuck in the OFF state, this means that the LPCXpresso did not
    complete initializeation.

  - Each time the OS enters an interrupt (or a signal) it will turn the LED OFF and
    restores its previous stated upon return from the interrupt (or signal).

    The normal state, after initialization will be a dull glow.  The brightness of
    the glow will be inversely related to the proportion of time spent within interrupt
    handling logic.  The glow may decrease in brightness when the system is very
    busy handling device interrupts and increase in brightness as the system becomes
    idle.

    Stuck in the OFF state suggests that that the system never completed
    initialization;  Stuck in the ON state would indicated that the system
    intialialized, but is not takint interrupts.

  - If a fatal assertion or a fatal unhandled exception occurs, the LED will flash
    strongly as a slow, 2Hz rate.

LPCXpresso Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  General Architecture Settings:

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
     be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM0=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=lpc11xx

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_LPC1115=y

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=lpcxpresso-lpc1115

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_LPCEXPRESSO=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (CPU SRAM in this case):

       CONFIG_RAM_SIZE=(8*1024) (8Kb)

       There is an additional 32Kb of SRAM in AHB SRAM banks 0 and 1.

    CONFIG_RAM_START - The start address of installed DRAM

       CONFIG_RAM_START=0x10000000

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
      CONFIG_LPC11_MAINOSC=y
      CONFIG_LPC11_PLL0=y
      CONFIG_LPC11_UART0=y
      CONFIG_LPC11_CAN1=n
      CONFIG_LPC11_SPI=n
      CONFIG_LPC11_SSP0=n
      CONFIG_LPC11_SSP1=n
      CONFIG_LPC11_I2C0=n
      CONFIG_LPC11_I2S=n
      CONFIG_LPC11_TMR0=n
      CONFIG_LPC11_TMR1=n
      CONFIG_LPC11_PWM0=n
      CONFIG_LPC11_ADC=n
      CONFIG_LPC11_FLASH=n

  LPC11xx specific device driver settings

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

  LPC11xx specific CAN device driver settings.  These settings all
  require CONFIG_CAN:

    CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default
      Standard 11-bit IDs.
    CONFIG_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_LPC11_CAN1 is defined.
    CONFIG_CAN1_DIVISOR - CAN1 is clocked at CCLK divided by this number.
      (the CCLK frequency is divided by this number to get the CAN clock).
      Options = {1,2,4,6}. Default: 4.
    CONFIG_CAN_TSEG1 - The number of CAN time quanta in segment 1. Default: 6

Configurations
^^^^^^^^^^^^^^

Each LPCXpresso configuration is maintained in a sub-directory and can be
selected as follow:

    cd tools
    ./configure.sh lpcxpresso-lpc1115/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  dhcpd:
    This builds the DCHP server using the apps/examples/dhcpd application
    (for execution from FLASH.) See apps/examples/README.txt for information
    about the dhcpd example.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configurations using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Jumpers: Nothing special.  Use the default base board jumper
       settings.

  nsh:
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables both the serial and telnet NSH interfaces.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configurations using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. This configuration has been used for testing the microSD card.
       This support is, however, disabled in the base configuration.

       At last attempt, the SPI-based mircroSD does not work at
       higher fequencies.  Setting the SPI frequency to 400000
       removes the problem.   There must be some more optimal
       value that could be determined with additional experimetnation.

       Jumpers: J55 must be set to provide chip select PIO1_11 signal as
       the SD slot chip select.

  nx:
    And example using the NuttX graphics system (NX).  This example
    uses the UG-9664HSWAG01 driver.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configurations using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

