README
======

This README discusses issues unique to NuttX configurations for the
STM32 Tiny development board.

This board is available from several vendors on the net, and may
be sold under different names. It is based on a STM32 F103C8T6 MCU, and
is (always ?) bundled with a nRF24L01 wireless communication module.

Contents
========

  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI "buildroot" Toolchain
  - NuttX OABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - LEDs
  - PWM
  - UARTs
  - Timer Inputs/Outputs
  - STM32 Tiny -specific Configuration Options
  - Configurations

Development Environment
=======================

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems.

GNU Toolchain Options
=====================

  Toolchain Configurations
  ------------------------
  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The CodeSourcery GNU toolchain,
  2. The Atollic Toolchain,
  3. The devkitARM GNU toolchain,
  4. Raisonance GNU toolchain, or
  5. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the CodeSourcery toolchain for Windows.  To use
  the Atollic, devkitARM, Raisonance GNU, or NuttX buildroot toolchain, you simply need to
  add one of the following configuration options to your .config (or defconfig)
  file:

    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y  : CodeSourcery under Windows
    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_ARMV7M_TOOLCHAIN_ATOLLIC=y        : The Atollic toolchain under Windows
    CONFIG_ARMV7M_TOOLCHAIN_DEVKITARM=y      : devkitARM under Windows
    CONFIG_ARMV7M_TOOLCHAIN_RAISONANCE=y     : Raisonance RIDE7 under Windows
    CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin (default)

  If you change the default toolchain, then you may also have to modify the PATH in
  the setenv.h file if your make cannot find the tools.

  NOTE: the CodeSourcery (for Windows), Atollic, devkitARM, and Raisonance toolchains are
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

  The CodeSourcery Toolchain (2009q1)
  -----------------------------------
  The CodeSourcery toolchain (2009q1) does not work with default optimization
  level of -Os (See Make.defs).  It will work with -O0, -O1, or -O2, but not with
  -Os.

  The Atollic "Pro" and "Lite" Toolchain
  --------------------------------------
  One problem that I had with the Atollic toolchains is that the provide a gcc.exe
  and g++.exe in the same bin/ file as their ARM binaries.  If the Atollic bin/ path
  appears in your PATH variable before /usr/bin, then you will get the wrong gcc
  when you try to build host executables.  This will cause to strange, uninterpretable
  errors build some host binaries in tools/ when you first make.

  Also, the Atollic toolchains are the only toolchains that have built-in support for
  the FPU in these configurations.  If you plan to use the Cortex-M4 FPU, you will
  need to use the Atollic toolchain for now.  See the FPU section below for more
  information.

  The Atollic "Lite" Toolchain
  ----------------------------
  The free, "Lite" version of the Atollic toolchain does not support C++ nor
  does it support ar, nm, objdump, or objdcopy. If you use the Atollic "Lite"
  toolchain, you will have to set:

    CONFIG_HAVE_CXX=n

  In order to compile successfully.  Otherwise, you will get errors like:

    "C++ Compiler only available in TrueSTUDIO Professional"

  The make may then fail in some of the post link processing because of some of
  the other missing tools.  The Make.defs file replaces the ar and nm with
  the default system x86 tool versions and these seem to work okay.  Disable all
  of the following to avoid using objcopy:

    CONFIG_RRLOAD_BINARY=n
    CONFIG_INTELHEX_BINARY=n
    CONFIG_MOTOROLA_SREC=n
    CONFIG_RAW_BINARY=n

  devkitARM
  ---------
  The devkitARM toolchain includes a version of MSYS make.  Make sure that the
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
  3) Set up include pathes:  You will need include/, arch/arm/src/stm32,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/stm32/stm32_vectors.S.  With RIDE, I have to build NuttX
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
     ./configure.sh stm32_tiny/<sub-dir>

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
====

The STM32Tiny board has only one software controllable LED.
This LED can be used by the board port when CONFIG_ARCH_LEDS option is
enabled.

If enabled the LED is simply turned on when the board boots
succesfully, and is blinking on panic / assertion failed.

PWM
===

The STM32 Tiny has no real on-board PWM devices, but the board can be
configured to output a pulse train using TIM3 CH2 on the GPIO line B.5
(connected to the LED).
Please note that the CONFIG_STM32_TIM3_PARTIAL_REMAP option must be enabled
in this case.

UARTs
=====

UART/USART PINS
---------------

USART1
  RX      PA10
  TX      PA9
USART2
  CK      PA4
  CTS     PA0*
  RTS     PA1
  RX      PA3
  TX      PA2
USART3
  CK      PB12*
  CTS     PB13*
  RTS     PB14*
  RX      PB11
  TX      PB10

* theses IO lines are intended to be used by the wireless module on the board.


Default USART/UART Configuration
--------------------------------

USART1 (RX & TX only) is available through the RS-232 port on the board. A MAX232 chip converts
voltage to RS-232 level. This serial port can be used to flash a firmware using the boot loader
integrated in the MCU.


Timer Inputs/Outputs
====================

TIM1
  CH1     PA8
  CH2     PA9*
  CH3     PA10*
  CH4     PA11*
TIM2
  CH1     PA0*, PA15, PA5
  CH2     PA1, PB3
  CH3     PA2, PB10*
  CH4     PA3, PB11
TIM3
  CH1     PA6, PB4
  CH2     PA7, PB5*
  CH3     PB0
  CH4     PB1*
TIM4
  CH1     PB6*
  CH2     PB7
  CH3     PB8
  CH4     PB9*

 * Indicates pins that have other on-board functions and should be used only
   with care (See board datasheet).


STM32 Tiny - specific Configuration Options
===============================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM3=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=stm32

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_STM32F103C8=y

    CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
       configuration features.

       CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=stm32_tiny

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_STM32_TINY=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_RAM_SIZE=20480 (20Kb)

    CONFIG_RAM_START - The start address of installed DRAM

       CONFIG_RAM_START=0x20000000

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
        stack in bytes.  If not defined, the user task stacks will be
      used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

    CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
       cause a 100 second delay during boot-up.  This 100 second delay
       serves no purpose other than it allows you to calibratre
       CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
       the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
       the delay actually is 100 seconds.

  Individual subsystems can be enabled:

    AHB
    ---
    CONFIG_STM32_CRC
    CONFIG_STM32_BKPSRAM

    APB1
    ----
    CONFIG_STM32_TIM2
    CONFIG_STM32_TIM3
    CONFIG_STM32_TIM4
    CONFIG_STM32_WWDG
    CONFIG_STM32_IWDG
    CONFIG_STM32_SPI2
    CONFIG_STM32_USART2
    CONFIG_STM32_USART3
    CONFIG_STM32_I2C1
    CONFIG_STM32_I2C2
    CONFIG_STM32_CAN1
    CONFIG_STM32_PWR -- Required for RTC

    APB2
    ----
    CONFIG_STM32_TIM1
    CONFIG_STM32_USART1
    CONFIG_STM32_ADC1
    CONFIG_STM32_ADC2
    CONFIG_STM32_SPI1

  Timer devices may be used for different purposes.  One special purpose is
  to generate modulated outputs for such things as motor control.  If CONFIG_STM32_TIMn
  is defined (as above) then the following may also be defined to indicate that
  the timer is intended to be used for pulsed output modulation or ADC conversion.
  Note that ADC require two definitions:  Not only do you have
  to assign the timer (n) for used by the ADC, but then you also have to
  configure which ADC (m) it is assigned to.

    CONFIG_STM32_TIMn_PWM   Reserve timer n for use by PWM, n=1,..,14
    CONFIG_STM32_TIMn_ADC   Reserve timer n for use by ADC, n=1,..,14
    CONFIG_STM32_TIMn_ADCm  Reserve timer n to trigger ADCm, n=1,..,14, m=1,..,3

  For each timer that is enabled for PWM usage, we need the following additional
  configuration settings:

    CONFIG_STM32_TIMx_CHANNEL - Specifies the timer output channel {1,..,4}

  NOTE: The STM32 timers are each capable of generating different signals on
  each of the four channels with different duty cycles.  That capability is
  not supported by this driver:  Only one output channel per timer.

  JTAG Enable settings (by default only SW-DP is enabled):

    CONFIG_STM32_JTAG_FULL_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)
    CONFIG_STM32_JTAG_NOJNTRST_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)
      but without JNTRST.
    CONFIG_STM32_JTAG_SW_ENABLE - Set JTAG-DP disabled and SW-DP enabled

  STM32Tiny specific device driver settings

    CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the USARTn (n=1,2,3)
       for the console and ttys0 (default is the USART1).
    CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_U[S]ARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_U[S]ARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_U[S]ARTn_2STOP - Two stop bits

  STM32Tiny CAN Configuration

    CONFIG_CAN - Enables CAN support (one or both of CONFIG_STM32_CAN1 or
      CONFIG_STM32_CAN2 must also be defined)
    CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default
      Standard 11-bit IDs.
    CONFIG_CAN_FIFOSIZE - The size of the circular buffer of CAN messages.
      Default: 8
    CONFIG_CAN_NPENDINGRTR - The size of the list of pending RTR requests.
      Default: 4
    CONFIG_CAN_LOOPBACK - A CAN driver may or may not support a loopback
      mode for testing. The STM32 CAN driver does support loopback mode.
    CONFIG_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32_CAN1 is defined.
    CONFIG_CAN2_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32_CAN2 is defined.
    CONFIG_CAN_TSEG1 - The number of CAN time quanta in segment 1. Default: 6
    CONFIG_CAN_TSEG2 - the number of CAN time quanta in segment 2. Default: 7
    CONFIG_CAN_REGDEBUG - If CONFIG_DEBUG is set, this will generate an
      dump of all CAN registers.

  STM32Tiny SPI Configuration

    CONFIG_STM32_SPI_INTERRUPTS - Select to enable interrupt driven SPI
      support. Non-interrupt-driven, poll-waiting is recommended if the
      interrupt rate would be to high in the interrupt driven case.
    CONFIG_STM32_SPI_DMA - Use DMA to improve SPI transfer performance.
      Cannot be used with CONFIG_STM32_SPI_INTERRUPT.

Configurations
==============

Each STM32Tiny configuration is maintained in a sub-directory and
can be selected as follow:

    cd tools
    ./configure.sh STM32Tiny/<subdir>
    cd -
    . ./setenv.sh

If this is a Windows native build, then configure.bat should be used
instead of configure.sh:

    configure.bat STM32Tiny\<subdir>

Where <subdir> is one of the following:

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh. This
    configuration enables a console on UART1. Support for
    builtin applications is enabled, but in the base configuration no
    builtin applications are selected (see NOTES below).

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

       CONFIG_HOST_WINDOWS=y                   : Builds under Windows
       CONFIG_WINDOWS_CYGWIN=y                 : Using Cygwin
       CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y : CodeSourcery for Windows

    3. This example supports the PWM test (apps/examples/pwm) but this must
       be manually enabled by selecting:

       CONFIG_PWM=y              : Enable the generic PWM infrastructure
       CONFIG_STM32_TIM3=y       : Enable TIM3
       CONFIG_STM32_TIM3_PWM=y   : Use TIM3 to generate PWM output
       CONFIG_STM32_TIM3_PARTIAL_REMAP=y  : Required to have the port B5 as timer PWM output  (channel 2)
       CONFIG_STM32_TIM3_CHANNEL=2

       See also apps/examples/README.txt

       Note that the only supported board configuration uses the board LED as PWM output.

       Special PWM-only debug options:

       CONFIG_DEBUG_PWM

     7. USB Support (CDC/ACM device)

        CONFIG_STM32_OTGFS=y          : STM32 OTG FS support
        CONFIG_USBDEV=y               : USB device support must be enabled
        CONFIG_CDCACM=y               : The CDC/ACM driver must be built
        CONFIG_NSH_BUILTIN_APPS=y     : NSH built-in application support must be enabled
        CONFIG_NSH_ARCHINIT=y         : To perform USB initialization

     8. Using the USB console.

        The STM32Tiny NSH configuration can be set up to use a USB CDC/ACM
        (or PL2303) USB console.  The normal way that you would configure the
        the USB console would be to change the .config file like this:

        CONFIG_STM32_OTGFS=y           : STM32 OTG FS support
        CONFIG_USART2_SERIAL_CONSOLE=n : Disable the USART2 console
        CONFIG_DEV_CONSOLE=n           : Inhibit use of /dev/console by other logic
        CONFIG_USBDEV=y                : USB device support must be enabled
        CONFIG_CDCACM=y                : The CDC/ACM driver must be built
        CONFIG_CDCACM_CONSOLE=y        : Enable the CDC/ACM USB console.

        NOTE: When you first start the USB console, you have hit ENTER a few
        times before NSH starts.  The logic does this to prevent sending USB data
        before there is anything on the host side listening for USB serial input.

    9.  Here is an alternative USB console configuration.  The following
        configuration will also create a NSH USB console but this version
        will use /dev/console.  Instead, it will use the normal /dev/ttyACM0
        USB serial device for the console:

        CONFIG_STM32_OTGFS=y           : STM32 OTG FS support
        CONFIG_USART2_SERIAL_CONSOLE=y : Keep the USART2 console
        CONFIG_DEV_CONSOLE=y           : /dev/console exists (but NSH won't use it)
        CONFIG_USBDEV=y                : USB device support must be enabled
        CONFIG_CDCACM=y                : The CDC/ACM driver must be built
        CONFIG_CDCACM_CONSOLE=n        : Don't use the CDC/ACM USB console.
        CONFIG_NSH_USBCONSOLE=y        : Instead use some other USB device for the console

        The particular USB device that is used is:

        CONFIG_NSH_USBCONDEV="/dev/ttyACM0"

        The advantage of this configuration is only that it is easier to
        bet working.  This alternative does has some side effects:

        - When any other device other than /dev/console is used for a user
          interface, linefeeds (\n) will not be expanded to carriage return /
          linefeeds (\r\n).  You will need to set your terminal program to account
          for this.

        - /dev/console still exists and still refers to the serial port. So
          you can still use certain kinds of debug output (see include/debug.h, all
          of the interfaces based on lowsyslog will work in this configuration).

        - But don't enable USB debug output!  Since USB is console is used for
          USB debug output and you are using a USB console, there will be
          infinite loops and deadlocks:  Debug output generates USB debug
          output which generatates USB debug output, etc.  If you want USB
          debug output, you should consider enabling USB trace
          (CONFIG_USBDEV_TRACE) and perhaps the USB monitor (CONFIG_SYSTEM_USBMONITOR).

          See the usbnsh configuration below for more information on configuring
          USB trace output and the USB monitor.

  usbnsh:
  -------

    This is another NSH example.  If differs from other 'nsh' configurations
    in that this configurations uses a USB serial device for console I/O.

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

       CONFIG_HOST_WINDOWS=y                   : Builds under Windows
       CONFIG_WINDOWS_CYGWIN=y                 : Using Cygwin
       CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y : CodeSourcery for Windows

    3. This configuration does have UART2 output enabled and set up as
       the system logging device:

       CONFIG_SYSLOG=y                    : Enable output to syslog, not console
       CONFIG_SYSLOG_CHAR=y               : Use a character device for system logging
       CONFIG_SYSLOG_DEVPATH="/dev/ttyS0" : UART2 will be /dev/ttyS0

       However, there is nothing to generate SYLOG output in the default
       configuration so nothing should appear on UART2 unless you enable
       some debug output or enable the USB monitor.

    4. Enabling USB monitor SYSLOG output.  If tracing is enabled, the USB
       device will save encoded trace output in in-memory buffer; if the
       USB monitor is enabled, that trace buffer will be periodically
       emptied and dumped to the system loggin device (UART2 in this
       configuraion):

       CONFIG_USBDEV_TRACE=y                   : Enable USB trace feature
       CONFIG_USBDEV_TRACE_NRECORDS=128        : Buffer 128 records in memory
       CONFIG_NSH_USBDEV_TRACE=n               : No builtin tracing from NSH
       CONFIG_NSH_ARCHINIT=y                   : Automatically start the USB monitor
       CONFIG_SYSTEM_USBMONITOR=y              : Enable the USB monitor daemon
       CONFIG_SYSTEM_USBMONITOR_STACKSIZE=2048 : USB monitor daemon stack size
       CONFIG_SYSTEM_USBMONITOR_PRIORITY=50    : USB monitor daemon priority
       CONFIG_SYSTEM_USBMONITOR_INTERVAL=2     : Dump trace data every 2 seconds

       CONFIG_SYSTEM_USBMONITOR_TRACEINIT=y    : Enable TRACE output
       CONFIG_SYSTEM_USBMONITOR_TRACECLASS=y
       CONFIG_SYSTEM_USBMONITOR_TRACETRANSFERS=y
       CONFIG_SYSTEM_USBMONITOR_TRACECONTROLLER=y
       CONFIG_SYSTEM_USBMONITOR_TRACEINTERRUPTS=y

    5. By default, this project assumes that you are *NOT* using the DFU
       bootloader.

    Using the Prolifics PL2303 Emulation
    ------------------------------------
    You could also use the non-standard PL2303 serial device instead of
    the standard CDC/ACM serial device by changing:

      CONFIG_CDCACM=y               : Disable the CDC/ACM serial device class
      CONFIG_CDCACM_CONSOLE=y       : The CDC/ACM serial device is NOT the console
      CONFIG_PL2303=y               : The Prolifics PL2303 emulation is enabled
      CONFIG_PL2303_CONSOLE=y       : The PL2303 serial device is the console
