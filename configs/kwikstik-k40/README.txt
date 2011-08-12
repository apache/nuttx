README.txt
==========

This is the README file for the port of NuttX to the Freescale Kinetis
KwiStick K40.  Refer to the Freescale web site for further information
about this part:
http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=KWIKSTIK-K40

Contents
========

  o Kinetis KwikStik Features
  o Development Environment
  o GNU Toolchain Options
  o IDEs
  o NuttX buildroot Toolchain
  
Kinetis KwikStik Features:
=========================

  o Kinetis K40 MCU in 144 LQFP
    - 100 MHz ARM Cortex-M4 core
    - 256Kb program flash, 256Kb FlexMemory
    - Full-speed USB 2.0 device
    - Low-pwer segment LCD controller
    - SPI, UART, CAN and more
  o Large segment LCD display with 306 segments
  o 2.3mm audio output and 2 micro USB connectors
  o Omnidirectional microphone and a buzzer
  o On-board Segger J-Link debugger interface
  o Infrared communication port
  o microSD card slot
  o Capacitive touch sensing interface
  o Freescale Tower System connectivity for UART, timers, CAN, SPI, I2C, and DAC
  o Freescale Tower plug-in (TWRPI) socket connectivity for ADC, SPI, I2C, and GPIO

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

  1. The CodeSourcery GNU toolchain,
  2. The devkitARM GNU toolchain,
  3. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the CodeSourcery Windows toolchain.  To
  use the devkitARM or the NuttX GNU toolchain, you simply need to change the
  the following configuration options to your .config (or defconfig) file:

    CONFIG_KINETIS_CODESOURCERYW=y  : CodeSourcery under Windows
    CONFIG_KINETIS_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_KINETIS_DEVKITARM=y      : devkitARM under Windows
    CONFIG_KINETIS_BUILDROOT=y	    : NuttX buildroot under Linux or Cygwin (default)

  If you are not using CONFIG_KINETIS_BUILDROOT, then you may also have to modify
  the PATH in the setenv.h file if your make cannot find the tools.

  NOTE: the CodeSourcery (for Windows) and devkitARM toolchains are
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
     a file in a "linked" directory and find that your changes had not effect.
     That is because you are building the copy of the file in the "fake" symbolic
     directory.  If you use a Windows toolchain, you should get in the habit of
     making like this:

       make clean_context all

     An alias in your .bashrc file might make that less painful.

  3. Dependencies are not made when using Windows versions of the GCC.  This is
     because the dependencies are generated using Windows pathes which do not
     work with the Cygwin make.

     Support has been added for making dependencies with the windows-native toolchains.
     That support can be enabled by modifying your Make.defs file as follows:

    -  MKDEP                = $(TOPDIR)/tools/mknulldeps.sh
    +  MKDEP                = $(TOPDIR)/tools/mkdeps.sh --winpaths "$(TOPDIR)"

     If you have problems with the dependency build (for example, if you are not
     building on C:), then you may need to modify tools/mkdeps.sh

  NOTE 1: The CodeSourcery toolchain (2009q1) does not work with default optimization
  level of -Os (See Make.defs).  It will work with -O0, -O1, or -O2, but not with
  -Os.

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
  3) Set up include pathes:  You will need include/, arch/arm/src/k40,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/kinetis/k40_vectors.S.

NuttX buildroot Toolchain
=========================

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M4 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M4 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/project/showfiles.php?group_id=189573).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  NOTE:  The NuttX toolchain is an OABI toolchain (vs. the more common EABI)
  and does not include optimizations for Cortex-M4 (ARMv7E-M).

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh kwikstik-k40/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-defconfig-4.3.3 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  detailed PLUS some special instructions that you will need to follow if you are
  building a Cortex-M4 toolchain for Cygwin under Windows.

KwikStik-K40-specific Configuration Options
============================================

	CONFIG_ARCH - Identifies the arch/ subdirectory.  This sould
	   be set to:

	   CONFIG_ARCH=arm

	CONFIG_ARCH_family - For use in C code:

	   CONFIG_ARCH_ARM=y

	CONFIG_ARCH_architecture - For use in C code:

	   CONFIG_ARCH_CORTEXM4=y

	CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

	   CONFIG_ARCH_CHIP=k40

	CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
	   chip:

	   CONFIG_ARCH_CHIP_MK40X256VLQ100

	CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
	   hence, the board that supports the particular chip or SoC.

	   CONFIG_ARCH_BOARD=kwikstik-k40 (for the KwikStik-K40 development board)

	CONFIG_ARCH_BOARD_name - For use in C code

	   CONFIG_ARCH_BOARD_KWIKSTIK_K40=y

	CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
	   of delay loops

	CONFIG_ENDIAN_BIG - define if big endian (default is little
	   endian)

	CONFIG_DRAM_SIZE - Describes the installed DRAM (SRAM in this case):

	   CONFIG_DRAM_SIZE=0x00010000 (64Kb)

	CONFIG_DRAM_START - The start address of installed DRAM

	   CONFIG_DRAM_START=0x20000000

	CONFIG_DRAM_END - Last address+1 of installed RAM

	   CONFIG_DRAM_END=(CONFIG_DRAM_START+CONFIG_DRAM_SIZE)

	CONFIG_ARCH_IRQPRIO - The Kinetis K40 supports interrupt prioritization

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

  	CONFIG_KINETIS_UART0
  	CONFIG_KINETIS_UART1
  	CONFIG_KINETIS_UART2
  	CONFIG_KINETIS_UART3
  	CONFIG_KINETIS_UART4
  	CONFIG_KINETIS_UART5
  	CONFIG_KINETIS_ETHERNET (K60 only)
  	CONFIG_KINETIS_RNGB (K60 only)
    CONFIG_KINETIS_FLEXCAN0
    CONFIG_KINETIS_FLEXCAN1
    CONFIG_KINETIS_SPI0
    CONFIG_KINETIS_SPI1
    CONFIG_KINETIS_SPI2
    CONFIG_KINETIS_I2C0
    CONFIG_KINETIS_I2C1
    CONFIG_KINETIS_I2S
  	CONFIG_KINETIS_DAC0
  	CONFIG_KINETIS_DAC1
    CONFIG_KINETIS_ADC0
    CONFIG_KINETIS_ADC1
    CONFIG_KINETIS_CMP
    CONFIG_KINETIS_VREF
    CONFIG_KINETIS_SDHC
    CONFIG_KINETIS_FTM0
    CONFIG_KINETIS_FTM1
    CONFIG_KINETIS_FTM2
    CONFIG_KINETIS_LPTIMER
    CONFIG_KINETIS_RTC
    CONFIG_KINETIS_SLCD (K40 only)
    CONFIG_KINETIS_EWM
    CONFIG_KINETIS_CMT
    CONFIG_KINETIS_USBOTG
    CONFIG_KINETIS_USBDCD
    CONFIG_KINETIS_LLWU
    CONFIG_KINETIS_REGFILE
    CONFIG_KINETIS_TSI
    CONFIG_KINETIS_PORTA
    CONFIG_KINETIS_PORTB
    CONFIG_KINETIS_PORTC
    CONFIG_KINETIS_PORTD
    CONFIG_KINETIS_PORTE
    CONFIG_KINETIS_FTFL
    CONFIG_KINETIS_DMA
    CONFIG_KINETIS_DMAMUX
    CONFIG_KINETIS_CRC
    CONFIG_KINETIS_PDB
    CONFIG_KINETIS_PIT
    CONFIG_KINETIS_FLEXBUS
    CONFIG_KINETIS_MPU

  Kinetis K40 specific device driver settings

	CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the USARTn (n=1,2,3) or UART
           m (m=4,5) for the console and ttys0 (default is the USART1).
	CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
	   This specific the size of the receive buffer
	CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
	   being sent.  This specific the size of the transmit buffer
	CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART.  Must be
	CONFIG_U[S]ARTn_BITS - The number of bits.  Must be either 7 or 8.
	CONFIG_U[S]ARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
	CONFIG_U[S]ARTn_2STOP - Two stop bits

	CONFIG_KINETIS_SPI_INTERRUPTS - Select to enable interrupt driven SPI
	  support. Non-interrupt-driven, poll-waiting is recommended if the
	  interrupt rate would be to high in the interrupt driven case.
	CONFIG_KINETIS_SPI_DMA - Use DMA to improve SPI transfer performance.
	  Cannot be used with CONFIG_KINETIS_SPI_INTERRUPT.

	CONFIG_SDIO_DMA - Support DMA data transfers.  Requires CONFIG_KINETIS_SDIO
	  and CONFIG_KINETIS_DMA2.
	CONFIG_SDIO_PRI - Select SDIO interrupt prority.  Default: 128
	CONFIG_SDIO_DMAPRIO - Select SDIO DMA interrupt priority. 
	  Default:  Medium
	CONFIG_SDIO_WIDTH_D1_ONLY - Select 1-bit transfer mode.  Default:
	  4-bit transfer mode.

  KwikStik-K40 LCD Hardware Configuration

    CONFIG_LCD_LANDSCAPE - Define for 320x240 display "landscape"
      support. Default is this 320x240 "landscape" orientation
      (this setting is informative only... not used).
    CONFIG_LCD_PORTRAIT - Define for 240x320 display "portrait"
      orientation support.  In this orientation, the KwikStik-K40's
      LCD ribbon cable is at the bottom of the display. Default is
      320x240 "landscape" orientation.
    CONFIG_LCD_RPORTRAIT - Define for 240x320 display "reverse
      portrait" orientation support.  In this orientation, the
      KwikStik-K40's LCD ribbon cable is at the top of the display.
      Default is 320x240 "landscape" orientation.
    CONFIG_LCD_BACKLIGHT - Define to support an adjustable backlight
      using timer 1.  The granularity of the settings is determined
      by CONFIG_LCD_MAXPOWER.  Requires CONFIG_KINETIS_TIM1.

Configurations
==============

Each KwikStik-K40 configuration is maintained in a sudirectory and
can be selected as follow:

	cd tools
	./configure.sh kwikstik-k40/<subdir>
	cd -
	. ./setenv.sh

Where <subdir> is one of the following:

  ostest:
  ------
    This configuration directory, performs a simple OS test using
    examples/ostest.  By default, this project assumes that you are
    using the DFU bootloader.

    CONFIG_KINETIS_BUILDROOT=y	  : NuttX buildroot under Linux or Cygwin
