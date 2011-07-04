README
======

This README discusses issues unique to NuttX configurations for the
STMicro STM3210E-EVAL development board.

Contents
========

  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX buildroot Toolchain
  - DFU
  - LEDs
  - STM3210E-EVAL-specific Configuration Options
  - Configurations

Development Environment
=======================

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment because the Raisonance R-Link emulatator and some RIDE7 development tools
  were used and those tools works only under Windows.

GNU Toolchain Options
=====================

  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The CodeSourcery GNU toolchain,
  2. The devkitARM GNU toolchain,
  3. Raisonance GNU toolchain, or
  4. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the NuttX buildroot toolchain.  However,
  the make system is setup to default to use the devkitARM toolchain.  To use
  the CodeSourcery, devkitARM or Raisonance GNU toolchain, you simply need to
  add one of the following configuration options to your .config (or defconfig)
  file:

    CONFIG_STM32_CODESOURCERYW=y  : CodeSourcery under Windows
    CONFIG_STM32_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_STM32_DEVKITARM=y      : devkitARM under Windows
    CONFIG_STM32_RAISONANCE=y     : Raisonance RIDE7 under Windows
    CONFIG_STM32_BUILDROOT=y	  : NuttX buildroot under Linux or Cygwin (default)

  If you are not using CONFIG_STM32_BUILDROOT, then you may also have to modify
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
  3) Set up include pathes:  You will need include/, arch/arm/src/stm32,
     arch/arm/src/common, arch/arm/src/cortexm3, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/stm32/stm32_vectors.S.  With RIDE, I have to build NuttX
  one time from the Cygwin command line in order to obtain the pre-built
  startup object needed by RIDE.

NuttX buildroot Toolchain
=========================

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/project/showfiles.php?group_id=189573).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh stm3210e-eval/<sub-dir>

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
  building a Cortex-M3 toolchain for Cygwin under Windows.

DFU
===

  The linker files in these projects can be configured to indicate that you
  will be loading code using STMicro built-in USB Device Firmware Upgrade (DFU)
  loader or via some JTAG emulator.  You can specify the DFU bootloader by
  adding the following line:

    CONFIG_STM32_DFU=y

  to your .config file. Most of the configurations in this directory are set
  up to use the DFU loader.

  If CONFIG_STM32_DFU is defined, the code will not be positioned at the beginning
  of FLASH (0x08000000) but will be offset to 0x08003000.  This offset is needed
  to make space for the DFU loader and 0x08003000 is where the DFU loader expects
  to find new applications at boot time.  If you need to change that origin for some
  other bootloader, you will need to edit the file(s) ld.script.dfu for each
  configuration.

  The DFU SE PC-based software is available from the STMicro website,
  http://www.st.com.  General usage instructions:
  
  1. Convert the NuttX Intel Hex file (nuttx.ihx) into a special DFU
     file (nuttx.dfu)... see below for details.
  2. Connect the STM3210E-EVAL board to your computer using a USB
     cable.
  3. Start the DFU loader on the STM3210E-EVAL board.  You do this by
     resetting the board while holding the "Key" button.  Windows should
     recognize that the DFU loader has been installed.
  3. Run the DFU SE program to load nuttx.dfu into FLASH.

  What if the DFU loader is not in FLASH?  The loader code is available
  inside of the Demo dirctory of the USBLib ZIP file that can be downloaded
  from the STMicro Website.  You can build it using RIDE (or other toolchains);
  you will need a JTAG emulator to burn it into FLASH the first time.

  In order to use STMicro's built-in DFU loader, you will have to get
  the NuttX binary into a special format with a .dfu extension.  The
  DFU SE PC_based software installation includes a file "DFU File Manager"
  conversion program that a file in Intel Hex format to the special DFU
  format.  When you successfully build NuttX, you will find a file called
  nutt.ihx in the top-level directory.  That is the file that you should
  provide to the DFU File Manager.  You will need to rename it to nuttx.hex
  in order to find it with the DFU File Manager. You will end up with
  a file called nuttx.dfu that you can use with the STMicro DFU SE program.

LEDs
====

The STM3210E-EVAL board has four LEDs labeled LD1, LD2, LD3 and LD4 on the
the board.  Usage of these LEDs is defined in include/board.h and src/up_leds.c.
They are encoded as follows:

	SYMBOL				Meaning					LED1*	LED2	LED3	LED4
	-------------------	-----------------------	-------	-------	-------	------
	LED_STARTED			NuttX has been started	ON		OFF		OFF		OFF
	LED_HEAPALLOCATE	Heap has been allocated	OFF		ON		OFF		OFF
	LED_IRQSENABLED		Interrupts enabled		ON		ON		OFF		OFF
	LED_STACKCREATED	Idle stack created		OFF		OFF		ON		OFF
	LED_INIRQ			In an interrupt**		ON		N/C		N/C		OFF
	LED_SIGNAL			In a signal handler***  N/C		ON		N/C		OFF
	LED_ASSERTION		An assertion failed		ON		ON		N/C		OFF
	LED_PANIC			The system has crashed	N/C		N/C		N/C		ON
    LED_IDLE            STM32 is is sleep mode  (Optional, not used)

  * If LED1, LED2, LED3 are statically on, then NuttX probably failed to boot
    and these LEDs will give you some indication of where the failure was
 ** The normal state is LED3 ON and LED1 faintly glowing.  This faint glow
    is because of timer interupts that result in the LED being illuminated
    on a small proportion of the time.
*** LED2 may also flicker normally if signals are processed.

STM3210E-EVAL-specific Configuration Options
============================================

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

	   CONFIG_ARCH_CHIP_STM32F103ZET6

    CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
       configuration features.

       CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n
 
	CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
	   hence, the board that supports the particular chip or SoC.

	   CONFIG_ARCH_BOARD=stm3210e_eval (for the STM3210E-EVAL development board)

	CONFIG_ARCH_BOARD_name - For use in C code

	   CONFIG_ARCH_BOARD_STM3210E_EVAL=y

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

	CONFIG_ARCH_IRQPRIO - The STM32F103Z supports interrupt prioritization

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
	AHB
	---
	CONFIG_STM32_DMA1
	CONFIG_STM32_DMA2
	CONFIG_STM32_CRC
	CONFIG_STM32_FSMC
	CONFIG_STM32_SDIO

	APB1
	----
	CONFIG_STM32_TIM2
	CONFIG_STM32_TIM3
	CONFIG_STM32_TIM4
	CONFIG_STM32_TIM5
	CONFIG_STM32_TIM6
	CONFIG_STM32_TIM7
	CONFIG_STM32_WWDG
	CONFIG_STM32_SPI2
	CONFIG_STM32_SPI4
	CONFIG_STM32_USART2
	CONFIG_STM32_USART3
	CONFIG_STM32_UART4
	CONFIG_STM32_UART5
	CONFIG_STM32_I2C1
	CONFIG_STM32_I2C2
	CONFIG_STM32_USB
	CONFIG_STM32_CAN
	CONFIG_STM32_BKP
	CONFIG_STM32_PWR
	CONFIG_STM32_DAC
	CONFIG_STM32_USB

	APB2
	----
	CONFIG_STM32_ADC1
	CONFIG_STM32_ADC2
	CONFIG_STM32_TIM1
	CONFIG_STM32_SPI1
	CONFIG_STM32_TIM8
	CONFIG_STM32_USART1
	CONFIG_STM32_ADC3

  Alternate pin mappings (should not be used with the STM3210E-EVAL board):

	CONFIG_STM32_TIM1_FULL_REMAP
	CONFIG_STM32_TIM1_PARTIAL_REMAP
	CONFIG_STM32_TIM2_FULL_REMAP
	CONFIG_STM32_TIM2_PARTIAL_REMAP_1
	CONFIG_STM32_TIM2_PARTIAL_REMAP_2
	CONFIG_STM32_TIM3_FULL_REMAP
	CONFIG_STM32_TIM3_PARTIAL_REMAP
	CONFIG_STM32_TIM4_REMAP
	CONFIG_STM32_USART1_REMAP
	CONFIG_STM32_USART2_REMAP
	CONFIG_STM32_USART3_FULL_REMAP
	CONFIG_STM32_USART3_PARTIAL_REMAP
	CONFIG_STM32_SPI1_REMAP
	CONFIG_STM32_SPI3_REMAP
	CONFIG_STM32_I2C1_REMAP
	CONFIG_STM32_CAN1_FULL_REMAP
	CONFIG_STM32_CAN1_PARTIAL_REMAP
	CONFIG_STM32_CAN2_REMAP

  STM32F103Z specific device driver settings

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

	CONFIG_STM32_SPI_INTERRUPTS - Select to enable interrupt driven SPI
	  support. Non-interrupt-driven, poll-waiting is recommended if the
	  interrupt rate would be to high in the interrupt driven case.
	CONFIG_STM32_SPI_DMA - Use DMA to improve SPI transfer performance.
	  Cannot be used with CONFIG_STM32_SPI_INTERRUPT.

	CONFIG_SDIO_DMA - Support DMA data transfers.  Requires CONFIG_STM32_SDIO
	  and CONFIG_STM32_DMA2.
	CONFIG_SDIO_PRI - Select SDIO interrupt prority.  Default: 128
	CONFIG_SDIO_DMAPRIO - Select SDIO DMA interrupt priority. 
	  Default:  Medium
	CONFIG_SDIO_WIDTH_D1_ONLY - Select 1-bit transfer mode.  Default:
	  4-bit transfer mode.

Configurations
==============

Each STM3210E-EVAL configuration is maintained in a sudirectory and
can be selected as follow:

	cd tools
	./configure.sh stm3210e-eval/<subdir>
	cd -
	. ./setenv.sh

Where <subdir> is one of the following:

  nsh and nsh2:
  ------------
    Configure the NuttShell (nsh) located at examples/nsh.

    Differences between the two NSH configurations:

    =========== ======================= ================================
                nsh                     nsh2
    =========== ======================= ================================
    Toolchain:  NuttX buildroot for     Codesourcery for Windows *
                Linux or Cygwin *,**
    ----------- ----------------------- --------------------------------
    Loader:     DfuSe                   DfuSe
    ----------- ----------------------- --------------------------------
    Serial      Debug output: USART1    Debug output: USART1
    Console:    NSH output:   USART1    NSH output:   USART2 ***
    ----------- ----------------------- --------------------------------
    I2C1        Disabled                Enabled
    =========== ======================= ================================

    * You will probably need to modify nsh/setenv.sh or nsh2/setenv.sh
      to set up the correct PATH variable for whichever toolchain you
      may use.
   ** Since DfuSe is assumed, this configuration may only work under
      Cygwin without modification.
  *** When any other device other than /dev/console is used for a user
      interface, (1) linefeeds (\n) will not be expanded to carriage return
      / linefeeds \r\n). You will need to configure your terminal program
      to account for this. And (2) input is not automatically echoed so
      you will have to turn local echo on.

  ostest:
  ------
    This configuration directory, performs a simple OS test using
    examples/ostest.  By default, this project assumes that you are
    using the DFU bootloader.

  RIDE
  ----
    This configuration builds a trivial bring-up binary.  It is
    useful only because it words with the RIDE7 IDE and R-Link debugger.

  usbserial:
  ---------
    This configuration directory exercises the USB serial class
    driver at examples/usbserial.  See examples/README.txt for
    more information.

  usbstorage:
  ----------
    This configuration directory exercises the USB mass storage
    class driver at examples/usbstorage.  See examples/README.txt for
    more information.
