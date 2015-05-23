README
======

This README discusses issues unique to NuttX configurations for the
HY-MiniSTM32V development board.

Contents
========

  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI "buildroot" Toolchain
  - NuttX OABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - ST Bootloader
  - LEDs
  - RTC
  - HY-Mini specific Configuration Options
  - Configurations

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
  3. Raisonance GNU toolchain, or
  4. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the NuttX buildroot toolchain.  However,
  the make system is setup to default to use the devkitARM toolchain.  To use
  the CodeSourcery, devkitARM or Raisonance GNU toolchain, you simply need to
  add one of the following configuration options to your .config (or defconfig)
  file:

    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y  : CodeSourcery under Windows
    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_ARMV7M_TOOLCHAIN_DEVKITARM=y      : devkitARM under Windows
    CONFIG_ARMV7M_TOOLCHAIN_RAISONANCE=y     : Raisonance RIDE7 under Windows
    CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin (default)

  If you are not using CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT, then you may also have to modify
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
  is arch/arm/src/stm32/stm32_vectors.S.  You many have to build NuttX
  one time from the Cygwin command line in order to obtain the pre-built
  startup object needed by an IDE.

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
     ./configure.sh hymini-stm32v/<sub-dir>

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
     ./configure.sh hymini-stm32v/<sub-dir>

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

ST Bootloader
=============

  A bootloader code is available in an internal boot ROM memory (called
  'system memory' in STM documentation) in all STM32 MCUs. For the F103xx
  this bootloader can be used to upload & flash a firmware image through
  the USART1.

  Notes:

  - The bootloader is activated by the BOOT0 / BOOT1 pins after a MCU reset.
    See STM application note 2606 for more details.
  - On the hymini-stm32 board the USART1 is connected to a PL2303
    USB<->serial converter.

  To enter bootloader mode in the hymini-stm32 board:

  - Press the 'boot0' button  (located next to 'reset' button)
  - While boot0 button is pressed, reset the board through the reset button.
  - Once you pressed / released the 'reset' button, the MCU has (re)started
    in bootloader mode (and you can then release the boot0 button).

  A flash utility must be used on your development workstation to upload / flash
  a firmware image. (The 'stm32flash' open source tool, available at
  http://stm32flash.googlecode.com/ has been used sucessfully).

LEDs
====

The HY-MiniSTM32 board provides only two controlable LEDs labeled LED1 and LED2.
Usage of these LEDs is defined in include/board.h and src/up_leds.c.
They are encoded as follows:

    SYMBOL              Meaning                 LED1*   LED2
    ------------------- ----------------------- ------- -------
    LED_STARTED         NuttX has been started  OFF     OFF
    LED_HEAPALLOCATE    Heap has been allocated ON      OFF
    LED_IRQSENABLED     Interrupts enabled      OFF     ON
    LED_STACKCREATED    Idle stack created      ON      OFF
    LED_INIRQ           In an interrupt**       OFF     N/C
    LED_SIGNAL          In a signal handler***  N/C     ON
    LED_ASSERTION       An assertion failed     ON      ON
    LED_PANIC           The system has crashed  BLINK   BLINK
    LED_IDLE            STM32 is is sleep mode  (Optional, not used)

  * If Nuttx starts correctly, normal state is to have LED1 on and LED2 off.
 ** LED1 is turned off during interrupt.
*** LED2 is turned on during signal handler.

RTC
===

  The STM32 RTC may configured using the following settings.

    CONFIG_RTC - Enables general support for a hardware RTC. Specific
      architectures may require other specific settings.
    CONFIG_RTC_HIRES - The typical RTC keeps time to resolution of 1
      second, usually supporting a 32-bit time_t value.  In this case,
      the RTC is used to &quot;seed&quot; the normal NuttX timer and the
      NuttX timer provides for higher resoution time. If CONFIG_RTC_HIRES
      is enabled in the NuttX configuration, then the RTC provides higher
      resolution time and completely replaces the system timer for purpose of
      date and time.
      CONFIG_RTC_FREQUENCY - If CONFIG_RTC_HIRES is defined, then the
      frequency of the high resolution RTC must be provided.  If CONFIG_RTC_HIRES
      is not defined, CONFIG_RTC_FREQUENCY is assumed to be one.
    CONFIG_RTC_ALARM - Enable if the RTC hardware supports setting of an alarm.
      A callback function will be executed when the alarm goes off

  In hi-res mode, the STM32 RTC operates only at 16384Hz.  Overflow interrupts
  are handled when the 32-bit RTC counter overflows every 3 days and 43 minutes.
  A BKP register is incremented on each overflow interrupt creating, effectively,
  a 48-bit RTC counter.

  In the lo-res mode, the RTC operates at 1Hz.  Overflow interrupts are not handled
  (because the next overflow is not expected until the year 2106.

   WARNING:  Overflow interrupts are lost whenever the STM32 is powered down.  The
   overflow interrupt may be lost even if the STM32 is powered down only momentarily.
   Therefore hi-res solution is only useful in systems where the power is always on.

HY-Mini specific Configuration Options
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

       CONFIG_ARCH_CHIP_STM32F103VC

    CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
       configuration features.

       CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=hymini-stm32v (for the HY-Mini development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_HYMINI_STM32V=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_RAM_SIZE=0x0000C000 (48Kb)

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
    CONFIG_STM32_TIM3    (required for PWM control of LCD backlight)
    CONFIG_STM32_TIM4
    CONFIG_STM32_TIM5
    CONFIG_STM32_TIM6
    CONFIG_STM32_TIM7
    CONFIG_STM32_IWDG
    CONFIG_STM32_WWDG
    CONFIG_STM32_IWDG
    CONFIG_STM32_SPI2
    CONFIG_STM32_SPI4
    CONFIG_STM32_USART2
    CONFIG_STM32_USART3
    CONFIG_STM32_UART4
    CONFIG_STM32_UART5
    CONFIG_STM32_I2C1
    CONFIG_STM32_I2C2
    CONFIG_STM32_USB
    CONFIG_STM32_CAN1
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

  Timer and I2C devices may need to the following to force power to be applied
  unconditionally at power up.  (Otherwise, the device is powered when it is
  initialized).

    CONFIG_STM32_FORCEPOWER

  The Timer3 alternate mapping is required for PWM control of LCD backlight

    CONFIG_STM32_TIM3_PARTIAL_REMAP=y

  Timer devices may be used for different purposes.  One special purpose is
  to generate modulated outputs for such things as motor control.  If CONFIG_STM32_TIMn
  is defined (as above) then the following may also be defined to indicate that
  the timer is intended to be used for pulsed output modulation, ADC conversion,
  or DAC conversion.  Note that ADC/DAC require two definition:  Not only do you have
  to assign the timer (n) for used by the ADC or DAC, but then you also have to
  configure which ADC or DAC (m) it is assigned to.

    CONFIG_STM32_TIMn_PWM   Reserve timer n for use by PWM, n=1,..,8
    CONFIG_STM32_TIMn_ADC   Reserve timer n for use by ADC, n=1,..,8
    CONFIG_STM32_TIMn_ADCm  Reserve timer n to trigger ADCm, n=1,..,8, m=1,..,3
    CONFIG_STM32_TIMn_DAC   Reserve timer n for use by DAC, n=1,..,8
    CONFIG_STM32_TIMn_DACm  Reserve timer n to trigger DACm, n=1,..,8, m=1,..,2

  Others alternate pin mappings available:

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
    CONFIG_STM32_CAN1_REMAP1
    CONFIG_STM32_CAN1_REMAP2
    CONFIG_STM32_CAN2_REMAP

  STM32F103V specific device driver settings

    CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the USARTn (n=1,2,3) or UART
       m (m=4,5) for the console and ttys0 (default is the USART1).

       Note: USART1 is connected to a PL2303 serial to USB converter.
       So USART1 is available through USB port labeled CN3 on the board.

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
    CONFIG_MMCSD_HAVECARDDETECT - Select if SDIO driver card detection
      is 100% accurate  (it is on the  HY-MiniSTM32V)

  HY-MiniSTM32V CAN Configuration

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

  HY-MiniSTM32V LCD Hardware Configuration.  The HY-Mini board may be delivered with
  either an SSD1289 or an R61505U LCD controller.

    CONFIG_LCD_R61505U - Selects the R61505U LCD controller.
    CONFIG_LCD_SSD1289 - Selects the SSD1289 LCD controller.

  The following options apply for either LCD controller:

    CONFIG_NX_LCDDRIVER - To be defined to include LCD driver
    CONFIG_LCD_LANDSCAPE - Define for 320x240 display "landscape"
      support. In this orientation, the HY-MiniSTM32V's
      LCD used connector is at the right of the display.
      Default is this 320x240 "landscape" orientation
    CONFIG_LCD_PORTRAIT - Define for 240x320 display "portrait"
      orientation support.  In this orientation, the HY-MiniSTM32V's
      LCD used connector is at the bottom of the display. Default is
      320x240 "landscape" orientation.
    CONFIG_LCD_RPORTRAIT - Define for 240x320 display "reverse
      portrait" orientation support.  In this orientation, the
      HY-MiniSTM32V's LCD used connector is at the top of the display.
      Default is 320x240 "landscape" orientation.
    CONFIG_LCD_BACKLIGHT - Define to support an adjustable backlight
      using timer 3.  The granularity of the settings is determined
      by CONFIG_LCD_MAXPOWER.  Requires CONFIG_STM32_TIM3.

Configurations
==============

NOTES:

  - All configurations described below are using the mconf-based
    configuration tool.  To change their configuration using that tool, you
    should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       and misc/tools/

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  - All configurations use a generic GNU EABI toolchain for Linux by
    default.

  - They are all configured to generate a binary image that can be flashed
    through the STM32 internal bootloader.

Each HY-MiniSTM32V configuration is maintained in a sub-directory and
can be selected as follow:

    cd tools
    ./configure.sh hymini-stm32v/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  buttons:
  --------

    Uses apps/examples/buttons to exercise HY-MiniSTM32V buttons and
    button interrupts.

    CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIL=y  : Generic GNU EABI toolchain

  nsh and nsh2:
  ------------
    Configure the NuttShell (nsh) located at examples/nsh.

    Differences between the two NSH configurations:

    =========== ======================= ================================
                nsh                     nsh2
    =========== ======================= ================================
    Serial      Debug output: USART1    Debug output: USART1
    Console:    NSH output:   USART1    NSH output:   USART1 (2)
    ----------- ----------------------- --------------------------------
    microSD     Yes (5)                 Yes (5)
    Support
    ----------- ----------------------- --------------------------------
    FAT FS      CONFIG_FAT_LCNAMES=y    CONFIG_FAT_LCNAMES=y
    Config      CONFIG_FAT_LFN=n        CONFIG_FAT_LFN=y (3)
    ----------- ----------------------- --------------------------------
    LCD Driver  No                      Yes
    Support
    ----------- ----------------------- --------------------------------
    RTC Support No                      Yes
    ----------- ----------------------- --------------------------------
    Support for No                      Yes
    Built-in
    Apps
    ----------- ----------------------- --------------------------------
    Built-in    None                    apps/examples/nx
    Apps                                apps/examples/nxhello
                                        apps/system/usbmsc (4)
                                        apps/examples/buttons
                                        apps/examples/nximage
    =========== ======================= ================================

    (1) You will probably need to modify nsh/setenv.sh or nsh2/setenv.sh
        to set up the correct PATH variable for whichever toolchain you
        may use.
    (2) When any other device other than /dev/console is used for a user
        interface, (1) linefeeds (\n) will not be expanded to carriage return
        / linefeeds \r\n). You will need to configure your terminal program
        to account for this. And (2) input is not automatically echoed so
        you will have to turn local echo on.
    (3) Microsoft holds several patents related to the design of
        long file names in the FAT file system.  Please refer to the
        details in the top-level COPYING file.  Please do not use FAT
        long file name unless you are familiar with these patent issues.
    (4) When built as an NSH add-on command (CONFIG_NSH_BUILTIN_APPS=y),
        Caution should be used to assure that the SD drive is not in use when
        the USB storage device is configured.  Specifically, the SD driver
        should be unmounted like:

        nsh> mount -t vfat /dev/mmcsd0 /mnt/sdcard # Card is mounted in NSH
        ...
        nsh> umount /mnd/sdcard                    # Unmount before connecting USB!!!
        nsh> msconn                                # Connect the USB storage device
        ...
        nsh> msdis                                 # Disconnect USB storate device
        nsh> mount -t vfat /dev/mmcsd0 /mnt/sdcard # Restore the mount

        Failure to do this could result in corruption of the SD card format.
    (5) Option CONFIG_NSH_ARCHINIT must be enabled in order to call the SDIO slot
        initialization code.

  usbmsc:
  -------

    This configuration directory exercises the USB mass storage
    class driver at system/usbmsc.  See examples/README.txt for
    more information.

  usbnsh:
  -------

    This is another NSH example.  If differs from other 'nsh' configurations
    in that this configurations uses a USB serial device for console I/O.

    NOTES:

    1. This configuration does have UART2 output enabled and set up as
       the system logging device:

       CONFIG_SYSLOG=y                    : Enable output to syslog, not console
       CONFIG_SYSLOG_CHAR=y               : Use a character device for system logging
       CONFIG_SYSLOG_DEVPATH="/dev/ttyS0" : UART2 will be /dev/ttyS0

       However, there is nothing to generate SYLOG output in the default
       configuration so nothing should appear on UART2 unless you enable
       some debug output or enable the USB monitor.

    2. Enabling USB monitor SYSLOG output.  If tracing is enabled, the USB
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


    Using the Prolifics PL2303 Emulation
    ------------------------------------
    You could also use the non-standard PL2303 serial device instead of
    the standard CDC/ACM serial device by changing:

      CONFIG_CDCACM=y               : Disable the CDC/ACM serial device class
      CONFIG_CDCACM_CONSOLE=y       : The CDC/ACM serial device is NOT the console
      CONFIG_PL2303=y               : The Prolifics PL2303 emulation is enabled
      CONFIG_PL2303_CONSOLE=y       : The PL2303 serial device is the console

  usbserial:
  ---------
    This configuration directory exercises the USB serial class
    driver at examples/usbserial.  See examples/README.txt for
    more information.

      CONFIG_HOST_LINUX=y                      : Linux host
      CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYL=y  : CodeSourcery toolchain under Linux

    USB debug output can be enabled as by changing the following
    settings in the configuration file:

      -CONFIG_DEBUG=n
      -CONFIG_DEBUG_VERBOSE=n
      -CONFIG_DEBUG_USB=n
      +CONFIG_DEBUG=y
      +CONFIG_DEBUG_VERBOSE=y
      +CONFIG_DEBUG_USB=y

      -CONFIG_EXAMPLES_USBSERIAL_TRACEINIT=n
      -CONFIG_EXAMPLES_USBSERIAL_TRACECLASS=n
      -CONFIG_EXAMPLES_USBSERIAL_TRACETRANSFERS=n
      -CONFIG_EXAMPLES_USBSERIAL_TRACECONTROLLER=n
      -CONFIG_EXAMPLES_USBSERIAL_TRACEINTERRUPTS=n
      +CONFIG_EXAMPLES_USBSERIAL_TRACEINIT=y
      +CONFIG_EXAMPLES_USBSERIAL_TRACECLASS=y
      +CONFIG_EXAMPLES_USBSERIAL_TRACETRANSFERS=y
      +CONFIG_EXAMPLES_USBSERIAL_TRACECONTROLLER=y
      +CONFIG_EXAMPLES_USBSERIAL_TRACEINTERRUPTS=y

    By default, the usbserial example uses the Prolific PL2303
    serial/USB converter emulation.  The example can be modified
    serial/USB converter emulation.  The example can be modified
    to use the CDC/ACM serial class by making the following changes
    to the configuration file:

      -CONFIG_PL2303=y
      +CONFIG_PL2303=n

      -CONFIG_CDCACM=n
      +CONFIG_CDCACM=y

    The example can also be converted to use the alternative
    USB serial example at apps/examples/usbterm by changing the
    following:

      -CONFIG_EXAMPLES_USBSERIAL=y
      +CONFIG_EXAMPLES_USBSERIAL=n

      -CONFIG_EXAMPLES_USBTERM=n
      +CONFIG_EXAMPLES_USBTERM=y
