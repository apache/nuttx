README
======

This README discusses issues unique to NuttX configurations for the Spark Core board from Spark Devices (http://www.spark.io).  This board features the STM32103CBT6 MCU from STMicro.


  Microprocessor: 32-bit ARM Cortex M3 at 72MHz STM32F103CBT6
  Memory:         120 KB Flash and 20 KB SRAM, 2M serial Flash
  I/O Pins Out:   37, 17 On the Connector
  Network:        TI CC3000 Wifi Module
  ADCs:           9 (at 12-bit resolution)
  Peripherals:    4 timers, 2 I2Cs, 2 SPI ports, 3 USARTs, 2 led's one Blue and one RGB.
  Other:          Sleep, stop, and standby modes; serial wire debug and JTAG interfaces

  During the development of the SparkCore, the hardware was in limited supply
  As a work around David Sidrane <david_s5@nscdg.com> created a SparkCore Big board
  (http://nscdg.com/spark/sparkBB.png) that will interface with a maple mini
  (http://leaflabs.com/docs/hardware/maple-mini.html), and a CC3000BOOST
  (https://estore.ti.com/CC3000BOOST-CC3000-BoosterPack-P4258.aspx)

  It breaks out the Tx, Rx to connect to a FTDI TTL-232RG-VREG3V3-WE for the console and
  wires in the spark LEDs and serial flash to the same I/O as the sparkcore. It has a Jlink
  compatible Jtag connector on it.

Contents
========

  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI "buildroot" Toolchain
  - NuttX OABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - Hardware
    - Core Pin out
    - LEDs
    - Buttons
    - USARTS and Serial Consoles
  - DFU and JTAG
  - Spark -specific Configuration Options
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

  All testing has been conducted using the CodeSourcery toolchain for Linux.
  To use the Atollic, devkitARM, Raisonance GNU, or NuttX buildroot toolchain,
  you simply need to add one of the following configuration options to your
  .config (or defconfig) file:

    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=n  : CodeSourcery under Windows
    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_ARMV7M_TOOLCHAIN_ATOLLIC=y        : The Atollic toolchain under Windows
    CONFIG_ARMV7M_TOOLCHAIN_DEVKITARM=n      : devkitARM under Windows
    CONFIG_ARMV7M_TOOLCHAIN_RAISONANCE=y     : Raisonance RIDE7 under Windows
    CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=n      : NuttX buildroot under Linux or Cygwin (default)

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

       MKDEP = $(TOPDIR)/tools/mknulldeps.sh

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

  Using Sourcery CodeBench from http://www.mentor.com/embedded-software/sourcery-tools/sourcery-codebench/overview
    Download and install the latest version (as of this writting it was
    sourceryg++-2013.05-64-arm-none-eabi)

   Import the  project from git.
     File->import->Git-URI, then import a Exiting code as a Makefile progject
     from the working directory the git clone was done to.

   Select the Sourcery CodeBench for ARM EABI. N.B. You must do one command line
     build, before the make will work in CodeBench.

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

DFU and JTAG
============

  Enbling Support for the DFU Bootloader
  --------------------------------------
  The linker files in these projects can be configured to indicate that you
  will be loading code using STMicro built-in USB Device Firmware Upgrade (DFU)
  loader or via some JTAG emulator.  You can specify the DFU bootloader by
  adding the following line:

    CONFIG_STM32_DFU=y

  to your .config file. Most of the configurations in this directory are set
  up to use the DFU loader.

  If CONFIG_STM32_DFU is defined, the code will not be positioned at the beginning
  of FLASH (0x08000000) but will be offset to 0x08005000.  This offset is needed
  to make space for the DFU loader and 0x08005000 is where the DFU loader expects
  to find new applications at boot time.  If you need to change that origin for some
  other bootloader, you will need to edit the file(s) ld.script.dfu for the
  configuration.

  For Linux or Mac:
  ----------------

  While on Linux or Mac, we can use dfu-util to upload nuttx binary.

  1. Make sure we have installed dfu-util. (From yum, apt-get or build from source.)
  2. Start the DFU loader (bootloader) on the Spark board. You do this by
     resetting the board while holding the "Key" button. Windows should
     recognize that the DFU loader has been installed.
  3. Flash the nuttx.bin to the board use dfu-util. Here's an example:

      $ dfu-util -a1 -d 1eaf:0003 -D nuttx.bin -R

  For anything not clear, we can refer to LeafLabs official document:

    http://leaflabs.com/docs/unix-toolchain.html

  For Windows:
  -----------

  The DFU SE PC-based software is available from the STMicro website,
  http://www.st.com.  General usage instructions:

  1. Convert the NuttX Intel Hex file (nuttx.hex) into a special DFU
     file (nuttx.dfu)... see below for details.
  2. Connect the M3 Wildfire board to your computer using a USB
     cable.
  3. Start the DFU loader on the M3 Wildfire board.  You do this by
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
  nutt.hex in the top-level directory.  That is the file that you should
  provide to the DFU File Manager.  You will end up with a file called
  nuttx.dfu that you can use with the STMicro DFU SE program.

  Enabling JTAG
  -------------
  If you are not using the DFU, then you will probably also need to enable
  JTAG support.  By default, all JTAG support is disabled but there NuttX
  configuration options to enable JTAG in various different ways.

  These configurations effect the setting of the SWJ_CFG[2:0] bits in the AFIO
  MAPR register.  These bits are used to configure the SWJ and trace alternate function I/Os.
  The SWJ (SerialWire JTAG) supports JTAG or SWD access to the Cortex debug port.
  The default state in this port is for all JTAG support to be disable.

  CONFIG_STM32_JTAG_FULL_ENABLE - sets SWJ_CFG[2:0] to 000 which enables full
    SWJ (JTAG-DP + SW-DP)

  CONFIG_STM32_JTAG_NOJNTRST_ENABLE - sets SWJ_CFG[2:0] to 001 which enable
    full SWJ (JTAG-DP + SW-DP) but without JNTRST.

  CONFIG_STM32_JTAG_SW_ENABLE - sets SWJ_CFG[2:0] to 010 which would set JTAG-DP
    disabled and SW-DP enabled

  The default setting (none of the above defined) is SWJ_CFG[2:0] set to 100
  which disable JTAG-DP and SW-DP.

Hardware
========

  The Spark comprises a STM32F103CB 72 Mhz, 128 Flash, 20K Ram, with 37 IO Pins, and
  a TI CC3000 Wifi Module. It has a 2MB serial flash, onboad regulation and 2 led's
  one Blue and one RGB.

  During the development of the SparkCore, the hardware was in limited supply
  As a work around david_s5 created a SparkCore Big board (http://nscdg.com/spark/sparkBB.png)
  that will interface with a maple mini (http://leaflabs.com/docs/hardware/maple-mini.html),
  and a CC3000BOOST (https://estore.ti.com/CC3000BOOST-CC3000-BoosterPack-P4258.aspx)

  It breaks out the Tx, Rx to connect to a FTDI TTL-232RG-VREG3V3-WE for the console and
  wires in the spark LEDs and serial flash to the same I/O as the sparkcore. It has a Jlink
  compatible Jtag connector on it.

Core Pin out
============

  There are 24 pis on the Spark Core module.

  Spark     Spark Function                                         STM32F103CBT6
  Name      Pin #                           Pin #
  -------- ------ ------------------------------------------------ ---------------
   RAW     JP1-1  Input Power                                       N/A
   GND     JP1-2  GND
   TX      JP1-3  PA[02] USART2_TX/ADC12_IN2/TIM2_CH3               12
   RX      JP1-4  PA[03] USART2_RX/ADC12_IN3/TIM2_CH4               13
   A7      JP1-5  PB[01] ADC12_IN9/TIM3_CH4                         19
   A6      JP1-6  PB[00] ADC12_IN8/TIM3_CH3                         18
   A5      JP1-7  PA[07] SPI1_MOSI/ADC12_IN7/TIM3_CH2               17
   A4      JP1-8  PA[06] SPI1_MISO/ADC12_IN6/TIM3_CH1               16
   A3      JP1-9  PA[05] SPI1_SCK/ADC12_IN5                         15
   A2     JP1-10  PA[04] SPI1_NSS/USART2_CK/ADC12_IN4               14
   A1     JP1-11  PA[01] USART2_RTS/ADC12_IN1/TIM2_CH2              11
   A0     JP1-12  PA[00] WKUP/USART2_CTS/ADC12_IN0/TIM2_CH1_ETR     10

  +3V3     JP2-1  V3.3 Out of Core                                  NA
   RST     JP2-2  NRST                                              7
   VDDA    JP2-3  ADC Voltage                                       9
   GND     JP2-4  GND
   D7      JP2-5  PA[13] JTMS/SWDIO                                 34 Common with Blue LED LED_USR
   D6      JP2-6  PA[14] JTCK/SWCLK                                 37
   D5      JP2-7  PA[15] JTDI                                       38
   D4      JP2-8  PB[03] JTDO                                       39
   D3      JP2-9  PB[04] NJTRST                                     40
   D2     JP2-10  PB[05] I2C1_SMBA                                  41
   D1     JP2-11  PB[06] I2C1_SCL/TIM4_CH1                          42
   D0     JP2-12  PB[07] I2C1_SDA/TIM4_CH2                          43

Core Internal IO
================

  Spark       Function                                          STM32F103CBT6
    Name                                    Pin #
  --------     ------------------------------------------------ ---------------
  BTN          PB[02] BOOT1                                      20
  LED1,D7      PA[13] JTMS/SWDIO                                 34
  LED2         PA[08] USART1_CK/TIM1_CH1/MCO                     29
  LED3         PA[09] USART1_TX/TIM1_CH2                         30
  LED4         PA[10] USART1_RX/TIM1_CH3                         31
  MEM_CS       PB[09] TIM4_CH4                                   46       SST25VF016B Chip Select
  SPI_CLK      PB[13] SPI2_SCK/USART3_CTS/TIM1_CH1N              26
  SPI_MISO     PB[14] SPI2_MISO/USART3_RTS/TIM1_CH2N             27
  SPI_MOSI     PB[15] SPI2_MOSI/TIM1_CH3N                        28
  USB_DISC     PB[10] I2C2_SCL/USART3_TX                         21
  WIFI_CS      PB[12] SPI2_NSS/I2C2_SMBA/USART3_CK/TIM1_BKIN     25        CC3000 Chip Select
  WIFI_EN      PB[08] TIM4_CH3                                   45        CC3000 Module enable
  WIFI_INT     PB[11] I2C2_SDA/USART3_RX                         22        CC3000 Host interface SPI interrupt

Buttons and LEDs
================

  Buttons
  -------
  The Spark has two mechanical buttons. One button is the RESET button
  connected to the STM32F103CB's reset line via /RST and the other is a
  generic user configurable button labeled BTN and connected to GPIO
  PB2/BOOT1. Since on the Spark, BOOT0 is tied to GND it is a moot point
  that BTN signal is connected to the BOOT1 signal. When a button is
  pressed it will drive the I/O line to GND.

  LEDs
  ----
  There are 4 user-controllable LEDs in two packages on board the Spark board:

      Sigal      Location     Color        GPIO    Active
      -------    ------------ -----------  -----  -----------
      LED1      LED_USR      Blue  LED    PA13    High  Common With D7
      LED2      LED_RGB      Red   LED    PA8     Low
      LED3      LED_RGB      Blue  LED    PA9     Low
      LED4      LED_RGB      Green LED    PA10    Low

  LED1 is connected to ground and can be illuminated by driving the PA13
  output high, it shares the Sparks D7 output. The LED2,LED3 and LED4
  are pulled high and can be illuminated by driving the corresponding GPIO output
  to low.

  The RGB LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/up_leds.c. The LEDs are used to encode OS-related
  events as follows:

      SYMBOL               Meaning                LED2    LED3   LED4
                                                  red     blue  green     Color
    ------------------- ----------------------- ------- ------- ------ ---------
    LED_STARTED         NuttX has been started  ON      OFF     OFF    Red
    LED_HEAPALLOCATE    Heap has been allocated OFF     ON      OFF    Blue
    LED_IRQSENABLED     Interrupts enabled      ON      OFF     ON     Orange
    LED_STACKCREATED    Idle stack created      OFF     OFF     ON     Green
    LED_INIRQ           In an interrupt**       ON      N/C     N/C    Orange Glow
    LED_SIGNAL          In a signal handler***  N/C     ON      N/C    Blue Glow
    LED_ASSERTION       An assertion failed     ON      ON      ON     White
    LED_PANIC           The system has crashed  ON      N/C     N/C    Red Flashing
    LED_IDLE            STM32 is is sleep mode  (Optional, not used)

    * If LED2, LED3, LED4 are statically on, then NuttX probably failed to boot
      and these LEDs will give you some indication of where the failure was
   ** The normal state is LED4 ON and LED2 faintly glowing.  This faint glow
      is because of timer interupts that result in the LED being illuminated
      on a small proportion of the time.
  *** LED3 may also flicker normally if signals are processed.

Serial Consoles
===============

  USART2
  -----
  If you have a 3.3 V TTL to RS-232 convertor then this is the most convenient
  serial console to use.  UART2 is the default in all of these
  configurations.

    USART2 RX  PA3   JP1 pin 4
    USART2 TX  PA2   JP1 pin 3
    GND             JP1 pin 2
    V3.3            JP2 pin 1

  Virtual COM Port
  ----------------
  Yet another option is to use UART0 and the USB virtual COM port.  This
  option may be more convenient for long term development, but was
  painful to use during board bring-up.

Spark -specific Configuration Options
==============
 WIP

Configurations
==============

  Composite: The composite is a super set of all the functions in nsh,
  usbserial, usbmsc. (usbnsh has not been rung out).

  Build it with

    make distclean;cd tools;./configure.sh spark/composite;cd ..

  then run make menuconfig if you wish to customize things.

  N.B. Memory is tight, both Flash and RAM are taxed. If you enable
  debugging you will need to add -Os following the line -g in the line:

    ifeq ($(CONFIG_DEBUG_SYMBOLS),y)
      ARCHOPTIMIZATION = -g

  in the top level Make.degs or the code will not fit.

  Stack space has been hand optimized using the stack coloring by enabling
  "Stack usage debug hooks" (CONFIG_DEBUG_STACK) in Build Setup-> Debug
  Options. I have selected values that have 8-16 bytes of headroom with
  network debugging on. If you enable more debugging and get a hard fault
  or any weirdness like commands hanging. Then the Idle, main or Interrupt
  stack my be too small. Stop the target and have a look a memory for a
  blown stack: No DEADBEEF at the lowest address of a given stack.

  Given the RAM memory constraints it is not possible to be running the
  network and USB CDC/ACM and MSC at the same time. But on the bright
  side, you can export the FLASH memory to the PC. Write files on the
  Flash. Reboot and mount the FAT FS and run network code that will have
  access the files.

  You can use the scripts/cdc-acm.inf file to install the windows
  composite device.

  SPI2 is enabled and support is included for the FAT file system on the
  16Mbit (2M) SST25 device and control of the CC3000 on the spark core.

  When the system boots, you should have a dev/mtdblock0 that can be
  mounted using the command:

     mount -t vfat /dev/mtdblock0 /mnt/p0

  or /dev/mtdblock0 can be exported as MSC on the USB interface along with
  a Virtual serial port as a CDC/ACM interface.

  Use the command conn* and disconn to manage the USB interface.

  N.B. *If /dev/mtdblock0 is mounted then You must unmount it prior to
  exporting it via the conn command.  Bad things will happen if not.

  Network control is facilitated by running the c3b (cc3000basic) application.

  Run c3b from the nsh prompt.

    +-------------------------------------------+
    |      Nuttx CC3000 Demo Program            |
    +-------------------------------------------+

      01 - Initialize the CC3000
      02 - Show RX & TX buffer sizes, & free RAM
      03 - Start Smart Config
      04 - Manually connect to AP
      05 - Manually add connection profile
      06 - List access points
      07 - Show CC3000 information
      08 - Telnet

     Type 01-07 to select above option:

  Select 01. Then use 03 and the TI Smart config application running on an
  IOS or Android device to configure join your network.

  Use 07 to see the IP address of the device.

  (On the next reboot running c3b 01 the CC3000 will automaticaly rejoin the
  network after the 01 give it a few seconds and enter 07 or 08)

  Use 08 to start Telnet. Then you can connect to the device using the
  address listed in command 07.

  qq will exit the c3b with the telnet deamon running (if started)

  Slow.... You will be thinking 300 bps. This is because of packet sizes and
  how the select thread runs in the telnet session. Telnet is not the best
  showcase for the CC3000, but simply a proof of network connectivity.

  http POST and GET should be more efficient.
