README
======

  This README discusses issues unique to NuttX configurations for the
  STMicro STM32L-Discovery development board.  The STM32L-Discovery board
  is based on the STM32L152RBT6 MCU (128KB FLASH and 16KB of SRAM).

  The STM32L-Discovery and 32L152CDISCOVERY kits are functionally
  equivalent. The difference is the internal Flash memory size (STM32L152RBT6
  with 128 Kbytes or STM32L152RCT6 with 256 Kbytes).

  Both boards feature:

    - An ST-LINK/V2 embedded debug tool interface,
    - LCD (24 segments, 4 commons),
    - LEDs,
    - Pushbuttons,
    - A linear touch sensor, and
    - Four touchkeys.

Contents
========

  - Status
  - GPIO Pin Usage
  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI "buildroot" Toolchain
  - NuttX OABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - LEDs
  - Serial Console
  - Debugging
  - STM32L-Discovery-specific Configuration Options
  - Configurations

Status
======

  The basic port is complete.  A NuttShell (NSH) configuration exists for the
  STM32L-Discovery board.  A driver has been developed for the segment LCD on
  board the STM32L-Discovery.  In the NSH configuration discription below,
  there is information about how the basic NSH extension can be extended to
  use apps/examples/slcd to exercise the segment LCD.

  * The following subsystem have header files, drivers and have been
    exercised: PWR, RCC, GPIO, SYSCFG, LCD, USART.

  * The following subsystenms have header files and ported drivers, but are
    untested:  DMA

  * The following subystems have counterparts with other STM32 parts, but
    have not been ported or verified:  ADC, DAC, TIM2-15, TIM9-11, RTC,
    IWDG, WWDG, I2C, SPI, DBG.  These may be close to functional depending
    upon how close the IP is on the STM32L15X.

    This might include also USB, FSMC, and SDIO.

  * The following subsystems are unique to the STM32L and have not been
    developed: COMP, TSIO, RI, OPAMP

  * The STM32L15X does support USB, however, USB is not available on the
    STM32L-Discovery board.

  * These subystems are available on other STM32L15x/16x parts, but not on
    the part used in the STM32L-Discovery board: CRC, AES, FSMC, SDIO

GPIO Pin Usage
==============

  ----- --------------------- -------------------------------- ----------------
  GPIO  ALTERNATE FUNCTION    BOARD FUNCTION                   P1/P2
  ----- --------------------- -------------------------------- ----------------
  PA0   WKUP1/USART2_CTS/     Push button (PA0), WAKE UP (Iuu) P1, pin 15
        ADC_IN0/TIM2_CH1_ETR
        /COMP1_INP
  PA1   USART2_RTS/ADC_IN1/   LCD SEG0                         P1, pin 16
        TIM2_CH2/LCD_SEG0/
        COMP1_INP
  PA2   USART2_TX/ADC_IN2/    LCD SEG1                         P1, pin 17
        TIM2_CH3/TIM9_CH1/
        LCD_SEG1/COMP1_INP
  PA3   USART2_RX/ADC_IN3/    LCD SEG2                         P1, pin 18
        TIM2_CH4/TIM9_CH2/
        LCD_SEG2/COMP1_INP
  PA4   SPI1_NSS/USART2_CK/   Measurement (Iuu)                P1, pin 19
        ADC_IN4/DAC_OUT1/
        COMP1_INP
  PA5   SPI1_SCK/ADC_IN5/     ---                              P1, pin 20
        DAC_OUT2/
        TIM2_CH1_ETR/COMP1_
        INP
  PA6   SPI1_MISO/ADC_IN6/    Linear Touch Sensor (PA6)        ---
        TIM3_CH1/TIM1_BKIN/
        LCD_SEG3/TIM10_CH1/
        COMP1_INP
  PA7   SPI1_MOSI/ADC_IN7/    Linear Touch Sensor (PA7)        ---
        TIM3_CH2/TIM1_CH1N
        /LCD_SEG4/TIM11_CH1/
  PA8   USART1_CK/MCO/        LCD glass COM0                   P2, pin 23
        LCD_COM0
  PA9   USART1_TX/LCD_COM1    LCD glass COM1                   P2, pin 22
  PA10  USART1_RX/LCD_COM2    LCD glass COM2                   P2, pin 21
  PA11  USART1_CTS/USBDM/     ---                              P2, pin 20
        SPI1_MISO
  PA12  USART1_RTS/USBDP/     ---                              P2, pin 19
        SPI1_MOSI
  JTDI  TIM2_CH1_ETR/PA15/    LCD_SEG12                        P2, pin 16
        SPI1_NSS/LCD_SEG17
  ----- --------------------- -------------------------------- ----------------
  PB0   ADC_IN8/TIM3_CH3/     Linear Touch Sensor (PB0)        ---
        LCD_SEG5/COMP1_INP/
        VREF_OUT
  PB1   ADC_IN9/TIM3_CH4/     Linear Touch Sensor (PB1)        ---
        LCD_SEG6/COMP1_INP/
        VREF_OUT
  PB2/  ---                   ---                              P1, pin 21
  BOOT1
  JTDO  TIM2_CH2/PB3/TRACES   LCD_SEG3, SWO                    P2, pin 11
        WO/SPI1_SCK/COMP2_I
        NM/LCD_SEG7
 JNTRST TIM3_CH1/PB4/SPI1_MIS SEG4                             P2, pin 10
        O/COMP2_INP/LCD_SEG8
  PB5   I2C1_SMBAl/TIM3_CH2/  LCD SEG5                         P2, pin 9
        SPI1_MOSI/COMP2_INP/
        LCD_SEG9
  PB6   I2C1_SCL/TIM4_CH1/    LED Blue                         P2, pin 8
        USART1_TX/LCD_SEG8
  PB7   I2C1_SDA/TIM4_CH2/    LED Green                        P2, pin 7
        USART1_RX/PVD_IN
  PB8   TIM4_CH3/I2C1_SCL/    LCD SEG13                        P2, pin 4
        LCD_SEG16/TIM10_CH1
  PB9   TIM4_CH4/I2C1_SDA/    LCD glass COM3                   P2, pin 3
        LCD_COM3/TIM11_CH1
  PB10  I2C2_SCL/USART3_TX/   LCD SEG6                         P1, pin 22
        TIM2_CH3/LCD_SEG10
  PB11  I2C2_SDA/USART3_RX/   LCD SEG7                         P1, pin 23
        TIM2_CH4/LCD_SEG11
  PB12  SPI2_NSS/I2C2_SMBA/   LCD SEG8                         P1, pin 24
        USART3_CK/LCD_SEG12
        2/ADC_IN18/COMP1_INP
        / TIM10_CH1
  PB13  SPI2_SCK/USART3_CTS/  LCD SEG9                         P1, pin 25
        LCD_SEG13/ADC_IN19/
        COMP1_INP/TIM9_CH1
  PB14  SPI2_MISO/USART3_RT   LCD SEG10                        P1, pin 26
        S/LCD_SEG14/ADC_IN20
        / COMP1_INP/TIM9_CH2
  PB15  SPI2_MOSI/TIM1_CH3N/  LCD SEG11                        P1, pin 27
        LCD_SEG15/ADC_IN21/
        COMP1_INP/TIM11_CH1/
        RTC_50_60Hz
  ----- --------------------- -------------------------------- ----------------
  PC0   ADC_IN10/LCD_SEG18/   LCD SEG14                        P1, pin 11
        COMP1_INP
  PC1   ADC_IN11/LCD_SEG19/   LCD SEG15                        P1, pin 12
        COMP1_INP
  PC2   ADC_IN12/LCD_SEG20/   LCD SEG16                        P1, pin 13
        COMP1_INP
  PC3   ADC_IN13/LCD_SEG21/   LCD SEG17                        P1, pin 14
        COMP1_INP
  PC4   ADC_IN14/LCD_SEG22/   Linear Touch Sensor (PC4)        ---
        COMP1_INP
  PC5   ADC_IN15/LCD_SEG23/   Linear Touch Sensor (PC5)        ---
        COMP1_INP
  PC6   TIM3_CH1/LCD_SEG24    LCD SEG18                        P2, pin 27
  PC7   TIM3_CH2/LCD_SEG25    LCD SEG19                        P2, pin 26
  PC8   TIM3_CH3/LCD_SEG26    LCD SEG20                        P2, pin 25
  PC9   TIM3_CH4/LCD_SEG27    LCD SEG21                        P2, pin 24
  PC10  USART3_TX/LCD_SEG28   LCD SEG22                        P2, pin 15
        /LCD_SEG40/LCD_COM4
  PC11  USART3_RX/LCD_SEG2    LCD SEG23                        P2, pin 14
        9/LCD_SEG41/
        LCD_COM5
  PC12  USART3_CK/LCD_SEG3    ---                              P2, pin 13
        0/LCD_SEG42/
        LCD_COM6
  PC13  RTC_AF1/WKUP2 2 CNT_  IDD CNT_EN                       P1, pin 4
        EN 4
  PC14  OSC32_IN 3 OSC32_IN   OSC32_IN                         P1, pin 5
  PC15  OSC32_OUT 4 OSC32_OUT OSC32_OUT                        P1, pin 6
  ----- --------------------- -------------------------------- ----------------
  PD2   TIM3_ETR/LCD_SEG31/   ---                              P2, pin 12
        LCD_SEG43/LCD_COM7
  ----- --------------------- -------------------------------- ----------------

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

    CONFIG_STM32_CODESOURCERYW=y  : CodeSourcery under Windows
    CONFIG_STM32_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_STM32_ATOLLIC_LITE=y   : The free, "Lite" version of Atollic toolchain under Windows
    CONFIG_STM32_ATOLLIC_PRO=y    : The paid, "Pro" version of Atollic toolchain under Windows
    CONFIG_STM32_DEVKITARM=y      : devkitARM under Windows
    CONFIG_STM32_RAISONANCE=y     : Raisonance RIDE7 under Windows
    CONFIG_STM32_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin (default)

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
     ./configure.sh stm32ldiscovery/<sub-dir>

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

  The STM32L-Discovery board has four LEDs.  Two of these are controlled by
  logic on the board and are not available for software control:

    LD1 COM:   LD2 default status is red. LD2 turns to green to indicate
               that communications are in progress between the PC and the
               ST-LINK/V2.
    LD2 PWR:   Red LED indicates that the board is powered.

  And two LEDs can be controlled by software:

    User LD3:  Green LED is a user LED connected to the I/O PB7 of the
               STM32L152 MCU.
    User LD4:  Blue LED is a user LED connected to the I/O PB6 of the
               STM32L152 MCU.

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/up_leds.c. The LEDs are used to encode OS-related
  events as follows:

    SYMBOL                Meaning                 LED state
                                                    LED3     LED4
    -------------------  -----------------------  -------- --------
    LED_STARTED          NuttX has been started     OFF      OFF
    LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF
    LED_IRQSENABLED      Interrupts enabled         OFF      OFF
    LED_STACKCREATED     Idle stack created         ON       OFF
    LED_INIRQ            In an interrupt              No change
    LED_SIGNAL           In a signal handler          No change
    LED_ASSERTION        An assertion failed          No change
    LED_PANIC            The system has crashed     OFF      Blinking
    LED_IDLE             STM32 is is sleep mode       Not used

Serial Console
==============

  The STM32L-Discovery has no on-board RS-232 driver.  Further, there are no
  USART pins that do not conflict with the on board resources, in particular,
  the LCD:  Most USART pins are available if the LCD is enabled; USART2 may
  be used if either the LCD or the on-board LEDs are disabled.

    PA9   USART1_TX  LCD glass COM1  P2, pin 22
    PA10  USART1_RX  LCD glass COM2  P2, pin 21
    PB6   USART1_TX  LED Blue        P2, pin 8
    PB7   USART1_RX  LED Green       P2, pin 7

    PA2   USART2_TX  LCD SEG1        P1, pin 17
    PA3   USART2_RX  LCD SEG2        P1, pin 18

    PB10  USART3_TX LCD SEG6         P1, pin 22
    PB11  USART3_RX LCD SEG7         P1, pin 23
    PC10  USART3_TX LCD SEG22        P2, pin 15
    PC11  USART3_RX LCD SEG23        P2, pin 14

  NOTES:

  - GND and (external) 5V are available on both P1 and P2.  Note:  These
    signals may be at lower voltage levels and, hence, may not properly
    drive an external RS-232 transceiver.

  - The crystal X3 is not installed on the STM32L3-Discovery.  As a result,
    the HSE clock is not availabled and the less acurate HSI must be used.
    This may limit the accuracy of the computed baud, especially at higher
    BAUD.  The HSI is supposedly calibrated in the factory to within 1% at
    room temperatures so perhaps this not a issue.

  - According to the STM32L-Discovery User Manual, the LCD should be removed
    from its socket if you use any of the LCD pins for any other purpose.

    I have had no problems using the USART1 with PA9 and PA10 with a 3.3-5V
    RS-232 transceiver module at 57600 baud.  I have not tried higher baud
    rates.

  - There is no support for a USB serial connector on the STM32L-Discovery
    board.  The STM32L152 does support USB, but the USB pins are "free I/O"
    on the board and no USB connector is provided. So the use of a USB
    console is not option.  If you need console output, you will need to
    disable either LCD (and use any USART) or the LEDs (and use USART1)

Debugging
=========

  STM32 ST-LINK Utility
  ---------------------
  For simply writing to FLASH, I use the STM32 ST-LINK Utility.  At least
  version 2.4.0 is required (older versions do not recognize the STM32 F3
  device).  This utility is available from free from the STMicro website.

  Debugging
  ---------
  If you are going to use a debugger, you should make sure that the following
  settings are selection in your configuration file:

    CONFIG_DEBUG_SYMBOLS=y     : Enable debug symbols in the build
    CONFIG_ARMV7M_USEBASEPRI=y : Use the BASEPRI register to disable interrupts

  OpenOCD
  -------
  I am told that OpenOCD will work with the ST-Link, but I have never tried
  it.

  https://github.com/texane/stlink
  --------------------------------
  This is an open source server for the ST-Link that I have never used.

  Atollic GDB Server
  ------------------
  You can use the Atollic IDE, but I have never done that either.

STM32L-Discovery-specific Configuration Options
===============================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM4=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=stm32

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_STM32L152RB=y

    CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
       configuration features.

       CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=stm32fldiscovery (for the STM32L-Discovery development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_STM32FLDISCOVERY=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_DRAM_SIZE=16384 (16Kb)

    CONFIG_DRAM_START - The start address of installed DRAM

       CONFIG_DRAM_START=0x20000000

    CONFIG_STM32_CCMEXCLUDE - Exclude CCM SRAM from the HEAP

    CONFIG_ARCH_IRQPRIO - The STM32L-Discovery supports interrupt prioritization

       CONFIG_ARCH_IRQPRIO=y

    CONFIG_ARCH_FPU - The STM32L-Discovery does not support a floating point unit (FPU)

       CONFIG_ARCH_FPU=n

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
    ----
    (GPIOs are always enabled)
    CONFIG_STM32_FLITF
    CONFIG_STM32_DMA1
    CONFIG_STM32_DMA2

    APB2
    ----
    CONFIG_STM32_SYSCFG
    CONFIG_STM32_TIM9
    CONFIG_STM32_TIM10
    CONFIG_STM32_TIM11
    CONFIG_STM32_ADC1
    CONFIG_STM32_SPI1
    CONFIG_STM32_USART1

    APB1
    ----
    CONFIG_STM32_TIM2
    CONFIG_STM32_TIM3
    CONFIG_STM32_TIM4
    CONFIG_STM32_TIM5
    CONFIG_STM32_TIM6
    CONFIG_STM32_TIM7
    CONFIG_STM32_LCD
    CONFIG_STM32_WWDG
    CONFIG_STM32_IWDG
    CONFIG_STM32_SPI2
    CONFIG_STM32_SPI3
    CONFIG_STM32_USART2
    CONFIG_STM32_USART3
    CONFIG_STM32_I2C1
    CONFIG_STM32_I2C2
    CONFIG_STM32_USB
    CONFIG_STM32_PWR -- Required for RTC
    CONFIG_STM32_DAC1
    CONFIG_STM32_COMP

  Timer devices may be used for different purposes.  One special purpose is
  to generate modulated outputs for such things as motor control.  If CONFIG_STM32_TIMn
  is defined (as above) then the following may also be defined to indicate that
  the timer is intended to be used for pulsed output modulation, ADC conversion,
  or DAC conversion. Note that ADC/DAC require two definition:  Not only do you have
  to assign the timer (n) for used by the ADC or DAC, but then you also have to
  configure which ADC or DAC (m) it is assigned to.

    CONFIG_STM32_TIMn_PWM   Reserve timer n for use by PWM, n=1,..,14
    CONFIG_STM32_TIMn_ADC   Reserve timer n for use by ADC, n=1,..,14
    CONFIG_STM32_TIMn_ADCm  Reserve timer n to trigger ADCm, n=1,..,14, m=1,..,3
    CONFIG_STM32_TIMn_DAC   Reserve timer n for use by DAC, n=1,..,14
    CONFIG_STM32_TIMn_DACm  Reserve timer n to trigger DACm, n=1,..,14, m=1,..,2

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

  STM32L-Discovery specific device driver settings

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

  STM32L-Discovery CAN Configuration

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

  STM32L-Discovery SPI Configuration

    CONFIG_STM32_SPI_INTERRUPTS - Select to enable interrupt driven SPI
      support. Non-interrupt-driven, poll-waiting is recommended if the
      interrupt rate would be to high in the interrupt driven case.
    CONFIG_STM32_SPI_DMA - Use DMA to improve SPI transfer performance.
      Cannot be used with CONFIG_STM32_SPI_INTERRUPT.

Configurations
==============

  Each STM32L-Discovery configuration is maintained in a sub-directory and
  can be selected as follow:

    cd tools
    ./configure.sh STM32L-Discovery/<subdir>
    cd -
    . ./setenv.sh

  If this is a Windows native build, then configure.bat should be used
  instead of configure.sh:

    configure.bat STM32L-Discovery\<subdir>

  Where <subdir> is one of the following sub-directories.

  NOTE:  These configurations use the mconf-based configuration tool.  To
  change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       and misc/tools/

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

Configuration sub-directories
-----------------------------

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.

    NOTES:

    1. The serial console is on UART1 and NuttX LED support is enabled.
       Therefore, you will need an external RS232 driver or TTL serial-to-
       USB converter.  The UART1 TX and RX pins should be available on
       PA9 and PA10, respectively.

       The serial console is configured for 57600 8N1 by default.

    2. Support for NSH built-in applications is *not* enabled.

    3. By default, this configuration uses the CodeSourcery toolchain
       for Windows and builds under Cygwin (or probably MSYS).  That
       can easily be reconfigured, of course.

       Build Setup:
         CONFIG_HOST_WINDOWS=y                   : Builds under Windows
         CONFIG_WINDOWS_CYGWIN=y                 : Using Cygwin

       System Type:
         CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y : CodeSourcery for Windows

    4. To enable SLCD support:

       Board Selection:
         CONFIG_ARCH_LEDS=n                      : Disable board LED support

       Library Routines:
         CONFIG_LIB_SLCDCODEC=y                  : Enable the SLCD CODEC

       System Type -> STM32 Peripheral Support:
         CONFIG_STM32_LCD=y                      : Enable the Segment LCD

       When the LCD is enabled and the LEDs are disabled, the USART1
       serial console will automaticall move to PB6 and PB7 (you will get
       a compilation error if you forget to disable the LEDs).

       SIGNAL FUNCTION   LED        CONNECTION
       ------ ---------- ---------- -----------
        PB6   USART1_TX  LED Blue   P2, pin 8
        PB7   USART1_RX  LED Green  P2, pin 7

       To enable apps/examples/slcd to test the SLCD:

       Binary Formats:
         CONFIG_BINFMT_DISABLE=n                 : Don't disable binary support
         CONFIG_BUILTIN=y                        : Enable support for built-in binaries

       Application Configuration -> NSH Library:
         CONFIG_NSH_BUILTIN_APPS=y               : Enable builtin apps in NSH
         CONFIG_NSH_ARCHINIT=y                   : Needed to initialize the SLCD

       Application Configuration -> Examples:
         CONFIG_EXAMPLES_SLCD=y                  : Enable apps/examples/slcd

       To enable LCD debug output:

       Device Drivers:
         CONFIG_LCD=y                            : (Needed to enable LCD debug)

       Build Setup -> Debug Options:
         CONFIG_DEBUG=y                          : Enable debug features
         CONFIG_DEBUG_VERBOSE=y                  : Enable LCD debug

       NOTE:  At this point in time, testing of the SLCD is very limited because
       there is not much in apps/examples/slcd.  Certainly there are more bugs
       to be found.  There are also many segment-encoded glyphs in stm32_lcd.c
       But there is a basically functional driver with a working test setup
       that can be extended if you want a fully functional SLCD driver.
