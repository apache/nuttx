README
======

This README discusses issues unique to NuttX configurations for the ST NucleoF401RE board
from ST Micro (http://www.st.com/web/catalog/mmc/FM141/SC1169/SS1577/LN1810/PF258797)


  Microprocessor: 32-bit ARM Cortex M4 at 84MHz STM32F104RE
  Memory:         512 KB Flash and 96 KB SRAM
  I/O Pins Out:   37, 17 On the Connector
  Network:        TI CC3000 Wifi Module
  ADCs:           1 (at 12-bit resolution)
  Peripherals:    10 timers, 2 I2Cs, 2 SPI ports, 3 USARTs, 1 led
  Other:          Sleep, stop, and standby modes; serial wire debug and JTAG interfaces
  Expansion I/F   Ardino and Morpho Headers

  Uses a STM32F103 to provide a ST-Link for programming, debug similar to the OpenOcd
  FTDI function - USB to JTAG front-end.

  Wireless WIFI + SD Card SDIO via a "CC3000 WiFi Arduino Shield" added card
  RS232 console support via a "RS232 Arduino Shield" added card

Contents
========

  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - Hardware
    - Button
    - LED
    - USARTs and Serial Consoles
  - LQFP64
  - mbed
  - Shields
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

  NOTE: There are several limitations to using a Windows based toolchain in a
  Cygwin environment.  The three biggest are:

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

       V=1 make clean_context all 2>&1 |tee mout

     An alias in your .bashrc file might make that less painful.

  3. Dependencies are not made when using Windows versions of the GCC.  This is
     because the dependencies are generated using Windows pathes which do not
     work with the Cygwin make.

       MKDEP = $(TOPDIR)/tools/mknulldeps.sh

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

     $ (cd tools; ./configure.sh nucleo-f401re/nsh)
     $ make qconfig
     $ V=1 make context all 2>&1 | tee mout

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
  use the GCC 4.6.3 EABI toolchain; instead use the GCC 4.3.3 EABI toolchain.

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

mbed
====

  The Nucleo-F401RE includes boot loader from mbed:

    https://mbed.org/platforms/ST-Nucleo-F401RE/
    https://mbed.org/handbook/Homepage

  Using the mbed loader:

  1. Connect the Nucleo-F401RE to the host PC using the USB connector.
  2. A new file system will appear called NUCLEO; open it with Windows
     Explorer (assuming that you are using Windows).
  3. Drag and drop nuttx.bin into the MBED window.  This will load the
     nuttx.bin binary into the Nucleo-F401RE.  The NUCLEO window will
     close then re-open and the Nucleo-F401RE will be running the new code.

Hardware
========

  GPIO
  ----
  SERIAL_TX=PA_2    USER_BUTTON=PC_13
  SERIAL_RX=PA_3    LED1       =PA_5

  A0=PA_0  USART2RX D0=PA_3            D8 =PA_9
  A1=PA_1  USART2TX D1=PA_2            D9 =PC_7
  A2=PA_4           D2=PA_10   WIFI_CS=D10=PB_6 SPI_CS
  A3=PB_0  WIFI_INT=D3=PB_3            D11=PA_7 SPI_MOSI
  A4=PC_1      SDCS=D4=PB_5            D12=PA_6 SPI_MISO
  A5=PC_0   WIFI_EN=D5=PB_4       LED1=D13=PA_5 SPI_SCK
               LED2=D6=PB_10  I2C1_SDA=D14=PB_9 Probe
                    D7=PA_8   I2C1_SCL=D15=PB_8 Probe

  From: https://mbed.org/platforms/ST-Nucleo-F401RE/

  Buttons
  -------
  B1 USER: the user button is connected to the I/O PC13 (pin 2) of the STM32
  microcontroller.

  LEDs
  ----
  The Nucleo F401RE and a single user LED, LD2.  LD2 is the green LED
  connected to Arduino signal D13 corresponding to MCU I/O PA5 (pin 21) or
  PB13 (pin 34) depending on the STM32target.

    - When the I/O is HIGH value, the LED is on.
    - When the I/O is LOW, the LED is off.

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
  events as follows when the red LED (PE24) is available:

    SYMBOL                Meaning                   LD2
    -------------------  -----------------------  -----------
    LED_STARTED          NuttX has been started     OFF
    LED_HEAPALLOCATE     Heap has been allocated    OFF
    LED_IRQSENABLED      Interrupts enabled         OFF
    LED_STACKCREATED     Idle stack created         ON
    LED_INIRQ            In an interrupt            No change
    LED_SIGNAL           In a signal handler        No change
    LED_ASSERTION        An assertion failed        No change
    LED_PANIC            The system has crashed     Blinking
    LED_IDLE             MCU is is sleep mode       Not used

  Thus if LD2, NuttX has successfully booted and is, apparently, running
  normally.  If LD2 is flashing at approximately 2Hz, then a fatal error
  has been detected and the system has halted.

Serial Consoles
===============

  USART1
  ------
  Pins and Connectors:

    RXD: PA11  CN10 pin 14
         PB7   CN7 pin 21
    TXD: PA10  CN9 pin 3, CN10 pin 33
         PB6   CN5 pin 3, CN10 pin 17

  NOTE:  You may need to edit the include/board.h to select different USART1
  pin selections.

  TTL to RS-232 converter connection:

    Nucleo CN10 STM32F401RE
    ----------- ------------
    Pin 21 PA9  USART2_RX
    Pin 33 PA10 USART2_TX
    Pin 20 GND
    Pin 8  U5V

  To configure USART1 as the console:

    CONFIG_STM32_USART1=y
    CONFIG_USART1_ISUART=y
    CONFIG_USART1_SERIAL_CONSOLE=y
    CONFIG_USART1_RXBUFSIZE=256
    CONFIG_USART1_TXBUFSIZE=256
    CONFIG_USART1_BAUD=115200
    CONFIG_USART1_BITS=8
    CONFIG_USART1_PARITY=0
    CONFIG_USART1_2STOP=0

  USART2
  -----
  Pins and Connectors:

    RXD: PA3   CN9 pin 1 (See SB13, 14, 62, 63). CN10 pin 37
         PD6
    TXD: PA2   CN9 pin 2(See SB13, 14, 62, 63). CN10 pin 35
         PD5

  UART2 is the default in all of these configurations.

  TTL to RS-232 converter connection:

    Nucleo CN9  STM32F401RE
    ----------- ------------
    Pin 1  PA3  USART2_RX
    Pin 2  PA2  USART2_TX

  Solder Bridges.  This configuration requires:

  - SB62 and SB63 Closed: PA2 and PA3 on STM32 MCU are connected to D1 and D0
    (pin 7 and pin 8) on Arduino connector CN9 and ST Morpho connector CN10
    as USART signals.  Thus SB13 and SB14 should be OFF.

  - SB13 and SB14 Open:  PA2 and PA3 on STM32F103C8T6 (ST-LINK MCU) are
    disconnected to PA3 and PA2 on STM32 MCU.

  To configure USART2 as the console:

    CONFIG_STM32_USART2=y
    CONFIG_USART2_ISUART=y
    CONFIG_USART2_SERIAL_CONSOLE=y
    CONFIG_USART2_RXBUFSIZE=256
    CONFIG_USART2_TXBUFSIZE=256
    CONFIG_USART2_BAUD=115200
    CONFIG_USART2_BITS=8
    CONFIG_USART2_PARITY=0
    CONFIG_USART2_2STOP=0

  USART6
  ------
  Pins and Connectors:

    RXD: PC7    CN5 pin2, CN10 pin 19
         PA12   CN10, pin 12
    TXD: PC6    CN10, pin 4
         PA11   CN10, pin 14

  To configure USART6 as the console:

    CONFIG_STM32_USART6=y
    CONFIG_USART6_ISUART=y
    CONFIG_USART6_SERIAL_CONSOLE=y
    CONFIG_USART6_RXBUFSIZE=256
    CONFIG_USART6_TXBUFSIZE=256
    CONFIG_USART6_BAUD=115200
    CONFIG_USART6_BITS=8
    CONFIG_USART6_PARITY=0
    CONFIG_USART6_2STOP=0

  Virtual COM Port
  ----------------
  Yet another option is to use UART2 and the USB virtual COM port.  This
  option may be more convenient for long term development, but is painful
  to use during board bring-up.

  Solder Bridges.  This configuration requires:

  - SB62 and SB63 Open: PA2 and PA3 on STM32 MCU are disconnected to D1
    and D0 (pin 7 and pin 8) on Arduino connector CN9 and ST Morpho
    connector CN10.

  - SB13 and SB14 Closed:  PA2 and PA3 on STM32F103C8T6 (ST-LINK MCU) are
    connected to PA3 and PA2 on STM32 MCU to have USART communication
    between them. Thus SB61, SB62 and SB63 should be OFF.

  Configuring USART2 is the same as given above.

  Question:  What BAUD should be configure to interface with the Virtual
  COM port?  115200 8N1?

  Default
  -------
  As shipped, SB62 and SB63 are open and SB13 and SB14 closed, so the
  virtual COM port is enabled.

Shields
=======

  1. RS-232 from Cutedigi.com.  Supports a single RS-232 connected via

       Nucleo CN9  STM32F401RE  Cutedigi
       ----------- ------------ --------
       Pin 1  PA3  USART2_RX    RXD
       Pin 2  PA2  USART2_TX    TXD

     Support for this shield is enabled by selecting USART2 and configuring
     SB13, 14, 62, and 63 as described above under "Serial Consoles"

  2. CC3000 Wireless shield

     Support this shield is enabled by configuring the CC3000 networking:

       CONFIG_WL_CC3000

Configurations
==============

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables the serial interfaces on UART2.  Support for
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
       for Linux.  That can easily be reconfigured, of course.

       CONFIG_HOST_LINUX=y                     : Builds under Linux
       CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYL=y : CodeSourcery for Linux

    3. Although the default console is USART2 (which would correspond to
       the Virtual COM port) I have done all testing with the console 
       device configured for USART1 (see instruction above under "Serial
       Consoles).  I have been using a TTL-to-RS-232 converted connected
       as shown below:

       Nucleo CN10 STM32F401RE
       ----------- ------------
       Pin 21 PA9  USART2_RX
       Pin 33 PA10 USART2_TX
       Pin 20 GND
       Pin 8  U5V

  cc3000:
  ------
    This configuration adds support for the CC3000 Shield.

    Build it with

      make distclean;(cd tools;./configure.sh nucleo-f401re/nsh)

    then run make menuconfig if you wish to customize things.

    or

    $ make qconfig

    You can use the scripts/cdc-acm.inf file to install the windows
    composite device.

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
