README
======

This README discusses issues unique to NuttX configurations for the ST
Nucleo-l432kc board from ST Micro.  See

  http://www.st.com/nucleo-l432kc

NucleoL432KC:

  Microprocessor: 32-bit ARM Cortex M4 at 80MHz STM32L432KCU6
  Memory:         256 KB Flash and 64 KB SRAM
  ADC:            1×12-bit, 5 MSPS A/D converter: up to 10 channels
  DMA:            16-stream DMA controllers with FIFOs and burst support
  Timers:         Up to 11 timers: up to five 16-bit, one 32-bit, two low-power
                  16 bit timers, two watchdog timers, and a SysTick timer
  GPIO:           Up to 26 I/O ports with interrupt capability, most 5v tolerant
  I2C:            Up to 2 × I2C interfaces
  USARTs:         Up to 3 USARTs, 2 UARTs, 1 LPUART
  SPIs:           Up to 2 SPIs
  SAIs:           1 dual-channel audio interface
  CAN interface
  QSPI interface
  USB:            USB 2.0 full-speed device/host/OTG controller with on-chip PHY
  CRC calculation unit
  RTC

Board features:

  Peripherals:    1 led
  Debug:          Serial wire debug and JTAG interfaces via on-board micro-usb stlink v2.1
  Expansion I/F   Arduino Nano Headers

  Uses a STM32F103 to provide a ST-Link for programming, debug similar to the
  OpenOcd FTDI function - USB to JTAG front-end.

  See http://mbed.org/platforms/ST-Nucleo-L432KC for more
  information about these boards.

Contents
========

  - Nucleo-32 Boards
  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - Hardware
    - Button
    - LED
    - USARTs and Serial Consoles
  - QFN32
  - mbed
  - SPI Flash support
  - Configurations

Nucleo-32 Boards
================

The Nucleo-L432KC is a member of the Nucleo-64 board family.  The Nucleo-64
is a standard board for use with several STM32 parts in the LQFP64 package.
Variants include

  Order code    Targeted STM32
  ------------- --------------
  NUCLEO-F031K6 STM32F031K6T6
  NUCLEO-F042K6 STM32F042K6T6
  NUCLEO-F303K8 STM32F303K8T6
  NUCLEO-L011K4 STM32L011K4T6
  NUCLEO-L031K6 STM32L031K6T6
  NUCLEO-L432KC STM32L432KCU6

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

  1. The NuttX buildroot Toolchain (see below), or
  2. Any generic arm-none-eabi GNU toolchain.

  All testing has been conducted using the NuttX CodeSourcery toolchain.  To use
  a different toolchain, you simply need to modify the configuration.  As an
  example:

    CONFIG_ARM_TOOLCHAIN_GNU_EABI : Generic arm-none-eabi toolchain

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
    Download and install the latest version (as of this writing it was
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
  3) Set up include paths:  You will need include/, arch/arm/src/stm32,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/stm32/stm32_vectors.S.  With RIDE, I have to build NuttX
  one time from the Cygwin command line in order to obtain the pre-built
  startup object needed by RIDE.

NuttX EABI "buildroot" Toolchain
================================

  A GNU GCC-based toolchain is assumed.  The PATH environment variable should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  Bitbucket download site (https://bitbucket.org/nuttx/buildroot/downloads/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured NuttX in <some-dir>/nuttx.

     $ tools/configure.sh nucleo-l432kc:nsh
     $ make qconfig
     $ V=1 make context all 2>&1 | tee mout

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp boards/cortexm3-eabi-defconfig-4.6.3 .config

  6. make oldconfig

  7. make

  8. Make sure that the PATH variable includes the path to the newly built
     binaries.

  See the file boards/README.txt in the buildroot source tree.  That has more
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
  be downloaded from the NuttX Bitbucket download site
  (https://bitbucket.org/nuttx/nuttx/downloads/).

  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured NuttX in <some-dir>/nuttx.

     tools/configure.sh lpcxpresso-lpc1768:<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp boards/cortexm3-defconfig-nxflat .config

  6. make oldconfig

  7. make

  8. Make sure that the PATH variable includes the path to the newly built
     NXFLAT binaries.

mbed
====

  The Nucleo-L432KC includes boot loader from mbed:

    https://mbed.org/handbook/Homepage

  Using the mbed loader:

  1. Connect the Nucleo-L432kc to the host PC using the USB connector.
  2. A new file system will appear called NUCLEO; open it with Windows
     Explorer (assuming that you are using Windows).
  3. Drag and drop nuttx.bin into the MBED window.  This will load the
     nuttx.bin binary into the Nucleo-L432kc.  The NUCLEO window will
     close then re-open and the Nucleo-L432KC will be running the new code.

Hardware
========

  LEDs
  ----
  The Nucleo L432KC provides a single user LED, LD3.  LD3
  is the green LED connected to Arduino signal D13 corresponding to MCU I/O
  PB3 (pin 26).

    - When the I/O is HIGH value, the LED is on.
    - When the I/O is LOW, the LED is off.

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
  events as follows when the LED is available:

    SYMBOL                Meaning                   LD3
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

  Thus if LD3, NuttX has successfully booted and is, apparently, running
  normally.  If LD3 is flashing at approximately 2Hz, then a fatal error
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

    Nucleo CN10 STM32L432KC
    ----------- ------------
    Pin 21 PA9  USART1_RX   *Warning you make need to reverse RX/TX on
    Pin 33 PA10 USART1_TX    some RS-232 converters
    Pin 20 GND
    Pin 8  U5V

  To configure USART1 as the console:

    CONFIG_STM32_USART1=y
    CONFIG_USART1_SERIALDRIVER=y
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

    Nucleo CN9  STM32L432KC
    ----------- ------------
    Pin 1  PA3  USART2_RX   *Warning you make need to reverse RX/TX on
    Pin 2  PA2  USART2_TX    some RS-232 converters

  Solder Bridges.  This configuration requires:

  - SB62 and SB63 Closed: PA2 and PA3 on STM32 MCU are connected to D1 and D0
    (pin 7 and pin 8) on Arduino connector CN9 and ST Morpho connector CN10
    as USART signals.  Thus SB13 and SB14 should be OFF.

  - SB13 and SB14 Open:  PA2 and PA3 on STM32F103C8T6 (ST-LINK MCU) are
    disconnected to PA3 and PA2 on STM32 MCU.

  To configure USART2 as the console:

    CONFIG_STM32_USART2=y
    CONFIG_USART2_SERIALDRIVER=y
    CONFIG_USART2_SERIAL_CONSOLE=y
    CONFIG_USART2_RXBUFSIZE=256
    CONFIG_USART2_TXBUFSIZE=256
    CONFIG_USART2_BAUD=115200
    CONFIG_USART2_BITS=8
    CONFIG_USART2_PARITY=0
    CONFIG_USART2_2STOP=0

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

SPI Flash support:
=====================

  We can use an external SPI Serial Flash with nucleo-l432kc board. In this
  case we tested with AT45DB081D (8Mbit = 1MiB).

  You can connect the AT45DB081D memory in the nucleo-l432kc board this way:

  --------------------------------
  | Memory        nucleo-l432kc  |
  |------------------------------|
  | SI      --->  D11 (PB5)      |
  | SCK     --->  D13 (PB3)      |
  | /RESET  --->  3V3            |
  | /CS     --->  D10 (PA11)     |
  | /WP     --->  3V3            |
  | VCC     --->  3V3            |
  | GND     --->  GND            |
  | SO      --->  D12 (PB4)      |
  --------------------------------

  You can start with default "nucleo-l432kc/nsh" configuration option and
  enable/disable these options using "make menuconfig" :

  System Type  --->
      STM32L4 Peripheral Support  --->
          [*] SPI1

  Device Drivers  --->
      -*- Memory Technology Device (MTD) Support  --->
              -*-   SPI-based AT45DB flash
              (1000000) AT45DB Frequency

  File Systems  --->
      [*] NXFFS file system

  Then after compiling and flashing the file nuttx.bin you can test the flash
  this way:

  nsh> ls /mnt
  /mnt:
   at45db/

  nsh> echo "Testing" > /mnt/at45db/file.txt

  nsh> ls /mnt/at45db
  /mnt/at45db:
   file.txt

  nsh> cat /mnt/at45db/file.txt
  Testing

  nsh>

Configurations
==============

  nsh:
  ---------
    Configures the NuttShell (nsh) located at apps/examples/nsh for the
    Nucleo-L432KC board.  The Configuration enables the serial interfaces
    on UART2.  Support for builtin applications is enabled, but in the base
    configuration no builtin applications are selected (see NOTES below).

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. By default, this configuration uses the ARM EABI toolchain
       for Linux.  That can easily be reconfigured, of course.

       CONFIG_HOST_LINUX=y                     : Builds under Linux
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y      : GNU EABI toolchain for Linux

    3. Although the default console is USART2 (which would correspond to
       the Virtual COM port) I have done all testing with the console
       device configured for USART1 (see instruction above under "Serial
       Consoles).  I have been using a TTL-to-RS-232 converter connected
       as shown below:

       Nucleo CN10 STM32L432KC
       ----------- ------------
       Pin 21 PA9  USART1_RX   *Warning you make need to reverse RX/TX on
       Pin 33 PA10 USART1_TX    some RS-232 converters
       Pin 20 GND
       Pin 8  U5V

  spwm
  ----

    Configures the sinusoidal PWM (SPWM) example which presents a simple use case
    of the STM32L4 PWM lower-half driver without generic upper-half PWM logic.

    It uses TIM1 to generate PWM and TIM6 to change waveform samples

    At the moment, the waveform parameters are hardcoded, but it should be easy to
    modify this example and make it more functional.
