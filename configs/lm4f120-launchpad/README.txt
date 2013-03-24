README
^^^^^^

README for NuttX port to the Stellaris LM4F120 LaunchPad.  The Stellaris® LM4F120 LaunchPad Evaluation Board is a low-cost evaluation platform for ARM® Cortex™-M4F-based microcontrollers from Texas Instruments.

Contents
^^^^^^^^

  Stellaris LM4F120 LaunchPad
  On-Board GPIO Usage
  Development Environment
  GNU Toolchain Options
  IDEs
  NuttX EABI "buildroot" Toolchain
  NuttX OABI "buildroot" Toolchain
  NXFLAT Toolchain
  LEDs
  Serial Console
  USB Device Controller Functions
  Using OpenOCD and GDB with an FT2232 JTAG emulator
  LM4F120 LaunchPad Configuration Options
  Configurations

Stellaris LM4F120 LaunchPad
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Stellaris® LM4F120 LaunchPad Evaluation Kit offers these features:

o A Stellaris® LaunchPad Evaluation board (EK-LM4F120XL)
o On-board Stellaris® In-Circuit Debug Interface (ICDI)
o Programmable user buttons and an RGB LED for custom applications.
o USB Micro-B plug to USB-A plug cable

Features of the LM4F120H5QR Microcontroller

o 32-bit ARM® Cortex™-M4F 80-MHz processor core.
o On-chip memory, featuring 256 KB single-cycle Flash up to 40 MHz (a
  prefetch buffer improves performance above 40 MHz), 32 KB single-cycle
  SRAM; internal ROM loaded with StellarisWare® software; 2KB EEPROM
o Two Controller Area Network (CAN) modules, using CAN protocol version
  2.0 part A/B and with bit rates up to 1 Mbps
o Universal Serial Bus (USB) controller with USB 2.0 full-speed (12 Mbps)
  and low-speed (1.5 Mbps) operation, 32 endpoints, and USB OTG/Host/Device
  mode
o Advanced serial integration, featuring: eight UARTs with IrDA, 9-bit, and
  ISO 7816 support (one UART with modem status and modem flow control); four
  Synchronous Serial Interface (SSI) modules, supporting operation for
  Freescale SPI, MICROWIRE, or Texas Instruments synchronous serial
  interfaces; four Inter-Integrated Circuit (I2C) modules, providing
  Standard (100 Kbps) and Fast (400 Kbps) transmission and support for
  sending and receiving data as either a master or a slave
o ARM PrimeCell® 32-channel configurable µDMA controller, providing a way to
  offload data transfer tasks from the Cortex™-M4F processor, allowing for
  more efficient use of the processor and the available bus bandwidth
o Analog support, featuring: two 12-bit Analog-to-Digital Converters (ADC)
  with 12 analog input channels and a sample rate of one million
  samples/second; two analog comparators; 16 digital comparators; on-chip
  voltage regulator
o Advanced motion control, featuring: eight Pulse Width Modulation (PWM)
  generator blocks, each with one 16-bit counter, two PWM comparators, a
  PWM signal generator, a dead-band generator, and an interrupt/ADC-trigger
  selector; two PWM fault inputs to promote low-latency shutdown; two
  Quadrature Encoder Interface (QEI) modules, with position integrator to 
  rack encoder position and velocity capture using built-in timer
o Two ARM FiRM-compliant watchdog timers; six 32-bit general-purpose timers
  (up to twelve 16-bit); six wide 64-bit general-purpose timers (up to twelve
  32-bit); 12 16/32-bit and 12 32/64-bit Capture Compare PWM (CCP) pins
o Up to 43 GPIOs (depending on configuration), with programmable control for
  GPIO interrupts and pad configuration, and highly flexible pin muxing
o Lower-power battery-backed Hibernation module with Real-Time Clock
o Multiple clock sources for microcontroller system clock: Precision
  Oscillator (PIOSC), Main Oscillator (MOSC), 32.768-kHz external oscillator
  for the Hibernation Module, and Internal 30-kHz Oscillator
o Full-featured debug solution with debug access via JTAG and Serial Wire
  interfaces, and IEEE 1149.1-1990 compliant Test Access Port (TAP) controller
o Industrial-range (-40°C to 85°C) RoHS-compliant 64-pin LQFP

On-Board GPIO Usage
===================

PIN SIGNAL(S)                                LanchPad Function
--- ---------------------------------------- ---------------------------------------
 17 PA0/U0RX                                 DEBUG/VCOM, Virtual COM port receive
 18 PA1/U0TX                                 DEBUG/VCOM, Virtual COM port transmit
 19 PA2/SSIOCLK                              GPIO, J2 pin 10
 20 PA3/SSIOFSS                              GPIO, J2 pin 9
 21 PA4/SSIORX                               GPIO, J2 pin 8
 22 PA5/SSIOTX                               GPIO, J1 pin 8
 23 PA6/I2CLSCL                              GPIO, J1 pin 9 
 24 PA7/I2CLSDA                              GPIO, J1 pin 10

 45 PB0/T2CCP0/U1Rx                          GPIO, J1 pin 3
 46 PB1/T2CCP1/U1Tx                          GPIO, J1 pin 4
 47 PB2/I2C0SCL/T3CCP0                       GPIO, J2, pin 3
 48 PB3/I2C0SDA/T3CCP1                       GPIO, J4 pin 3
 58 PB4/AIN10/CAN0Rx/SSI2CLK/T1CCP0          GPIO, J1 pin 7
 57 PB5/AIN11/CAN0Tx/SSI2FSS/T1CCP1          GPIO, J1 pin 2
 01 PB6/SSI2RX/T0CCP0                        GPIO, J2 pin 7
 04 PB7/SSI2TX/T0CCP1                        GPIO, J2 pin 6

 52 PC0/SWCLK/T4CCP0/TCK                     DEBUG/VCOM
 51 PC1/SWDIO/T4CCP1/TMS                     DEBUG/VCOM
 50 PC2/T5CCP0/TDI                           DEBUG/VCOM
 49 PC3/SWO/T5CCP1/TDO                       DEBUG/VCOM
 16 PC4/C1-/U1RTS/U1RX/U4RX/WT0CCP0          GPIO, J4 pin 4
 15 PC5/C1+/U1CTS/U1TX/U4TX/WT0CCP1          GPIO, J4 pin 5
 14 PC6/C0+/U3RX/WT1CCP0                     GPIO, J4 pin 6
 13 PC7/C0-/U3TX/WT1CCP1                     GPIO, J4 pin 7

 61 PD0/AIN7/I2C3SCL/SSI1CLK/SSI3CLKWT2CCP0  Connects to PB6 via resistor, GPIO, J3 pin 3
 62 PD1/AIN6/I2C3SDA/SSI1Fss/SSI3Fss/WT2CCP1 Connects to PB7 via resistor, GPIO, J3 Pin 4
 63 PD2/AIN5/SSI1RX/SSI3RX/WT3CCP0           GPIO, J3 pin 5
 64 PD3/AIN4/SSI1TX/SSI3TX/WT3CCP1           GPIO, J3 pin 6
 43 PD4/U6RX/USB0DM/WT4CCP0                  USB_DM
 44 PD5/U6TX/USB0DP/WT4CCP1                  USB_DP
 53 PD6/U2RX/WT5CCP0                         GPIO, J4 pin 8
 10 PD7/NMI/U2TX/WT5CCP1                     +USB_VBUS, GPIO, J4 pin 9
                                             Used for VBUS detection when
                                             configured as a self-powered USB
                                             Device

 09 PE0/AIN3/U7RX                            GPIO, J2 pin 3
 08 PE1/AIN2/U7TX                            GPIO, J3 pin 7
 07 PE2/AIN1                                 GPIO, J3 pin 8
 06 PE3/AIN0                                 GPIO, J3 pin 9
 59 PE4/AIN9/CAN0RX/I2C2SCL/U5RX             GPIO, J1 pin 5
 60 PE5/AIN8/CAN0TX/I2C2SDA/U5TX             GPIO, J1 pin 6

 28 PF0/C0O/CAN0RX/NMI/SSI1RX/T0CCP0/U1RTS   USR_SW2 (Low when pressed), GPIO, J2 pin 4
 29 PF1/C1O/SSI1TX/T0CCP1/TRD1/U1CTS         LED_R, GPIO, J3 pin 10
 30 PF2/SSI1CLK/T1CCP0/TRD0                  LED_B, GPIO, J4 pin 1
 31 PF3/CAN0TX/SSI1FSS/T1CCP1/TRCLK          LED_G, GPIO, J4 pin 2
 05 PF4/T2CCP0                               USR_SW1 (Low when pressed), GPIO, J4 pin 10

Using OpenOCD and GDB with an FT2232 JTAG emulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  Building OpenOCD under Cygwin:

    Refer to configs/olimex-lpc1766stk/README.txt

  Installing OpenOCD in Linux:

      sudo apt-get install openocd

    As of this writing, there is no support for the lm4f120 in the package
    above. You will have to build openocd from its source (as of this writing
    the latest commit was b9b4bd1a6410ff1b2885d9c2abe16a4ae7cb885f):

      git clone http://git.code.sf.net/p/openocd/code openocd
      cd openocd

    Then, add the patches provided by http://openocd.zylin.com/922:

      git fetch http://openocd.zylin.com/openocd refs/changes/22/922/14 && git checkout FETCH_HEAD
      ./bootstrap
      ./configure --enable-maintainer-mode --enable-ti-icdi
      make 
      sudo make install

    For additional help, see http://processors.wiki.ti.com/index.php/Stellaris_Launchpad_with_OpenOCD_and_Linux

  Helper Scripts.

    I have been using the on-board In-Circuit Debug Interface (ICDI) interface.
    OpenOCD requires a configuration file.  I keep the one I used last here:
    
      configs/lm4f120-launchpad/tools/lm4f120-launchpad.cfg

    However, the "correct" configuration script to use with OpenOCD may
    change as the features of OpenOCD evolve.  So you should at least
    compare that lm4f120-launchpad.cfg file with configuration files in
    /usr/share/openocd/scripts.  As of this writing, the configuration
    files of interest were:

      /usr/local/share/openocd/scripts/board/ek-lm4f120xl.cfg
      /usr/local/share/openocd/scripts/interface/ti-icdi.cfg
      /usr/local/share/openocd/scripts/target/stellaris_icdi.cfg

    There is also a script on the tools/ directory that I use to start
    the OpenOCD daemon on my system called oocd.sh.  That script will
    probably require some modifications to work in another environment:
  
    - Possibly the value of OPENOCD_PATH and TARGET_PATH
    - It assumes that the correct script to use is the one at
      configs/lm4f120-launchpad/tools/lm4f120-launchpad.cfg

  Starting OpenOCD

    If you are in the top-level NuttX build directlory then you should
    be able to start the OpenOCD daemon like:

      oocd.sh $PWD

    The relative path to the oocd.sh script is configs/lm4f120-launchpad/tools,
    but that should have been added to your PATH variable when you sourced
    the setenv.sh script.
    
    Note that OpenOCD needs to be run with administrator privileges in
    some environments (sudo).

  Connecting GDB

    Once the OpenOCD daemon has been started, you can connect to it via
    GDB using the following GDB command:

      arm-nuttx-elf-gdb
      (gdb) target remote localhost:3333

    NOTE:  The name of your GDB program may differ.  For example, with the
    CodeSourcery toolchain, the ARM GDB would be called arm-none-eabi-gdb.

    After starting GDB, you can load the NuttX ELF file:

      (gdb) symbol-file nuttx
      (gdb) monitor reset
      (gdb) monitor halt
      (gdb) load nuttx

    NOTES:
    1. Loading the symbol-file is only useful if you have built NuttX to
       include debug symbols (by setting CONFIG_DEBUG_SYMBOLS=y in the
       .config file).
    2. The MCU must be halted prior to loading code using 'mon reset'
       as described below.
 
    OpenOCD will support several special 'monitor' commands.  These
    GDB commands will send comments to the OpenOCD monitor.  Here
    are a couple that you will need to use:
  
     (gdb) monitor reset
     (gdb) monitor halt

    NOTES:
    1. The MCU must be halted using 'mon halt' prior to loading code.
    2. Reset will restart the processor after loading code.
    3. The 'monitor' command can be abbreviated as just 'mon'.

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

  1. The NuttX buildroot Toolchain (default, see below),
  2. The CodeSourcery GNU toolchain,
  3. The devkitARM GNU toolchain,
  4. The Atollic toolchain, or
  5. The Code Red toolchain

  All testing has been conducted using the Buildroot toolchain for Cygwin/Linux.
  To use a different toolchain, you simply need to add one of the following
  configuration options to your .config (or defconfig) file:

    CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin (default)
    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y  : CodeSourcery under Windows or Cygwin
    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_ARMV7M_TOOLCHAIN_DEVKITARM=y      : The Atollic toolchain under Windows or Cygwin
    CONFIG_ARMV7M_TOOLCHAIN_CODEREDW=y       : The Code Red toolchain under Windows
    CONFIG_ARMV7M_TOOLCHAIN_CODEREDL=y       : The Code Red toolchain under Linux

    CONFIG_ARMV7M_OABI_TOOLCHAIN=y           : If you use an older, OABI buildroot toolchain

  If you change the default toolchain, then you may also have to modify the PATH in
  the setenv.h file if your make cannot find the tools.

  NOTE: the CodeSourcery (for Windows), Atollic, devkitARM, and Code Red (for Windows)
  toolchains are Windows native toolchains.  The CodeSourcey (for Linux) and NuttX
  buildroot toolchains are Cygwin and/or Linux native toolchains. There are several
  limitations to using a Windows based toolchain in a Cygwin environment.  The three
  biggest are:

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
  3) Set up include pathes:  You will need include/, arch/arm/src/lm,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/lm/lm_vectors.S.

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
     ./configure.sh lm4f120-launchpad/<sub-dir>

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
  The LM4F120 has a single RGB LED.  If CONFIG_ARCH_LEDS is defined, then
  support for the LaunchPad LEDs will be included in the build.  See:

  - configs/lm4f120-launchpad/include/board.h - Defines LED constants, types and
    prototypes the LED interface functions.

  - configs/lm4f120-launchpad/src/lm4f120-launchpad.h - GPIO settings for the LEDs.

  - configs/lm4f120-launchpad/src/up_leds.c - LED control logic.

  OFF:
  - OFF means that the OS is still initializing. Initialization is very fast so
    if you see this at all, it probably means that the system is hanging up
    somewhere in the initialization phases.

  GREEN or GREEN-ish
  - This means that the OS completed initialization.

  Bluish:
  - Whenever and interrupt or signal handler is entered, the BLUE LED is
    illuminated and extinguished when the interrupt or signal handler exits.
    This will add a BLUE-ish tinge to the LED.

  Redish:
  - If a recovered assertion occurs, the RED component will be illuminated
    briefly while the assertion is handled.  You will probably never see this.

  Flashing RED:
  - In the event of a fatal crash, the BLUE and GREEN components will be
    extinguished and the RED component will FLASH at a 2Hz rate.

Serial Console
^^^^^^^^^^^^^^

  By default, all configurations use UART0 which connects to the USB VCOM
  on the DEBUG port on the LM4F120 LaunchPad:

    UART0 RX - PA.0
    UART0 TX - PA.1

  However, if you use an external RS232 driver, then other options are
  available.  UART1 has option pin settings and flow control capabilities
  that are not available with the other UARTS::

    UART1 RX - PB.0 or PC.4 (Need disambiguation in board.h)
    UART1 TX - PB.1 or PC.5 ("  " "            " "" "     ")

    UART1_RTS - PF.0 or PC.4
    UART1_CTS - PF.1 or PC.5

  NOTE: board.h currently selects PB.0, PB.1, PF.0 and PF.1 for UART1, but
  that can be changed by editting board.h

  UART2-5, 7 are also available, UART2 is not recommended because it shares
  some pin usage with USB device mode.  UART6 is not available because its
  only RX/TX pin options are dedicated to USB support.

    UART2 RX - PD.6
    UART2 TX - PD.7 (Also used for USB VBUS detection)

    UART3 RX - PC.6
    UART3 TX - PC.7

    UART4 RX - PC.4
    UART4 TX - PC.5

    UART5 RX - PE.4
    UART5 TX - PE.5

    UART6 RX - PD.4, Not available.  Dedicated for USB_DM
    UART6 TX - PD.5, Not available.  Dedicated for USB_DP

    UART7 RX - PE.0
    UART7 TX - PE.1

USB Device Controller Functions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  Device Overview

    An FT2232 device from Future Technology Devices International Ltd manages
    USB-to-serial conversion. The FT2232 is factory configured by Luminary
    Micro to implement a JTAG/SWD port (synchronous serial) on channel A and
    a Virtual COM Port (VCP) on channel B. This feature allows two simultaneous
    communications links between the host computer and the target device using
    a single USB cable. Separate Windows drivers for each function are provided
    on the Documentation and Software CD.

  Debugging with JTAG/SWD

    The FT2232 USB device performs JTAG/SWD serial operations under the control 
    of the debugger or the Luminary Flash Programmer.  It also operate as an
    In-Circuit Debugger Interface (ICDI), allowing debugging of any external
    target board.  Debugging modes:

    MODE DEBUG FUNCTION       USE                         SELECTED BY
    1    Internal ICDI        Debug on-board LM4F120     Default Mode
                              microcontroller over USB
                              interface.
    2    ICDI out to JTAG/SWD The EVB is used as a USB    Connecting to an external
         header               to SWD/JTAG interface to    target and starting debug
                              an external target.         software. The red Debug Out
                                                          LED will be ON.
    3    In from JTAG/SWD     For users who prefer an     Connecting an external
         header               external debug interface    debugger to the JTAG/SWD
                              (ULINK, JLINK, etc.) with   header.
                              the EVB.

  Virtual COM Port

    The Virtual COM Port (VCP) allows Windows applications (such as HyperTerminal)
    to communicate with UART0 on the LM4F120 over USB. Once the FT2232 VCP
    driver is installed, Windows assigns a COM port number to the VCP channel.

LM4F120 LaunchPad Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM3=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=lm

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_LM4F120

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=lm4f120-launchpad (for the LM4F120 LaunchPad)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_LM4F120_LAUNCHPAD

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_DRAM_SIZE=0x00010000 (64Kb)

    CONFIG_DRAM_START - The start address of installed DRAM

       CONFIG_DRAM_START=0x20000000

    CONFIG_ARCH_IRQPRIO - The LM4F120 supports interrupt prioritization

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

  There are configurations for disabling support for interrupts GPIO ports.
  GPIOJ must be disabled because it does not exist on the LM4F120.
  Additional interrupt support can be disabled if desired to reduce memory
  footprint.

    CONFIG_LM_DISABLE_GPIOA_IRQS=n
    CONFIG_LM_DISABLE_GPIOB_IRQS=n
    CONFIG_LM_DISABLE_GPIOC_IRQS=n
    CONFIG_LM_DISABLE_GPIOD_IRQS=n
    CONFIG_LM_DISABLE_GPIOE_IRQS=n
    CONFIG_LM_DISABLE_GPIOF_IRQS=n
    CONFIG_LM_DISABLE_GPIOG_IRQS=n
    CONFIG_LM_DISABLE_GPIOH_IRQS=n
    CONFIG_LM_DISABLE_GPIOJ_IRQS=y
 
  LM4F120 specific device driver settings

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

    CONFIG_SSI0_DISABLE - Select to disable support for SSI0
    CONFIG_SSI1_DISABLE - Select to disable support for SSI1
    CONFIG_SSI_POLLWAIT - Select to disable interrupt driven SSI support.
      Poll-waiting is recommended if the interrupt rate would be to
      high in the interrupt driven case.
    CONFIG_SSI_TXLIMIT - Write this many words to the Tx FIFO before
      emptying the Rx FIFO.  If the SPI frequency is high and this
      value is large, then larger values of this setting may cause
      Rx FIFO overrun errors.  Default: half of the Tx FIFO size (4).

    CONFIG_LM_ETHERNET - This must be set (along with CONFIG_NET)
      to build the Stellaris Ethernet driver
    CONFIG_LM_ETHLEDS - Enable to use Ethernet LEDs on the board.
    CONFIG_LM_BOARDMAC - If the board-specific logic can provide
      a MAC address (via lm_ethernetmac()), then this should be selected.
    CONFIG_LM_ETHHDUPLEX - Set to force half duplex operation
    CONFIG_LM_ETHNOAUTOCRC - Set to suppress auto-CRC generation
    CONFIG_LM_ETHNOPAD - Set to suppress Tx padding
    CONFIG_LM_MULTICAST - Set to enable multicast frames
    CONFIG_LM_PROMISCUOUS - Set to enable promiscuous mode
    CONFIG_LM_BADCRC - Set to enable bad CRC rejection.
    CONFIG_LM_DUMPPACKET - Dump each packet received/sent to the console.

Configurations
^^^^^^^^^^^^^^

Each LM4F120 LaunchPad configuration is maintained in a
sub-directory and can be selected as follow:

    cd tools
    ./configure.sh lm4f120-launchpad/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    configuration enables the serial VCOM interfaces on UART0.  Support for
    builtin applications is enabled, but in the base configuration no
    builtin applications are selected.

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

       CONFIG_HOST_LINUX=y                 : Linux (Cygwin under Windows okay too).
       CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y : Buildroot (arm-nuttx-elf-gcc)
       CONFIG_RAW_BINARY=y                 : Output formats: ELF and raw binary

  ostest:
    This configuration directory, performs a simple OS test using
    examples/ostest. The configuration enables the serial VCOM interfaces
    on UART0. 

    NOTES:
    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Default platform/toolchain:

       CONFIG_HOST_LINUX=y                 : Linux (Cygwin under Windows okay too).
       CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y : Buildroot (arm-nuttx-elf-gcc)
       CONFIG_RAW_BINARY=y                 : Output formats: ELF and raw binary
