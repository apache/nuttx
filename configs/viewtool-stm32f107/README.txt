README
======

  This README discusses issues unique to NuttX configurations for the
  ViewTool STM32F103/F107 V1.2 board.  This board may be fitted with either

    - STM32F107VCT6, or
    - STM32F103VCT6

  The board is vary modular with connectors for a variety of peripherals.
  Features on the base board include:

    - User and Wake-Up Keys
    - LEDs

  See http://www.viewtool.com/ for further information.

Contents
========

  o User and Wake-Up keys
  o LEDs
  o Serial Console
    - Console Configuration
    - J5 - USART1
    - PL-2013 USB-to-Serial Interface
    - RS-232 Module
  o Toolchains
    - NOTE about Windows native toolchains
  o Configurations
    - Information Common to All Configurations
    - Configuration Sub-directories

User and Wake-Up keys
=====================

  All pulled high and will be sensed low when depressed.

    SW2 PC11  Needs J42 closed
    SW3 PC12  Needs J43 closed
    SW4 PA0   Needs J44 closed

LEDs
====

  There are four LEDs on the ViewTool STM32F103/F107 board that can be controlled
  by software:  LED1 through LED4.  All pulled high and can be illuminated by
  driving the output to low

    LED1 PA6
    LED2 PA7
    LED3 PB12
    LED4 PB13

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/stm32_leds.c. The LEDs are used to encode OS-related
  events as follows:

    SYMBOL            Meaning                      LED state
                                               LED1 LED2 LED3 LED4
    ----------------- -----------------------  ---- ---- ---- ----
    LED_STARTED       NuttX has been started   ON   OFF  OFF  OFF
    LED_HEAPALLOCATE  Heap has been allocated  OFF  ON   OFF  OFF
    LED_IRQSENABLED   Interrupts enabled       ON   ON   OFF  OFF
    LED_STACKCREATED  Idle stack created       OFF  OFF  ON   OFF
    LED_INIRQ         In an interrupt          N/C  N/C  N/C  Soft glow
    LED_SIGNAL        In a signal handler      N/C  N/C  N/C  Soft glow
    LED_ASSERTION     An assertion failed      N/C  N/C  N/C  Soft glow
    LED_PANIC         The system has crashed   N/C  N/C  N/C  2Hz Flashing
    LED_IDLE          MCU is is sleep mode         Not used

  After booting, LED1-3 are not longer used by the system and can be used for
  other purposes by the application (Of course, all LEDs are available to the
  application if CONFIG_ARCH_LEDS is not defined.

Serial Console
==============

  Console Configuration
  ---------------------
  The NuttX console is configured by default on USART1 at 115200 BAUD 8N1
  (8-bits, not parity, one stop bit).  These setting can, of course, easily
  be changed by reconfiguring NuttX.

  J5 - USART1
  -----------
  The boards come with a PL-2303 based USB-to-serial board.  Also available
  as an option is an RS-232 board.  Both have the same pin out on a 6-pin
  connector that mates with the upper row of J5.

    PIN MODULE BOARD J5
    --- ------ ---------------------------
     1   5V    1  POWER Power jumper
     2   GND   3  GND   Ground
     3   TXD   5  RXD1  PA10    USART1_RXD
     4   RXD   7  TXD1  PA9     USART1_TXD
     5   RTS?  9  CTS?  PA12    USART1_RTS
     6   CTS?  11 RTS?  PA11    USART1_CTS

  PL-2013 USB-to-Serial Interface
  -------------------------------

    J37 - CON4.  Jumper Settings:
      1 <-> 3 : Connects PA9 to the RXD1 output pin
      2 <-> 4 : Connects PA10 to the TXD1 input pin

    J35 - CON2.  Jumper Setting:
      Open.  the PL2303 adapter receives its power from the USB host.

  RS-232 Module
  -------------

    J37 - CON4.  Jumper Settings:
      1 <-> 3 : Connects PA9 to the RXD1 output pin
      2 <-> 4 : Connects PA10 to the TXD1 input pin

    J35 - CON2.  Jumper Setting:
      1 <-> 2 : Proves 3.3V to the RS-232 module.

Toolchains
==========

  NOTE about Windows native toolchains
  ------------------------------------

  There are several limitations to using a Windows based toolchain in a
  Cygwin environment.  The three biggest are:

  1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
     performed automatically in the Cygwin makefiles using the 'cygpath'
     utility but you might easily find some new path problems.  If so, check
     out 'cygpath -w'

  2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic
     links are used in Nuttx (e.g., include/arch).  The make system works
     around these problems for the Windows tools by copying directories
     instead of linking them.  But this can also cause some confusion for
     you:  For example, you may edit a file in a "linked" directory and find
     that your changes had no effect.  That is because you are building the
     copy of the file in the "fake" symbolic directory.  If you use a\
     Windows toolchain, you should get in the habit of making like this:

       make clean_context all

     An alias in your .bashrc file might make that less painful.

  3. Dependencies are not made when using Windows versions of the GCC.  This is
     because the dependencies are generated using Windows pathes which do not
     work with the Cygwin make.

       MKDEP                = $(TOPDIR)/tools/mknulldeps.sh

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each SAM3U-EK configuration is maintained in a sub-directory and
  can be selected as follow:

    cd tools
    ./configure.sh viewtool-stm32f107/<subdir>
    cd -
    . ./setenv.sh

  Before sourcing the setenv.sh file above, you should examine it and perform
  edits as necessary so that TOOLCHAIN_BIN is the correct path to the directory
  than holds your toolchain binaries.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

    make

  The <subdir> that is provided above as an argument to the tools/configure.sh
  must be is one of the following.

  NOTES:

  1. These configurations use the mconf-based configuration tool.  To
    change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       and misc/tools/

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output on USART1.

  3. Unless otherwise stated, the configurations are setup for
     Cygwin under Windows:

     Build Setup:
       CONFIG_HOST_WINDOWS=y                   : Windows operating system
       CONFIG_WINDOWS_CYGWIN=y                 : POSIX environment under windows

  4. All of these configurations use the CodeSourcery for Windows toolchain
     (unless stated otherwise in the description of the configuration).  That
     toolchain selection can easily be reconfigured using 'make menuconfig'.
     Here are the relevant current settings:

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y : GNU EABI toolchain for windows

     The setenv.sh file is available for you to use to set the PATH
     variable.  The path in the that file may not, however, be correct
     for your installation.

     See also the "NOTE about Windows native toolchains" in the section call
     "GNU Toolchain Options" above.

  4. These configurations all assume that the STM32F107VCT6 is mounted on
     board.  This is configurable; you can select the STM32F103VCT6 as an
     alternative.

  5. These configurations all assume that you are loading code using
     something like the ST-Link v2 JTAG.  None of these configurations are
     setup to use the DFU bootloader but should be easily reconfigured to
     use that bootloader is so desired.

  Configuration Sub-directories
  -----------------------------

  nsh:

    This configuration directory provide the basuic NuttShell (NSH).

    NOTES:
    1. This configuration uses the default USART1 serial console.  That
       is easily changed by reconfiguring to (1) enable a different
       serial peripheral, and (2) selecting that serial peripheral as
       the console device.

    2. By default, this configuration is set up to build on Windows
       under either a Cygwin or MSYS environment using a recent, Windows-
       native, generic ARM EABI GCC toolchain (such as the CodeSourcery
       toolchain).  Both the build environment and the toolchain
       selection can easily be changed by reconfiguring:

       CONFIG_HOST_WINDOWS=y                   : Windows operating system
       CONFIG_WINDOWS_CYGWIN=y                 : POSIX environment under windows
       CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y : CodeSourcery for Windows
