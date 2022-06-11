README
======

  This is a README file for the port of NuttX to the Teensy-3.1 from PJRC
  (https://www.pjrc.com/).  The Teensy-3.1 features the Freescale
  MK20DX256VLH7 chip (now NXP).  The MK20DX256VLH7 is a 64-pin Cortex-M4
  running at 72MHz.  It has 256KiB of program FLASH memory and 64KiB of
  SRAM.  For more information about the Teensy 3.1, see

    https://www.pjrc.com/teensy/teensy31.html
    https://www.pjrc.com/store/teensy31.html

  This board configuration can also be used with the older Teensy-3.0.  The
  Teensy-3.0 has the same schematic (although some pins are not used on the
  Teensy-3.0).  The primary difference is that the Teensy 3.0 has a
  MK20DX128VLH5 with slightly less capability.  There are many difference
  between the MK20DX256VLH7 and the MK20DX128VLH5 but the basic differences
  that effect how you configure NuttX are:

    --------------- -------------- -------------- ---------------------------
    Feature         Teensy 3.0     Teensy 3.1     Teensy 3.0 CONFIGURATION
    --------------- -------------- -------------- ---------------------------
    Processor
      Core          MK20DX128VLH5  MK20DX256VLH7  CONFIG_ARCH_CHIP_MK20DX128VLH5=y
      Rated Speed    48 MHz         72 MHz        See Note 1
      Overclockable  96 MHz         96 MHz        CONFIG_TEENSY_3X_OVERCLOCK=y
    Flash Memory    128 KB         256 KB         See Note 1
    SRAM             16 KB          64 KB         CONFIG_RAM_SIZE=16384 and
                                                  see Note 2
    --------------- -------------- -------------- ---------------------------

  NOTES:
  1. Settings in boards/arm/kinetis/teensy-3.x/include/board.h will automatically
     select the correct clocking based on CONFIG_ARCH_CHIP_MK20DX128VLH5=y.
  2. The linker script at boards/arm/kinetis/teensy-3.x/scripts/mk30dx128vlh5.ld will
     automatically be selected when CONFIG_ARCH_CHIP_MK20DX128VLH5=y.  It
     will use the correct FLASH and SRAM sizes.

  The initial Teensy-3.1 port is largely the effort of Jakob Odersky.
  https://github.com/jodersky/nuttx/tree/teensy31-7.6 and
  https://github.com/jodersky/px4-nuttx

Contents
========

  o STATUS
  o Pin Configuration
  o Serial Console
  o LEDs
  o Using the Halfkey Loader
  o Debugging
  o Teensy-3.1 Configuration settings
  o Configurations

STATUS
======

  2015-06-11:
    After some extended tinkering with the PLL setup, the Teensy-3.1 is
    fully functional using the basic NSH configuration.
  2015-06-12:
    Fix LED (need high drive strength).  Calibrate delay loop.

Pin Configuration
=================

  Nearly all pins are available to the user.  The few port pins used on
  board are listed below:

    ----- --------------------------------------
    P0RT  BOARD USAGE
    ----- --------------------------------------
    PTA0  MINI54TAN / Bootloader
    PTA1  MINI54TAN / Bootloader
    PTA2  MINI54TAN / Bootloader
    PTA3  MINI54TAN / Bootloader
    PTA18 16MHz XTAL  (XTAL32 is not populated).
    PTA19 16MHz XTAL
    PTB1  MINI54TAN / Bootloader
    PTB2  MINI54TAN / Bootloader
    PTC5  LED
    ----- --------------------------------------

Serial Console
==============

  The K20 has three UARTs with pin availability as follows:

    --------- ------ ----------- -------------------------
    UART      PORT   BOARD       PJRC PINOUT DESCRIPTION
    FUNCTION         LABEL
    --------- ------ ----------- -------------------------
    UART0_RX  PTA1   (See above) MINI54TAN / Bootloader
              PTB16  Pin 0       RX1 / Touch
              PTD6   Pin 21 / A7 RX1 / CS / PWM
    UART0_TX  PTA2   (See above) MINI54TAN / Bootloader
              PTB17  Pin 1       TX1 / Touch
              PTD7   Pin 5       TX1 / PWM
    --------- ------ ----------- -------------------------
    UART1_RX  PTC3   Pin 9       RX2 / CS / PWM
              PTE1   Pad 26      (Pad on back of board)
    UART1_TX  PTC4   Pin 10      TX2 / CS / PWM
              PTE0   Pad 31      (Pad on back of board)
    --------- ------ ----------- -------------------------
    UART2_RX  PTD2   Pin 7       RX3 / DOUT
    UART2_TX  PTD3   Pin 8       TX3 / DIN
    --------- ------ ----------- -------------------------

  The default serial console is UART0 on pins 0 (RX) and 1 (TX).

LEDs
====

  A single LED is available driven by PTC5.  The LED is grounded
  so bringing PTC5 high will illuminate the LED.

  When CONFIG_ARCH_LEDS is defined in the NuttX configuration, NuttX will
  control the LED as follows:

    SYMBOL              Meaning                 LED
    ------------------- ----------------------- ------
    LED_STARTED         NuttX has been started  OFF
    LED_HEAPALLOCATE    Heap has been allocated OFF
    LED_IRQSENABLED     Interrupts enabled      OFF
    LED_STACKCREATED    Idle stack created      ON
    LED_INIRQ           In an interrupt         N/C
    LED_SIGNAL          In a signal handler     N/C
    LED_ASSERTION       An assertion failed     N/C
    LED_PANIC           The system has crashed  FLASH

  Thus is LED is statically on, NuttX has successfully  booted and is,
  apparently, running normally.  If LED is flashing at approximately
  2Hz, then a fatal error has been detected and the system has halted.

Using the Halfkey Loader
========================

  See https://www.pjrc.com/teensy/first_use.html
      https://www.pjrc.com/teensy/loader_cli.html

Debugging
=========

  And, at this point, I don't know how to debug the board.  There is no
  way to connect a JTAG SWD debuggger, at least not without cutting leads
  to the MINI54TAN device:

    See: http://mcuoneclipse.com/2014/08/09/hacking-the-teensy-v3-1-for-swd-debugging/

Teensy-3.1 Configuration settings
=================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM4=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP="kinetis"

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_MK20DX256VLH7=y

    CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD="teensy-3.x"

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_TEENSY_3X=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_RAM_SIZE=0x00008000 (32Kb)

    CONFIG_RAM_START - The start address of installed DRAM

       CONFIG_RAM_START=0x20000000

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
        stack in bytes.  If not defined, the user task stacks will be
      used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

  Individual subsystems can be enabled:

    To be provided

Configurations
==============

  Common Configuration Information
  --------------------------------
  Each Teensy-3.x configurations are maintained in sub-directories and
  can be selected as follow:

    tools/configure.sh teensy-3.x:<subdir>
    make oldconfig

  Before building, make sure that your PATH environment variable includes
  the correct path to the directory than holds your toolchain binaries.

  If this is a Windows native build, then configure.bat should be used
  instead of configure.sh:

    configure.bat teensy-3.x\<subdir>

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply,
  nuttx.

    make oldconfig
    make

  The <subdir> that is provided above as an argument to the
  tools/configure.sh must be is one of the directories listed below.

NOTES:

  1. These configurations use the mconf-based configuration tool.  To
     change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output on [To be provided].

  3. All of these configurations are set up to build under Windows using the
     "GNU Tools for ARM Embedded Processors" that is maintained by ARM
     (unless stated otherwise in the description of the configuration).

       https://developer.arm.com/open-source/gnu-toolchain/gnu-rm

     That toolchain selection can easily be reconfigured using
     'make menuconfig'.  Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_WINDOWS=y               : Window environment
       CONFIG_WINDOWS_CYGWIN=y             : Cywin under Windows

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABI=y  : GNU ARM EABI toolchain

     NOTE: As of this writing, there are issues with using this tool at
     the -Os level of optimization.  This has not been proven to be a
     compiler issue (as least not one that might not be fixed with a
     well placed volatile qualifier).  However, in any event, it is
     recommend that you use not more that -O2 optimization.

  Configuration sub-directories
  -----------------------------

  nsh:

    Configures the NuttShell (nsh) located at apps/examples/nsh.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. By default, this configuration uses the ARM EABI toolchain
       for Windows and builds under Cygwin (or probably MSYS).  That
       can easily be reconfigured, of course.

       CONFIG_HOST_WINDOWS=y                   : Builds under Windows
       CONFIG_WINDOWS_CYGWIN=y                 : Using Cygwin
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABI=y      : GNU EABI toolchain for Windows

 Support
    for  builtin applications is enabled, but in the base configuration
    no builtin applications are selected (see NOTES below).
