README
======

README for NuttX port to the IMXRT1020-EVK, an IMXRT1020 based
board with various capabilities, featuring the NXP
IMXRT1021DAG5A CPU.

    o Processor

      - MIMXRT1021DAG5A processor

    o Memory

      - 256 Mb SDRAM memory
      - 64 Mb QSPI Flash
      - TF socket for SD card

    o Display and Audio

      - Audio CODEC
      - 4-pole audio headphone jack
      - External speaker connection
      - Microphone

    o Connectivity

      - Micro USB host and OTG connectors
      - Ethernet (10/100T) connector
      - CAN transceivers
      - Arduino® interface

Serial Console
==============

  The EVK default console is on LPUART1, which is multiplexed onto
  the debug port (either OpenSDA or SEGGER JLink).

  It runs at 115200,n,8,1.

LEDs and Buttons
================

  There is one user accessible LED status indicator located on the 1020-EVK,
  USERLED.  The function of the LEDs include:

    D3: Power (Green) & Overpower (Red)
    D5: User LED (Green) GPIO_AD_B0_05
    D15: RST LED (Red)

  This LED is not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/imxrt_autoleds.c. The LED is used to encode
  OS-related events as documented in board.h

     ---------------------------------------------------
     SYMBOL                    Meaning           USERLED
    ---------------------------------------------------

    LED_STARTED          NuttX has been started  OFF
    LED_HEAPALLOCATE     Heap has been allocated OFF
    LED_IRQSENABLED      Interrupts enabled      OFF
    LED_STACKCREATED     Idle stack created      ON
    LED_INIRQ            In an interrupt         N/C
    LED_SIGNAL           In a signal handler     N/C
    LED_ASSERTION        An assertion failed     N/C
    LED_PANIC            The system has crashed  FLASH
    LED_IDLE             Not used

  In addition the LED is illuminated during an interrupt.

  This IMXRT board has three external buttons

    1. SW2 (IRQ88, ONOFF)  Not on a GPIO, No muxing
    2. SW3 (IRQ88, POR)    Not on a GPIO, No muxing
    3. SW4 (IRQ88, USER)   Wakeup, GPIO5-0

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------

  Each i.MX RT 1020 configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh [OPTIONS] imxrt1020-evk:<subdir>

  Where typical options are -l to configure to build on Linux or -c to
  configure for Cygwin under Linux.  'tools/configure.sh -h' will show
  you all of the options.

  Before building, make sure the PATH environment variable include the
  correct path to the directory than holds your toolchain binaries.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

    make

  The <subdir> that is provided above as an argument to the tools/configure.sh
  must be is one of the following.

  NOTES:

  1. These configurations use the mconf-based configuration tool.  To
    change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output on UART1 (i.e. the multiplexed OpenSDA/JLink serial port).

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
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y : GNU ARM EABI toolchain

Configuration sub-directories
-----------------------------

  netnsh:

    This configuration is similar to the nsh configuration except that is
    has networking enabled, both IPv4 and IPv6.  This NSH configuration is
    focused on network-related testing.

    NOTES:

    1. LED support is disabled because there is a conflict between the LED
       GPIO and PHY pin usage.

    2. Telnet is enabled.  But since both IPv4 and IPv6 are enabled, it
       will default to IPv6.  That means that to connect a Telnet session
       from a PC, you will need to use the IPv6 address which by defaault
       is:

         telnet fc00::2

       Or, disable IPv4 support so that only IPv4 addressing is used.

    3. The network monitor is not enabled in this configuration.  As a
       result, the Ethernet cable must be connected when the board is
       powered up.  Otherwise, it will stall for a long period of time
       before the NSH prompt appears and you will not be able to used
       the board.

  nsh:

    Configures the NuttShell (nsh) located at examples/nsh.  This NSH
    configuration is focused on low level, command-line driver testing.
    Built-in applications are supported, but none are enabled.  This
    configuration does not support a network.
