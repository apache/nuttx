README
======

  This README file provides information about the port of NuttX to the NXP
  i.MXRT evaluation kit, MIMXRT1050-EVKB.  This board features the
  MIMXRT1052DVL6A MCU.  Some of the features of this board include:

    o Processor

      - MIMXRT1052DVL6A processor

    o Memory

      - 256 Mb SDRAM memory
      - 512 Mb Hyper Flash
      - Footprint for QSPI Flash
      - TF socket for SD card

    o Display and Audio

      - Parallel LCD connector
      - Camera connector
      - Audio CODEC
      - 4-pole audio headphone jack
      - External speaker connection
      - Microphone
      - SPDIF connector

    o Connectivity

      - Micro USB host and OTG connectors
      - Ethernet (10/100T) connector
      - CAN transceivers
      - Arduino® interface

Contents
========

  o Serial Console
  o LEDs and buttons
  o Configurations
    - Configuration sub-directories

Serial Console
==============

  Virtual console port provided by OpenSDA:

           UART1_TXD   GPIO_AD_B0_12  LPUART1_TX
           UART1_RXD   GPIO_AD_B0_13  LPUART1_RX

  Arduino RS-232 Shield:

    J22 D0 UART_RX/D0  GPIO_AD_B1_07  LPUART3_RX
    J22 D1 UART_TX/D1  GPIO_AD_B1_06  LPUART3_TX

LEDs and buttons
================

  LEDs
  ----

  There are four LED status indicators located on the EVK Board.  The
  functions of these LEDs include:

    - Main Power Supply(D3)
      Green: DC 5V main supply is normal.
      Red:   J2 input voltage is over 5.6V.
      Off:   The board is not powered.
    - Reset RED LED(D15)
    - OpenSDA LED(D16)
    - USER LED(D18)

  Only a single LED, D18, is under software control.  It connects to
  GPIO_AD_B0_09 which is shared with JTAG_TDI and ENET_RST

  This LED is not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/sam_autoleds.c. The LED is used to encode
  OS-related events as follows:

    ------------------- ----------------------- ------
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

  Thus if the LED is statically on, NuttX has successfully  booted and is,
  apparently, running normally.  If the LED is flashing at approximately
  2Hz, then a fatal error has been detected and the system has halted.

  Buttons
  -------

  There are four user interface switches on the MIMXRT1050 EVK Board:

    - SW1: Power Switch (slide switch)
    - SW2: ON/OFF Button
    - SW3: Reset button
    - SW8: User button

  Only the user button is available to the software.  It is sensed on the
  WAKEUP pin which will be pulled low when the button is pressed.

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each i.MX RT 10050 configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh [OPTIONS] imxrt1050-evk/<subdir>

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
     output on UART3 (i.e., for the Arduino serial shield).

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

       The following configuration options should be added to your
       configuration in order to use the network monitor:

         CONFIG_IMXRT_ENET_PHYINIT=y
         CONFIG_IMXRT_GPIO1_0_15_IRQ=y
         CONFIG_IMXRT_GPIO_IRQ=y
         CONFIG_NETDEV_IOCTL=y
         CONFIG_NETDEV_PHY_IOCTL=y
         CONFIG_NSH_NETINIT_MONITOR=y
         CONFIG_NSH_NETINIT_RETRYMSEC=2000
         CONFIG_NSH_NETINIT_SIGNO=18
         CONFIG_NSH_NETINIT_THREAD=y
         CONFIG_NSH_NETINIT_THREAD_PRIORITY=80
         CONFIG_NSH_NETINIT_THREAD_STACKSIZE=1568

       STATUS: As of this writing, I get a hardfault when I enable the PHY
       interrupt so I suspect that there is something incorrect in that
       pin configuration.

  nsh:

    Configures the NuttShell (nsh) located at examples/nsh.  This NSH
    configuration is focused on low level, command-line driver testing.
    Built-in applications are supported, but none are enabled.  This
    configuration does not support a network.
