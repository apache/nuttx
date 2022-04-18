README for the XMC4500 Relax
============================

  The directory provides board support for the Infineon XMC4500 Relax v1
  boards.  There are to variants of this board:  There is a Lite version
  that has fewer features, for example, no 32.768KHz crystal.

  The current configurations support only the Lite version of the board.

Status
======

  2017-03-21:   The XMC4500 Relax boots into NSH, provides the NSH prompt,
    and the LEDs are working.  But there is a problem with serial input.
    The most likely reason for this is there are no serial RX interripts.

Serial Console
==============

  Be default, UART0 (aka, USIC0, channel 0) is used as the serial console.
  The RX and TX pins is available:

    RX     - P1.4, Connector X2, pin 17
    TX     - P1.5, Connector X2, pin 16
    GND    -       Available on pins 1-4 of either connector X1 or X2
    VDD3.3 -       Available on pins 37-38 of either connector X1 or X2
    VDD5   -       Available on pins 39-40 of either connector X1 or X2

  A TTL to RS-232 converter or a USB TTL-to-USB serial adaptor is required.
  The notion of what is TX and what is RX depends on your point of view.
  With the TTL to RS-232 converter, I connect pin 17 to the pin labeled
  TX on the converter and pin 16 to the RX pin on the converter.

LEDs
====

  The XMC4500 Relax Lite v1 board has two LEDs:

    LED1 P1.1 High output illuminates
    LED2 P1.0 High output illuminates

  If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
  any way.  The definitions provided in the board.h header file can be used
  to access individual LEDs.

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/sam_autoleds.c. The LEDs are used to encode
  OS-related events as follows:

    SYMBOL              Meaning                  LED state
                                                LED1   LED2
    ------------------ ------------------------ ------ ------
    LED_STARTED        NuttX has been started   OFF    OFF
    LED_HEAPALLOCATE   Heap has been allocated  OFF    OFF
    LED_IRQSENABLED    Interrupts enabled       OFF    OFF
    LED_STACKCREATED   Idle stack created       ON     OFF
    LED_INIRQ          In an interrupt           No change
    LED_SIGNAL         In a signal handler       No change
    LED_ASSERTION      An assertion failed       No change
    LED_PANIC          The system has crashed   N/C  Blinking
    LED_IDLE           MCU is is sleep mode      Not used

  Thus if LED1 is statically on, NuttX has successfully booted and is,
  apparently, running normally.  If LED2 is flashing at approximately
  2Hz, then a fatal error has been detected and the system has halted.

Buttons
=======

  The XMC4500 Relax Lite v1 board has two buttons:

    BUTTON1 P1.14 Low input sensed when button pressed
    BUTTON2 P1.15 Low input sensed when button pressed

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each XMC4500 Relax configuration is maintained in a sub-directory and
  can be selected as follow:

    .tools/configure.sh xmc5400-relax:<subdir>

  See '.tools/configure.sh -h' for a list of all options.  The most typical
  are -l to select the Linux host or -c to select the Windows Cygwin host.

  Before starting the build, make sure that your PATH environment variable
  includes the correct path to your toolchain.

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
       output on UART0 (aka USIC0, channel 0) as described above under
       "Serial Console".  The relevant configuration settings are listed
       below:

         CONFIG_XMC4_USIC0=y
         CONFIG_XMC4_USIC0_CHAN0_ISUART=y
         CONFIG_XMC4_USIC0_CHAN1_NONE=y

         CONFIG_UART0_SERIALDRIVER=y
         CONFIG_UART0_SERIAL_CONSOLE=y

         CONFIG_UART0_RXBUFSIZE=256
         CONFIG_UART0_TXBUFSIZE=256
         CONFIG_UART0_BAUD=115200
         CONFIG_UART0_BITS=8
         CONFIG_UART0_PARITY=0
         CONFIG_UART0_2STOP=0

    3. All of these configurations are set up to build under Windows using
       the  "GNU Tools for ARM Embedded Processors" that is maintained by
       ARM (unless stated otherwise in the description of the configuration).

         https://developer.arm.com/open-source/gnu-toolchain/gnu-rm

       That toolchain selection can easily be reconfigured using
       'make menuconfig'.  Here are the relevant current settings:

       Build Setup:
         CONFIG_HOST_WINDOWS=y               : Window environment
         CONFIG_WINDOWS_CYGWIN=y             : Cywin under Windows

       System Type -> Toolchain:
         CONFIG_ARMV7M_TOOLCHAIN_GNU_EABI=y  : GNU ARM EABI toolchain

  Configuration sub-directories
  -----------------------------

  nsh:

    Configures the NuttShell (nsh) located at examples/nsh.  This
    configuration is focused on low level, command-line driver testing.  It
    has no network.

    NOTES:

    1. NSH built-in applications are supported.

       Binary Formats:
         CONFIG_BUILTIN=y           : Enable support for built-in programs

       Application Configuration:
         CONFIG_NSH_BUILTIN_APPS=y  : Enable starting apps from NSH command line

SPI
===

  Using MAX6675 Thermocouple
  --------------------------

  There is a board support to use a MAX6675 connected to SPI2. In other to use
  it you need to enable these options:

    CONFIG_XMC4_USIC=y
    CONFIG_XMC4_USCI_UART=y
    CONFIG_XMC4_USCI_SPI=y
    CONFIG_XMC4_SPI2=y
    CONFIG_XMC4_USIC1=y
    CONFIG_XMC4_USIC1_CHAN0_ISSPI=y
    CONFIG_XMC4_USIC1_CHAN1_ISUART=y
    CONFIG_UART3_SERIAL_CONSOLE=y
    CONFIG_SENSORS_MAX6675=y

  These are the used SPI pins: SCLK = P0.11, MISO = P0.4 and CS = P0.2
