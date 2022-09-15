Nucleo-L552ZE-Q README
======================

  This README file discusses the port of NuttX to the STMicro Nucleo-L552ZE
  board.  That board features the STM32L552ZET6Q MCU with 512KiB of FLASH
  and 256KiB of SRAM.

Contents
========

  - Status
  - LEDs
  - Buttons
  - Serial Console
  - Configurations

Status
======
  2021-03-03: The board now boots and the basic NSH configuration works
    without problem.

LEDs
====

  The Board provides a 3 user LEDs, LD1-LD3
  LED1 (Green)      PC_7
  LED2 (Blue)       PB_7
  LED3 (Red)        PA_9

    - When the I/O is HIGH value, the LEDs are on.
    - When the I/O is LOW, the LEDs are off.

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/stm32_autoleds.c. The LEDs are used to encode OS
  related events as follows when the LEDs are available:

  SYMBOL                Meaning                  RED  GREEN BLUE
  -------------------  -----------------------   ---  ----- ----

  LED_STARTED          NuttX has been started    OFF  OFF   OFF
  LED_HEAPALLOCATE     Heap has been allocated   OFF  OFF   ON
  LED_IRQSENABLED      Interrupts enabled        OFF  ON    OFF
  LED_STACKCREATED     Idle stack created        OFF  ON    ON
  LED_INIRQ            In an interrupt           NC   NC    ON  (momentary)
  LED_SIGNAL           In a signal handler       NC   ON    OFF (momentary)
  LED_ASSERTION        An assertion failed       ON   NC    ON  (momentary)
  LED_PANIC            The system has crashed    ON   OFF   OFF (flashing 2Hz)
  LED_IDLE             MCU is is sleep mode      ON   OFF   OFF

OFF -    means that the OS is still initializing. Initialization is very fast
         so if you see this at all, it probably means that the system is
         hanging up somewhere in the initialization phases.

GREEN -  This means that the OS completed initialization.

BLUE  -  Whenever and interrupt or signal handler is entered, the BLUE LED is
         illuminated and extinguished when the interrupt or signal handler
         exits.

VIOLET - If a recovered assertion occurs, the RED and blue LED will be
         illuminated briefly while the assertion is handled.  You will
         probably never see this.

Flashing RED - In the event of a fatal crash, all other LEDs will be
          extinguished and RED LED will FLASH at a 2Hz rate.

  Thus if the GREEN LED is lit, NuttX has successfully booted and is,
  apparently, running normally.  If the RED LED is flashing at
  approximately 2Hz, then a fatal error has been detected and the system has
  halted.

Buttons
=======

  B1 USER: the user button is connected to the I/O PC13 (pin 2) of the STM32
  microcontroller.

Serial Consoles
===============

  USART3
  ------

  Default board is configured to use USART3 as console.

  Pins and Connectors:

    FUNC GPIO  Connector
                   Pin  NAME
    ---- ---   ------- -----
    TXD: PD8   CN10-14 D1 TX
    RXD: PD9   CN10-16 D0 RX
    ---- ---   ------- -----

  You must use a 3.3 TTL to RS-232 converter or a USB to 3.3V TTL

    Nucleo 144           FTDI TTL-232R-3V3
    -------------       -------------------
    TXD - CN10-14   -   RXD - Pin 5 (Yellow)
    RXD - CN10-16   -   TXD - Pin 4 (Orange)
    GND             -   GND   Pin 1  (Black)
    -------------       -------------------

    *Note you will be reverse RX/TX

  Use make menuconfig to configure USART3 as the console:

    CONFIG_STM32L5_USART3=y
    CONFIG_USART3_SERIALDRIVER=y
    CONFIG_USART3_SERIAL_CONSOLE=y
    CONFIG_USART3_RXBUFSIZE=256
    CONFIG_USART3_TXBUFSIZE=256
    CONFIG_USART3_BAUD=115200
    CONFIG_USART3_BITS=8
    CONFIG_USART3_PARITY=0
    CONFIG_USART3_2STOP=0

  Virtual COM Port
  ----------------
  Yet another option is to use LPUART1 and the USB virtual COM port.  This
  option may be more convenient for long term development, but is painful
  to use during board bring-up. However as LPUART peripheral has not been
  implemented for STM32L5, this cannot currently be used.

  Solder Bridges.  This configuration requires:

    PG7 LPUART1 TX SB127 ON and SB124 OFF (Default)
    PG8 LPUART1 RX SB129 ON and SB126 OFF (Default)

  You can also put USART3 on the virtual COM port by reworking the solder
  bridges as follows:

    PD8 USART3 TX SB127 OFF and SB124 ON
    PD9 USART3 RX SB129 OFF and SB126 ON

  Default
  -------
  As shipped, the virtual COM port is enabled.

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each configuration is maintained in a sub-directory and can be
  selected as follow:

    tools/configure.sh nucleo-l552ze:<subdir>

  Before building, make sure the PATH environment variable includes the
  correct path to the directory than holds your toolchain binaries.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

    make oldconfig
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
       output on USART3, as described above under "Serial Console".  The
       elevant configuration settings are listed below:

         CONFIG_STM32L5_USART3=y
         CONFIG_STM32L5_USART3_SERIALDRIVER=y
         CONFIG_STM32L5_USART=y

         CONFIG_USART3_SERIALDRIVER=y
         CONFIG_USART3_SERIAL_CONSOLE=y

         CONFIG_USART3_RXBUFSIZE=256
         CONFIG_USART3_TXBUFSIZE=256
         CONFIG_USART3_BAUD=115200
         CONFIG_USART3_BITS=8
         CONFIG_USART3_PARITY=0
         CONFIG_USART3_2STOP=0

  3. All of these configurations are set up to build under Linux using the
     "GNU Tools for ARM Embedded Processors" that is maintained by ARM
     (unless stated otherwise in the description of the configuration).

       https://developer.arm.com/open-source/gnu-toolchain/gnu-rm

     That toolchain selection can easily be reconfigured using
     'make menuconfig'.  Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_LINUX=y                 : Linux environment

     System Type -> Toolchain:
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y : GNU ARM EABI toolchain

  Configuration sub-directories
  -----------------------------

  nsh:

    Configures the NuttShell (nsh) located at examples/nsh.  This
    configuration is focused on low level, command-line driver testing.

