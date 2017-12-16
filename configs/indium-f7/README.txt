README
======

This README discusses issues unique to NuttX configurations for the RAF Research 
Indium-F7 board and using STMicro Nucleo-144 boards for interim support.  

Contents
========

  - Indium-F7 Boards
  - Indium-F7 interim boards (Nucleo-F722ZE, Nucleo-F746ZG, Nucleo-F767ZI)
  - Development Environment
  - Basic configuaration & build steps
  - Configurations
     f722-nsh, f746-nsh, and f767-nsh

Indium-F7 Boards:
=================

The Indium-F7 board is a is a special purpose board created by RAF Research LLC.
Currently very few Indium-F7 boards exist and those that do are undergoing hardware
feature checkout. However, it is possible to develop basic Indium-F7 software using
STMicro Nucleo-144 development boards. This config directory provides support for 
developing software on both native Indium-F7 hardware and for three STM32F7 Nucleo-144
development boards.

The configurations supported include:

  STM32 MCU      Board Variant   Config used
  -------------  -------------   ------------------
  STM32F722RET6  Indium-F7       indium-f7/f722-nsh Note1
  STM32F722ZET6  NUCLEO-F722ZE   indium-f7/f722-nsh Note1
  STM32F746ZGT6  NUCLEO-F746ZG   indium-f7/f746-nsh
  STM32F767ZIT6  NUCLEO-F767ZI   indium-f7/f767-nsh
  Note1: Chip selection ('R' vs 'Z') designates the board being used.
  ------------- ------------------

Common Board Features:
---------------------

  Peripherals:    4 leds, 1 push button (3 LEDs, 1 button) under software
                  control
  Debug:          Indium-F7 board need separate ST-Link/V2 programmers.
                  Nucleo have built-in ST-Link/V2 equivalent programmers.
  Serial Console: Indium-F7 boards require nsh console on UART4 (Morpho Connector).
                  Nucleo boards can use the UART4 (Morpho Connector) console or
                  the NuttX "Virtual Console" (USART3).

Basic configuration & build steps
==================================

  A GNU GCC-based toolchain is assumed.  The PATH environment variable should
  be modified to point to the correct path to the Cortex-M7 GCC toolchain (if
  different from the default in your PATH variable).

   - Configures nuttx creating .config file in the nuttx directory.
     $ cd tools && ./configure.sh indium-f7/f7nn-nsh && cd ..
   - Refreshes the .config file with the latest available configurations.
     $ make oldconfig
   - Select the features you want in the build.
     $ make menuconfig
   - Builds Nuttx with the features you selected.
     $ make

Nucleo Hardware Notes
=====================

  GPIO - there are 144 I/O lines on the STM32F7xxZxT6 with various pins pined out
  on the Nucleo 144.

  See https://developer.mbed.org/platforms/ST-Nucleo-F746ZG/ for slick graphic
  pinouts.

  Keep in mind that:
   1) The I/O is 3.3 Volt not 5 Volt like on the Arduino products.
   2) The Nucleo-144 board family has 3 pages of Solder Bridges AKA Solder
      Blobs (SB) that can alter the factory configuration. We will note SB
      in effect but will assume the facitory defualt settings.

  Our main concern is establishing a console and LED utilization for
  debugging. Because so many pins can be multiplexed with so many functions,
  the above mentioned graphic may be helpful in indentifying a serial port.

  There are 5 choices that can be made from the menuconfig:

  CONFIG_NUCLEO_CONSOLE_ARDUINO or CONFIG_NUCLEO_CONSOLE_MORPHO or
  CONFIG_NUCLEO_CONSOLE_MORPHO_UART4 or CONFIG_NUCLEO_CONSOLE_VIRTUAL or
  CONFIG_NUCLEO_CONSOLE_NONE

  For Indium software development we strongly recommend selecting 
  CONFIG_NUCLEO_CONSOLE_MORPHO_UART4. However, CONFIG_NUCLEO_CONSOLE_VIRTUAL
  is also supported when using Nucleo boards.

  The CONFIG_NUCLEO_CONSOLE_MORPHO_UART4 configurations uses Serial Port 4 (UART4)
  with TX on PA1 and RX on PA0. Zero Ohm resistor / solder short at
  SB13 must be removed/open. (Disables Ethernet MII clocking.)
          Serial
          ------
          SERIAL_RX         PA_1  CN11 30
          SERIAL_TX         PA_0  CN11 28

  The CONFIG_NUCLEO_CONSOLE_VIRTUAL configurations uses Serial Port 3 (USART3)
  with TX on PD8 and RX on PD9.
          Serial
            ------
            SERIAL_RX         PD9
            SERIAL_TX         PD8

  These signals are internally connected to the on-board ST-Link.

  Buttons
  -------
  The button is connected to the I/O PC15.

  LEDs
  ----
  The Board provides a 3 user LEDs, LD1-LD3
  LED1 (Green)      PC1
  LED2 (Blue)       PC6
  LED3 (Red)        PH1

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



  You must use a 3.3 TTL to RS-232 converter or a USB to 3.3V TTL

    Nucleo 144           FTDI TTL-232R-3V3
    -------------       -------------------
    TXD - CN11-61   -   RXD - Pin 5 (Yellow)
    RXD - CN12-64   -   TXD - Pin 4 (Orange)
    GND   CN12-63   -   GND   Pin 1  (Black)
    -------------       -------------------

    *Note you will be reverse RX/TX

  Virtual COM Port (CONFIG_NUCLEO_CONSOLE_VIRTUAL)
  ----------------
  Yet another option is to use USART3 and the USB virtual COM port.  This
  option may be more convenient for long term development, but is painful
  to use during board bring-up.

Configurations
==============

f7xx-nsh:
---------
  Configures the NuttShell (nsh) located at apps/examples/nsh for the
  Nucleo-144 boards.  The Configuration enables the serial interfaces
  on USART6.  Support for builtin applications is enabled, but in the base
  configuration no builtin applications are selected (see NOTES below).

  NOTES:

  1. This configuration uses the mconf-based configuration tool.  To
     change this configuration using that tool, you should:

     a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
        see additional README.txt files in the NuttX tools repository.

     b. If this is the intall configuration then Execute
           'cd tools && ./configure.sh indium-f7/nsh && cd ..'
        in nuttx/ in order to start configuration process.
        Caution: Doing this step more than once will overwrite .config with
        the contents of the indium-f7/nsh/defconfig file.

     c. Execute 'make oldconfig' in nuttx/ in order to refresh the
        configuration.

     d. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

     e. Save the .config file to reuse it in the future starting at step d.

  2. By default, this configuration uses the ARM GNU toolchain
     for Linux.  That can easily be reconfigured, of course.

     CONFIG_HOST_LINUX=y                     : Builds under Linux
     CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIL=y     : ARM GNU for Linux

  3. The serial console may be configured to use either USART3 (which would
     correspond to the Virtual COM port) or with the console device
     configured for USART6 to support an Arduino serial shield (see
     instructions above under "Serial Consoles).  You will need to check the
     defconfig file to see how the console is set up and, perhaps, modify
     the configuration accordingly.

     To select the Virtual COM port:

       -CONFIG_NUCLEO_CONSOLE_ARDUINO
       +CONFIG_NUCLEO_CONSOLE_VIRTUAL=y
       -CONFIG_USART6_SERIAL_CONSOLE=y
       +CONFIG_USART3_SERIAL_CONSOLE=y

     To select the Arduino serial shield:

       -CONFIG_NUCLEO_CONSOLE_VIRTUAL=y
       +CONFIG_NUCLEO_CONSOLE_ARDUINO
       -CONFIG_USART3_SERIAL_CONSOLE=y
       +CONFIG_USART6_SERIAL_CONSOLE=y

     Default values for other settings associated with the select USART should
     be correct.

f7xx-evalos:
------------
  This configuration is designed to test the features of the board.
    - Configures the NuttShell (nsh) located at apps/examples/nsh for the
      Nucleo-144 boards. The console is available on serial interface USART3,
      which is accessible over the USB ST-Link interface.
    - Configures nsh with advanced features such as autocompletion.
    - Configures the on-board LEDs to work with the 'leds' example app.
    - Configures the 'helloxx' example app.

  NOTES:

  1. This configuration uses the mconf-based configuration tool.  To
    change this configuration using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. If this is the intall configuration then Execute
          'cd tools && ./configure.sh indium-f7/evalos && cd ..'
       in nuttx/ in order to start configuration process.
       Caution: Doing this step more than once will overwrite .config with
       the contents of the indium-f7/evalos/defconfig file.

    c. Execute 'make oldconfig' in nuttx/ in order to refresh the
       configuration.

    d. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

    e. Save the .config file to reuse it in the future starting at step d.

  2. By default, this configuration uses the ARM GNU toolchain
    for Linux.  That can easily be reconfigured, of course.

    CONFIG_HOST_LINUX=y                     : Builds under Linux
    CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIL=y     : ARM GNU for Linux
