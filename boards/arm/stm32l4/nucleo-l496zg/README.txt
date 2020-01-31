README
======

This README discusses issues unique to NuttX configurations for the STMicro
Nucleo-144 board for STM32L4 chips.

Contents
========

  - Nucleo-144 Boards
  - Nucleo L496ZG
  - Hardware
    - Button
    - LED
    - U[S]ARTs and Serial Consoles
    - SPI
    - SDIO - MMC
  - SPI Test
  - Configurations
     nsh

Nucleo-144 Boards:
=================

The Nucleo-144 is a standard board for use with several STM32 parts in the
LQFP144 package.  Variants with a STM32L4 MCU include:

  STM32 Part     Board Variant Name
  -------------  ------------------
  STM32L496ZGT6  NUCLEO-L496ZG
  STM32L496ZGT6P NUCLEO-L496ZG-P
  STM32L4A6ZGT6  NUCLEO-L4A6ZG
  STM32L4R5ZIT6  NUCLEO-L4R5ZI
  STM32L4R5ZIT6P NUCLEO-L4R5ZI-P

  ------------- ------------------

This directory supports only the STM32L4 variants of Nucleo-144. For others,
see boards/arm/stm32f7/nucleo-144 configuration.

Please read the User Manual UM2179: Getting started with STM32 Nucleo board
software development tools and take note of the Powering options for the
board (6.3 Power supply and power selection) and the Solder bridges based
hardware configuration changes that are configurable (6.11 Solder bridges).

Also note that UM1727 is not valid for L4 Nucleo-144 boards!

Common Board Features:
---------------------

  Peripherals:    8 leds, 2 push button (3 LEDs, 1 button) under software
                  control
  Debug:          STLINK/V2-1 debugger/programmer Uses a STM32F103CB to
                  provide a ST-Link for programming, debug similar to the
                  OpenOcd FTDI function - USB to JTAG front-end.

  Expansion I/F:  ST Zio and Extended Ardino and Morpho Headers

Nucleo L496ZG
=============

ST Nucleo L496ZG board from ST Micro is supported.  See

http://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-eval-tools/stm32-mcu-eval-tools/stm32-mcu-nucleo/nucleo-l496zg.html

The Nucleo L496ZG order part number is NUCLEO-L496ZG. It is one member of
the STM32 Nucleo-144 board family.

NUCLEO-L496ZG Features:
----------------------

  Microprocessor: STM32L496ZGT6 Core: ARM 32-bit Cortex®-M4 CPU with FPU,
                  80 MHz, MPU, and DSP instructions.
  Memory:         1024 KB Flash 320KB of SRAM (including 64KB of SRAM2)
  ADC:            3×12-bit: up to 24 channels
  DMA:            2 X 7-stream DMA controllers with FIFOs and burst support
  Timers:         Up to 13 timers: (2x 16-bit lowpower), two 32-bit timers,
                  2x watchdogs, SysTick
  GPIO:           114 I/O ports with interrupt capability
  LCD:            LCD-TFT Controller, Parallel interface
  I2C:            4 × I2C interfaces (SMBus/PMBus)
  U[S]ARTs:       3 USARTs, 2 UARTs (27 Mbit/s, ISO7816 interface, LIN, IrDA,
                  modem control)
  SPI/12Ss:       6/3 (simplex) (up to 50 Mbit/s), 3 with muxed simplex I2S
                  for audio class accuracy via internal audio PLL or external
                  clock
  QSPI:           Dual mode Quad-SPI
  SAIs:           2 Serial Audio Interfaces
  CAN:            2 X CAN interface
  SDMMC interface
  USB:            USB 2.0 full-speed device/host/OTG controller with on-chip
                  PHY
  Camera Interface: 8/14 Bit
  CRC calculation unit
  TRG:            True random number generator
  RTC

See https://developer.mbed.org/platforms/ST-Nucleo-L496ZG  for additional
information about this board.

Hardware
========
< Section needs updating >

  GPIO - there are 144 I/O lines on the STM32L4xxZx with various pins pined out
  on the Nucleo 144.

  Keep in mind that:
   1) The I/O is 3.3 Volt not 5 Volt like on the Arduino products.
   2) The Nucleo-144 board family has 3 pages of Solder Bridges AKA Solder
      Blobs (SB) that can alter the factory configuration. We will note SB
      in effect but will assume the factory default settings.

  Our main concern is establishing a console and LED utilization for
  debugging. Because so many pins can be multiplexed with so many functions,
  the above mentioned graphic may be helpful in identifying a serial port.

  There are 4 choices that can be made from the menuconfig:

  CONFIG_NUCLEO_CONSOLE_ARDUINO or CONFIG_NUCLEO_CONSOLE_MORPHO or
  CONFIG_NUCLEO_CONSOLE_VIRTUAL or CONFIG_NUCLEO_CONSOLE_NONE

  The CONFIG_NUCLEO_CONSOLE_NONE makes no preset for the console. You should still visit
  the U[S]ART selection and Device Drivers to disable any U[S]ART remaining.

  The CONFIG_NUCLEO_CONSOLE_ARDUINO configurations assume that you are using a
  standard Arduino RS-232 shield with the serial interface with RX on pin D0 and
  TX on pin D1 from USART6:

            -------- ---------------
                        STM32F7
            ARDUIONO FUNCTION  GPIO
            -- ----- --------- -----
            DO RX    USART6_RX PG9
            D1 TX    USART6_TX PG14
            -- ----- --------- -----

  The CONFIG_NUCLEO_CONSOLE_MORPHO configurations uses Serial Port 8 (USART8)
  with TX on PE1 and RX on PE0.

            Serial
            ------
            SERIAL_RX         PE_0
            SERIAL_TX         PE_1

  The CONFIG_NUCLEO_CONSOLE_VIRTUAL configurations uses Serial Port 3 (USART3)
  with TX on PD8 and RX on PD9.

            Serial
            ------
            SERIAL_RX         PD9
            SERIAL_TX         PD8

  These signals are internally connected to the on board ST-Link

  Of course if your design has used those pins you can choose a completely
  different U[S]ART to use as the console. In that Case, you will need to edit
  the include/board.h to select different U[S]ART and / or pin selections.

  Buttons
  -------
  B1 USER: the user button is connected to the I/O PC13 (Tamper support, SB173
           ON and SB180 OFF)

  LEDs
  ----
  The Board provides a 3 user LEDs, LD1-LD3
  LED1 (Green)      PB_0  (SB120 ON and SB119 OFF)
  LED2 (Blue)       PB_7  (SB139 ON)
  LED3 (Red)        PB_14 (SP118 ON)

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

Serial Consoles
===============

  USART3
  ------

  Default board is configured to use USART3 as console.

  Pins and Connectors:

    FUNC GPIO  Connector
                   Pin NAME
    ---- ---   ------- ----
    TXD: PC4   CN8-9,  A4
    RXD: PC5   CN8-11, A5
    ---- ---   ------- ----

  You must use a 3.3 TTL to RS-232 converter or a USB to 3.3V TTL

    Nucleo 144           FTDI TTL-232R-3V3
    -------------       -------------------
    TXD - CN8-9     -   RXD - Pin 5 (Yellow)
    RXD - CN8-11    -   TXD - Pin 4 (Orange)
    GND             -   GND   Pin 1  (Black)
    -------------       -------------------

    *Note you will be reverse RX/TX

  Use make menuconfig to configure USART3 as the console:

    CONFIG_STM32L4_USART3=y
    CONFIG_USART3_SERIALDRIVER=y
    CONFIG_USART3_SERIAL_CONSOLE=y
    CONFIG_USART3_RXBUFSIZE=256
    CONFIG_USART3_TXBUFSIZE=256
    CONFIG_USART3_BAUD=115200
    CONFIG_USART3_BITS=8
    CONFIG_USART3_PARITY=0
    CONFIG_USART3_2STOP=0

  USART2
  ------

  USART 2 could be used as console as well.

  Virtual COM Port
  ----------------
  Yet another option is to use LPUART1 and the USB virtual COM port.  This
  option may be more convenient for long term development, but is painful
  to use during board bring-up. However as LPUART peripheral has not been
  implemented for STM32L4, this cannot currently be used.

  Solder Bridges.  This configuration requires:

    PG7 LPUART1 TX SB131 ON and SB195 OFF (Default)
    PG8 LPUART1 RX SB130 ON and SB193 OFF (Default)

  Default
  -------
  As shipped, the virtual COM port is enabled.

SPI
---
  Since this board is so generic, having a quick way to vet the SPI
  configuration seams in order. So the board provides a quick test
  that can be selected vi CONFIG_NUCLEO_SPI_TEST that will initialize
  the selected buses (SPI1-SPI3) and send some text on the bus at
  application initialization time board_app_initialize.

SDIO
----
  To test the SD performance one can use a SparkFun microSD Sniffer
  from https://www.sparkfun.com/products/9419 or similar board
  and connect it as follows:

          VCC    V3.3 CN11  16
          GND    GND  CN11-8
          CMD    PD2  CN11-4
          CLK    PC12 CN11-3
          DAT0 - PC8  CN12-2
          DAT1 - PC9  CN12-1
          DAT2   PC10 CN11-1
          CD     PC11 CN11-2

Configurations
==============

nsh:
----
  Configures the NuttShell (nsh) located at apps/examples/nsh for the
  Nucleo-144 boards.  The Configuration enables the serial interfaces
  on USART6.  Support for builtin applications is enabled, but in the base
  configuration no builtin applications are selected (see NOTES below).

  NOTES:

  1. This configuration uses the mconf-based configuration tool.  To
     change this configuration using that tool, you should:

     a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
        see additional README.txt files in the NuttX tools repository.

     b. If this is the initial configuration then execute

           ./tools/configure.sh nucleo-l496zg:nsh

        in nuttx/ in order to start configuration process.
        Caution: Doing this step more than once will overwrite .config with
        the contents of the nucleo-l496zg/nsh/defconfig file.

     c. Execute 'make oldconfig' in nuttx/ in order to refresh the
        configuration.

     d. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

     e. Save the .config file to reuse it in the future starting at step d.

  2. By default, this configuration uses the ARM GNU toolchain
     for Linux.  That can easily be reconfigured, of course.

     CONFIG_HOST_LINUX=y                     : Builds under Linux
     CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIL=y     : ARM GNU for Linux

  3. Although the default console is LPUART1 (which would correspond to
     the Virtual COM port) I have done all testing with the console
     device configured for USART3 (see instruction above under "Serial
     Consoles).
