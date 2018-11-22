README
======

  This directory holds NuttX board support for the Maxim Integrated
  MAX32660-EVSYS board.  That board features:

  o MAX32660 Microcontroller
    - Arm Cortex-M4F, 96MHz
    - 256KB Flash Memory
    - 96KB SRAM
    - 16KB Instruction Cache
    - Two SPIs
    - Two I2Cs
    - Two UARTs
    - 14 GPIOs
  o DIP Breakout Board
    - 100mil Pitch Dual Inline Pin Headers
    - Breadboard Compatible
  o Integrated Peripherals
    - Red Indicator LED
    - User Pushbutton
  o MAX32625PICO-Based Debug Adapter
    - CMSIS-DAP SWD Debugger
    - Virtual UART Console

Contents
========

  o Status
  o Serial Console
  o LEDs and Buttons

Status
======

  2018-11-21:  The port is code complete but completely untested.  I am
    still waiting to receive hardware to perform the bringup.  This initial
    port will support an NSH console:  Clock configuration, timer, GPIO
    pin configuration, ICC, and UART.  Additional untested drivers are
    complete and ready for testing:  DMA, GPIO interrupts, RTC, WDT.  The
    following drivers are not implemented: I2C, SPI, I2S.

Serial Console
==============

  UART1 Tx and Rx signals at port P0.10 and P0.11 are connected to the
  programming and debug header JH2 pins 2 and 3 through 1kΩ resistors.
  This provides a convenient way to communicate with a PC though the
  virtual serial port available in Maxim’s CMSIS-DAP debug adapter. The
  series resistors allow for these signals to be overdriven by other
  circuits without modifying the board.

LEDs and Buttons
================

LEDs
----

  A single red LED is available driven by GPIO P0.13.

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

  An single button is available on GPIO P0.12 for use by software.
