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

  o Serial Console
  o LEDs and Buttons

Serial Console
==============

  A VCOM serial console is available.  This is provided by UART1 via pins
  P0.10 and P0.11.

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
