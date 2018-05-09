README
======

This README discusses issues unique to NuttX configurations for the
STMicro STM32F769I-DISCO development board featuring the STM32F769NIH6
MCU. The STM32F769NIH6 is a 216MHz Cortex-M7 operating with 2048K Flash
memory and 512Kb SRAM. The board features:

  - On-board ST-LINK/V2 for programming and debugging,
  - Mbed-enabled (mbed.org)
  - 4-inch 800x472 color LCD-TFT with capacitive touch screen
  - SAI audio codec
  - Audio line in and line out jack
  - Two ST MEMS microphones
  - SPDIF RCA input connector
  - Two pushbuttons (user and reset)
  - 512-Mbit Quad-SPI Flash memory
  - 128-Mbit SDRAM
  - Connector for microSD card
  - RF-EEPROM daughterboard connector
  - USB OTG HS with Micro-AB connectors
  - Ethernet connector compliant with IEEE-802.3-2002 and PoE

Refer to the http://www.st.com website for further information about this
board (search keyword: stm32f769i-disco)

Contents
========

  - STATUS
  - Development Environment
  - LEDs and Buttons
  - Serial Console
  - Configurations

STATUS
======

  2017-07:  The basic NSH configuration is functional using a serial
    console on USART1, which is connected to the "virtual com port"
    of the ST/LINK USB adapter.

  2017-07:  STM32 F7 Ethernet appears to be functional, but has had
    only light testing.

  Work in progress: Use LCD over DSI interface, rest of board.

Development Environment
=======================

  The Development environments for the STM32F769I-DISCO board are identical
  to the environments for other STM32F boards.  For full details on the
  environment options and setup, see the README.txt file in the
  config/stm32f769i-disco directory.

LEDs and Buttons
================

  LEDs
  ----
  The STM32F769I-DISCO board has numerous LEDs but only one, LD3 located
  near the reset button, that can be controlled by software.

  LD3 is controlled by PI1 which is also the SPI2_SCK at the Arduino
  interface.  One end of LD3 is grounded so a high output on PI1 will
  illuminate the LED.

  This LED is not used by the board port unless CONFIG_ARCH_LEDS is defined.
  In that case, the usage by the board port is defined in include/board.h
  and src/stm32_leds.c. The LEDs are used to encode OS-related events as
  follows:

    SYMBOL              Meaning                 LD3
    ------------------- ----------------------- ------
    LED_STARTED         NuttX has been started  OFF
    LED_HEAPALLOCATE    Heap has been allocated OFF
    LED_IRQSENABLED     Interrupts enabled      OFF
    LED_STACKCREATED    Idle stack created      ON
    LED_INIRQ           In an interrupt         N/C
    LED_SIGNAL          In a signal handler     N/C
    LED_ASSERTION       An assertion failed     N/C
    LED_PANIC           The system has crashed  FLASH

  Thus is LD3 is statically on, NuttX has successfully  booted and is,
  apparently, running normally.  If LD3 is flashing at approximately
  2Hz, then a fatal error has been detected and the system has halted.

  Buttons
  -------
  Pushbutton B1, labelled "User", is connected to GPIO PI11.  A high
  value will be sensed when the button is depressed.

Serial Console
==============

  Use the serial interface the ST/LINK provides to the USB host.

Configurations
==============

  Common Configuration Information
  --------------------------------
  Each STM32F769I-DISCO configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh stm32f769i-disco/<subdir>

  Where <subdir> is one of the sub-directories listed below.

Configuration Directories
-------------------------

  nsh:
  ---
    Configures the NuttShell (NSH) located at apps/examples/nsh.  The
    Configuration enables the serial interfaces on UART1.
    Otherwise nothing is enabled, so that config is a starting point
    for initial testing.
    Support for builtin applications is enabled, but in the base
    configuration no builtin applications are selected.

  nsh-ehternet:
  ---
    Same as above but a lot more hardware peripherals enabled,
    in particular ethernet, as well as networking stuff.
