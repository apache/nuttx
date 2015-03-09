README
======

This README file discusses the port of NuttX to the Atmel SAM V71 Xplained
Ultra Evaluation Kit (SAMV71-XULT).  This board features the ATSAMV71Q21 Cortex-M7
microcontroller.

Contents
========

  - Board Features
  - Serial Console
  - LEDs and Buttons
  - Configurations

Board Features
==============

  - ATSAMV71Q21 microcontroller: Cortex-M7, 300MHz, 2MiB FLASH, 384KiB SRAM,
    I/D-caches
  - One mechanical reset button
  - One power switch button
  - Two mechanical user pushbuttons
  - Two yellow user LEDs
  - Supercap backup
  - 12.0 MHz crystal
  - 32.768 kHz crystal
  - 2 MB SDRAM
  - 2 MB QSPI Flash
  - IEEE 802.3az 10Base-T/100Base-TX Ethernet RMII PHY
  - AT24MAC402 256KByte EEPROM with EUI-48 address
  - WM8904 stereo audio codec
  - ATA6561 CAN Transceiver
  - SD Card connector with SDIO support
  - Camera interface connector
  - MediaLB connector
  - Two Xplained Pro extension headers
  - One Xplained Pro LCD header
  - Coresight 20 connector for 4-bit ETM
  - Arduino due compatible shield connectors
  - External debugger connector
  - USB interface, device and host mode
  - Embedded Debugger with Data Gateway Interface and Virtual COM port (CDC)
  - External power input (5-14V) or USB powered

See the Atmel webite for further information about this board:

  - http://www.atmel.com/tools/atsamv71-xult.aspx

Serial Console
==============

The SAMV71-XULT has no on-board RS-232 drivers so it will be necessary to
use either the VCOM or an external RS-232 driver.  Here are some options.

  - Arduino Serial Shield:  One option is to use an Arduino-compatible
    serial shield.  This will use the RXD and TXD signals available at pins
    0 an 1, respectively, of the Arduino "Digital Low" connector.  On the
    SAMV71-XULT board, this corresponds to UART3:

    ------ ------ ------- ------- --------
    Pin on SAMV71 Arduino Arduino SAMV71
    J503   PIO    Name    Pin     Function
    ------ ------ ------- ------- --------
      1    PD28   RX0     0       URXD3
      2    PD30   TX0     1       UTXD3
    ------ ------ ------- ------- --------

  - SAMV7-XULT EXTn connectors.  USART pins are also available the EXTn
    connectors.  The following are labelled in the User Guide for USART
    functionality:

    ---- -------- ------ --------
    EXT1 EXTI1    SAMV71 SAMV71
    Pin  Name     PIO    Function
    ---- -------- ------ --------
    13   USART_RX PB00   RXD0
    14   USART_TX PB01   TXD0

    ---- -------- ------ --------
    EXT2 EXTI2    SAMV71 SAMV71
    Pin  Name     PIO    Function
    ---- -------- ------ --------
    13   USART_RX PA21   RXD1
    14   USART_TX PB04   TXD1

  - VCOM.  The Virtual Com Port gateway is available on USART1:

    ------ --------
    SAMV71 SAMV71
    PIO    Function
    ------ --------
    PB04   TXD1
    PA21   RXD1
    ------ --------

Any of these options can be selected as the serial console by:

  1. Enabling the UART/USART peripheral in the
     "System Type -> Peripheral Selection" menu, then
  2. Configuring the peripheral in the "Drivers -> Serial Configuration"
     menu.

LEDs and Buttons
================

LEDs
----
There are two yellow LED available on the SAM V71 Xplained Ultra board that
can be turned on and off.  The LEDs can be activated by driving the
connected I/O line to GND.

  ------ ----------- ---------------------
  SAMV71 Function    Shared functionality
  PIO
  ------ ----------- ---------------------
  PA23   Yellow LED0 EDBG GPIO
  PC09   Yellow LED1 LCD, and Shield
  ------ ----------- ---------------------

These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
defined.  In that case, the usage by the board port is defined in
include/board.h and src/sam_autoleds.c. The LEDs are used to encode
OS-related events as follows:

  -------------------  -----------------------  -------- --------
  SYMBOL                Meaning                     LED state
                                                  LED0     LED1
  -------------------  -----------------------  -------- --------
  LED_STARTED          NuttX has been started     OFF      OFF
  LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF
  LED_IRQSENABLED      Interrupts enabled         OFF      OFF
  LED_STACKCREATED     Idle stack created         ON       OFF
  LED_INIRQ            In an interrupt              No change
  LED_SIGNAL           In a signal handler          No change
  LED_ASSERTION        An assertion failed          No change
  LED_PANIC            The system has crashed     N/C      Blinking
  LED_IDLE             MCU is is sleep mode         Not used
  -------------------  -----------------------  -------- --------

Thus if LED0 is statically on, NuttX has successfully booted and is,
apparently, running normally.  If LED1 is flashing at approximately
2Hz, then a fatal error has been detected and the system has halted.

NOTE: That LED0 is not used after completion of booting and may
be used by other board-specific logic.

Buttons
-------
SAM V71 Xplained Ultra contains three mechanical buttons. One button is the
RESET button connected to the SAM V71 reset line and the others are generic
user configurable buttons. When a button is pressed it will drive the I/O
line to GND.

  ------ ----------- ---------------------
  SAMV71 Function    Shared functionality
  PIO
  ------ ----------- ---------------------
  RESET  RESET       Trace, Shield, and EDBG
  PA09   SW0         EDBG GPIO and Camera
  PB12   SW1         EDBG SWD and Chip Erase
  ------ ----------- ---------------------

NOTES:

  - There are no pull-up resistors connected to the generic user buttons so
    it is necessary to enable the internal pull-up in the SAM V71 to use the
    button.
  - PB12 is set up as a system flash ERASE pin when the firmware boots. To
    use the SW1, PB12 has to be configured as a normal regular I/O pin in
    the MATRIX module. For more information see the SAM V71 datasheet.

Configurations
==============
