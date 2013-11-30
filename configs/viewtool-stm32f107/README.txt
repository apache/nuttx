README
======

  This README discusses issues unique to NuttX configurations for the
  ViewTool STM32F103/F107 V1.2 board.  This board may be fitted with either

  - STM32F107VCT6, or
  - STM32F103VCT6

  The board is vary module with connectors for a variety of peripherals.
  Features on the base board include:

  - User and Wake-Up Keys
  - LEDs

  http://www.viewtool.com/ for further information.

User and Wake-Up keys
=====================

  All pulled high and will be sensed low when depressed.

    SW2 PC11  Needs J42 closed
    SW3 PC12  Needs J43 closed
    SW4 PA0   Needs J44 closed

LEDs
====

  All pulled high and can be illuminated by driving the output to low

    LED1 PA6
    LED2 PA7
    LED3 PB12
    LED4 PB13

Serial Console
==============

  The boards come with a PL-2303 based USB-to-serial board.  Also available
  as an option is an RS-232 board.  Both have the same pin out on a 6-pin
  connector that mates with the upper row of J5.

  PIN MODULE BOARD J5
  --- ------ ---------------------------
   1   5V    1  POWER Power jumper
   2   GND   3  GND   Ground
   3   TXD   5  RXD1  PA10    USART1_RXD
   4   RXD   7  TXD1  PA9     USART1_TXD
   5   RTS?  9  CTS?  PA12    USART1_RTS
   6   CTS?  11 RTS?  PA11    USART1_CTS

   Note:  This requires USART1 pin remapping